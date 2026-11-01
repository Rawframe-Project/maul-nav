# Bake

How a tile is made from host geometry, how its bytes are laid out and checked, and the two structures that make many bakes cheaper: the tile index and the tile cache. The entry points are in `src/baker.c`; each stage is a module of its own.

## Contents

- [Settings and input](#settings-and-input)
- [The pipeline](#the-pipeline)
- [The tile format](#the-tile-format)
- [The fingerprint](#the-fingerprint)
- [The tile index](#the-tile-index)
- [The tile cache](#the-tile-cache)

## Settings and input

A def's settings are meters (the slope in degrees), what a person picks. The bake works in cells, and the conversion must be the same everywhere: `src/bake_def.c` divides in binary32, counts a quotient within 2^-10 of an integer as that integer, then rounds heights and the radius up and the step height down, so that no agent loses a cell of clearance to rounding. `mnavValidateBakeDef` reports the result, which a tool can show.

Input is checked before any work (`src/input.c`, `src/bake_input.c`): NaN coordinates, indices out of range and area types past 63 (`mnav_errorInvalid`) and points past the extent, infinities included (`mnav_errorLimit`), are refused, naming the first vertex, then triangle. The extent (2^22 cells on the ground, 32,767 cell heights up or down) keeps every later integer within 32 bits. Degenerate and duplicate triangles are accepted and contribute nothing, since real level geometry has them.

## The pipeline

Each tile bakes alone, from the input that reaches it, with a border of cells beyond its sides: the agent's radius rounded up to cells, and three more. The border lets erosion and region tracing see across the tile's sides, so two neighboring tiles agree on their shared side without either seeing the other.

1. **Rasterize** (`raster.c`): every cell a triangle covers gets the lowest and highest height of the part of the triangle inside it, a fragment. This is the last stage in floating point. Terrains (`terrain.c`) give the triangles of only the cells near the tile; 2D outlines (`outline.c`) fill cells by the even-odd rule as a flat floor instead.
2. **Heightfield** (`heightfield.c`): fragments merge into solid spans per column, sorted so that the result depends only on the set of input triangles, never on their order.
3. **Walkable filters** (`filter.c`): which span tops an agent can stand on, by slope, by the free height above, and by the step at ledges. 2D bakes skip them, as they judge heights.
4. **Open space** (`compact.c`): the walkable span tops become the open space above them, each linked to the spans an agent can step to in the four directions.
5. **Volumes and erosion** (`volume.c`, `erode.c`): include and exclude volumes drop ground first, then the open space within the agent's radius of a boundary becomes unwalkable, so agents keep clear of what the volumes dropped as of a wall. Area volumes then set the area type of what is left, without shrinking it.
6. **Regions** (`region.c`): the open space is cut into regions by layers, monotone sweeps per row merged into non-overlapping layers of one area. Layers never overlap themselves, which keeps multi-floor buildings apart. Islands smaller than `minRegionArea`, away from the tile's sides, are dropped and counted.
7. **Contours and holes** (`contour.c`, `holes.c`): each region's boundary is traced along its cells and simplified within `maxEdgeError`, each vertex remembering what lies across its edge. A region so thin that simplification would leave it two vertices keeps the outline of its cells instead, so that it is not lost while a neighboring tile keeps the same cells up to their shared side. Holes are bridged into their outlines from their lowest-left vertex, so every region is one simple polygon. A hole no bridge reaches is dropped and counted.
8. **Polygons** (`polymesh.c`, `triangulate.c`): rings are triangulated, vertices welded, and triangles merged greedily into convex polygons of at most six vertices, the best merge first, ties to the first pair, so the result does not depend on hash order. Polygons are linked to their neighbors through an edge list.
9. **Border vertices** (`border_vertices.c`): vertices that tracing along the tile's border left inside the tile are removed where the polygons round them share an area, and the hole is triangulated and merged again, so polygons are not split where nothing in the world splits them.
10. **Detail** (`detail.c`, `delaunay.c`, `height_patch.c`): the floor's heights are laid over each polygon as small triangles, sampled every `detailSampleDistance` and kept within `detailMaxError`, in exact integer geometry: sixteenths of a cell on the ground, cell heights up. An edge's samples are the whole sixteenths on the edge nearest their even places, from the edge's own ends alone: the two polygons sharing an edge sample it alike, and each outline runs exactly along its polygon, so its triangulation is whole however sharp a corner. An edge takes at most `MNAV_MAX_DETAIL_EDGE_SAMPLES` samples, so that a polygon's six edges and corners always fit in `MNAV_MAX_DETAIL_VERTICES`; an edge that would need more, or has fewer whole sixteenths than samples, is sampled more widely and counted in `coarseEdgeCount`.
11. **Encode** (`tile.c`): the polygon mesh and detail mesh become the tile's bytes.

A stage that cannot finish a piece of work it can do without (a hole no bridge reaches, a ring whose triangulation stops short, a detail sample with no height) gives it up and counts it in the report rather than fail the tile. A named limit reached fails the tile with `mnav_errorCapacity` and the stage, and the baker then holds no tile.

## The tile format

A tile is an asset format in the family's sense ([maul-conventions.md](maul-conventions.md)): a host bakes it once and ships it. Little-endian, the same bytes on every platform, aligned to 8 bytes. A 32-byte header holds the magic `MNAV`, the format version `MNAV_TILE_FORMAT`, the bake's generation, the fingerprint, the payload's hash and size, and the number of fields. The payload is that many fields (`fields.c`), each a 32-bit tag and a 32-bit length, then its bytes, padded with zeros to the next 8 bytes, in increasing order of their names:

| Tag | Holds | Record | When absent |
|---|---|---|---|
| `DPRT` | per polygon, its detail vertices past its own and its detail triangles | 2 bytes | no polygons |
| `DTRI` | detail triangles, three corners each within their polygon's part | 3 bytes | no polygons |
| `DVRT` | detail vertices past the polygons' own, in sixteenths of a cell | 6 bytes | none |
| `POLY` | polygons: vertex count and area, then per edge its vertex, the neighbor across it and the tile side it lies on | 2 + 5 per edge | no polygons |
| `TILE` | the grid place, the side in cells, the agent in cells, the cell size and height as binary32 bits, the origin as binary64 bits | 48 bytes | required |
| `VERT` | vertices in cells | 6 bytes | none |

Every count is its field's length over its record, so nothing stores what the lengths give. A field that would be empty holds its default and is left out, so a tile has one encoding. The one stored value that could be derived is each polygon edge's neighbor and tile side, which `polymesh.c` finds from the geometry: the tile keeps them so that a load on the streaming path does not search for them, and the loader checks each against the geometry.

The writer (`mnavEncodeTile`, and `mnavEncodeFlightTile` for flight tiles) writes every byte of a tile value by value, little-endian, with the zero padding of each field, and never copies a struct whole, so the padding of a def or an input never reaches a tile. `test_baker` bakes a tile, 3D and 2D, from a def and input filled with one byte pattern and set field by field, then again over another pattern, and requires the same bytes; `test_flight_volume` does the same for a flight tile.

Loading (`mnavDecodeTile` in `tile.c`) reads every byte as hostile: the payload's hash, then the fields (a known tag at most once and in order, each length within the bytes and a whole number of records, padding zero, nothing after the last field), then each count against the bytes present before anything is allocated, and the mesh itself: convex polygons, walkable areas, each link returned by its neighbor across the same two vertices, each unlinked edge marked with exactly the tile side it lies on and no linked edge marked, detail indices within their polygon. A malformed tile is `mnav_errorFormat` naming the section and element; a tag this version does not know, or another format version, is `mnav_errorVersion`. The hash catches damage, not tampering: a host that needs authenticity signs the bytes.

Before 1.0.0 the format version rises whenever the bytes do, and the loader reads only the format this library writes, so a host rebakes its tiles from source when the format rises; it never guesses at older bytes. From 1.0.0 the version stays fixed: a new field gets a new tag, a reader gives an absent field its default, and an earlier reader refuses a tag it does not know. `test/stored_tiles.h` holds tiles in the current formats, written by `test_stored --write` in the pull request that raises a format, and the tests load them and pin what they answer. `test/stored_format1.h` holds ground and flight tiles in format 1, and the tests check that they are refused.

## The fingerprint

A 64-bit hash (`fingerprint.c`) of the bake's generation, the def's settings that shape the output, the tile's place, and every input triangle that reaches the tile as rasterization picks it (its vertices' bits and its area), in input order, and of the volumes that reach it. It is written into the tile. Geometry far from a tile leaves its fingerprint alone, so a host skips rebaking tiles whose fingerprint did not change.

The generation is `MNAV_BAKE_GENERATOR` (`MNAV_FLIGHT_GENERATOR` for flight tiles), not the library's version, so a release that changes no output keeps every tile and fingerprint as they were. Raise it in the pull request that changes what a bake writes for the same def and input: a stage's rule, a rounding, a tie, a default the bake reads. The golden tile hashes in the tests move in the same pull request, and that is the sign the number must rise. A change of layout raises `MNAV_TILE_FORMAT` too.

## The tile index

A bake tests every input triangle against its tile, so baking N tiles from one large mesh reads the mesh N times, though each triangle reaches one to four tiles. `mnavCreateTileIndex` (`tile_index.c`) checks every mesh once and lists for each tile the triangles whose ground bounds, widened by the tile's border and two cells, reach it, in input order. A bake given the index checks and reads only that list and still makes its exact test on each triangle, so the bytes, the fingerprint and the report equal a bake that reads everything. The margin of two cells covers the rounding of a tile's corner at the largest extent.

An index is never changed after it is made, so workers share one. A bake refuses an index whose grid or counts do not match its input. An edited vertex in a listed triangle is checked as it is read, so a stale index never reads out of bounds, but a triangle moved into a tile it was not listed for is missed until the index is made again. Terrains need no index, since a tile's cells are found by division.

Rejected: a cache inside the baker keyed by the meshes' addresses, which goes stale without notice after an edit in place, and a k-d tree over the triangles, whose reordering would change the fingerprint.

## The tile cache

Placing a crate should not mean reading every triangle again: rasterizing and building the open space take about two thirds of a bake. `mnavBaker_BakeTileCached` keeps, per tile, the open space as it stands before volumes apply, the volumes that reach the tile, the baker's settings and the triangles' fingerprint. `mnavBaker_RebuildTile` unpacks that and runs the stages from the volumes on, with the obstacles after the cached volumes. The result is, to the byte, the tile a full bake gives with the obstacles added to its volumes; the tests hold the rebuild to that on meshes, terrains, indexed input and every kind of volume.

- **Packing** (`field_pack.c`, `lz.c`): each column's span count and each span's floor, height and area, in planes of low bytes then high bytes, coded by a small LZ coder in LZ4's block format. The span links, two thirds of the field, are not kept: a rebuild links the spans again as the bake does. A 32 m tile of hilly terrain packs from about 310 KB to about 10 KB. No public call takes or gives packed bytes, so the cache unpacks only fields it packed: the decoder and the unpacking assert what the packing wrote (`MNAV_ASSERT`) rather than refuse it.
- **Refusals:** a tile not cached is `mnav_errorNotLoaded`; a baker whose settings differ from the cached ones in any word is `mnav_errorInvalid`. The settings are kept and compared in full, not by their hash, since a baker of another def that passed would run the stages over a field cut for another agent. A cached bake that fails leaves nothing cached for the tile.
- 2D bakes have no volumes and are not cached. The cache lives in memory and has no saved form: its packing is not a format the library promises to read again.

Rejected for now: entropy coding, which would make a tile about three times smaller again for several times the code; carving the polygons themselves, which is exact but numerically fragile and would break the match with a full bake; and a manager of obstacles over time, since the library owns no entities.
