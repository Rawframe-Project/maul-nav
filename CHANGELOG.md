# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and the tile formats.

## [0.1.0](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.1.0) (2026-10-27)

### Changed

- **Breaking:** a navmesh tile is a 32-byte header (magic `MNAV`, format
  version, generation, fingerprint, the payload's hash and size, and the
  field count) followed by tagged fields in name order: `DPRT`, `DTRI`,
  `DVRT`, `POLY`, `TILE` and `VERT`. Bytes nothing read are gone: the
  stored header size, reserved and pad words, the element counts, which
  follow from the fields' lengths, and each detail triangle's outline
  bits, with `mnavDetailTriangle.outline`. The loader refuses a field
  that is repeated, out of order, empty, not a whole number of records,
  or padded with non-zero bytes, and bytes after the last field, with
  `mnav_errorInvalid` naming the field's `mnavTileSection`; an unknown
  tag is `mnav_errorVersion`. `MNAV_TILE_FORMAT` is 3. Every tile's
  bytes and fingerprint change, so a host rebakes its tiles; the meshes
  they hold, and so paths and queries, do not change.
  ([#97](https://github.com/Rawframe-Project/maul-nav/pull/97))
- **Breaking:** a flight tile takes the same form: a 32-byte header with
  the magic `MNVF`, then the fields `FLHT`, `LEAF`, `NODE` and `ROOT`,
  refused as tiles are. `ROOT` must hold the def's cube count and `FLHT`
  exactly 56 bytes, and `NODE` and `LEAF` are checked against the def's
  limits before anything is allocated. `MNAV_FLIGHT_FORMAT` is 3 and
  `MNAV_FLIGHT_GENERATOR` 2. Every flight tile's bytes and fingerprint
  change; flight queries do not.
  ([#98](https://github.com/Rawframe-Project/maul-nav/pull/98))
- **Breaking:** one bake entry point per input kind. `mnavBakeTile` and
  `mnavBakeTile2D` now take an input record, `const mnavBakeInput*` and
  `const mnavBake2DInput*`, and the forms that took meshes or outlines
  alone are removed, as are the names `mnavBakeTileInput` and
  `mnavBakeTile2DInput`. A call that passed meshes alone passes
  `&(mnavBakeInput){meshes, count, NULL, 0, NULL, 0, NULL}`. Results do
  not change.
  ([#95](https://github.com/Rawframe-Project/maul-nav/pull/95))
- `mnavBakeDef.maxEdgeError`'s documentation says that walls simplified
  a cell or more can cross, that a ring which then does not triangulate
  whole loses the area it did not triangulate, and that
  `mnavBakeReport.partialRings` counts it.
  ([#91](https://github.com/Rawframe-Project/maul-nav/pull/91))
- `mnavBakeReport.partialRings` counts the polygon stage's rings alone:
  a detail outline, a simple ring since #109, always triangulates whole.
  ([#112](https://github.com/Rawframe-Project/maul-nav/pull/112))

### Fixed

- **Breaking:** `mnavGenerateLinks` refuses a spacing below the
  navmesh's cell size with `mnav_errorInvalid`. A spacing small enough
  used to make an out-of-range conversion of each edge's sample count.
  Generation now stops with `mnav_errorLimit` when a landing's search
  box meets more than `MNAV_MAX_LINK_LANDINGS` (32) polygons, which used
  to drop that link without a word, or at a link past `MNAV_MAX_LINKS`.
  On a stop, `countOut` gives the links found so far and the buffer
  holds the first of them. Links for a spacing of a cell or more do not
  change.
  ([#116](https://github.com/Rawframe-Project/maul-nav/pull/116))
- **Breaking:** `mnavFindRandomPointAround` returns `mnav_errorLimit`
  when a polygon clipped to its circle would have more corners than the
  clip holds. Rounding near a cut could add one corner too many, and the
  clip wrote past its buffer. Results otherwise do not change.
  ([#113](https://github.com/Rawframe-Project/maul-nav/pull/113))
- A raster cut stops at its piece's room of 12 corners: the bake fails
  at rasterizing with `mnav_errorLimit` instead of writing past the
  piece. Rounding of an interpolated corner near a cell line could make
  a cut add more corners than exact arithmetic allows. No known bake
  reaches it, and every tile is as it was.
  ([#115](https://github.com/Rawframe-Project/maul-nav/pull/115))
- Every call that reads an `mnavCorridor` refuses one that is not well
  formed, with `mnav_errorInvalid`, before reading its buffer: a
  corridor has a buffer, a capacity of at least 1, and from 1 to
  capacity polygons. A count past the capacity used to make the calls
  read past the caller's buffer. `mnavSetCorridor` refuses only a
  capacity below 1.
  ([#114](https://github.com/Rawframe-Project/maul-nav/pull/114))
- `mnavRebuildTile` compares a cached tile's bake settings in full, not
  by a 64-bit hash, so a baker of another def is always refused with
  `mnav_errorInvalid`. A cache's tile slots grow by 80 bytes each.
  ([#90](https://github.com/Rawframe-Project/maul-nav/pull/90))
- **Breaking:** the detail stage places each edge sample exactly on its
  edge, so a sliver polygon's outline is a simple ring and triangulates
  whole, where before a sample rounded off its edge could leave the ring
  cut short and counted in `partialRings`. Polygons sharing an edge
  still place the same points. Tile bytes and fingerprints change for
  polygons with sloped edges, and `MNAV_BAKE_GENERATOR` is 3, so a host
  rebakes its tiles.
  ([#109](https://github.com/Rawframe-Project/maul-nav/pull/109))

## [0.0.4](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.4) (2026-10-24)

### Changed

- A navmesh tile's rasterize stage takes about two thirds of the
  instructions it did: about 31% fewer at a 2 m cell and 33% fewer at
  a 0.25 m cell. A whole tile's bake takes about 14% and 19% fewer.
  A flight volume bake shares the rasterizer and gains too. Tile bytes
  are the same as before, and `MNAV_BAKE_GENERATOR` does not change.
  ([#25](https://github.com/Rawframe-Project/maul-nav/pull/25))

## [0.0.3](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.3) (2026-10-24)

### Changed

- A flight path search takes about a third of the instructions it
  did: the line-of-sight walk skips open blocks whole and steps through
  mixed leaves by bit tests. Found paths and raycasts are the same bits
  as before.
- A flight path whose end (or start) lies in a closed space the other
  cannot reach, such as the inside of a closed mesh, now ends at once
  with `mnav_pathNone` and the start alone as its path. This applies
  when the space is found closed within 512 blocks and the node budget.
  Before, the search flooded its whole node budget and returned
  `mnav_pathOutOfNodes` with a partial path toward the outside, which
  could never reach the end.
  ([#22](https://github.com/Rawframe-Project/maul-nav/pull/22))

### Fixed

- A host that runs its threads with flush-to-zero, denormals-are-zero
  or another rounding mode now gets the same bakes, paths and query
  results as every other host. Every public call that computes switches
  to the library's floating-point setting and restores the host's on
  return. The allocator and the bake clock run in the host's own
  setting. Results in the default setting do not change.
  ([#24](https://github.com/Rawframe-Project/maul-nav/pull/24))

## [0.0.2](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.2) (2026-10-24)

### Fixed

- A navmesh slot whose tile is removed or replaced at the last
  generation is now retired. Before, the next tile staged there wrapped
  to generation 0, and its polygon ids were refused by every query.

## [0.0.1](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.1) (2026-10-24)

The first release: navmesh and flight volume baking, queries, corridors,
flow fields and avoidance, with results that are bit-identical on every
platform, compiler and worker count.

### Added

- Navmesh baking, tile by tile, from triangle meshes, terrains and 2D
  outlines, with area types per triangle or cell and include, exclude
  and area volumes. Bakes check their settings and input as hostile,
  name the first setting or element refused, and report each tile's
  counts, memory, the work they gave up on and, with a clock, each
  stage's time. A tile index lets many bakes of one large mesh read only
  the triangles near each tile.
- A tile format, versioned and fingerprinted, whose loader checks every
  byte and names the section and element of a damaged tile.
- A navmesh that stages tiles and changes and commits them all at once,
  with ids that refuse a replaced tile, streaming that reports places
  with no tile instead of treating them as walls, and runtime tiers for
  area changes, link toggles and tile replacement.
- Off-mesh links point to point or edge to edge, one way or both, of six
  built-in kinds and the host's own, snapped at every commit, and
  generated drops and jumps with an optional clearance test.
- A tile cache that rebuilds tiles with obstacles to the same bytes as a
  full bake, in about a third of the time.
- Nearest points, A* paths and exact shortest paths, sliced searches,
  hierarchical paths for large worlds, rays, surface moves, heights,
  walls, polygons in a box, random points from a seed, reachability and
  grid paths, all with query filters for area costs and link kinds.
- Path corridors that follow moving agents and targets, with corners,
  area runs, shortcuts, checks and replanning after commits, and
  stateless steering toward the corners.
- Flow fields over grids, built and repaired in steps, and over
  navmeshes, built in steps.
- Flight volumes for agents that fly or swim: sparse voxel octree tiles
  baked from the same input, any-angle paths, sliced searches, rays,
  path checks and nearest open points.
- ORCA avoidance on the ground and in space, with obstacles, priorities
  and layers, the same velocities for the same agents in any order.
- Debug geometry for navmeshes, links, paths, corridors, flow fields,
  avoidance and flight volumes as plain vertices and indices.
- A shared library option, a CMake package and a pkg-config module, and
  support for Linux on x86-64 and arm64, macOS, Windows with `clang-cl`,
  and WebAssembly.

### Changed

- Tiles and flight tiles baked by earlier releases must be rebaked: they
  are refused with `mnav_errorVersion`.
- Against the withdrawn 1.x releases, `mnavQueryFilter` and
  `mnavLinkGenDef` lose their `version` field, and only the root defs
  carry `MNAV_ABI_VERSION`, which holds the whole release before 1.0. A
  program built against 1.x headers is refused with `mnav_errorVersion`
  and must be rebuilt.
