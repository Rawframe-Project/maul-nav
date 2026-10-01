# mnav-0003. The bake call and the tile format

Status: Accepted

## Context

A host bakes a navmesh tile by tile, on its own workers, from editor,
script, tool or CI alike, and stores each tile as cooked content that
it later loads from disk or the network. The bake must allocate only
through an owner object (record 0010) and report rather than log; the
tile bytes must be the same on every platform and are hostile input
when loaded.

## Decision

- **The baker:** `mnavCreateBaker` checks a def (mnav-0002) and keeps
  it with its allocator and memory limit. `mnavBakeTile` checks every
  input mesh, runs every stage for one tile and keeps the tile's bytes;
  `mnavCopyBakedTile` copies them into caller memory, reporting the
  size needed when the buffer is too small. A baker is used by one
  thread at a time; bakers share nothing, so a host bakes in parallel
  with one per worker. The bytes depend only on the def, the input and
  the tile's place, never on what the baker baked before.
- **The report:** `mnavBakeReport` gives the result, the stage it ended
  in, the first mesh and element its check refused, the tile's counts,
  its size, the memory peak, and the work the bake gave up on rather
  than fail: holes no bridge could join, triangulations that stopped
  short, detail samples without a height, detail at its vertex cap.
- **The fingerprint:** a 64-bit hash of the generator's version, the
  def's settings that shape the output, the tile's place, and every
  input triangle that reaches the tile (its vertices' bits and its area
  type), in input order. It is written into the tile; geometry far from
  a tile leaves its fingerprint alone.
- **The format:** little-endian. A 104-byte header (the magic `MNAV`,
  format version `MNAV_TILE_FORMAT`, header size, generator version,
  flags, the fingerprint, the payload's hash and size, the tile's grid
  place, its side in cells, cell size and height as binary32 bits, the
  agent in cells, the origin as binary64 bits, counts) and a payload of
  integers only: vertices in cells, polygons of 3 to 6 vertices with
  their area type, the neighbor across each edge or the tile side it
  lies on, per-polygon detail counts, detail vertices in sixteenths of
  a cell, detail triangles. Nothing derivable from the rest is stored.
- **Loading:** every field is read and checked: counts against caps
  and the bytes present before anything is allocated, the payload's
  hash, and the mesh (convex polygons with an area, walkable area
  types, neighbors that run the same edge back, tile sides where
  unlinked edges lie on the tile's sides, detail indices within their
  polygon). A malformed tile is `mnav_errorInvalid`; another format
  version is `mnav_errorVersion`. The loader is fuzzed on every push.

- **Islands:** the report counts the regions dropped for being smaller
  than the minimum region area and away from the tile's border
  (`droppedRegions`), the disconnected islands the requirements ask to
  be warned of. The library keeps no clock: hosts time the bake.

- **Terrains:** `mnavTerrain` is heights on a regular grid with an
  area per cell, `mnav_areaNone` cutting a hole. `mnavBakeTileInput`
  bakes meshes and terrains together; each cell is two triangles split
  from its lowest X and Z corner, made only for the cells near the
  tile, and the tile is byte for byte the one the same triangles as a
  mesh give.

## Consequences

- Tiles move between platforms and versions of a host unchanged, and a
  host can skip rebaking tiles whose fingerprint did not change.
- The hash catches damage, not tampering: a host that needs
  authenticity signs the bytes itself.
- A new field or section is a new format version.
