# mnav-0016. A tile cache for obstacles

Status: Accepted

## Context

Hosts place obstacles that stand still for a while: crates, barricades,
parked vehicles. A tile must change where they stand.
- **Baking the tile again** with the obstacle in its input already
  works (`mnav_tierDynamic` replaces tiles), but it reads every
  triangle again. Rasterizing and compacting take about two thirds of
  a bake.
- **DetourTileCache** (Recast/Detour) keeps each tile's heightfield
  layers compressed in memory, marks obstacle shapes into them and
  rebuilds tiles from the layers alone. Mononen reports a rebuild of
  about 1 ms where a full build took 10 ms.
- **Unity** carves a hole of an obstacle's shape into the navmesh, and
  by default only once the obstacle has stood still. Moving obstacles
  are left to local avoidance.
- **Maul Nav's bake volumes** (`mnavBakeVolume`) already apply to the
  open-space field, after it is compacted: exclude volumes carve,
  include volumes keep, and area volumes set the area type.

## Decision

- **The cache:** `mnavTileCache` is made by the host with its own
  allocator and limits: tiles held at once, 1 to
  `MNAV_MAX_CACHED_TILES`, and bytes. The default holds 1024 tiles in
  256 MiB.
- **What it keeps:** `mnavBakeTileCached` bakes a tile as
  `mnavBakeTileInput` does and keeps, per tile:
  - the open-space field as it stands before volumes apply;
  - the input's volumes that reach the tile, in input order, with their
    points copied. When the input has include volumes and none reach
    the tile, its first include is kept as well, so that the tile stays
    empty;
  - the hash of the settings and the fingerprint of the triangles.
- **The rebuild:** `mnavRebuildTile` takes obstacles as bake volumes.
  Each acts as the same bake volume would: an exclude obstacle carves,
  and an area obstacle changes the area within it. The rebuild
  copies the cached field and runs the stages from the volumes on: the
  cached volumes first, then the obstacles.
  - **The same bytes:** the tile, its report's counts and its
    fingerprint are those `mnavBakeTileInput` gives for the same input
    with the obstacles appended to its volumes. Tests hold the rebuild
    to this bit for bit on meshes, terrains, indexed input and every
    kind of volume.
  - **Shapes:** obstacles are rings with a height range. The host makes
    a box, cylinder or footprint into a ring, as for bake volumes.
- **Refusals:**
  - A tile the cache does not hold is refused with
    `mnav_errorNotLoaded`.
  - A baker whose settings hash differs from the one the tile was cached
    with is refused with `mnav_errorInvalid`.
  - Obstacles are checked as a bake checks volumes, each named by its
    own index. Their points, the cached volumes' points and the tile's
    triangles count against the input limit.
  - A cached bake that fails, for any reason, leaves nothing cached for
    the tile. A cache at its limit of tiles or bytes refuses the tile
    with `mnav_errorLimit`, and no tile is baked.
- **3D input only:** a 2D bake has no volumes, so its tiles are not
  cached.
- **Not now:**
  - compression, until measured against the field's size;
  - carving the polygons themselves, which is exact but numerically
    fragile and would break the bytes matching a bake;
  - a manager of obstacles over time, since the library owns no
    entities.
- **Measured:** on the benchmark's terrain of 8 by 8 tiles of 32 m, with
  hills and 256 boxes, at the default def:
  - a cached tile takes 297 to 316 KB, 310 KB on average and 19.9 MB for
    the 64 tiles;
  - a rebuild with one obstacle takes 1.2 ms per tile, against 3.8 ms
    for the cached bake.

## Consequences

- A host that places an obstacle rebuilds the tiles its ring reaches,
  widened by the agent's radius, in about a third of a bake, without
  the input geometry at hand. The tiles commit through the usual
  stage-and-commit path.
- The cache costs memory: about 300 KB per 32 m tile at the default
  cells, uncompressed. A large world caches only the tiles near where
  obstacles may go, or drops tiles as the host streams them out.
- The obstacles of one rebuild replace those of the last: the host
  passes every obstacle on the tile each time.
- The cache lives in memory only. Its settings hash includes the
  library's version, and the library gives no way to save it.
