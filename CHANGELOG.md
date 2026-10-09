# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and every data
format.

## [Unreleased]

### Added

- `mnavDebugAvoidance3D`: fliers drawn as three rings each, one in each
  axis plane, with a line to each neighbor `mnavAvoid3D` gives them, and
  the spheres they avoid as rings of a new kind, `mnav_debugObstacle`.
- Sample `flight`: two streams of fliers cross among pillars, each
  following its flight path with avoidance in space, a raycast keeping
  every step in open space; run as a test.

### Changed

- `mnavAvoid3D` finds the spheres near each flier through a grid
  rather than looking at every one, with the same velocities: a step of
  1,000 fliers among 4,096 spheres takes a third of the time.

## [0.7.0] - 2026-10-09

Avoidance among fliers: agents sharing a flight volume steer round each
other and round moving spheres in space, with the ground call's
determinism. Tiles baked by 0.6.0 still load; tiles baked now record
0.7.0 as their generator.

### Added

- `mnavAvoid3D`: avoidance among fliers in space, ORCA's planes and
  RVO2-3D's linear programs in binary64, with the ground call's
  neighbor grid in cubes, priorities and sidestep, and moving or still
  spheres as obstacles (`mnavAgent3D`, `mnavSphere`). The same agents
  give the same velocities in any order, on every platform.

### Changed

- Flight paths and raycasts look up the octree less, with the same
  results:
  - the voxel walk keeps the open block it is in and skips lookups and
    edge checks while it stays inside;
  - a face's neighbors are read from each leaf through a mask of the
    face's slab, not voxel by voxel.

  The bench's flight section runs 30% fewer instructions, with the
  same paths to the bit.
- The ground call's neighbor grid moved into a module of its own, which
  both calls share; its results are the same to the bit.
- The bench steers 1,000 fliers through a gap in a wall of spheres.

## [0.6.0] - 2026-10-09

Flight volumes, for agents that fly or swim: sparse voxel octree tiles
baked from the navmesh bake's input, streamed as navmesh tiles are, and
searched by any-angle paths.

### Added

- Flight volumes (`maul-nav/flight.h`, mnav-0015), where agents that
  fly or swim may go:
  - `mnavFlightDef`, with `mnavDefaultFlightDef` and
    `mnavValidateFlightDef`;
  - a baker (`mnavCreateFlightBaker`, `mnavBakeFlightTile`,
    `mnavCopyFlightTile`). It voxelizes the navmesh bake's input at
    the voxel size, makes the space below the ground solid unless told
    otherwise, and keeps the flier's radius clear of anything solid.
    Each tile is a compact sparse voxel octree in a fingerprinted,
    little-endian format.
  - a container (`mnavCreateFlightVolume`, `mnavStageFlightTile`,
    `mnavStageFlightTileRemoval`, `mnavCommitFlight`,
    `mnavGetFlightTile`) that checks every byte as hostile and commits
    all or nothing;
  - `mnavIsFlightOpen`, which tells whether a point is open to the
    flier's center;
  - `mnavFindFlightPath`, a flier's path by Lazy Theta* (Nash, Koenig
    and Tovey, 2010) over the octree's open blocks. Each block is
    entered at the point of its face nearest the way so far, and the
    search runs in an `mnavQuery` context with its node budget and path
    length limit. On the Warframe 3D benchmark maps (Brewer and
    Sturtevant, 2018) its paths averaged 0.925 to 0.944 of the
    26-connected grid's shortest;
  - `mnavFlightRaycast`, which stops at the first blocked voxel or
    unloaded tile and reports the fraction reached;
  - `mnavFindNearestFlightPoint`, the nearest point open to the flier
    within a radius.
- `mnavTileSection` names a flight tile's cube classes, nodes and
  leaves.
- `fuzz_flight`, a fuzz target over the flight tile loader and over
  paths, raycasts and nearest points in baked volumes, checked against
  every voxel.
- Bench lines for flight volumes: a tile's bake, a path query and a
  raycast over the bench terrain with 300 floating boxes.

### Changed

- A tile records the version that baked it, so tiles baked by 0.6.0
  differ from 0.5.0's for the same input in their bytes and
  fingerprint. The tile format is unchanged, and tiles baked by 0.5.0
  still load.

## [0.5.0] - 2026-10-08

The shortest path on the navmesh, exact, beside the A* search.

### Added

- `mnavFindShortestPath`: the shortest way on the ground between two
  points, by Polyanya (Cui, Harabor and Grastien, 2017) over the
  navmesh's portals, across tile sides and off-mesh links, with the
  same arguments and result as `mnavFindPath`. The A* search picks its
  corridor by costs through portal midpoints; among pillars its paths
  measured 11% longer than the shortest on average and up to 39%
  longer. The exact search needs every included area at one cost
  (`mnav_errorInvalid` otherwise). On the benchmark's 1,000 queries it
  ends as the A* search does, never longer, 1.9% shorter in all, in
  fewer nodes and instructions. When its nodes run out, the A* search
  tells whether the end can be reached at all.
- `fuzz_shortest`: random squares over two tiles with links, the exact
  search checked against the A* one; `fuzz_query` runs it on raw input.
- A benchmark line for the exact search.

## [0.4.4] - 2026-10-07

A fix found by fuzzing the tile index, and a fuller guide.

### Fixed

- `mnavCreateTileIndex` and `mnavCreateTileIndex2D` refused input
  spread over more than 2^31 - 1 tiles with `mnav_errorLimit` as
  documented, but counted the grid's tiles in 32 bits on the way out,
  an overflow undefined in C. The grid is now kept only when its tiles
  fit. Found by fuzz_bake.

### Documentation

- The README names the tile index; the guide covers checking a def and
  input before a bake, removing links, reading a link's state, the tile
  at a place, a polygon's area and the tier back, and the linked
  version.

## [0.4.3] - 2026-10-07

A faster neighbour search for crowds.

### Changed

- Avoidance's cell table keeps each occupied cell's run of agents, so
  a neighbour search reads a cell's agents without testing each one's
  cell: the benchmark's doorway of 1000 agents takes 3.6% fewer
  instructions and about 4% less time a step, every velocity the same.

## [0.4.2] - 2026-10-07

Faster grid flow fields and path searches, and a fuzz target for grid
flow fields.

### Added

- A fuzz target for grid flow fields, `fuzz_flow`, with its seed
  corpus: a field built in steps must be the one built at once, each
  cell's cost its next cell's plus the step, and a repair after area and
  goal changes a fresh build's, bit for bit.

### Changed

- The filter's area test is inline, as every search step asks it:
  building the benchmark's 512 by 512 grid flow field takes 21% fewer
  instructions and about 19% less time, and the whole benchmark 905 M
  fewer instructions, every result the same.

## [0.4.1] - 2026-10-07

Faster nearest queries and path searches, and a sample of a world baked
by workers sharing a tile index.

### Added

- `samples/bake_world.c`: a world baked from one large mesh by four
  worker threads sharing a tile index, checked against one thread's
  bake without it, to the byte.

### Changed

- `mnavFindNearest` looks up a polygon's detail height only when the
  polygon can still win (a point beside it whose ground distance alone
  loses is passed over), and a detail height stops at the first detail
  triangle holding the point. Streaming the benchmark's terrain with
  512 off-mesh links, whose ends are snapped again at each commit,
  takes 15% fewer instructions; the results are the same.
- Path searches name the node behind a portal by its four fields, not
  a whole search node zeroed for each edge: path queries on the
  benchmark take about 5% less time, the paths the same.

## [0.4.0] - 2026-10-07

Tile indexes of 2D outlines.

### Added

- Tile indexes of 2D outlines (mnav-0014): `mnavCreateTileIndex2D`, and
  `mnavBakeTile2DInput` with an `mnavBake2DInput` record holding the
  outlines and an optional index, which reads only the outlines listed
  for the tile and makes the same tile to the byte. `mnavBakeTile2D` is
  unchanged. A bake refuses an index made of the other kind of input.
  Baking the benchmark's flat world of 576 tiles takes about 9% fewer
  instructions with one, the tiles the same.

## [0.3.1] - 2026-10-07

Faster navmesh searches, the same paths.

### Changed

- Navmesh path searches keep each open node's total beside it in the
  heap, so ordering the open list reads the heap alone, and build each
  polygon's frame once per expansion: building the benchmark's
  hierarchy takes 7.9% fewer instructions and 7.3% fewer cycles, the
  paths the same. A query context holds 12 more bytes per node.
- `tools/mutants.txt` holds the tile index's 30 mutants, and
  `test_tile_index` places meshes below the origin, in shapes longer
  one way, and near the extent with a cell size that rounds, bakes
  three tiles past the index on every side, and checks every corner of
  a triangle read; `tools/mutate.py` names tests that abort.

## [0.3.0] - 2026-10-07

Tile indexes for bakes from large meshes, and a faster tile test.

### Added

- Tile indexes (mnav-0014): `mnavCreateTileIndex` lists, once, the
  triangles of a set of meshes that may reach each tile of a def's
  grid, and a bake given one through `mnavBakeInput.index` reads only
  its tile's list, making the same tile to the byte. 17% fewer
  instructions baking the benchmark's 64 tiles from one mesh, more
  for meshes spanning more tiles. `mnavDestroyTileIndex` frees one.

### Changed

- `mnavBakeInput` has a last field, `index`. Code that fills the record
  positionally gives it `NULL` (compilers warn of the missing field
  under `-Wextra`); zero-initialized and designated initializers need
  no change.
- Baking tests whether a triangle reaches the tile before its slope, and
  rejects triangles a cell or more outside the tile without dividing:
  11.8% fewer instructions in a Release build baking the benchmark's
  64 tiles from one mesh, the tiles the same to the byte.

## [0.2.5] - 2026-10-07

Hierarchy updates after area changes do a third of the work.

### Changed

- `mnavUpdateHierarchy` searches again the cluster before a changed
  tile only when the filter has come to include or exclude the polygon
  its transition crosses into, the only thing those searches see of
  it: a third of the work after an area change on the benchmark, the
  graph still the one a build makes.

## [0.2.4] - 2026-10-07

Documentation of three search boundaries, and the mutation tests behind
them.

### Changed

- `mnavFindWallDistance` says a wall exactly the radius away lies
  within it, and that of walls at the same distance it gives one, the
  same everywhere but not promised.
- `mnavFindHierarchicalPath` says that where routes through the
  clusters cost the same it takes one, the same everywhere but not
  promised, and the refined paths may differ a little in cost.

### Added

- `tools/mutate.py` and `tools/mutants.txt`: the 167 one-line mutants
  the tests were checked against, and the runner, not part of CI.

## [0.2.3] - 2026-10-07

Avoidance keeps agents out of circle obstacles they overlap.

### Fixed

- `mnavAvoid` walked an agent that overlapped a circle obstacle on into
  it, at its preferred velocity, whenever leaving within the step
  needed more than the agent's maximum speed. It now leaves at the
  most it can.

### Changed

- The raycast and navmesh flow field documentation says which tie
  choices are the same everywhere but not promised: the normal at a
  corner of two walls, and which of two equal-cost ways a polygon
  keeps.
- The guide's code snippets keep every result they get, and the tests
  run each one as written.

## [0.2.2] - 2026-10-06

Faster avoidance and outline bakes, the same results.

### Changed

- Avoidance finds neighbours through a table of grid cells half the
  neighbour range wide, ring by ring from each agent's cell: a quarter
  fewer instructions on a doorway of 1000 agents, the same velocities.
- Polygon merging in the bake pairs only polygons that share an edge,
  into the same bytes: the benchmark's outline world, with many holes,
  bakes and builds its hierarchy with a fifth fewer instructions.

## [0.2.1] - 2026-10-06

A path fault turned up by fuzzing, fixed; the fuzz targets now cover
the runtime navmesh's changes and debug output, and CI's fuzzing starts
from seed corpora.

### Added

- `fuzz_stream`, a fuzz target for sequences of runtime changes
  (tiles added, removed and replaced, links added, removed and toggled,
  areas changed) with commits, queries and a corridor between; and
  seed corpora for every fuzz target in `test/corpus`.

### Fixed

- A path across an off-mesh link whose ends lie on one polygon listed
  that polygon once, so the corridor set from it lost the link: its
  corners walked straight past. Each visit is now listed, before the
  link and after it.

## [0.2.0] - 2026-10-06

Faults found by measuring test coverage and by fuzzing the inputs no
target reached, and fixed: a random point in a circle of no radius,
avoidance given values of any size, vectors of no direction, hierarchy
updates next to a change, and link ends past the bake extent. Input
that made them is now refused or handled, which changes what some
calls accept: a minor release.

### Added

- `MNAV_MAX_AVOIDANCE_COORDINATE`, `MNAV_MAX_AVOIDANCE_SPEED`,
  `MNAV_MAX_AVOIDANCE_RADIUS` and `MNAV_MIN_AVOIDANCE_TIME`: the ranges
  of avoidance input.
- `fuzz_avoid`, a fuzz target for avoidance agents and obstacles, and
  `fuzz_graph`, one for hierarchies, navmesh flow fields, edge links and
  link generation.

### Changed

- `mnavStageLink` refuses with `mnav_errorRange` a link end farther
  from the def's origin than bake input may lie, and with
  `mnav_errorInvalid` an edge link whose ends are so near that the
  square of their distance rounds to 0. Such ends made an edge link's
  direction NaN and converted it to a tile index, undefined behaviour.
- `mnavUpdateHierarchy` searches more after an area change (on the
  test world 27 of 114 transitions for one tile, against 10); see
  Fixed.
- Fuzz builds stop on undefined behaviour.

- `mnavAvoid` refuses with `mnav_errorInvalid` a velocity, preferred
  velocity, obstacle velocity or maximum speed past 1e6 m/s, a radius
  past 1e6 m, and a step shorter than 1e-6 s; `mnavCreateAvoidance`
  refuses horizons shorter than 1e-6 s with `mnav_errorRange`.
  Velocities came out the same except where rounding had put them past
  the maximum speed.

### Fixed

- `mnavAvoid` with a huge velocity, only checked to be finite, gave an
  agent a velocity far past its maximum speed (3.7e210 m/s for a
  limit of 0.3), and a huge circle radius overflowed a conversion to a
  grid cell; now refused (see Changed), and grid cells saturate.
- `mnavAvoid` gave a NaN velocity to an agent a hair from an obstacle's
  corner (2.8e-312 m in the fuzz case): the vector to the corner was
  too short to square, and its unit vector came out infinite. Such
  vectors are now scaled up exactly before they are normalized.
- `mnavAvoid` could give a velocity a little faster than the agent's
  maximum speed (1.2e-8 of it, with deeply overlapping agents) when
  constraints met at a narrow angle; the speed is now kept exactly.
- The obstacle search walked every grid column an agent's reach spans;
  it now skips columns with no obstacles.
- `mnavUpdateHierarchy` after an area change kept the edges of the
  clusters next to the change, though their searches end on crossings
  inside it: the graph could differ from a build's, an edge kept to a
  crossing the filter now leaves out. Those clusters are searched
  again too.

- `mnavFindRandomPointAround` with a radius of 0, or one too small to
  tell its corners apart at the center's place, returned a point
  anywhere on the polygons it reached instead of the center: a circle
  of one point clipped nothing away.

## [0.1.0] - 2026-10-06

The first release: navmesh generation in 3D and 2D, the navmesh format
and its hostile-input loader, path and spatial queries, corridors,
off-mesh links, runtime tiers and streaming, hierarchical and sliced
searches, grid paths, flow fields over grids and navmeshes, avoidance,
and debug output; deterministic on every platform, compiler and worker
count, with named limits on every search and bake. A guide
(`docs/guide.md`), the API reference (`docs/api.md`) and samples come
with it.

### Added

- The library skeleton: the build, the family rules and tools, the
  version and result API (`mnavGetVersion`, `mnavResultName`) and the
  library profile.
- The bake's settings and input surface (`maul-nav/bake.h`): the bake
  def with its agent profile and named limits, checked and converted
  from meters to cells by `mnavValidateBakeDef`, and triangle meshes
  checked as hostile input by `mnavValidateTriangleMesh`.
- Results `mnav_errorLimit` and `mnav_errorRange`, the allocator, and
  the `mnavVec3` and `mnavPos3` types.
- Tile rasterization (internal): triangles clip into per-cell fragments,
  which sort and merge into each column's solid spans, so a tile's
  heightfield depends only on the set of input triangles, not their
  order. Its hash is pinned across platforms.
- The walkable filters (internal): spans within a step above walkable
  ground become walkable, ledges and steep neighbors are removed, and
  so is ground without the agent's height of free space.
- The open-space field and erosion (internal): walkable spans become
  open space linked to the neighbors an agent can step to, and space
  closer than the agent's radius to a boundary is removed. Heights,
  links and distances are 16 bits, which always suffice, so nothing is
  truncated. Pipeline hashes over a soup and a small level are
  pinned.
- `mnavHash64` and `MNAV_HASH_INIT`, the family's frozen 64-bit hash,
  for determinism checks and navmesh fingerprints.
- `mnavBakeDef.minRegionArea`, in square meters (2 by default), and
  region partitioning by layers (internal): monotone sweeps merged into
  non-overlapping layers of one area, small islands off the tile's
  border dropped, with 32-bit ids and sweep storage sized by the row's
  spans.
- `mnavBakeDef.maxEdgeError` and `maxEdgeLength`, in meters (0.3 and
  12 by default), and region contours (internal): each region's
  boundary traced edge by edge with corner heights and the region
  across each edge, simplified along walls and area borders, walls
  split past the maximum length.
- Hole merging (internal): each region's holes are bridged into its
  outline left to right through the nearest vertex whose diagonal
  crosses nothing, every tie broken by an index, and a hole that cannot
  be bridged is dropped and counted for the bake report.
- The polygon mesh (internal): each region's ring cut into triangles by
  ear clipping, shortest diagonal first, its vertices welded across the
  tile, merged into convex polygons of at most 6 vertices along their
  longest shared edges, and linked to the polygons across each edge or
  marked with the tile side it lies on. A ring that cannot be finished
  keeps its triangles and is counted.
- Removing tile-border vertices (internal): a vertex where two regions
  meet only because of the border around the tile is removed when its
  polygons keep more than 2 edges, share one area, leave at most 2
  edges at it unshared and the outline keeps its shape; the hole is
  triangulated, merged and checked before any polygon changes, and a
  vertex that cannot be replaced stays flagged. Linking polygons
  is now its own step after removal, and ear clipping refuses ears with
  a ring vertex inside them.
- The height patch under a polygon (internal), the first part of the
  detail mesh: each cell's floor in the polygon's region, flooded
  outward into the cells round it; a polygon of mixed regions floods
  from the spans nearest its vertices. A lookup in a cell without a
  height takes the nearest height in the first ring of cells that has
  one, and says so when none is within its radius rather than
  returning a marker as a height.
- Two bake settings, detailSampleDistance (1.5 m, 0 for none) and
  detailMaxError (0.125 m), with mnav_settingDetailSampleDistance and
  mnav_settingDetailMaxError; mnavBakeCells reports them in sixteenths
  of a cell and of a cell height.
- The detail mesh's outline (internal): each polygon edge sampled from
  its lexically first end at most a sample distance apart, at most 20
  samples, heights from the height patch, samples within the maximum
  error dropped, and the outline triangulated; all in sixteenths of a
  cell with integer arithmetic, so both sides of an edge get the same
  points and the result is the same on every platform.
- The detail mesh's interior (internal): the outline's triangulation
  made Delaunay by edge flips with an exact in-circle test, then grid
  samples well inside each polygon added worst first until every
  sample lies within the maximum error or the polygon holds 127
  vertices (counted); each insertion restores the Delaunay property
  round the new vertex, and only samples in changed triangles are
  measured again.
- mnav_errorVersion: data written in a format version this library does
  not read.
- The tile format (internal): a baked tile's polygon and detail meshes
  as little-endian integers behind a versioned header with the
  generator's version, the bake's input fingerprint and a payload
  hash; nothing derivable is stored. The loader reads field by field,
  checks every count against its cap and the bytes left before
  allocating, then the mesh itself (convex polygons, walkable areas,
  neighbors that link back, tile sides, detail indices), and names the
  section and element it refuses (mnav-0003). A libFuzzer target,
  fuzz_tile, runs with MAUL_NAV_FUZZ; test_tile --seed writes its
  seed.
- The public bake call (mnav-0003): mnavCreateBaker, mnavDestroyBaker,
  mnavBakeTile and mnavCopyBakedTile, with mnavBakeReport (result,
  stage, refused input, counts, memory peak, work given up) and
  mnavBakeStage; MNAV_TILE_FORMAT. A tile carries a fingerprint of the
  input that reaches it.
- The navmesh (mnav-0004): mnavCreateNavmesh, mnavDestroyNavmesh,
  mnavStageTile, mnavStageTileRemoval, mnavCommit and mnavGetTile in
  the new header maul-nav/navmesh.h, with mnavTileId, mnavPolygonId,
  mnavTileSection and mnavTileResult; mnav_errorNotLoaded. A commit
  applies everything staged or nothing, replaced and removed tiles'
  ids go stale through their slot's generation, and polygons on facing
  tile sides are linked wherever their edges overlap within the agent's
  step, computed exactly.
- The first query (mnav-0005): mnavFindNearest in the new header
  maul-nav/query.h, with mnavNearest. It returns the polygon whose
  nearest point lies in a box and scores best (over a polygon, the
  height beyond the agent's step; otherwise the distance), with ties
  broken by distance, tile place and polygon index, and reports when
  part of the box lies on places with no tile loaded.
- Path search (mnav-0005): query contexts (mnavQueryDef,
  mnavDefaultQueryDef, mnavCreateQuery, mnavDestroyQuery) sized by
  named limits on nodes and path length, and mnavFindPath, an A* over
  the portals between polygons that returns the polygon corridor, its
  cost and how the search ended (mnavPathEnd); mnav_errorStale for
  polygon ids whose tile was replaced or removed.
- Straight paths (mnav-0005): mnavPath gains the straight path, the
  corridor pulled tight with the funnel algorithm in binary64 with
  exact side tests, held in the query context and never cut short.
  The search's ties now go to the node made first; a tie on the
  distance still to go never decided anything and was dropped.
- Raycasts (mnav-0005): mnavRaycast with mnavRay and mnavRayEnd walks
  a segment along the navmesh in binary64 with exact link ranges, and
  ends reached, at a wall with its normal, at a side with no tile
  loaded, or out of nodes.
- Query filters (mnav-0005): mnavQueryFilter and
  mnavDefaultQueryFilter, a cost and an inclusion bit per area type.
  mnavFindNearest, mnavFindPath and mnavRaycast take a filter after the
  navmesh (NULL for the default); mnavPath gains the length in meters
  beside its cost, and the path length limit counts meters.
- Off-mesh links (mnav-0004): mnavLinkDef, mnavLinkId, mnavLinkState
  and mnavLinkKind; mnavStageLink, mnavStageLinkRemoval and
  mnavGetLink. Links are staged and committed with tiles and snapped
  again at every commit. The bake def gains limits.links
  (mnav_settingLinks). mnavCommit moves to its own module.
- Paths across off-mesh links (mnav-0005): mnavQueryFilter gains
  kinds, the link kinds an agent may cross; mnavPath gains links, an
  mnavPathLink per link crossed (its id, kind and takeoff point).
- Sliced path searches (mnav-0005): mnavBeginPath, mnavContinuePath
  and mnavFinishPath, with mnav_pathUnfinished; the result of any
  slicing equals mnavFindPath's, and a commit to the navmesh makes a
  search in progress stale. The funnel moves to its own module.
- Moving along the surface (mnav-0005): mnavMoveAlongSurface with
  mnavMove and mnavMoveEnd; a tile side's stretches no link covers are
  walls. The query context's code moves to its own module.
- Path corridors (mnav-0005): mnavCorridor, mnavCorners,
  mnavResetCorridor, mnavSetCorridor, mnavCheckCorridor and
  mnavCorridorCorners. A loaded path's corners equal its straight path
  bit for bit.
- Moving corridors (mnav-0005): mnavMoveCorridor and
  mnavMoveCorridorTarget. mnavMoveAlongSurface's search circle is
  widened by one cell: a move starting on a vertex could lose its
  neighbours to rounding and go nowhere.
- Corridor upkeep (mnav-0005): mnavShortcutCorridor and
  mnavReplanCorridor.
- Runtime tiers (mnav-0004): mnavTier and mnavBakeDef.tier
  (mnav_settingTier), mnavGetTier, mnavStageArea, mnavGetArea,
  mnavStageLinkEnabled and mnavLinkState.enabled, and the result
  mnav_errorTier. The default tier is static: replacing a loaded tile
  now needs mnav_tierDynamic.
- Polygons in a box (mnav-0005): mnavFindPolygons and mnavFound.
- A commit snaps again only the off-mesh links it adds and those near
  the places it changes; on the streaming benchmark with 512 links a
  step went from about 630 us to 200 us. bench/bench_main.c: baking,
  streaming and path timings.
- 2D generation (mnav-0002): mnavVec2, mnavOutline,
  mnav_elementPoint, mnavValidateOutline and mnavBakeTile2D; a 2D
  navmesh is an ordinary navmesh with a flat floor, served by every
  query.
- Grid paths (mnav-0005): mnavGrid, mnavCell, mnavGridPath,
  MNAV_MAX_GRID_SIDE and mnavFindGridPath, jump point search or A* by
  the filter's costs.
- Avoidance (mnav-0006): avoidance.h with mnavPos2, mnavAgent,
  mnavAvoidanceLimits, mnavAvoidanceDef, mnavDefaultAvoidanceDef,
  mnavCreateAvoidance, mnavDestroyAvoidance and mnavAvoid: ORCA in
  binary64 among agents, ordered by ids and split by priority.
- Avoidance obstacles (mnav-0006): mnavObstacle (circles, segments,
  counterclockwise polygons, static or moving), the def's
  obstacleTimeHorizon and limits.obstacleVertices and
  limits.obstacleNeighbors; mnavAvoid takes the obstacles beside the
  agents.
- bench/bench_main.c times 1,000 agents crossing a wall through a
  doorway 4 m wide with avoidance: about 1.2 ms a step, 870 agents per
  millisecond on one core.
- Avoidance finds obstacle candidates through a grid of their bounds:
  1,000 agents among 4,096 obstacle points went from 15.2 ms a step to
  0.57 ms; the lines are the same for any cell size.
- Flow fields (mnav-0007): flow.h with mnavFlowField, mnavBuildFlowField
  (Dijkstra from every goal over an mnavGrid with the grid path's steps
  and costs) and mnavFlowAt (a cell's cost to the nearest goal and its
  next cell).
- bench/bench_main.c times flow fields over 512 by 512 cells with
  walls: about 24 ms a build for one goal, 27 ms for four.
- Hierarchical paths (mnav-0008): hierarchy.h with mnavHierarchy,
  mnavBuildHierarchy (clusters of tiles, a transition per run of tile
  links leaving a cluster, edges by searches confined to a cluster) and
  mnavFindHierarchicalPath (A* over the transitions, refined step by
  step within one cluster at a time into an ordinary path).
- Hierarchies take off-mesh links between clusters as transitions.
- mnavUpdateHierarchy: after a commit that changed only polygon areas,
  only the clusters it touched are searched again; the report counts
  the searches.
- bench/bench_main.c builds a hierarchy on a flat world of 24 by 24
  tiles with pillars and walls and finds three long paths: about 13 ms
  each plainly with 131,072 nodes, 1.4 ms through the hierarchy with
  8,192, at the same lengths; the build takes about 84 ms.
- Spatial queries: mnavGetHeight, mnavFindWallDistance,
  mnavFindRandomPoint and mnavFindRandomPointAround (from a 64-bit seed,
  uniform by area, the latter within its circle), mnavCheckReachable.
- Link generation (mnav-0009): linkgen.h with mnavGenerateLinks, drops
  off ledges and jumps across gaps from a committed navmesh, kept by
  the host's clearance test, a walking-detour rule and a near-duplicate
  filter, for the host to stage.
- Debug output (mnav-0010): draw.h with mnavDebugBuffer and kinds;
  mnavDebugNavmesh, mnavDebugLinks, mnavDebugPath and mnavDebugCorridor
  in debug.h, mnavDebugFlowField in flow.h, mnavDebugAvoidance in
  avoidance.h.
- mnavBakeReport.droppedRegions counts the islands dropped under the
  minimum region area.
- Terrains (mnav-0003): mnavTerrain, mnavBakeInput and
  mnavBakeTileInput bake heightfields in a compact form beside meshes.
- Bake volumes (mnav-0003): mnavBakeVolume in mnavBakeInput, include,
  exclude and area override, applied around the agent radius's erosion.
- Flow fields over a region of a grid, built in budgeted steps:
  mnavFlowRegion, mnavBeginFlowField and mnavContinueFlowField;
  mnavDebugFlowField takes the grid's cell size from the field.
- mnavUpdateFlowField repairs a flow field after goals or cell areas
  change, searching only the cells whose ways change; the result is the
  field a rebuild gives, bit for bit.
- Fuzz targets for the bake input (fuzz_bake) and query inputs
  (fuzz_query).
- A differential harness against Recast and Detour (mnav-0011), test
  only, behind MAUL_NAV_DIFFERENTIAL.
- The benchmark prints named results and their ratios to
  bench/baseline.txt (mnav-0012); sliced path queries are timed too.
- Edge-to-edge links: mnavLinkDef.width, crossed at points spaced along
  it; mnavLinkState.crossings.
- Flow fields over the navmesh (mnav-0013): navflow.h, mnavNavFlow,
  mnavBuildNavFlow and mnavNavFlowAt.
- Navmesh flow fields over a region of tiles, in budgeted steps:
  mnavNavFlowRegion, mnavBeginNavFlow and mnavContinueNavFlow.
- Bake reports give each stage's time and memory peak: mnavClock in the
  bake def, stageTicks and stageMemory in mnavBakeReport.
- Navmesh flow fields take their memory when made, for named limits of
  polygons, tile slots and links (mnavNavFlowLimits; the def's polygons
  field moves into it), and draw their arrows (mnavDebugNavFlow).
- Samples: walk (bake to a walking agent), crowd (avoidance alone) and
  minimal (a consumer of the installed package, built in CI).
