# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and every data
format.

## [Unreleased]

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
