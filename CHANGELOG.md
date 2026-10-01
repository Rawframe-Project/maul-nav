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
  order (N12). Its hash is pinned across platforms.
- The walkable filters (internal): spans within a step above walkable
  ground become walkable, ledges and steep neighbors are removed, and
  so is ground without the agent's height of free space (N13).
- The open-space field and erosion (internal): walkable spans become
  open space linked to the neighbors an agent can step to, and space
  closer than the agent's radius to a boundary is removed. Heights,
  links and distances are 16 bits, which always suffice, so nothing is
  truncated (N14). Pipeline hashes over a soup and a small level are
  pinned.
- `mnavHash64` and `MNAV_HASH_INIT`, the family's frozen 64-bit hash,
  for determinism checks and navmesh fingerprints.
- `mnavBakeDef.minRegionArea`, in square meters (2 by default), and
  region partitioning by layers (internal): monotone sweeps merged into
  non-overlapping layers of one area, small islands off the tile's
  border dropped, with 32-bit ids and sweep storage sized by the row's
  spans (N15).
- `mnavBakeDef.maxEdgeError` and `maxEdgeLength`, in meters (0.3 and
  12 by default), and region contours (internal): each region's
  boundary traced edge by edge with corner heights and the region
  across each edge, simplified along walls and area borders, walls
  split past the maximum length (N16).
- Hole merging (internal): each region's holes are bridged into its
  outline left to right through the nearest vertex whose diagonal
  crosses nothing, every tie broken by an index, and a hole that cannot
  be bridged is dropped and counted for the bake report (N17).
- The polygon mesh (internal): each region's ring cut into triangles by
  ear clipping, shortest diagonal first, its vertices welded across the
  tile, merged into convex polygons of at most 6 vertices along their
  longest shared edges, and linked to the polygons across each edge or
  marked with the tile side it lies on. A ring that cannot be finished
  keeps its triangles and is counted (N18).
- Removing tile-border vertices (internal): a vertex where two regions
  meet only because of the border around the tile is removed when its
  polygons keep more than 2 edges, share one area, leave at most 2
  edges at it unshared and the outline keeps its shape; the hole is
  triangulated, merged and checked before any polygon changes, and a
  vertex that cannot be replaced stays flagged (N19). Linking polygons
  is now its own step after removal, and ear clipping refuses ears with
  a ring vertex inside them.
- The height patch under a polygon (internal), the first part of the
  detail mesh: each cell's floor in the polygon's region, flooded
  outward into the cells round it; a polygon of mixed regions floods
  from the spans nearest its vertices. A lookup in a cell without a
  height takes the nearest height in the first ring of cells that has
  one, and says so when none is within its radius rather than
  returning a marker as a height (N20).
