# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and the tile formats.

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
