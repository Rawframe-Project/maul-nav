# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and the tile formats.

## [0.0.3](https://github.com/Rawframe-Project/maul-nav/compare/v0.0.2...v0.0.3) (2026-10-24)


### Fixed

* compute in the library's floating-point environment ([#24](https://github.com/Rawframe-Project/maul-nav/issues/24)) ([63a9711](https://github.com/Rawframe-Project/maul-nav/commit/63a9711effabe595e5fe101618d374b3cd0e83ae)), closes [#20](https://github.com/Rawframe-Project/maul-nav/issues/20)


### Changed

* **flight:** cut a flight path's instructions to a third ([#22](https://github.com/Rawframe-Project/maul-nav/issues/22)) ([eeb9430](https://github.com/Rawframe-Project/maul-nav/commit/eeb9430c79d9d7afa1ef55da18ed1ae3a188115c)), closes [#17](https://github.com/Rawframe-Project/maul-nav/issues/17)

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
