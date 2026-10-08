# Maul Nav

Navigation for games, engines and simulations: navmesh generation from
triangle meshes, heightfields and 2D polygons, a versioned navmesh
format that loads hostile input safely, path and spatial queries, path
corridors, off-mesh links, flow fields for mass movement, and local
avoidance as a separate component. Written in C23 with public headers
any C17 or C++17 program can include, with no dependencies and an MIT
license.

It has:

- navmesh generation in 3D from triangle meshes and terrains, and in
  2D from polygons, shaped by bake volumes, tile by tile: each tile is
  an independent call the host may run on its own workers, giving the
  same bytes at any worker count, and a tile index of a large mesh or
  outline set that the workers share, so each tile reads only what
  reaches it;
- the navmesh data format, versioned and fingerprinted, and a loader
  that refuses malformed bytes with a typed error;
- nearest-point, path, raycast, height, wall-distance, surface-move,
  shape, random-point and reachability queries, with named limits on
  every search and typed results when one is hit;
- path corridors, sliced and hierarchical searches, grid pathfinding
  (A* and jump point search), and flow fields over grids and navmeshes;
- off-mesh links point to point or edge to edge, one or two way, of
  host-declared kinds, toggled at runtime and generated automatically;
- runtime changes by declared tier (static, modifiers only, dynamic),
  committed at a point the host chooses;
- velocity-space avoidance (ORCA), a component that needs no navmesh;
- debug geometry as plain vertices and indices, and bake reports.

It owns no world, entities, physics or animation: it takes geometry
the host extracts, and returns paths, corners and desired velocities
the host applies. Results are bit-identical on every platform,
compiler and worker count. It starts no threads. It calls host code
only when asked to, synchronously, on the calling thread and within
the call: an allocator, the clearance test link generation takes and
the clock a bake may read for its report.

## Status

Version 0.4.4. Everything listed above is built and tested on every CI
platform; fuzz targets cover tiles, bake input, queries, avoidance,
the searches over a whole navmesh and grid flow fields. Before 1.0.0 a
minor release may change the API, the ABI and the tile format.

## Documentation

- [The guide](docs/guide.md) walks through baking, the navmesh,
  queries, corridors, flow fields, avoidance and debug output.
- [The API reference](docs/api.md) lists all 90 public functions,
  generated from the headers.
- [The samples](docs/samples.md) are small complete programs.
- [The changelog](CHANGELOG.md) records every change.

## Building

Requirements: CMake 3.25 and GCC 14 or Clang 19 or newer; on Windows,
`clang-cl` (the Visual Studio component "C++ Clang tools for Windows").

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

The suite is checked against hand-picked mutants: `tools/mutants.txt`
lists 234 one-line changes to the library, each of which a test kills
(a few only in a sanitizer build) or which cannot change a result, the
file giving the reason above it. To run them, on a clean tree (hours;
not part of CI):

```sh
python3 tools/mutate.py --check tools/mutants.txt
python3 tools/mutate.py tools/mutants.txt build results.txt
```

## Design

The rules every Maul library follows are in `docs/conventions.md` and
`docs/adr/`; the records particular to this library are listed in
`docs/adr/mnav.md`.

## License

MIT; see `LICENSE`.
