# Maul Nav

Navigation for games, engines and simulations: navmesh generation from
triangle meshes, heightfields and 2D polygons, a versioned navmesh
format that loads hostile input safely, path and spatial queries, path
corridors, off-mesh links, flow fields for mass movement, and local
avoidance as a separate component. Written in C23 with public headers
any C17 or C++17 program can include, with no dependencies and an MIT
license.

It will own:

- navmesh generation in 3D and 2D, tiled, run in parallel through the
  host's task system, with the same bytes at any worker count;
- the navmesh data format and its loader;
- nearest-point, path, raycast and other spatial queries, with named
  limits on every search and typed results when one is hit;
- path corridors, sliced and hierarchical searches, grid pathfinding
  and flow fields;
- runtime changes by declared tier (static, modifiers only, dynamic),
  committed at a point the host chooses;
- velocity-space avoidance, in its own library target, usable without
  a navmesh.

It owns no world, entities, physics or animation: it takes geometry
the host extracts, and returns paths, corners and desired velocities
the host applies. Results are bit-identical on every platform,
compiler and worker count. It starts no threads and calls no
application code.

## Status

Not released. The first decisions are made and the skeleton builds;
the input surface, tiled 3D generation, the format and its loader come
first, then the queries.

## Building

Requirements: CMake 3.25 and GCC 14 or Clang 19 or newer; on Windows,
`clang-cl` (the Visual Studio component "C++ Clang tools for Windows").

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## Design

The rules every Maul library follows are in `docs/conventions.md` and
`docs/adr/`; the records particular to this library are listed in
`docs/adr/mnav.md`.

## License

MIT; see `LICENSE`.
