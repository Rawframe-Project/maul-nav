# Maul Nav

Game navigation in C: navmeshes baked from your geometry, paths, crowds and fliers, the same bits on every machine.

[![CI](https://github.com/Rawframe-Project/maul-nav/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul-nav/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/github/license/Rawframe-Project/maul-nav)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Rawframe-Project/maul-nav)](https://github.com/Rawframe-Project/maul-nav/releases)
[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/Rawframe-Project/maul-nav/badge)](https://scorecard.dev/viewer/?uri=github.com/Rawframe-Project/maul-nav)

Agents in a game need to know where they can walk, how to get somewhere, and how to get there without walking into each other. Maul Nav answers those questions for games, engines and simulations, and leaves the world to the host: it takes the geometry the host extracts and returns paths, corners and velocities the host applies.

- **Baking** navmesh tiles from triangle meshes, terrains and 2D outlines, shaped by include, exclude and area volumes. Each tile is an independent call the host runs on its own workers, with the same bytes at any worker count.
- **A tile format** that is loaded as hostile input: every byte is checked, and a damaged tile is refused with the section and element at fault.
- **Streaming and runtime changes:** tiles load and unload while queries run, changes commit all at once, areas and links toggle, and obstacles are carved into cached tiles in a third of a bake.
- **Queries:** nearest points, A* paths and exact shortest paths, sliced and hierarchical searches, rays, surface moves, walls, random points and grid paths, each under named limits.
- **Path corridors and steering**, and **off-mesh links** (jumps, drops, ladders, doors, teleports, or the host's own kinds), placed by hand or generated.
- **Flow fields** over grids and navmeshes, for many agents sharing goals.
- **Flight volumes** for agents that fly or swim: sparse voxel octree tiles with any-angle paths, rays and nearest open points.
- **Avoidance** (ORCA) on the ground and in space, a component that needs no navmesh.
- **Debug geometry** as plain vertices and indices any renderer can draw.

Results are bit-identical on every platform, compiler and worker count, so a server and its clients agree on every path. The library starts no threads, keeps no clock and takes no dependencies beyond the C library.

## Quick start

It builds with CMake 3.25 or newer and GCC 14 or newer, Clang 19 or newer, or Apple Clang. On Linux and macOS:

```sh
git clone https://github.com/Rawframe-Project/maul-nav.git
cd maul-nav
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
./build/sample_walk
```

Ubuntu 24.04's default compiler is GCC 13, which the build refuses ("GCC 14 or newer is required"). Install GCC 14 and configure again in a fresh build directory, since `build/` keeps the compiler it first found; then go on from `cmake --build build`:

```sh
sudo apt install gcc-14
rm -rf build && CC=gcc-14 cmake -B build -DCMAKE_BUILD_TYPE=Release
```

On Windows the library builds with `clang-cl`, and on the web with Emscripten; [Getting started](docs/user/getting-started.md) lists what each needs.

Install it with `cmake --install build --prefix <prefix>` and use it from CMake:

```cmake
find_package(maul-nav REQUIRED CONFIG)
target_link_libraries(app PRIVATE maul-nav::maul-nav)
```

or through pkg-config as `maul-nav`. [Getting started](docs/user/getting-started.md) covers the build options, the samples and the model the API follows.

## Platforms

Linux on x86-64 and arm64 with GCC 14 or Clang 19 and newer, macOS with Apple Clang, Windows with `clang-cl`, and WebAssembly with Emscripten; CI builds and tests each. The public headers are C17 and C++17, so any C or C++ program can include them.

## Documentation

- [The documentation site](https://rawframe-project.github.io/maul-nav/), with the API reference generated from the headers.
- [User documentation](docs/README.md): one page per part of the library.
- [How it works](docs/internals/overview.md) and [how to develop it](docs/operations/development.md).
- [Changelog](CHANGELOG.md).

Maul Nav belongs to the Maul family of standalone C libraries, which share one set of conventions.

## Contributing

Issues are welcome. Pull requests are accepted for issues a maintainer has approved; see [CONTRIBUTING.md](https://github.com/Rawframe-Project/.github/blob/main/CONTRIBUTING.md). The project is developed with AI coding agents, directed and reviewed by its maintainers; outside contributions follow the [AI policy](https://github.com/Rawframe-Project/.github/blob/main/AI_POLICY.md).

## License

MIT; see [LICENSE](LICENSE).
