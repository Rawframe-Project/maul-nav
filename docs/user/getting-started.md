# Getting started

Maul Nav turns level geometry into a navmesh and answers the questions agents ask of it: where can I stand, how do I get there, what is in my way. This page covers building and installing the library and the rules every part of its API follows.

## Requirements

- CMake 3.25 or newer.
- GCC 14 or Clang 19 or newer; on Windows, `clang-cl` (the Visual Studio component "C++ Clang tools for Windows"). The library is C23.
- The public headers are the common subset of C17 and C++17, so any C or C++ program can include them.

## Building and installing

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix "$PWD/package"
```

A program finds the installed package with CMake:

```cmake
find_package(maul-nav REQUIRED CONFIG)
target_link_libraries(app PRIVATE maul-nav::maul-nav)
```

or through pkg-config as `maul-nav`. `samples/minimal` is such a program: it checks the linked version, bakes one tile and finds a path across it.

| Option | Default | Effect |
|---|---|---|
| `MAUL_NAV_BUILD_SHARED` | `OFF` | A shared library instead of a static one. |
| `MAUL_NAV_BUILD_TESTS`, `_BENCH`, `_SAMPLES` | on when built on its own | The test suites, the benchmark and the samples. |
| `MAUL_NAV_INSTALL` | on when built on its own | Install rules and the CMake and pkg-config packages. |

## Samples

The programs in `samples/` are built with the library when it is the top-level project.

| Program | What it shows |
|---|---|
| `walk.c` | From geometry to a walking agent: four tiles baked from a floor with a wall, a path round the wall, and an agent following it through a corridor at the velocity `mnavSteer` gives. |
| `bake_world.c` | A world baked from one large mesh on four worker threads sharing one tile index, checked to the byte against a one-thread bake without the index. |
| `crowd.c` | Avoidance alone, with no navmesh: two rows of agents swap sides round a pillar. |
| `navcrowd.c` | A crowd on a navmesh: two groups cross a room and meet head on in a corridor, with corners, steering, walls, avoidance and surface moves each step. |
| `flight.c` | Fliers among pillars in a flight volume, with avoidance in space; a new pillar rises and only its tile is baked again. |
| `obstacles.c` | Crates dropped on a navmesh through a tile cache; only the tiles they reach are rebuilt, and the agents whose corridors broke plan again. |

## How the API works

- **Units and axes.** Meters and seconds, right-handed, +Y up and -Z forward. World positions are doubles (`mnavPos3`), so precision does not depend on the distance from the world origin. Positions inside a frame the library names (a bake's origin, a debug buffer's origin) are floats.
- **Results.** Every function that can fail returns `mnavResult`, or a small struct that starts with one and names what was refused. Zero is success and negative values are errors; `mnavResultName` names any of them. Results are `[[nodiscard]]` where the compiler supports it. [Troubleshooting](troubleshooting.md) lists what each error means.
- **Defs.** Every object (baker, navmesh, query context, tile cache, flow field, hierarchy, avoidance set, flight baker and volume) is made from a def that its `mnavDefault...Def` function returns, adjusted, and is given back with its `mnavDestroy...` function. The defaults are built in the program from the headers it includes, so always start from one: a def that is not is refused with `mnav_errorInvalid`, and a creation's def built against headers of another library version with `mnav_errorVersion`.
- **Limits.** A def carries named limits: nodes per search, polygons per tile, agents per step. The object takes the memory they need when it is made. Reaching a limit is a typed result, never unbounded work or a silent cut.
- **Memory.** Every def has an allocator; a zeroed one means the C library's. Queries never allocate: their scratch is the query context's.
- **Threads.** The library starts none and calls host code only when asked to, on the calling thread. Each object is used by one thread at a time, and any number of threads may query one navmesh while no stage or commit call runs, each with its own query context. Every function's documentation says what may run beside it.
- **Hostile input.** Every argument, every input triangle and every byte of a tile is checked, and what is wrong is refused with a status in every build.
- **Determinism.** The same inputs give the same bytes and the same results on every platform, compiler and worker count. Random queries take a seed from the caller. `mnavHash64` hashes bytes the same way everywhere, so a host can pin its own results as the library's tests do.
- **The host moves things.** The library owns no world, entities or animation. It takes geometry the host extracts and returns paths, corners and velocities the host applies.

## The API reference

Every public function is documented in its header, with what each parameter takes, what each result means and what may run beside it. The [documentation site](https://rawframe-project.github.io/maul-nav/) has the same reference generated from the headers.
