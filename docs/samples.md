# Samples

Small programs in `samples/`, built with the library when it is the
top-level project (`MAUL_NAV_BUILD_SAMPLES`).

| Program | What it shows |
|---|---|
| `walk.c` | From geometry to a walking agent: a floor with a wall baked into four tiles, loaded into a navmesh, a path round the wall, and an agent following it through a path corridor a meter a step. |
| `crowd.c` | Avoidance on its own, with no navmesh: two rows of agents swap sides round a pillar; the host passes the agents each step and moves them itself. |
| `minimal/` | A separate project that uses only the installed package, in C17: it checks the linked version, bakes one tile and finds a path across it. CI builds it against a fresh install. |

Build the minimal sample against an install:

```sh
cmake --install build --prefix "$PWD/package"
cmake -S samples/minimal -B build-minimal -DCMAKE_PREFIX_PATH="$PWD/package"
cmake --build build-minimal
./build-minimal/minimal
```
