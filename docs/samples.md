# Samples

Small programs in `samples/`, built with the library when it is the
top-level project (`MAUL_NAV_BUILD_SAMPLES`).

| Program | What it shows |
|---|---|
| `walk.c` | From geometry to a walking agent: a floor with a wall baked into four tiles, loaded into a navmesh, a path round the wall, and an agent following it through a path corridor at the velocity `mnavSteer` gives. |
| `bake_world.c` | Baking a world from one large mesh: hills with walls, about 33,000 triangles over 4 by 4 tiles. A tile index is made once and shared by four worker threads, each with its own baker; the tiles are checked against a one-thread bake without the index, to the byte, then loaded and crossed by a path. |
| `crowd.c` | Avoidance on its own, with no navmesh: two rows of agents swap sides round a pillar; the host passes the agents each step and moves them itself. |
| `flight.c` | Fliers among pillars: a ground with four pillars baked into a flight volume, and two streams of twelve fliers crossing it at right angles. Each follows its flight path with avoidance in space; a raycast each step keeps it in open space, sliding along the axes when a step is blocked. Three seconds in, a fifth pillar rises: its tile alone is baked again and committed, and the fliers whose paths it blocks, by `mnavCheckFlightPath`, search again. |
| `obstacles.c` | Obstacles from a tile cache: a floor baked into four tiles through a tile cache and loaded into a navmesh of the dynamic tier, with eight agents crossing it. A barricade of crates falls across it, a crate drops in one tile, and the barricade is lifted; each change rebuilds only the tiles the changed crates reach, with every crate standing on them, and commits them together, and the agents whose corridors broke, by `mnavCheckCorridor`, plan again. |
| `minimal/` | A separate project that uses only the installed package, in C17: it checks the linked version, bakes one tile and finds a path across it. CI builds it against a fresh install. |

Build the minimal sample against an install:

```sh
cmake --install build --prefix "$PWD/package"
cmake -S samples/minimal -B build-minimal -DCMAKE_PREFIX_PATH="$PWD/package"
cmake --build build-minimal
./build-minimal/minimal
```
