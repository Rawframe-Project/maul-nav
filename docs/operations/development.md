# Development

How to build, test and change Maul Nav as a maintainer. Building and installing as a user is in [getting-started.md](../user/getting-started.md).

## Toolchain

The organization's CI image, `ghcr.io/rawframe-project/ci-cpp`, has everything below; CI runs every Linux job in it, pinned by digest. On another machine:

- CMake 3.25 or newer and Ninja.
- GCC 14 and Clang 20 (`clang-20`, `clang-tidy-20`), with the sanitizer and libFuzzer runtimes (`libclang-rt-20-dev` on Debian and Ubuntu). GCC 13 is refused by `cmake/CompilerFlags.cmake`.
- `clang-format` at the version CI pins: `pip install clang-format==22.1.5`. Another version formats differently.
- Python 3 for the checks and the documentation site (`pip install markdown==3.6` for the site only).
- A C++17 compiler for the header tests and the differential harness; Emscripten for web builds.

## Build options for development

| Option | Effect |
|---|---|
| `MAUL_NAV_WERROR` | Warnings as errors, as CI builds. |
| `MAUL_NAV_SANITIZE` | AddressSanitizer and UBSan. |
| `MAUL_NAV_TSAN` | ThreadSanitizer, for the suites that bake and query on several threads. |
| `MAUL_NAV_COVERAGE` | gcov instrumentation. |
| `MAUL_NAV_FUZZ` | The libFuzzer targets, `fuzz_<area>` at the top of the build tree (Clang only), with the library instrumented too. |
| `MAUL_NAV_DIFFERENTIAL` | The harness that compares with Recast and Detour. |

`MNAV_ASSERT` is on in every build without `NDEBUG`: Debug, the sanitizer builds and the fuzz builds. A broken invariant traps with `__builtin_trap`, so a debugger or a core dump stops at the failing check.

## Presets and running one test

Every CI cell is a CMake preset in `CMakePresets.json`: a configure, a build and a test preset of the same name, built in `out/<preset>`. The Linux presets name `gcc-14` or `clang-20`; `linux-clang-20-asan` and `linux-clang-20-tsan` are Debug builds with assertions on. Every suite is `test/test_<name>.c`, one executable and one CTest test labeled with its name:

```sh
cmake --preset linux-clang-20-asan
cmake --build --preset linux-clang-20-asan
ctest --preset linux-clang-20-asan -L corridor
./out/linux-clang-20-asan/test_corridor
```

The test presets leave out the tests labeled `nightly`, the large scenes; `ctest --preset nightly` runs them in the `linux-clang-20` build. `linux-clang-20-asan` also leaves out the tests labeled `dense`, the slow suites' full sweeps, which `ctest --preset dense-asan` runs in its build. `linux-clang-20`, `linux-shared`, `macos-clang` and `windows-clang-cl` also run the `package` test (`MAUL_NAV_TEST_PACKAGE`, `cmake/PackageTest.cmake`), which installs the build and builds `samples/minimal` against it.

A suite prints one line per failed check with its file and line. Suites marked `WHITEBOX` in `CMakeLists.txt` include `src/` and are not built in shared builds, where internal symbols are hidden. `test_guide.c` runs the C snippets of `docs/user/` as written; `tools/check_guide.py` fails when a snippet and its copy differ.

Golden values (tile bytes and results of a few scenes, by `mnavHash64`) are constants in the suites. A test that prints a changed hash names it; change the constant only for a change of results that the pull request explains. Every query golden runs on tiles read from `test/stored_tiles.h` (the test world, the terrain, the link generation trench and the doorway crowd's rooms) rather than baked, and a suite that bakes its own scene checks it exactly instead of by hash. The query suites' hand-built tiles, whose polygons lie where a test needs them, are bytes too: `test/hand_tiles.h`, written by `test_hand_tiles --write` from the squares in `test/test_hand_tiles.c`, each with what it is built to exercise; `test_hand_tiles --check test/hand_tiles.h` fails when the file differs. A bake change therefore moves only the bake goldens (`LEVEL_TILE_HASH`, `TILES_HASH`, `ROOM_HASH`, `VOLUME_HASH`, the scenarios' `TERRAIN_HASH`, `FLIGHT_BYTES_HASH`); when it does, raise `MNAV_BAKE_GENERATOR` or `MNAV_FLIGHT_GENERATOR` in the same pull request.

## Fuzzing

Each target, `test/fuzz_<area>.c`, reads bytes as the library's input: tile and flight tile bytes, bake input, queries, the searches over a whole navmesh, avoidance, grid flow fields, the exact shortest path against the A* search, flight queries and the tile cache's coder. `test/corpus/fuzz_<area>.tar.gz` holds a minimized seed corpus for each, which every target must have, and `test_tile --seed <file>` adds the level tile to `fuzz_tile`'s.

```sh
tools/fuzz.sh build out/fuzz                     # every target, Clang 20 with the sanitizers
tools/fuzz.sh run out/fuzz 300 4                 # each for 300 s from its seeds, 4 at a time
tools/fuzz.sh seeds out/fuzz                     # every target's seeds, into out/corpus-run/<target>
tools/fuzz.sh check out/fuzz fuzz_tile 20        # one target as CI's fuzz job runs it, over out/corpus/fuzz_tile
tools/fuzz.sh night out/fuzz fuzz_query 600      # one target from out/corpus/fuzz_query and its seeds, merged back
tools/fuzz.sh targets out/fuzz                   # the targets the build defines, as JSON
./out/fuzz/fuzz_query out/fuzz-crashes/fuzz_query/crash-<hash>
```

`tools/fuzz.sh` always bounds a run by `-max_total_time` and runs each target through `tools/fuzz_target.sh`, which writes the log, its seed first, to `out/fuzz-logs/<target>.log`, a finding to `out/fuzz-crashes/<target>/` and, on a finding, its report without the input to `out/fuzz-reports/<target>.txt`; run long campaigns through the workspace's heavy-job wrapper. A corpus is merged with `tools/fuzz_target.sh --merge TARGET MERGED INPUT_DIR...`, which keeps the merge's log there too and fails when an input crashes. To grow a target's committed seeds, pack a merged corpus with `tar -czf test/corpus/fuzz_<area>.tar.gz -C out/corpus/fuzz_<area> .` when its new inputs reach code the seeds do not. A crash becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`).

## The differential harness

`test/differential/` bakes four scenes (the test world, a two-floor building, a terrain bowl and plateaus with two bridges) with Maul Nav and with Recast at the same settings, and compares reachability and straight path lengths over 200 pairs of points per scene against the floors at the top of `differential.cpp`. It also casts a ray from each pair's first point toward its second with both libraries and compares whether each stops at a wall, and where. Each library keeps a simplified wall within 0.3 m of the cells it follows (Recast's `maxSimplificationError` of 1.2 cells, Maul Nav's `maxEdgeError`), so the same wall may stand up to 0.6 m apart in the two navmeshes, and a ray may stop at one and pass the other's corner. A pair the rays disagree on passes only when the other library has a wall within 0.6 m of every point a ray stopped at, along that wall's normal, and then in two ways. When both stop, the pair is banded if the library whose ray went farther, cast again toward 0.6 m past the other's stop, stops within 0.6 m of it, so a ray that went through a wall both navmeshes have still fails. When one stops and the other reaches the end, the pair is grazed if the ray that reached the end crosses none of its own library's walls within 0.6 m of the other's stop (Maul Nav's from `mnavQuery_FindWalls` around that point, Detour's from `getPolyWallSegments` over the polygons its ray went through): the ray passed the other library's corner where its own wall is a chord, and a ray that went through a wall of its own navmesh fails. So that this check is seen to work, every ray that stopped, gone on to the end, must cross one of its own walls near its stop, and walls that could not be read (no polygon of the ray under the point, the node limit, too many walls) fail either check rather than pass it. A pair passes only as `kRayAllowed` in `differential.cpp` lists it, by scene, index and kind, every one explained in the pull request that listed it, so a change in which pairs disagree always fails: the world lists 10 banded pairs (41, 61, 64, 83, 100, 132, 155, 181, 191 and 198) and 1 grazed, pair 8, which grazes the corner of the box at x 30 to 34 and z 10 to 12 that Maul Nav cuts with a chord, and every other scene none. A listed pair the rays no longer disagree on fails too, so the list stays the set of known disagreements: a fix removes its pairs from the list in the same pull request, and no entry stays to cover a later relapse. It also finds the nearest wall within 5 m of each of the 400 points with both libraries, from the polygon nearest the point (`mnavQuery_FindWallDistance`, Detour's `findDistanceToWall`), and compares whether each finds one and how far. Their distances may differ by 0.6 m plus how far apart the two snap the point, and a wall only one finds lies within the band when it is within that much of the 5 m radius. A point that differs, by whether a wall is found or by more than 0.25 m, passes only within the band and as `kWallAllowed` lists it, by scene, the point and the wall Maul Nav finds there (its distance and z, within 1 mm), so a change of that wall fails until it is explained, and a listed point that no longer differs fails: the world lists points 12 and 156, south of the wall Maul Nav keeps at z 30.5 and Detour 0.25 m farther, where only Maul Nav's lies within 5 m. It also moves along the surface from each pair's first point toward its second, at most 5 m on the ground so that Detour's move stays within the 48 polygons and 64 nodes it keeps without a status (`mnavQuery_MoveAlongSurface`, Detour's `moveAlongSurface`), and compares whether each reaches the wanted point and where each ends. A pair whose moves end more than 0.25 m apart passes only when the two ends' distances from the wanted point lie within 0.6 m plus how far apart the two snap the start, and as `kMoveAllowed` lists it with the point Maul Nav's move ends at (within 1 mm), and a listed pair that no longer differs fails. CMake fetches recastnavigation at a pinned commit for the harness alone; `-DFETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION=<dir>` uses a local copy. Neither Recast nor Detour is ever linked into the library.

```sh
cmake -B out/differential -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-14 \
  -DCMAKE_CXX_COMPILER=g++-14 -DMAUL_NAV_DIFFERENTIAL=ON
cmake --build out/differential --target differential
./out/differential/differential
```

After the agreement table it prints what each library costs on the same scenes, the best of five runs, reported and never gated. When `GITHUB_STEP_SUMMARY` names a file, as in a GitHub Actions job, it appends the whole report to it too: CI shows a passing test's output nowhere in the job's log, so the report is on the job's summary page. The header says Maul Nav is measured in 32 m tiles and Recast untiled:

| Measure | What both sides do |
|---|---|
| `bake` | The whole scene. Maul Nav bakes every 32 m tile and copies its bytes; Recast builds one untiled mesh and Detour's navmesh data from it. |
| `path` | One pair, averaged over the pairs both sides snap to a polygon (as the agreement table counts them), so both search every pair measured: the nearest polygon at each end, the search and the straight path. |
| `nodes` | The nodes one search makes, averaged over the pairs: Maul Nav's query context's count, Detour's node pool's. |
| `bytes` | The navmesh's bytes: Maul Nav's tiles in all, Detour's navmesh data. |

`ratio` is Maul Nav's over Detour's, so below 1 is cheaper. `bake` and `path` appear twice, once in user-space instructions retired (`M ins`, `k ins`) and once in time (`ms`, `us`). The instruction count barely moves with the machine's load, so it is the one to compare runs by; the time shows the cache and memory too, and the header prints the load average it was taken under. Where `perf_event_open` is not available (macOS, Windows, or a sandbox that refuses it), the header says so and only times are printed. A pull request that claims a speedup gives both counts from the same build before and after.

## Benchmarks

`maul-nav_bench` times a terrain of 8 by 8 tiles: bakes and cached rebuilds, streaming, paths, avoidance in crowds, flow fields, a hierarchy over a flat world, and flight volumes. It prints each section's counts, which do not depend on the machine, then its timings, the best of five runs. Timings are never a CI gate.

To measure a change, build the base and the change in Release with the same compiler, run each pinned to one core (`taskset -c 2 ./maul-nav_bench > base.txt`) on a quiet machine, and pass the base's output to the change's run to print the ratios. On a loaded machine, compare instruction counts (`perf stat -e instructions`) instead. Give both numbers in the pull request.

`maul-nav_bench --count flight-paths <runs>` bakes the flight volume, picks the fliers' 200 ends and then finds their paths `runs` times, printing nothing. The difference between two counts is the paths' alone:

```sh
perf stat -e instructions:u out/linux-gcc-14/maul-nav_bench --count flight-paths 0
perf stat -e instructions:u out/linux-gcc-14/maul-nav_bench --count flight-paths 1
```

## Debugging a failure

- **A red CI cell.** A failed test cell uploads its ctest logs as `logs-<preset>` for a week (`gh run download <run> -n logs-<preset>`); the `web` job's are `logs-web-emscripten` and the large scenes' `logs-nightly`. A fuzz job that finds something uploads nothing: its log prints each target's report, with the kind of each finding and the sanitizers' report and stack, while the input, the log and the run's seed stay on the runner. Reproduce it by fuzzing that target here (`tools/fuzz.sh run`, or `tools/fuzz.sh night` for a target the night found), where the input lands in `out/fuzz-crashes/<target>/`. The same preset reproduces a cell here: `cmake --preset <preset> && cmake --build --preset <preset> && ctest --preset <preset> -L <suite> --output-on-failure`.
- **A fuzz finding.** The target run on the one input replays it and stops: `./out/fuzz/fuzz_query out/fuzz-crashes/fuzz_query/crash-<hash>`. `-minimize_crash=1 -max_total_time=60` shrinks it before it becomes a test in its module's suite.
- **A broken assertion.** `MNAV_ASSERT` traps where the check fails. On x86-64 the trap is `ud2`, a SIGILL, and in the sanitizer presets `ASAN_OPTIONS=handle_sigill=1 out/linux-clang-20-asan/test_<suite>` prints its stack with file and line. On AArch64 it is `brk`, a SIGTRAP, which that option does not cover; there a debugger stops at the check.
- **A golden that differs under one host floating-point setting.** `EveryFloatHost` (`test/float_hosts.h`) prints `float host <i>` with the hash it got, and `the setting changed` when a call left the host's setting changed. On x86-64 and AArch64, host `i` is rounding mode `i / 2` (to nearest, downward, upward, toward zero), with flush-to-zero and denormals-are-zero on when `i` is odd; elsewhere it is rounding mode `i`. A public call that computes outside its entry, or host code that runs inside one, is the usual cause.
- **A moved golden.** Each suite prints the hash it got (`LEVEL_TILE_HASH=...`), so the commit that moved it is found by building commits and comparing. A moved bake golden with unmoved query goldens is a bake change, and a moved golden on `test/stored_tiles.h` is a query change. `FIELD_HASH` and `GROUP_HASH` use no navmesh, so a bake leaves them alone.
- **The differential below a floor.** Its table gives each scene's shares, and the floors are at the top of `test/differential/differential.cpp`. `pairs` below 180 means too few pairs snapped to both navmeshes; `reach` below the floor means the libraries disagree on which pairs connect; `length`, `notlonger` and `sums` mean Maul Nav's straight paths grew or shrank against Detour's on that scene. `rays differ past the wall band or the list` means a ray of one library stopped where the other has no wall within 0.6 m, went more than 0.6 m past the other's stop, or ended another way (a node limit), that a ray reached the end through one of its own walls near the other's stop (`through its own wall near the other's stop`), that a ray which stopped crosses none of its own walls near its stop (`crosses none of its walls near there`), that a library's walls near a stop could not be read (`could not be read`), or that a pair disagrees that `kRayAllowed` does not list as it is, or that a listed pair no longer disagrees (`remove it from kRayAllowed`). The report prints each pair the rays disagree on with its verdict, and the ray table's `differ` counts them, `banded` and `grazed` each kind and `listed` those the list allows; `stops` counts the rays that stopped and `crossed` those whose own walls near the stop the check saw them cross, which must be all. A new pair within the band is explained, as a difference of simplification or a Maul Nav error with its own fix, before it is listed. `walls differ past the list` means a point's walls differ past the band or as `kWallAllowed` does not list, that Maul Nav finds another wall at a listed point than the list gives (`listed with another Maul Nav wall`), or that a listed point no longer differs (`remove it from kWallAllowed`); the wall table's `differ` counts the points that differ and `listed` those the list allows. `moves end apart past the list` means a pair's moves end more than 0.25 m apart past the band, or as `kMoveAllowed` does not list, that Maul Nav's move at a listed pair ends elsewhere than the list gives (`listed with another Maul Nav end`), or that a listed pair no longer differs (`remove it from kMoveAllowed`); the move table's `ends` gives the share of moves both libraries end the same way, reached or stopped, `stops` counts the moves both stop, `close` the share of those that end within 0.25 m, `differ` the pairs that end apart and `listed` those the list allows.
- **The web cell.** With an Emscripten SDK installed and `EMSDK` set (`source <emsdk>/emsdk_env.sh`): `cmake --preset web-emscripten && cmake --build --preset web-emscripten && ctest --preset web-emscripten`, the tests running under Node.
- **ThreadSanitizer in docker.** Where docker's seccomp blocks `personality(ADDR_NO_RANDOMIZE)`, TSan does not start in the CI image: run the `linux-clang-20-tsan` preset on the host, or add `--security-opt seccomp=unconfined`.
