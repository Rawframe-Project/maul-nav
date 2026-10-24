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

The test presets leave out the tests labeled `nightly`, the large scenes; `ctest --preset nightly` runs them in the `linux-clang-20` build. `linux-clang-20`, `linux-shared`, `macos-clang` and `windows-clang-cl` also run the `package` test (`MAUL_NAV_TEST_PACKAGE`, `cmake/PackageTest.cmake`), which installs the build and builds `samples/minimal` against it.

A suite prints one line per failed check with its file and line. Suites marked `WHITEBOX` in `CMakeLists.txt` include `src/` and are not built in shared builds, where internal symbols are hidden. `test_guide.c` runs the C snippets of `docs/user/` as written; `tools/check_guide.py` fails when a snippet and its copy differ.

Golden values (tile bytes and results of a few scenes, by `mnavHash64`) are constants in the suites. A test that prints a changed hash names it; change the constant only for a change of results that the pull request explains. Every query golden runs on tiles read from `test/stored_tiles.h` (the test world, the terrain and the link generation trench) rather than baked, and a suite that bakes its own scene checks it exactly instead of by hash. A bake change therefore moves only the bake goldens (`LEVEL_TILE_HASH`, `TILES_HASH`, `ROOM_HASH`, `VOLUME_HASH`, the scenarios' `TERRAIN_HASH`, `FLIGHT_BYTES_HASH`); when it does, raise `MNAV_BAKE_GENERATOR` or `MNAV_FLIGHT_GENERATOR` in the same pull request.

## Fuzzing

Each target, `test/fuzz_<area>.c`, reads bytes as the library's input: tile and flight tile bytes, bake input, queries, the searches over a whole navmesh, avoidance, grid flow fields, the exact shortest path against the A* search, flight queries and the tile cache's coder. `test/corpus/fuzz_<area>.tar.gz` holds a minimized seed corpus for each, which every target must have, and `test_tile --seed <file>` adds the level tile to `fuzz_tile`'s.

```sh
tools/fuzz.sh build out/fuzz                     # every target, Clang 20 with the sanitizers
tools/fuzz.sh run out/fuzz 300 4                 # each for 300 s from its seeds, 4 at a time
tools/fuzz.sh night out/fuzz fuzz_query 600      # one target from out/corpus/fuzz_query and its seeds, merged back
tools/fuzz.sh targets out/fuzz                   # the targets the build defines, as JSON
./out/fuzz/fuzz_query out/fuzz-crashes/crash-<hash>
```

`tools/fuzz.sh` always bounds a run by `-max_total_time`; run long campaigns through the workspace's heavy-job wrapper. To grow a target's committed seeds, pack a merged corpus with `tar -czf test/corpus/fuzz_<area>.tar.gz -C out/corpus/fuzz_<area> .` when its new inputs reach code the seeds do not. A crash becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`).

## The differential harness

`test/differential/` bakes four scenes (the test world, a two-floor building, a terrain bowl and plateaus with two bridges) with Maul Nav and with Recast at the same settings, and compares reachability and straight path lengths over 200 pairs of points per scene against the floors at the top of `differential.cpp`. CMake fetches recastnavigation at a pinned commit for the harness alone; `-DFETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION=<dir>` uses a local copy. Neither Recast nor Detour is ever linked into the library.

```sh
cmake -B out/differential -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-14 \
  -DCMAKE_CXX_COMPILER=g++-14 -DMAUL_NAV_DIFFERENTIAL=ON
cmake --build out/differential --target differential
./out/differential/differential
```

After the agreement table it prints what each library costs on the same scenes, the best of five runs, reported and never gated. The header says Maul Nav is measured in 32 m tiles and Recast untiled:

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

- **A red CI cell.** A failed test cell uploads its ctest logs as `logs-<preset>` for a week (`gh run download <run> -n logs-<preset>`); the `web` job's are `logs-web-emscripten` and the large scenes' `logs-nightly`. A fuzz job that finds something uploads `fuzz` (or `fuzz-<target>` at night): the logs and the input that failed. The same preset reproduces a cell here: `cmake --preset <preset> && cmake --build --preset <preset> && ctest --preset <preset> -L <suite> --output-on-failure`.
- **A fuzz finding.** The target run on the one input replays it and stops: `./out/fuzz/fuzz_query out/fuzz-crashes/crash-<hash>`. `-minimize_crash=1 -max_total_time=60` shrinks it before it becomes a test in its module's suite.
- **A broken assertion.** `MNAV_ASSERT` traps where the check fails. In the sanitizer presets, `ASAN_OPTIONS=handle_sigill=1 out/linux-clang-20-asan/test_<suite>` prints the stack of the trap with file and line; under a debugger, the trap stops at the check.
- **A golden that differs under one host floating-point setting.** `EveryFloatHost` (`test/float_hosts.h`) prints `float host <i>` with the hash it got, and `the setting changed` when a call left the host's setting changed. On x86-64 and AArch64, host `i` is rounding mode `i / 2` (to nearest, downward, upward, toward zero), with flush-to-zero and denormals-are-zero on when `i` is odd; elsewhere it is rounding mode `i`. A public call that computes outside its entry, or host code that runs inside one, is the usual cause.
- **A moved golden.** Each suite prints the hash it got (`LEVEL_TILE_HASH=...`), so the commit that moved it is found by building commits and comparing. A moved bake golden with unmoved query goldens is a bake change; a moved query golden is a query change, since those run on `test/stored_tiles.h`.
- **The differential below a floor.** Its table gives each scene's shares, and the floors are at the top of `test/differential/differential.cpp`. `pairs` below 180 means too few pairs snapped to both navmeshes; `reach` below the floor means the libraries disagree on which pairs connect; `length`, `notlonger` and `sums` mean Maul Nav's straight paths grew or shrank against Detour's on that scene.
- **The web cell.** With an Emscripten SDK installed and `EMSDK` set (`source <emsdk>/emsdk_env.sh`): `cmake --preset web-emscripten && cmake --build --preset web-emscripten && ctest --preset web-emscripten`, the tests running under Node.
- **ThreadSanitizer in docker.** Where docker's seccomp blocks `personality(ADDR_NO_RANDOMIZE)`, TSan does not start in the CI image: run the `linux-clang-20-tsan` preset on the host, or add `--security-opt seccomp=unconfined`.
