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

## Running tests

Every suite is `test/test_<name>.c`, one executable and one CTest test labeled with its name:

```sh
cmake -B out/debug -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL_NAV_SANITIZE=ON
cmake --build out/debug
ctest --test-dir out/debug -L corridor
./out/debug/test_corridor
```

A suite prints one line per failed check with its file and line. Suites marked `WHITEBOX` in `CMakeLists.txt` include `src/` and are not built in shared builds, where internal symbols are hidden. `test_guide.c` runs the C snippets of `docs/user/` as written; `tools/check_guide.py` fails when a snippet and its copy differ.

Golden values (tile bytes and results of a few scenes, by `mnavHash64`) are constants in the suites. A test that prints a changed hash names it; change the constant only for a change of results that the pull request explains.

## Fuzzing

Each target, `test/fuzz_<area>.c`, reads bytes as the library's input: tile and flight tile bytes, bake input, queries, the searches over a whole navmesh, avoidance, grid flow fields, the exact shortest path against the A* search, flight queries and the tile cache's coder. `test/corpus/fuzz_<area>.tar.gz` holds a minimized seed corpus for each, and `test_tile --seed <file>` writes `fuzz_tile`'s first seed.

```sh
cmake -B out/fuzz -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL_NAV_FUZZ=ON -DMAUL_NAV_BUILD_TESTS=OFF
cmake --build out/fuzz
mkdir -p out/corpus/fuzz_query && tar -xzf test/corpus/fuzz_query.tar.gz -C out/corpus/fuzz_query
./out/fuzz/fuzz_query -max_total_time=300 out/corpus/fuzz_query
./out/fuzz/fuzz_query crash-<hash>
```

Always pass `-max_total_time`, and run long campaigns through the workspace's heavy-job wrapper. A crash becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`).

## The differential harness

`test/differential/` bakes four scenes (the test world, a two-floor building, a terrain bowl and plateaus with two bridges) with Maul Nav and with Recast at the same settings, and compares reachability and straight path lengths over 200 pairs of points per scene against the floors at the top of `differential.cpp`. CMake fetches recastnavigation at a pinned commit for the harness alone; `-DFETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION=<dir>` uses a local copy. Neither Recast nor Detour is ever linked into the library.

```sh
cmake -B out/differential -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-14 \
  -DCMAKE_CXX_COMPILER=g++-14 -DMAUL_NAV_DIFFERENTIAL=ON
cmake --build out/differential --target differential
./out/differential/differential
```

## Benchmarks

`maul-nav_bench` times a terrain of 8 by 8 tiles: bakes and cached rebuilds, streaming, paths, avoidance in crowds, flow fields, a hierarchy over a flat world, and flight volumes. It prints each section's counts, which do not depend on the machine, then its timings, the best of five runs. Timings are never a CI gate.

To measure a change, build the base and the change in Release with the same compiler, run each pinned to one core (`taskset -c 2 ./maul-nav_bench > base.txt`) on a quiet machine, and pass the base's output to the change's run to print the ratios. On a loaded machine, compare instruction counts (`perf stat -e instructions`) instead. Give both numbers in the pull request.
