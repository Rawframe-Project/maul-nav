# Checks

What verifies a change, locally and in CI.

## Locally

Before pushing, build with warnings as errors under the sanitizers and run the suites the change reaches, and run the rule checks:

```sh
cmake -B out/asan -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL_NAV_SANITIZE=ON -DMAUL_NAV_WERROR=ON
cmake --build out/asan
ctest --test-dir out/asan -j 8
python3 tools/check_source.py && python3 tools/check_modules.py && python3 tools/check_docs.py
python3 tools/check_lengths.py && python3 tools/check_guide.py
```

On the shared development machine, run full builds and fuzzing through the workspace's heavy-job wrapper.

## In CI

`.github/workflows/ci.yml` runs on every push and pull request:

| Job | What it guards |
|---|---|
| `checks` | `clang-format`, the family's source, module, API documentation, snippet and length rules, and clang-tidy. |
| build cells | GCC 14 and Clang 20 in Release, Clang with AddressSanitizer and UBSan, Clang with ThreadSanitizer, the shared library with the package test, arm64, macOS, and `clang-cl` on Windows with the headers under MSVC's `cl`. |
| `web-emscripten` | The library and its tests under Emscripten and Node, and the static library's size. |
| `differential` | Paths and reachability against Recast and Detour. |
| `fuzz` | Every fuzz target for a minute from its seed corpus. |

## Tests

Each suite in `test/` is one executable, registered with CTest under its name and labeled with it. The white-box suites include `src/` and are not built in shared builds. `test_guide.c` runs the C snippets of `docs/user/` as written.
