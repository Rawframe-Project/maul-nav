# Checks

What verifies a change, locally and in CI, and the time each may take.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one before it. Its builds are the CI cells' presets from `CMakePresets.json`.

| Tier | Runs | Budget |
|---|---|---|
| `changed` | The source, module, length, API documentation and snippet rules, test hygiene, the organization's documentation check, `clang-format` on the changed C files, and the `linux-clang-20-asan` preset (assertions on) with the suites the changes reach: a suite's own file reaches its suite, library code reaches every suite. | under 2 minutes |
| `pr` | Every rule on every file, clang-tidy, the `linux-clang-20-asan` and `linux-gcc-14` presets (the Recast differential with the latter), the hand tiles' check, and 10 seconds of fuzzing per target. | under 10 minutes |
| `full` | Also the other Linux presets (ThreadSanitizer, Clang Release with the package test, the shared library), the large scenes, the dense forms under AddressSanitizer, and 60 seconds per fuzz target. | under 20 minutes |

Run `changed` before every push. On the shared development machine, run `pr` and `full` through the workspace's heavy-job wrapper. Builds go to `out/<preset>` and the other trees to `out/check-*`, each with its log beside it. After each preset's configure, the test hygiene check reads its CTest registrations.

## In CI

`.github/workflows/ci.yml` runs on every pull request, on every push to `main` and every night. The Linux jobs run in the organization's CI image, pinned by digest, and install nothing. The ruleset requires only its `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs are skipped. |
| `docs` | The organization's documentation check, run from the commit of `.github` the workflow pins: allowed files, the em dash, record numbers in docs and code, page sizes, links. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules, the module graph, lengths, API documentation, documentation snippets, test hygiene (no sleep, wall clock, retry or unexplained skip in a test, and every registered test labeled, in the default and the shared configurations), that the documentation site builds, that `test/hand_tiles.h` is what `test_hand_tiles` writes, and clang-tidy. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 in Release building every part (tests, samples, benchmark, and the Recast differential, which it runs and whose log also reports costs against Detour, read as [development.md](development.md#the-differential-harness) explains); Clang 20 in Release with the package test; Clang with AddressSanitizer and UBSan, each test repeated up to three times on a pull request so a flaky one fails before it merges; Clang with ThreadSanitizer; the shared library with the package test; GCC 14 on arm64; macOS and `clang-cl` on Windows, both with the package test. All build with warnings as errors and check the golden values; a skipped or missing test fails the cell. ccache is restored on every run and saved only from `main`. |
| `msvc-headers` | Every public header compiled by MSVC's `cl` as C17 and C++17 with `/W4 /WX`. |
| `web` | The library and its tests under Emscripten and Node, and the static library's size. The image has no Emscripten, so this job uses the runner and the emsdk action. |
| `fuzz` | Every fuzz target for 20 seconds from its committed seeds (`tools/fuzz.sh seeds`), four at a time, each through `tools/fuzz_target.sh`. A finding prints its report (the target, its findings by kind and count, and the sanitizers' report, never the input, its file name or the run's seed) and fails the job. Nothing is uploaded: the log, which starts with the seed, and the input stay on the runner. |
| `fuzz-night` | At night, each target the build defines (`tools/fuzz.sh targets`, so a new target joins without a list to keep) alone for ten minutes, from the corpus the last night left in its Actions cache and the committed seeds together; the merged corpus is saved for the next night when the target found nothing and no input crashed the merge (`tools/fuzz.sh night`, through `tools/fuzz_target.sh` and its `--merge`). No pull request run restores the cache, and no corpus, log or finding is uploaded. A target without committed seeds fails, never starts empty. A manual run (`workflow_dispatch`) takes `fuzz-seconds` (a whole number from 1 to 1500, so that the merge and the corpus's save fit in the job's 30 minutes) to run each target for less, to try the night's jobs. |
| `scenes` | At night, what the pull request check leaves out: the tests labeled `nightly` (the 1,000-agent doorway crowd), through the `nightly` test preset, and the suites' dense forms under AddressSanitizer, through `dense-asan`. |

Only the `build` cells cache with ccache. The other jobs compile too little for a cache to pay for its restore and save steps; on the hosted runners their builds take 5 s (`fuzz`, every target with the sanitizers), 14 s (`scenes`, at night), 18 s (`msvc-headers`) and 57 s (`web`, under Emscripten, which runs beside the AddressSanitizer cell, the pull request check's slowest job at about 4.5 minutes). The `checks` job spends its time in clang-tidy's analysis, which ccache does not keep. CodeQL (a 16 s build in the image) builds without a cache on purpose: a cache hit skips the compile that CodeQL traces, and the analysis would miss that file.

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on every pull request and on `main`, building with GCC 14 in the CI image (an advanced setup, since the default one builds with the runner's GCC 13). `pages.yml` publishes the documentation site with the API reference generated from the headers. `scorecard.yml` grades the repository's security practices weekly for the README badge, and Dependabot proposes updates of the pinned actions once a week in one pull request.

## Tests

Each suite in `test/` is one executable, registered with CTest under its name and labeled with it; the samples run as tests labeled `samples`. The white-box suites include `src/` and are not built in shared builds. Golden values pin a few scenes: the bake goldens hash tile bytes, and every query golden runs on tiles read from `test/stored_tiles.h` (the test world, the terrain, the link generation trench, the doorway crowd's rooms), so a change to the bake leaves them alone. Suites that bake scenes of their own (the hierarchy's large world, the world in 8 m tiles, the building) check them exactly, not by hash. Large scenes carry the label `nightly`, and every test preset but `nightly` leaves them out. A suite whose sweeps or trials are too slow for a unit test (`alloc_failures`, `flight_query`, `hierarchy`, `scenarios`, `tile_cache`) runs less of them, a subset (or, for `alloc_failures`, a coarser bake of the same scene that reaches the same allocation stacks), and with the argument `dense` all of them, as the test `<suite>_dense`, labeled `dense` (`test/dense.h`). Every cell runs the dense tests on a pull request but the AddressSanitizer one, the pull request check's slowest job, which leaves them to `dense-asan` at night.

## Budgets

Over a budget, tests are pruned or merged before anything else is added. The cells, the fuzz job and the checks run in parallel; the pull request check's wall time is its slowest job.
