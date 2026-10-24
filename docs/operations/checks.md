# Checks

What verifies a change, locally and in CI, and the time each may take.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one before it. Its builds are the CI cells' presets from `CMakePresets.json`.

| Tier | Runs | Budget |
|---|---|---|
| `changed` | The source, module, length, API documentation and snippet rules, the organization's documentation check, `clang-format` on the changed C files, and the `linux-clang-20-asan` preset (assertions on) with the suites the changes reach: a suite's own file reaches its suite, library code reaches every suite. | under 2 minutes |
| `pr` | Every rule on every file, clang-tidy, the `linux-clang-20-asan` and `linux-gcc-14` presets (the Recast differential with the latter), and 10 seconds of fuzzing per target. | under 10 minutes |
| `full` | Also the other Linux presets (ThreadSanitizer, Clang Release with the package test, the shared library), the large scenes, and 60 seconds per fuzz target. | under 20 minutes |

Run `changed` before every push. On the shared development machine, run `pr` and `full` through the workspace's heavy-job wrapper. Builds go to `out/<preset>` and the other trees to `out/check-*`, each with its log beside it.

## In CI

`.github/workflows/ci.yml` runs on every pull request, on every push to `main` and every night. The Linux jobs run in the organization's CI image, pinned by digest, and install nothing. The ruleset requires only its `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs are skipped. |
| `docs` | The organization's documentation check: allowed files, the em dash, record numbers in docs and code, page sizes, links. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules, the module graph, lengths, API documentation, documentation snippets, that the documentation site builds, and clang-tidy. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 in Release building every part (tests, samples, benchmark, and the Recast differential, which it runs); Clang 20 in Release with the package test; Clang with AddressSanitizer and UBSan, each test repeated up to three times on a pull request so a flaky one fails before it merges; Clang with ThreadSanitizer; the shared library with the package test; GCC 14 on arm64; macOS and `clang-cl` on Windows, both with the package test. All build with warnings as errors and check the golden values; a skipped or missing test fails the cell. ccache is restored on every run and saved only from `main`. |
| `msvc-headers` | Every public header compiled by MSVC's `cl` as C17 and C++17 with `/W4 /WX`. |
| `web` | The library and its tests under Emscripten and Node, and the static library's size. The image has no Emscripten, so this job uses the runner and the emsdk action. |
| `fuzz` | Every fuzz target for 20 seconds from its committed seeds, four at a time (`tools/fuzz.sh run`). A finding uploads the logs and the input. |
| `fuzz-night` | At night, each target the build defines (`tools/fuzz.sh targets`, so a new target joins without a list to keep) alone for ten minutes, from the corpus the last night left in its Actions cache and the committed seeds together; the merged corpus is saved for the next night (`tools/fuzz.sh night`). A target without committed seeds fails, never starts empty. A manual run (`workflow_dispatch`) takes `fuzz-seconds` to run each target for less, to try the night's jobs. |
| `scenes` | At night, the large scenes the pull request check leaves out: the tests labeled `nightly` (the 1,000-agent doorway crowd), through the `nightly` test preset. |

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on every pull request and on `main`, building with GCC 14 (an advanced setup, since the default one builds with the runner's GCC 13). `pages.yml` publishes the documentation site with the API reference generated from the headers. `scorecard.yml` grades the repository's security practices weekly for the README badge, and Dependabot proposes updates of the pinned actions once a week in one pull request.

## Tests

Each suite in `test/` is one executable, registered with CTest under its name and labeled with it. The white-box suites include `src/` and are not built in shared builds. Golden values pin a few scenes: the bake goldens hash tile bytes, and every query golden runs on tiles read from `test/stored_tiles.h` (the test world, the terrain, the link generation trench), so a change to the bake leaves them alone. Suites that bake scenes of their own (the hierarchy's large world, the world in 8 m tiles, the building) check them exactly, not by hash. Large scenes carry the label `nightly`, and every test preset but `nightly` leaves them out.

## Budgets

Over a budget, tests are pruned or merged before anything else is added. The cells, the fuzz job and the checks run in parallel; the pull request check's wall time is its slowest job.
