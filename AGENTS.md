# Maul Nav

Maul Nav is a C library of game navigation: navmesh and flight volume baking from host geometry, a tile format loaded as hostile input, path and spatial queries, corridors, off-mesh links, flow fields, and ORCA avoidance as a component that needs no navmesh. It is part of the Maul family; the family rules are in [docs/internals/maul-conventions.md](docs/internals/maul-conventions.md), and [docs/internals/overview.md](docs/internals/overview.md) explains how the parts fit.

It never compromises on:

1. **Same bits everywhere.** Tile bytes, query results, corridors, flow fields and avoidance velocities are bit-identical on every platform, compiler, architecture and worker count.
2. **Hostile input.** Every argument, every input triangle and every byte of a tile is validated and refused with a status in every build.
3. **Bounded work.** Every search and bake is bounded by a named limit in its def; reaching one is a typed result, never a silent cut. Queries never allocate.
4. **The host owns the world.** No threads, no clock, no entities: the library returns paths and velocities, and the host applies them.

## Commands

```sh
tools/check.sh changed          # before every push: rules, format, ASan build, reached suites (< 2 min)
tools/check.sh pr               # what the pull request check runs (< 10 min)
tools/check.sh full             # adds TSan, Clang Release, shared, the large scenes, longer fuzzing (< 20 min)
cmake --preset linux-clang-20-asan && cmake --build --preset linux-clang-20-asan   # one CI cell
ctest --preset linux-clang-20-asan -L corridor   # one suite, by label
tools/fuzz.sh build out/fuzz && tools/fuzz.sh run out/fuzz 30 4   # every fuzz target, 30 s each
```

Run the `pr` and `full` tiers, fuzzing and the ThreadSanitizer build through the workspace's heavy-job wrapper on the shared machine. The server's default `gcc` is 13, which the family refuses: the presets name `gcc-14` and `clang-20`. Every CI cell is a preset in `CMakePresets.json` (configure, build and test of one name, built in `out/<preset>`); add a cell there, never as commands in a workflow. Each suite is `test/test_<label>.c`, labeled with its name; large scenes also carry `nightly`, which every test preset but `nightly` leaves out.

## Ways to hurt yourself

- **Killing by pattern.** Never `pkill -f`, `pgrep | kill` or `kill` a PID found by matching a name or path: other sessions build and fuzz on the same machine with the same binary names. Kill only a PID you started.
- **Endless fuzzing.** A libFuzzer target runs until it finds something. Always pass `-max_total_time`, and stop what you started.
- **Moving a golden value to make a test pass.** The hashes in the suites pin promised output. A changed hash is a change of results: find out why, and change the constant only when the change is meant, saying so in the pull request and the changelog.
- **Rebaking the stored tiles.** `test/stored_tiles.h` holds tiles in the current formats, and every query golden runs on its tiles (`LoadWorldTiles` in `test/world.h` for the test world) so that a bake change moves only the bake goldens. `test/stored_format1.h` holds tiles of format 1, which must be refused with `mnav_errorVersion`. Regenerate `stored_tiles.h` (`test_stored --write`) only in the pull request that raises a format; never change `stored_format1.h`.
- **Editing shared family files.** These are synced from the canonical copies in the organization's `.github` repository (`maul/`) and fail CI when they differ: `docs/internals/maul-conventions.md`; `.clang-format`, `.clang-tidy`, `.editorconfig`, `.gitattributes`, `.gitignore`; `cmake/CompilerFlags.cmake`, `cmake/Install.cmake`, `cmake/Library.cmake`, `cmake/ReadVersion.cmake`, `cmake/Testing.cmake`, `cmake/Config.cmake.in`, `cmake/pkg-config.pc.in`, `cmake/HeaderCheck/CMakeLists.txt`; `bench/CMakeLists.txt`; `test/test_harness.h`; `src/hash.c`; and `tools/build_docs.py`, `check_docs.py`, `check_guide.py`, `check_lengths.py`, `check_modules.py`, `check_source.py`, `gen_api.py`. Change the canonical copy first.

## Invariants

- **Determinism:** no fast math and no floating-point contraction (`cmake/CompilerFlags.cmake`); only the `<math.h>` functions in `tools/libm-allowed.txt`; bake stages after rasterization in integers; every tie broken by a stable key the function documents, never by order, address or hash iteration. [determinism.md](docs/internals/determinism.md) has the rules.
- **Modules:** `tools/modules.txt` lists which module may include which. Avoidance, grid flow fields and steering reach no navmesh module. A new include needs a line there.
- **Public headers** are the common subset of C17 and C++17 and declare only the API; every public function has a `///` block with `@param`, `@return` and a thread safety paragraph, which `tools/check_docs.py` checks.
- **Defs:** every def starts with its cookie, set by its `static inline` default and checked first. The defs of the roots (every creation that takes no other library object, and `mnavSteer`) carry `MNAV_ABI_VERSION` next, checked second, before anything else is read; the query filter and the link generation def carry only the cookie. Before 1.0 the stamp is the whole version.
- **Version:** `MNAV_VERSION_*` between the release-please markers in `include/maul-nav/base.h`; CMake reads it from there. Only release-please changes it.
- **Bake generations:** `MNAV_BAKE_GENERATOR` and `MNAV_FLIGHT_GENERATOR` rise in the pull request that changes the bytes a bake writes for the same def and input; moved golden tile hashes are the sign.
- **Formats:** a change to the layout of tile or flight tile bytes bumps `MNAV_TILE_FORMAT` or `MNAV_FLIGHT_FORMAT`. The library reads only the formats it writes: a tile of another format is `mnav_errorVersion`, and the host rebakes from source. Tiles are little-endian; nothing derivable is stored.
- **Limits are API.** Defaults and `MNAV_MAX_*` ranges are part of the contract; a limit is checked before work starts, and the memory it needs is taken when the object is made.
- **Assertions:** `MNAV_ASSERT` (`src/invariant.h`) checks internal invariants only. It traps in builds without `NDEBUG` (Debug, sanitizers, fuzzing) and compiles out of Release. Never assert on caller input or tile bytes; validate and return a status. Library code never calls `assert` or `abort`.
- **Committed data:** `test/corpus/fuzz_<area>.tar.gz` are minimized seed corpora, written by a fuzz target's merge step (`-merge=1`) and packed with `tar -czf`; `test/stored_tiles.h` is written by `test_stored --write` when a format rises, and `test/stored_format1.h` is never changed.

## Testing

Follow the [repository standard's Tests section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#tests); in this repository:

- Test through the public headers. A suite that reaches `src/` is `WHITEBOX` and needs a reason the public API cannot give, such as an internal format's exact bytes.
- Golden hashes pin a few scenes; a new behavior gets an exact check of its own, not another hash. A query golden runs on stored tiles (`LoadWorldTiles`, or a scene of `test/stored_tiles.h`), never on a fresh bake; a scene a suite bakes itself gets exact checks only.
- A large scene (thousands of agents, many tiles) is labeled `nightly` beside its suite's label and runs only at night.
- Inputs a reader could run past go in a heap allocation of exactly their length, so AddressSanitizer sees an overread.
- Every C snippet of `docs/user/` appears unchanged in `test/test_guide.c`; `tools/check_guide.py` enforces it.
- A fuzz finding becomes a test in its module's suite with the minimized input.
- Bake and query suites share the scenes in `test/world.h`, `test/soup.h`, `test/terrain.h`, `test/trench.h` and `test/bake_tile.h`; build on them before adding geometry.

## Documentation

Follow the [repository standard's Documentation section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#documentation). A change in what a caller must do or can rely on updates the page of `docs/user/` for that part and, if it is in a snippet, the matching code in `test/test_guide.c`. The API reference is generated from the headers into the documentation site; never commit it.

## Pull requests

Titles are conventional commits. Scopes: `bake`, `format`, `cache`, `navmesh`, `links`, `query`, `corridor`, `hierarchy`, `flow`, `avoidance`, `flight`, `debug`, `samples`, `build`, `bench`, `family`. Example: `fix(corridor): keep the target polygon when the target moves`.

A change of results (tile bytes, fingerprints, paths, velocities) says which and why in the pull request, since hosts store tiles and compare runs. A change meant to affect performance gives numbers before and after from the same machine.
