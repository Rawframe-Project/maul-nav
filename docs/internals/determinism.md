# Determinism

Every output of Maul Nav is the same bits on every supported platform, compiler, architecture and worker count: tile bytes, query results, corridors, flow fields and avoidance velocities. Hosts replicate navigation as authoritative state (a server's path is the client's path), cache tiles by fingerprint, and compare runs across machines, so a difference in one bit is a bug.

## Where the numbers live

- **World positions are doubles** (`mnavPos3`) across the API, so precision does not depend on the distance from the world origin.
- **Bake geometry is binary32 in the tile's own frame**, relative to the def's origin, through rasterization. Every stage after it (spans, regions, contours, polygons, detail) is integer arithmetic: cells, cell heights and sixteenths of a cell. An integer stage cannot round differently on another machine.
- **Queries read each tile's integers in its own frame, in binary64.** Results do not depend on how far a tile lies from the origin, nor on where in memory a tile is stored.
- **Avoidance is binary64 throughout**; lines and planes count as parallel within 1e-9.

## What the compiler may not do

`cmake/CompilerFlags.cmake` turns floating-point contraction off and refuses fast math in any build type: a fused multiply-add changes the last bit of a result, and GCC and Clang fuse differently. Only the operations IEEE 754 rounds exactly come from the platform: `+`, `-`, `*`, `/`, `sqrt` and the functions in `tools/libm-allowed.txt` (rounding, absolute value, minimum and maximum). `tools/check_source.py` refuses a call to any other `<math.h>` function. Cosines (the bake's slope limit), and anything else the library needs, are its own code.

## The floating-point environment

A host may run its threads with flush-to-zero, denormals-are-zero or another rounding mode, and those change results: a bake rounds its vertices differently, and avoidance between agents whose squared distance is subnormal turns the wrong way. So every public function that computes floats runs in one environment of its own: round to nearest, denormals kept, every exception masked. It enters that environment through the family's module (`src/fpenv.c`, shared from the organization's `.github` repository) and leaves the host's as it found it, sticky exception flags included.

- **How a function enters it.** The public function does nothing but enter, call a worker marked `MNAV_NOINLINE` and leave, so the compiler cannot move arithmetic across the control register's writes. Entries nest: a public function another one calls only counts.
- **Host code.** The allocator and a bake's clock are host code: they run in the host's own environment, each call wrapped by `mnavFloatHostEnter` and `mnavFloatHostLeave` in a helper of its own. The link generator's clearance test decides which links are kept, so it runs in the library's environment.
- **Threads.** maul-nav has no task hook. A host's threads call the public functions themselves, each with its own baker or query context, and each call enters the environment on its thread.
- **The list.** `tools/check_float_env.py` checks that every public function enters the environment, unless `tools/float-env-allowed.txt` names it with a reason, such as computing no floats at all.

The golden suites (`test_tile`, `test_workers`, `test_bake2d`, `test_volume`, `test_scenarios`, `test_flight_volume`, `test_stored` and `test_avoidance`) also compute their pinned values under every rounding mode, with flush-to-zero and denormals-are-zero off and on where the platform has them (`test/float_hosts.h`). They check that each call gives the host's setting back. `test_workers` runs each worker thread under another setting.

## Order

No result depends on insertion order, hash-table iteration, pointer values, thread count or timing.

- Every tie is broken by a stable key, which each function documents: the measured value first, then the tile's place, then the polygon index, then a host-given id.
- Within a commit, staged tiles take slots in order of place, so the order in which they were staged does not matter.
- Avoidance sorts neighbors by distance, then the host's id, so the same agents in any order get the same velocities. Agents and obstacles with equal ids fall back to input order, which is why ids should differ.
- Searches order their open lists by cost, then by the order nodes were made, never by address.

## Seeds and clocks

Random queries take their state from the caller (a 64-bit seed through SplitMix64); the library keeps none. The only clock is the one a host may pass to a bake for its report; it never reaches the tile or its fingerprint.

## How it is checked

`mnavHash64` is a frozen 64-bit hash over bytes, read eight at a time in the host's byte order. The tests hash tile bytes, paths, flow fields and avoidance velocities of a few golden scenes and compare them with committed values on every CI cell, from x86-64 and arm64 Linux through macOS, Windows and WebAssembly. A change that moves a golden value is a change of results: its pull request says why, and the changelog says who sees it, since hosts store tiles and fingerprints.
