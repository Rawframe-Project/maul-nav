# Overview

Maul Nav bakes navmesh and flight tiles from host geometry, loads them as hostile bytes into containers that change only at commits, and answers path and spatial queries over them; avoidance, flow fields and steering sit beside the navmesh. It follows the family rules in [maul-conventions.md](maul-conventions.md); this page states what the library's domain adds and how the parts fit.

## Contents

- [Library profile](#library-profile)
- [How the parts fit](#how-the-parts-fit)
- [Defs, limits and versions](#defs-limits-and-versions)
- [Hostile input and invariants](#hostile-input-and-invariants)
- [Testing](#testing)

## Library profile

- **Determinism:** bit-exact on every supported platform, compiler, architecture and worker count. It covers the bytes of baked tiles, every query result (positions, polygon ids, costs, end reasons), corridors, flow fields and avoidance velocities. Navigation results become replicated, authoritative state in the programs that use them, so they must be the same everywhere; [determinism.md](determinism.md) explains how.
- **Threads:** none. Every function runs on the caller's thread. Tiles bake in parallel on the host's workers, one baker each, with the same bytes at any worker count. Queries are concurrent readers of a committed navmesh or flight volume; a stage or commit call is the single writer, and no query runs beside a commit.
- **Memory:** the owner objects are the baker, tile index, tile cache, navmesh, query context, hierarchy, grid flow field, navmesh flow field, avoidance set, flight baker and flight volume. Each takes its allocator in its def and its memory for its named limits. Queries never allocate: their scratch is the query context's. A bake allocates within its def's memory limit, and loaded data is never trusted for a size before it is checked.
- **Stale ids:** ids outlive their tiles by design, since corridors and hosts keep polygon ids across commits. A stale id is refused with `mnav_errorStale` and counts nothing; it is never dereferenced.
- **Misuse:** each owner object counts the calls refused as misuse, which its `mnav<Object>_GetMisuse` call reads at any time, so a host catches bugs in release builds. A call refused with `mnav_errorInvalid` counts one, on the first owner object it is given that is not NULL (the context for a search, the navmesh for a read with no context); a call that runs others inside, such as `mnavQuery_FindPath`, counts once, since the library calls its own workers, which count nothing. A call back in from host code refused with `mnav_errorBusy` counts one too. Nothing else counts: a stale id (`mnav_errorStale`), bad tile bytes (`mnav_errorFormat`, `mnav_errorVersion`), input geometry a bake refuses and its report names, a place outside the object (`mnav_errorOutside`), a value past a published bound (`mnav_errorLimit`), room too small and the wrong state are all things a correct program can meet. A call with no owner object, such as making one from a def, counts nowhere.
- **Outputs on failure:** every output says in its header what a failure leaves in it, by these rules (`src/outputs.h`). A call whose first parameter is an owner object clears its outputs on any failure, before it checks the owner, unless the failure itself defines the output: the size or count a buffer too small needed, with the first capacity items written; the links counted and kept when generation stops at a bound; a bake's report, which names the failure. A caller's array is an output like any other: it is zeroed over its capacity, a tile copy's bytes included. No output has a size only its owner knows: every array comes with its capacity. Such a call that answers "no" without failing clears its outputs the same way: a nearest polygon search that finds none gives slot 0 and a zeroed point, and a nearest open point not found within the radius is zeroed. One refusal leaves them: a call refused as busy, which only a call back in from host code the object runs can be (the navmesh's allocator and a link generation's clearance test, or the allocator of a context, hierarchy, flow field or set), writes none of its outputs, arrays and single values alike, since any of them may be one the call in progress has written or is filling. Any other failure of that call still clears them, and the id `mnavStageLink` gives, a creation's, still becomes the null id. A refused call never writes more than a success could: avoidance clears its velocities only when the set's agent limit holds the count, which bounds the caller's array. A creation sets the handle or id it gives to NULL or the null id on any failure, whether or not it takes an owner. Any other call without an owner (a def or input check, `mnavSteer`, `mnavResetCorridor`) leaves its outputs as they were. No output may overlap an input.
- **Host code:** the allocator, the clearance test that link generation takes, and the clock a bake reads for its report, each called only within the call that was given it, on the calling thread. While an owner object's allocator runs, while a baker's clock runs, and while a clearance test runs, the object, the baker, and the navmesh and generating context are marked busy (`src/host_code.h`), with no need to know the calling thread; an object's mark stays set from the release of its own block on. A call that would change a busy object is refused with `mnav_errorBusy`, one misuse counted (`mnavNavmesh_GetMisuse`, `mnavQuery_GetMisuse` and one getter per owner object, such as `mnavBaker_GetMisuse`), and changes nothing. For every owner object but the navmesh and the context, that is every call taking it that is not a read, its destroy call included; `mnavBaker_RebuildTile` counts as such a call of the tile cache it reads. While the navmesh's allocator runs, that is every stage call, commit and destroy; while a clearance test runs, every one of them but `mnavNavmesh_StageTile` and `mnavNavmesh_StageTileRemoval`, which write only the staged list that neither link generation nor any query reads, and those two stay allowed from the test itself and from any other thread. Every call that takes the context is refused while either runs on it. A clearance test may call whatever only reads the navmesh with a query context of its own (a ray, a nearest point, a height), `mnavNavmesh_StageTile` and `mnavNavmesh_StageTileRemoval`, and any call on other objects; it may not use the generating context or otherwise change the navmesh. A bake also holds the baker, the tile index it reads and the tile cache it fills or rebuilds from for the whole call, and a flight bake its index, so host code the bake runs (any of their allocators, the clock) changes none of them: their calls listed above are refused from there too. A hierarchy's build or update marks the hierarchy, and the navmesh it reads as a clearance test does, for the whole call, and the context it searches with while the hierarchy's allocator runs: the build searches through the public path calls, which refuse a marked context, and a context's allocator runs only when the context is made and destroyed. Beyond these refusals, host code may call no maul-nav function on an object the running call reads or writes, and may not read the object it serves: a baker's allocator, for example, must not drop the cached tile `mnavBaker_RebuildTile` is reading, which the hold refuses, and a navmesh's allocator must not search the navmesh it is committing, which nothing refuses.
- **Platform dependencies:** the C library only. `tools/libm-allowed.txt` lists the `<math.h>` functions the sources may call, all exactly rounded; anything else (cosines, arc tangents) is the library's own.

## How the parts fit

```text
host geometry ── bake_input, input (checks) ── tile_index (optional, shared)
       │
   raster ─ heightfield ─ filter ─ compact ─ volume, erode ─ region
       ─ contour ─ holes ─ polymesh ─ border_vertices ─ detail ─ tile (bytes)
       │                       │
   flight_baker ─ flight_tile  tile_cache ─ field_pack ─ lz
       │                       │
   flight_volume            navmesh ─ commit, modifiers, offmesh
       │                       │
   flight_search,           nearest, path, funnel, shortest, raycast, surface,
   flight_probe             corridor, spatial, grid, linkgen, hierarchy, navflow

   avoidance, avoidance3 (orca, orca3, obstacle, crowd)   flow, flow_repair   steer
   debug, flight_debug ── draw
```

`tools/modules.txt` declares which module may include which, and `tools/check_modules.py` enforces it. Avoidance, grid flow fields and steering reach no navmesh module: a host can use them alone. The parts and their reasoning:

- [bake.md](bake.md): the bake pipeline, the tile format and its fingerprint, the tile index and the tile cache.
- [navmesh.md](navmesh.md): staging and commits, ids, links across tiles, off-mesh links and tiers.
- [queries.md](queries.md): nearest points, the A* and shortest path searches, the funnel, rays, surface moves, corridors, walls and random points.
- [hierarchy.md](hierarchy.md): clusters, transitions and their updates.
- [flow-fields.md](flow-fields.md): grid fields with repairs, navmesh fields without.
- [avoidance.md](avoidance.md): ORCA on the ground and in space, neighbor order and the input ranges.
- [flight.md](flight.md): the voxel octree tiles and their searches.

## Defs, limits and versions

Every creation takes a def from its `mnavDefault...Def` function, which is `static inline`, so it is built in the program. Each def carries a cookie, checked first, so an uninitialized def is refused instead of read. Maul Nav has no single root object: a program may start with a baker, a navmesh, a query context, an avoidance set or a grid flow field. So every creation that takes no other library object, and `mnavSteer`, is a root: its def also carries `MNAV_ABI_VERSION`, the version of the headers the program was built with, checked second, before anything else is read. The query filter and the link generation def carry only a cookie, since every call that reads them also takes an object a root made.

A def's named limits count work, never time: nodes per search, polygons per tile, agents per step, bytes held. Each object takes the memory its limits need when it is made, so a call cannot fail for memory midway, and reaching a limit is `mnav_errorCapacity` with no work half done. A limit in seconds would make a bake's outcome depend on the machine.

## Hostile input and invariants

Input is hostile: geometry, tile bytes, ids and every argument. Each is validated and refused with a status in every build, and the result names what was refused (the setting, the input element, the tile section and element) where a caller needs it to find the fault. The tile and flight tile loaders check every count against the def's limits and the bytes present before allocating, the payload's hash, and every structural rule of the mesh, and are fuzzed.

The library writes two asset formats in the family's sense, the tile ([bake.md](bake.md#the-tile-format)) and the flight tile ([flight.md](flight.md#the-format-and-loading)), which a host bakes once and ships. It writes no state format: nothing a running program saves is written by the library, and the tile cache lives in memory only.

Internal invariants are asserted with `MNAV_ASSERT` (`src/invariant.h`): on in builds without `NDEBUG` (Debug, sanitizer and fuzz builds), where a broken invariant traps at the fault, and compiled out of Release, whose per-span and per-node loops must not pay for them.

## Testing

Tests pin promised outputs, not implementation: tile bytes of a few golden scenes by hash, stored tiles in the current formats (`test/stored_tiles.h`), the refusal of tiles in format 1 (`test/stored_format1.h`), and query and avoidance results by hash, compared on every CI cell. Fuzz targets cover the tile and flight tile loaders, bake input, queries, the searches over a whole navmesh, avoidance, grid flow fields, the exact shortest path against the A* search, flight queries and the tile cache's coder. A test-only harness compares paths and reachability with Recast and Detour (`test/differential/`). See [checks.md](../operations/checks.md).
