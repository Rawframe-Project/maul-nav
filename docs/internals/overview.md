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
- **Memory:** the owner objects are the baker, tile index, tile cache, navmesh, query context, hierarchy, flow field, navmesh flow field, avoidance set, flight baker and flight volume. Each takes its allocator in its def and its memory for its named limits. Queries never allocate: their scratch is the query context's. A bake allocates within its def's memory limit, and loaded data is never trusted for a size before it is checked.
- **Stale ids:** ids outlive their tiles by design, since corridors and hosts keep polygon ids across commits. A stale id is refused with `mnav_errorStale` and counts nothing; it is never dereferenced.
- **Host code:** the allocator, the clearance test that link generation takes, and the clock a bake reads for its report, each called only within the call that was given it, on the calling thread.
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

Every creation takes a def from its `mnavDefault...Def` function, which is `static inline`, so it is built in the program. Each def carries a cookie, checked first, so an uninitialized def is refused instead of read. Maul Nav has no single root object: a program may start with a baker, a navmesh, a query context, an avoidance set or a flow field. So every creation that takes no other library object, and `mnavSteer`, is a root: its def also carries `MNAV_ABI_VERSION`, the version of the headers the program was built with, checked second, before anything else is read. The query filter and the link generation def carry only a cookie, since every call that reads them also takes an object a root made.

A def's named limits count work, never time: nodes per search, polygons per tile, agents per step, bytes held. Each object takes the memory its limits need when it is made, so a call cannot fail for memory midway, and reaching a limit is `mnav_errorLimit` with no work half done. A limit in seconds would make a bake's outcome depend on the machine.

## Hostile input and invariants

Input is hostile: geometry, tile bytes, ids and every argument. Each is validated and refused with a status in every build, and the result names what was refused (the setting, the input element, the tile section and element) where a caller needs it to find the fault. The tile and flight tile loaders check every count against the def's limits and the bytes present before allocating, the payload's hash, and every structural rule of the mesh, and are fuzzed.

Internal invariants are asserted with `MNAV_ASSERT` (`src/invariant.h`): on in builds without `NDEBUG` (Debug, sanitizer and fuzz builds), where a broken invariant traps at the fault, and compiled out of Release, whose per-span and per-node loops must not pay for them.

## Testing

Tests pin promised outputs, not implementation: tile bytes of a few golden scenes by hash, the bytes of tiles an earlier format version wrote (`test/stored_1_0_0.h`), and query and avoidance results by hash, compared on every CI cell. Fuzz targets cover the tile and flight tile loaders, bake input, queries, the searches over a whole navmesh, avoidance, grid flow fields, the exact shortest path against the A* search, flight queries and the tile cache's coder. A test-only harness compares paths and reachability with Recast and Detour (`test/differential/`). See [checks.md](../operations/checks.md).
