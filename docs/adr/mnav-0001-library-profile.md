# mnav-0001. Library profile

Status: Accepted

## Context

Every Maul library states in one record what its domain adds to the
family rulebook (family record 0005). Navigation results become
replicated, authoritative state in the programs that use them, so they
must be the same everywhere.

## Decision

- **Determinism:** bit-exact on every supported platform, compiler,
  architecture and worker count. It covers the bytes of generated
  navmesh data, every query result (positions, polygon references,
  costs, end reasons), path corridors, flow fields and avoidance
  velocities. The rules that follow from it:
  - geometry computes in binary32 within a tile's own frame, every
    stage after rasterization in integers, and world positions cross
    the API as doubles;
  - only the operations IEEE 754 rounds exactly and `sqrt` come from
    the platform; anything else (cosines, arc tangents) is the
    library's own; `tools/libm-allowed.txt` lists the `<math.h>`
    functions the sources may call, and the source check refuses any
    other (amended 2026-10-10, after a `hypot` in steering went
    unnoticed from 0.10.0 to 0.11.1);
  - no result depends on registration or insertion order, hash-table
    iteration, pointer values, thread count or timing; every tie is
    broken by a stable key (a polygon reference, then tile
    coordinates, then a rule each function documents);
  - randomized queries take their random state from the caller, and
    the library has none of its own;
  - a clock is read only for bake reports, which never feed output;
  - the tests print determinism hashes of navmesh bytes and query
    results, compared across every CI cell and against committed
    values. A change that moves one says why, in its commit message
    and in `CHANGELOG.md`.
- **Threads:** none of its own (family record 0017). Tiles build in
  parallel through the host's task hooks, and the result is the same
  at any worker count. Queries are concurrent readers of a navmesh;
  committing a change is the single writer, and no query may run
  during it.
- **Memory:** the owner objects are the navmesh, the query context
  (one per querying thread), the avoidance set and the flow field,
  each created with the caller's allocator. Queries are allocation
  free: their scratch memory is the query context's, sized by named
  limits when it is created. The bake allocates through its allocator
  within a named memory limit, and loaded data is never trusted for a
  size before it is checked.
- **Platform dependencies:** the C library only, with `sqrt` from
  libm.
- **Versions:** every def and the query filter carry the version of
  the headers the program was built with, `MNAV_ABI_VERSION` (major and
  minor), stamped by defaults that are `static inline` in the headers.
  maul-nav has no root object: each creation, and `mnavSteer`, may be
  the first call a program makes, so every function that takes a def
  or a filter refuses another version with `mnav_errorVersion` before
  it reads anything else of it (conventions section 12; amended
  2026-10-10).
- **Commit areas:** `api`, `avoidance`, `bake`, `bench`, `build`,
  `ci`, `corridor`, `docs`, `field`, `format`, `grid`, `query`,
  `samples`, `tests`, `tools`.

## Consequences

A host gets the same navmesh from the same input on every machine, the
same path from the same query, and runs queries from as many threads
as it likes between commits. The library pays for this with its own
transcendental functions and stable tie-breaking in every search.
