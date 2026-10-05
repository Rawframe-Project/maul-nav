# mnav-0011. Differential checks against Recast and Detour

Status: Accepted

## Context

The requirements name Recast as the quality bar and ask for path
lengths and reachability to agree with Recast and Detour within
tolerance on the scenario corpus, in a test-only harness never linked
into the library.

## Decision

- **Where:** `test/differential/`, built only with
  `MAUL_NAV_DIFFERENTIAL=ON`. CMake fetches recastnavigation at commit
  `9f4ce64` for the harness alone; a local copy stands in through
  `FETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION`. The scenes and Maul Nav's
  side are C; the C++ file holds Recast's and Detour's side.
- **Corpus:** the test world, the two-floor building, the terrain bowl
  and the plateaus with two bridges, each one triangle mesh.
- **Builds:** Maul Nav's default def in 32 m tiles; Recast's untiled
  solo build at the same settings in its units, with layer regions as
  Maul Nav partitions.
- **Comparison:** 200 pairs of random points per scene, snapped to each
  navmesh within 0.5 m across and 2 m up or down, then a path in each.
  Floors: reachability agreeing for 97% of pairs; straight path lengths
  within 5% or 0.25 m for 90%; Maul Nav's no longer than that beyond
  Detour's for 95%; and the scene's summed lengths within 2%.

## Consequences

- At `9f4ce64`, reachability agrees for every pair; lengths agree for
  94% to 100% of pairs, Maul Nav's are no longer for 95.5% to 100%, and
  the sums differ by at most 0.26%. The pairs that differ go both ways:
  where corridors through edge midpoints cost nearly the same, the two
  searches pick differently, and around the ramp's low end the two
  builds keep slightly different ground under it.
- CI runs the harness as its own job; the network fetch is the
  harness's only outside dependency.
