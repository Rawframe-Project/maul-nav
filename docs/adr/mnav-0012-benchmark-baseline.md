# mnav-0012. Benchmarks against a recorded baseline

Status: Accepted

## Context

The requirements ask for benchmarks against a recorded baseline: bake
time per tile, path queries per second, sliced query throughput,
avoidance agents per millisecond and flow-field update cost. The
benchmark printed its timings only.

## Decision

- **Named results:** `maul-nav_bench` prints each section's counts and
  bytes as comment lines, then named results, the best of five runs:
  a tile's bake, a streaming step with and without links, a path query
  and queries per second, a sliced path query in slices of 64 nodes and
  one slice, an avoidance step of 1,000 agents and agents per
  millisecond, flow builds with one and four goals, flow repairs of a
  cell, a moved goal and a wall's gap, a hierarchy's build, and long
  paths with and without it. Later sections add their own lines (amended
  2026-10-09): flight volumes' bakes, searches and rays, a tile rebuilt
  from the tile cache with a crate on it, and a crowd step of 1,000
  agents on the terrain's navmesh (corners, steering, avoidance and
  surface moves, each part's share printed).
- **Baseline:** `bench/baseline.txt` holds those lines as the benchmark
  wrote them, pinned to one core, with the machine, compiler and date
  in its comments. Given the file, the benchmark prints each result's
  ratio to the recorded one, as Maul RHI's does.
- **No gate:** wall time varies with the machine and its load, so the
  ratios are read, not checked; a change that moves them says why in
  its commit.

## Consequences

- The first baseline was recorded with other work on the machine; it
  was recorded again on a quiet machine (load average about 4), the
  better of two runs, with the navmesh flow fields (mnav-0013) added.
