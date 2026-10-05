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
  paths with and without it.
- **Baseline:** `bench/baseline.txt` holds those lines as the benchmark
  wrote them, pinned to one core, with the machine, compiler and date
  in its comments. Given the file, the benchmark prints each result's
  ratio to the recorded one, as Maul RHI's does.
- **No gate:** wall time varies with the machine and its load, so the
  ratios are read, not checked; a change that moves them says why in
  its commit.

## Consequences

- The first baseline was recorded with other work on the machine, and
  its comments say so; it is to be recorded again on a quiet machine.
