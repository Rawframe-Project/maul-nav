# mnav-0007. Flow fields

Status: Accepted

## Context

The requirements ask for flow fields toward a goal or a set of goals,
for unit counts where a path per agent does not scale. Supreme
Commander 2's design keeps a cost field, an integration field and a
flow field per sector, joined by portals, with a line-of-sight pass;
the grid path (mnav-0005) already has areas, filters and a step rule.

## Decision

- **Over the grid:** `mnavBuildFlowField` takes an `mnavGrid`, a
  filter and goal cells, and searches from every goal at once with
  Dijkstra's algorithm and the grid path's steps: the 8 neighbours,
  never across a blocked corner, a step costing its length times the
  mean of its two cells' area costs. The rule is symmetric, so a cell's
  cost from the goals is the cost of its way to them.
- **What a cell holds:** its cost to the nearest goal, infinite where
  none is reached, and the next cell on that way, the cell itself at a
  goal; `mnavFlowAt` reads them. Following the next cells walks a
  cheapest grid path, each step strictly downhill.
- **Order:** the open list is a heap by cost, then by cell index, so
  ties go the same way on every platform and in any goal order.
- **Memory:** `mnavFlowField` owns arrays for `cells` cells (20 bytes
  each), up to `MNAV_MAX_FLOW_CELLS`; a larger grid is refused with
  `mnav_errorLimit`, and a failed build leaves no grid to read.
- Blocked goals and goals in areas the filter leaves out are dropped.

## Consequences

- One build costs about one grid search over the reachable cells, and
  any number of agents then read their way in constant time.
- The field's ways are grid ways; agents smooth them by steering and
  avoidance. Sectors and portals, the line-of-sight pass, density and
  fields over the navmesh are later work, each on measured need.
