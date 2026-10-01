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
- **Memory:** `mnavFlowField` owns arrays for `cells` cells (29 bytes
  each: cost, next cell, heap place and heap entry, a goal list, a
  repair's touched list and marks), up to `MNAV_MAX_FLOW_CELLS`; a
  larger region is refused with `mnav_errorLimit`, and a failed begin
  leaves nothing to read.
- Blocked goals and goals in areas the filter leaves out are dropped.

- **Regions and budgets:** a field covers a region of its grid, the
  cells outside as blocked, checked against the cell limit;
  `mnavBeginFlowField` and `mnavContinueFlowField` do the work in steps
  of a cell budget, with the same result for any budgets.
- **Repairs:** `mnavUpdateFlowField` takes the new goals and the cells
  whose areas changed; the cells whose next-cell chains ran through a
  removed goal, a changed cell or a changed corner are reset and
  searched again, lowered costs spread from new goals and changed
  cells, and every touched cell and its neighbours take the next cell a
  build chooses (the neighbour giving the cost exactly, lowest by cost
  then index). Costs are the fixed point of the neighbours' offers, so
  the repaired field is the rebuilt one, bit for bit.

## Consequences

- One build costs about one grid search over the reachable cells, and
  any number of agents then read their way in constant time.
- The field's ways are grid ways; agents smooth them by steering and
  avoidance. Sectors and portals, the line-of-sight pass, density and
  fields over the navmesh are later work, each on measured need.
