# Flow fields

A flow field answers every agent's way to a set of goals from one search, for unit counts where a path per agent does not scale. There are two: over a host's grid (`src/flow.c`, `src/flow_repair.c`) and over the navmesh (`src/navflow.c`).

## Grid fields

- **The search.** `mnavGridFlow_Build` searches from every goal at once with Dijkstra's algorithm and the grid path's steps: eight neighbors, never across a blocked corner, a step costing its length times the mean of its two cells' area costs. The rule is symmetric, so a cell's cost from the goals is the cost of its way to them.
- **What a cell holds.** Its cost to the nearest goal, infinite where none is reached, and its next cell, itself at a goal. Following next cells walks a cheapest grid path, each step strictly downhill.
- **Order.** The open list is a heap by cost, then by cell index, so ties go the same way on every platform and in any goal order.
- **Memory.** A field owns arrays for its `cells` limit (cost, next cell, heap place and entry, a goal list, a repair's touched list and marks, 29 bytes a cell), up to `MNAV_MAX_GRID_FLOW_CELLS`. A region of more cells is refused, and a failed begin leaves nothing to read.
- **Regions and budgets.** A field covers a region of its grid, cells outside counting as blocked; `mnavGridFlow_Continue` settles a budget of cells per call, with the same field for any budgets.

### Repairs

`mnavGridFlow_Update` takes the new goals and the cells whose areas changed. Cells whose next-cell chains ran through a removed goal, a changed cell or a changed corner are reset and searched again; lowered costs spread from new goals and changed cells; and every touched cell and its neighbors take the next cell a build would choose (the neighbor giving the cost exactly, lowest by cost then index). Costs are the fixed point of the neighbors' offers, so the repaired field is the rebuilt one, bit for bit. A test compares them after random changes.

## Navmesh fields

- **The search.** Dijkstra's search backward from goal points, the path search's pricing reversed: a polygon stands at the midpoint of the portal it is left by toward the goals, a goal polygon at its goal point, and its cost is the next polygon's plus the walk between their places times the next polygon's area cost. An off-mesh link adds its cost and stands the polygon at the takeoff point. Ties go to the lower cost, then slot and polygon.
- **Memory.** A field takes its memory when it is made, for polygons, tile slots and off-mesh links (two ends each); a build allocates nothing and refuses a navmesh past a limit. It numbers the polygons slot by slot at each build and sorts the links by the polygon they land on.
- **Regions and steps.** `mnavNavFlow_Begin` takes a box of tile places: polygons outside are left out and count nothing toward the limit. `mnavNavFlow_Continue` settles a budget of polygons, with the same field for any budgets. A commit during the work, or after it, makes the field stale.

### Why no repairs

A polygon's place depends on the way it takes (the midpoint of its own portal), so the cost of a step into it depends on its choice, not on the graph alone. Repairs in the manner of Ramalingam and Reps, which the grid fields use, need fixed step costs; here a repair could not promise the build's field. A build is cheap instead: the whole 768 m flat world of the benchmark in about 14 ms, a region less, and in steps of any size. A host whose goals move keeps two fields, building the next in steps while agents read the last.
