# Flow fields

When many agents share goals, one search serves them all: a flow field holds, for every cell or polygon, its cost to the nearest goal and the way to go next. Agents then read their way in constant time instead of searching each.

## Over a grid

- `mnavCreateGridFlow` with a def of the most cells it covers (65,536 by default), then `mnavGridFlow_Build` over an `mnavGrid` of area types, a [filter](paths.md#filters) and a set of goal cells.
- `mnavGridFlow_GetFlow` gives a cell's cost and its next cell; following next cells walks a cheapest grid path, each step strictly downhill. A cell no goal reaches has an infinite cost; a cell outside the field's region is `mnav_errorOutside`.
- Steps are the grid path's: eight neighbors, never across a blocked corner, each costing its length times the mean of its two cells' area costs. Blocked goals and goals in areas the filter leaves out are dropped.
- **In steps.** `mnavGridFlow_Begin` takes a region of the grid, an `mnavCellRange` from one corner cell to the other, both included (cells outside count as blocked), and `mnavGridFlow_Continue` does the work a budget of cells at a time. The field is the same for any budgets; read it when the work is done.
- **Repairs.** When goals move or cells change area, `mnavGridFlow_Update` takes the new goals and the changed cells and repairs only what they reach, in steps too. The repaired field is the one a rebuild gives, bit for bit.

## Over a navmesh

- `mnavCreateNavFlow` with a def of the most polygons, tile slots and off-mesh links it covers, then `mnavNavFlow_Build` toward goal points on polygons.
- `mnavNavFlow_GetFlow` gives a polygon's cost, its next polygon, and the portal to head for on the way out of it, or the off-mesh link to take. An agent heads for its polygon's portal and the next polygon's after it; within a polygon, the host steers. A polygon outside the field's region is `mnav_errorOutside`.
- **In steps.** `mnavNavFlow_Begin` takes a box of tile places (all by default), and `mnavNavFlow_Continue` settles a budget of polygons at a time, with the same field for any budgets.
- **Commits.** A commit to the navmesh leaves the field out of date, and reads refuse it (`mnav_errorState`). There are no repairs over a navmesh, since a polygon's way decides where in it the cost is measured; a build is cheap instead. A host whose goals move keeps two fields, building the next in steps while agents read the last.

Both kinds draw as arrows into a [debug buffer](debug-drawing.md).
