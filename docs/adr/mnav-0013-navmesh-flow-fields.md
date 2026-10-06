# mnav-0013. Flow fields over the navmesh

Status: Accepted

## Context

The requirements ask for flow fields over the navmesh or a grid;
mnav-0007 gave grids theirs. Hosts whose units walk a navmesh want each
polygon's way to the goals from one search.

## Decision

- **Owner object:** `mnavNavFlow` holds memory for up to `polygons`
  polygons; it numbers a navmesh's polygons slot by slot at each build
  and sorts the off-mesh links by the polygon they land on.
- **Search:** Dijkstra's search backward from goal points on
  polygons, the path search's pricing reversed: a polygon stands at the
  midpoint of the portal it is left by toward the goals, a goal polygon
  at its goal point, and its cost is the next polygon's plus the walk
  between their places times the next polygon's area cost; an off-mesh
  link adds its cost and stands the polygon at the takeoff point. Inner
  edges, tile links and links of kinds the filter crosses are stepped
  over; ties go to the lower cost, then slot and polygon.
- **Reads:** `mnavNavFlowAt` gives a polygon's cost, next polygon, the
  portal's ends and the link taken; after a later commit the field is
  stale.

## Consequences

- An agent heads for its polygon's portal and the next polygon's after
  it; within a polygon the host steers, as with a corridor's corners.
- **Regions and steps:** `mnavBeginNavFlow` takes a box of tile places
  (all by default): polygons outside are as left out and count nothing
  toward the limit; `mnavContinueNavFlow` settles a budget of polygons,
  and the field is the same for any budgets. Reads wait for the end; a
  commit during the work makes it stale.
- Repairs after goals move are a later slice; a change to the navmesh
  calls for a new build.
