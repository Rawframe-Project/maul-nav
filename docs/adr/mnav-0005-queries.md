# mnav-0005. Queries

Status: Accepted

## Context

Queries read a committed navmesh (mnav-0004) from any number of
threads. Their answers must be the same on every platform and must not
depend on how the navmesh stores its tiles, and an answer near tiles
that are not loaded must say so rather than treat them as walls.

## Decision

- **Read-only, in world coordinates:** a query takes the navmesh as
  const, positions as binary64 meters (right-handed, +Y up), and
  extents as binary32 meters; inputs that are not finite and negative
  extents are `mnav_errorInvalid`. Internally each tile's integer data
  is read in its own frame in binary64, with no contraction and no
  library transcendentals, so results are bit-identical everywhere.
- **Ties are part of the contract:** every query states how it breaks
  ties, by measured values first and then by tile place and polygon
  index, never by storage order.
- **Unloaded places are reported:** a result that may have been
  different with more tiles loaded says so.
- **Nearest point:** `mnavFindNearest` returns the polygon whose nearest
  point lies in a box round the query point and scores best: over a
  polygon, the height beyond the agent's step; otherwise the distance.
  Ties go to the shorter distance, then the tile first by place, then
  the lower polygon index. The result tells whether the point lies over
  the polygon and whether part of the box is not loaded.

- **Query contexts:** searches run in a caller-owned `mnavQuery` whose
  memory is sized by its named limits when it is made: nodes per search
  and the longest path in meters. Searches never allocate; one thread
  uses a context at a time, and many contexts read one navmesh at once.
- **Path search:** `mnavFindPath` runs A* from a point on one polygon to
  a point on another. Its nodes are the portals between polygons, at
  their midpoints, and the two points; costs and the heuristic are 3D
  distances, so the heuristic is consistent and no node is reopened.
  Ties go to the node made first. The
  result names how the search ended (found, out of nodes, too long,
  stopped at places with no tile loaded, no path, checked in that
  order), its cost, and the corridor of polygons, which short of the
  end runs to the node nearest it. A stale polygon id is
  `mnav_errorStale`.
- **Straight path:** the corridor is pulled tight with the funnel
  algorithm over its portals, in binary64 world coordinates with exact
  side tests and no tolerance: a portal end on a funnel side narrows
  it, one on or past the other side makes that side's point a corner.
  The points live in the context, sized from the node limit, so a
  straight path is never cut short; short of the end it ends at the
  midpoint of the last portal crossed.
- **Raycast:** `mnavRaycast` walks a segment on the ground from a point
  on a polygon, polygon to polygon through the first edge it crosses
  outward, across tile sides where a link's exact cell range holds the
  exit point; at a corner it goes on through an edge that leads on,
  the lowest-numbered first. It ends reached, at a wall (with the
  wall's normal), at a side with no tile loaded, or out of nodes when
  the polygons crossed reach the node limit, and reports the fraction
  travelled and the polygons crossed.

- **Filters:** every query takes an `mnavQueryFilter` (NULL for the
  default): a cost per area type, from 0.001 to 1,000,000, and a mask of
  the area types included. A search step costs its length times the
  cost of the polygon it lies in, and the heuristic is the distance
  times the cheapest included cost, so it stays consistent at any
  costs. Excluded polygons are not entered, are walls to rays, and are
  skipped by the nearest point; the start polygon is always usable. The
  path length limit counts meters, and a path reports its cost and its
  length apart.

## Consequences

- Queries scan the polygons of each loaded tile in their box; a search
  tree per tile is added when measurements call for it, without
  changing any answer.
