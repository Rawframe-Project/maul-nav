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

- **Polygons in a box:** `mnavFindPolygons` lists the included
  polygons whose bounds meet a box, by tile in place order then by
  index; a buffer too small holds the first ones, the count covers
  all, and the result is `mnav_errorCapacity`.
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
  end runs to the node nearest it. The corridor lists each visit to a
  polygon once: a polygon an off-mesh link leaves and lands back on is
  listed before the link and after it, so that a corridor set from the
  path still crosses the link (in 0.2.0 such visits merged, which a
  fuzz target found). A stale polygon id is `mnav_errorStale`.
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

- **Paths across off-mesh links:** the filter's kind mask says which
  links an agent may cross. The search crosses an attached link from
  its takeoff polygon at its declared cost when its kind and its
  landing polygon's area are included; the heuristic's scale drops to
  the lowest cost per meter of any included kind's attached links, so
  cheap teleports keep the search exact. The funnel runs stretch by
  stretch between links, and the path lists each link crossed with its
  id, its kind and the index of its takeoff point.

- **Sliced searches:** `mnavBeginPath`, `mnavContinuePath` (up to a
  budget of nodes per call) and `mnavFinishPath` run the path search
  across calls; `mnavFindPath` is the three at once, and any slicing
  gives its result bit for bit. The context keeps its own copy of the
  filter. The navmesh counts its commits, and a search on a navmesh
  committed to since it began is `mnav_errorStale`, so no search mixes
  navmesh versions. Finished early, a search gives the corridor toward
  the node nearest the end, ended `mnav_pathUnfinished`.

- **Moving along the surface:** `mnavMoveAlongSurface` walks
  breadth-first, in edge order, from a point on a polygon over the
  included polygons whose shared edges meet the circle round the move,
  widened by one cell so that a start on a vertex keeps its
  neighbours.
  It reaches the wanted point in the polygon holding it, or ends at the
  nearest point on the walls met, ties to the wall met first. Walls are
  every edge, and every part of a tile-side edge, with no included
  polygon across, so gaps between linked polygons are walls. It ends
  reached, at a wall, at a side with no tile loaded, or out of nodes,
  with the point on the detail surface and the polygons walked.

- **Path corridors:** `mnavCorridor` is a plain struct over a polygon
  buffer the caller owns, with the agent's position and its target:
  `mnavResetCorridor` starts one, `mnavSetCorridor` loads a path (one
  longer than the buffer is refused, never cut), `mnavCorridorCorners`
  pulls it tight with the funnel over the portals between its polygons
  (edges, tile links, off-mesh links), and `mnavCheckCorridor` counts
  the leading polygons still current, included and joined, where the
  caller trims and replans. `mnavMoveCorridor` and
  `mnavMoveCorridorTarget` move its ends along the surface and merge
  the polygons walked into its start or end; a merge that would
  outgrow the buffer is refused and changes nothing.
  `mnavShortcutCorridor` casts a ray toward a point the caller picks
  and, when it reaches it, puts the polygons it crossed in place of
  the corridor's start if fewer; `mnavReplanCorridor` searches again
  between the corridor's ends, finding their polygons again within a
  box when their own have gone.

- **Grid paths:** `mnavFindGridPath` searches a caller's grid of cell
  areas in a query context: eight neighbours, no blocked corner cut, a
  step costing its length times the mean of its cells' area costs.
  Jump point search when every included area costs the same, A*
  otherwise; the context's node limit and path length, the navmesh
  search's ends, closed nodes final, and the turning cells as the
  path.

- **Heights:** `mnavGetHeight` reads a polygon's detail surface at a
  ground point inside its footprint, `mnav_errorRange` outside it.
- **Walls:** `mnavFindWallDistance` searches from the center's polygon
  through the edges within a radius, as Detour's findDistanceToWall
  does, and returns the nearest wall point, its ground distance and the
  unit direction from it to the center; an edge with nothing the filter
  includes across it is a wall, an unloaded tile side included; off-mesh
  links are left aside; the node limit's cut is reported.
- **Random points:** `mnavFindRandomPoint` picks uniformly over the
  ground area of every included polygon from a 64-bit seed through
  SplitMix64, where Detour picks a tile first;
  `mnavFindRandomPointAround` samples the polygons the walls search reaches by the area each shares
  with the circle, an inscribed 32-gon written out as constants, so the
  point lies in the circle, where Detour's may lie a polygon away.
- **Reachability:** `mnavCheckReachable` runs the path search without
  string pulling and returns its end: found, none, or the limit or
  unloaded tile that kept it from telling.

## Consequences

- Queries scan the polygons of each loaded tile in their box; a search
  tree per tile is added when measurements call for it, without
  changing any answer.
