# Queries

How the searches and spatial queries over a committed navmesh work, and the choices that make their answers the same everywhere. A query takes the navmesh as const and its scratch from an `mnavQuery` context sized by its limits when it is made; no query allocates.

## Contents

- [Common rules](#common-rules)
- [Nearest point and boxes](#nearest-point-and-boxes)
- [The A* search](#the-a-search)
- [The straight path](#the-straight-path)
- [The shortest path](#the-shortest-path)
- [Rays and surface moves](#rays-and-surface-moves)
- [Corridors and steering](#corridors-and-steering)
- [Walls and random points](#walls-and-random-points)
- [Grids](#grids)

## Common rules

- **World in, tiles inside.** Positions arrive as binary64 world meters and extents as binary32; inputs that are not finite and negative extents are refused. Each tile's integer data is read in its own frame in binary64, so an answer does not depend on how far a tile lies from the origin or how tiles are stored.
- **Ties are part of the contract.** Every query breaks ties by measured values first, then by tile place and polygon index, never by storage order. Each header states its rule.
- **Unloaded places are reported.** A result that could differ with more tiles loaded says so.
- **Filters.** A step costs its length times the cost of the polygon it lies in; excluded polygons are not entered, are walls to rays and are skipped by the nearest point, except the start polygon, which is always usable. The heuristic is the distance times the cheapest cost any included area or crossable link reaches per meter, so it stays consistent at any costs and cheap teleports keep the search exact.

## Nearest point and boxes

`mnavFindNearest` returns, among polygons whose nearest point lies in the box, the one that scores best: over a polygon, the height beyond the agent's step; otherwise the distance. Ties go to the shorter distance, then the tile first by place, then the lower polygon index. It tells whether the point lies over the polygon and whether part of the box is not loaded. `mnavFindPolygons` lists polygons whose bounds meet a box, by tile in place order then index. Both scan each loaded tile's polygons in the box; a search tree per tile would be added on measured need, without changing any answer.

## The A* search

`mnavFindPath` (`path.c`) runs A* over the portals between polygons. Its nodes stand at portal midpoints, plus the two end points; costs and the heuristic are 3D distances, so the heuristic is consistent and no node is reopened. Ties go to the node made first. It ends found, out of nodes, too long, stopped at a place with no tile, or with no path, checked in that order. The corridor lists each visit to a polygon once: a polygon an off-mesh link leaves and lands back on appears before the link and after it, so a corridor set from the path still crosses the link.

Sliced searches (`mnavBeginPath`, `mnavContinuePath`, `mnavFinishPath`) keep the search in the context between calls, with a copy of the filter; `mnavFindPath` is the three at once, so any slicing gives the same result bit for bit. The navmesh's commit count makes a search across a commit stale.

## The straight path

The corridor is pulled tight with the funnel algorithm (`funnel.c`) over its portals, in binary64 world coordinates with exact side tests and no tolerance: a portal end on a funnel side narrows it, one on or past the other side makes that side's point a corner. Across a tile side, the portal is the exact cell range the tile link covers. The funnel runs stretch by stretch between off-mesh links. The points live in the context, sized from the node limit, so a straight path is never cut short.

## The shortest path

Portal midpoints make A* cheap and the straight path often longer than it need be: on the benchmark's flat world of pillars, `mnavFindPath`'s paths are 11% longer than the shortest on average and up to 39% longer. `mnavFindShortestPath` (`shortest.c`) runs Polyanya (Cui, Harabor and Grastien, 2017) beside it rather than in its place.

- Nodes are intervals of portals, each with a root that sees all of it: the start, a corner the way turns at, or an off-mesh link's landing point. The heuristic is the distance from the root through the interval to the end. The best cost known at each root's position prunes a dearer way to it, and a node coming back round a root to a polygon an ancestor with that root entered is dropped.
- Every vertex and every end of a link along a tile side counts as a corner, a superset of the corners where walls meet. A line through a vertex no wall meets, common where tiles and blocks align, is then a turn of no angle there; counting only wall corners would lose it.
- An end the start cannot reach would have Polyanya spend its nodes on every way there is. When the nodes run out, the A* search's reachability, at most a node per portal, decides between out of nodes and no path.
- Polyanya is exact only for one cost, so every included area must have one cost. Off-mesh links are crossed at their declared cost, the takeoff point reached like the end and the landing point a new root.

## Rays and surface moves

`mnavRaycast` (`raycast.c`) walks a segment on the ground from polygon to polygon through the first edge it crosses outward, across tile sides where a link's cell range holds the exit point; at a corner it goes on through an edge that leads on, the lowest-numbered first. It ends reached, at a wall with the wall's normal, at a side with no tile, or out of nodes.

`mnavMoveAlongSurface` (`surface.c`) walks breadth-first, in edge order, over the included polygons whose shared edges meet the circle round the move, widened by one cell so that a start on a vertex keeps its neighbors. It reaches the wanted point in the polygon holding it, or stops at the nearest point on the walls met. Walls are every edge, and every part of a tile-side edge, with no included polygon across, so gaps between linked polygons are walls too.

## Corridors and steering

`mnavCorridor` (`corridor.c`) is a plain struct over a polygon buffer the caller owns, so a host keeps thousands of agents without the library holding any. Loading a path longer than the buffer is refused, never cut, and a move whose merge would outgrow it changes nothing: a silently shortened corridor would steer an agent toward the wrong place. Moves walk the surface and merge the polygons walked into the corridor's start or end; `mnavCheckCorridor` counts the leading polygons still current, included and joined, where the host trims and replans.

`mnavSteer` (`steer.c`) keeps no state and reads only the corners, so it is the same for an agent on any thread. It anticipates turns as Detour's crowd does, swinging wide of the next corner to meet the turn already heading out of it.

## Walls and random points

`mnavFindWallDistance` and `mnavFindWalls` (`spatial.c`) search from the center's polygon through the edges within a radius, as Detour's findDistanceToWall does. An edge with nothing included across it is a wall, an unloaded tile side included, and so are the parts of a tile side no polygon beyond covers. Off-mesh links are left aside. `mnavFindWalls` keeps the nearest walls when its buffer is short, as Detour's crowd keeps a local boundary.

`mnavFindRandomPoint` picks uniformly over the ground area of every included polygon from a 64-bit seed through SplitMix64, where Detour picks a tile first and so favors sparse tiles. `mnavFindRandomPointAround` samples the polygons the wall search reaches by the area each shares with the circle, an inscribed 32-gon written out as constants, so the point lies in the circle, where Detour's may lie a polygon away.

## Grids

`mnavFindGridPath` (`grid.c`) searches a caller's grid of areas in a query context: eight neighbors, no blocked corner cut, a step costing its length times the mean of its cells' area costs. When every included area costs the same it runs jump point search, which needs uniform costs; otherwise A*. It shares the navmesh search's ends and limits, and returns the cells the path turns at.
