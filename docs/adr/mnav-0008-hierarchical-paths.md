# mnav-0008. Hierarchical paths

Status: Accepted

## Context

The requirements ask for hierarchical pathfinding for large worlds: an
abstract graph over tiles or clusters for long paths, refined locally.
On a world of 24 by 24 tiles (768 m, 28,229 polygons) with pillars and
walls, the plain search needs up to 70,360 nodes and about 30 ms for a
long path, and the default 8,192 nodes find none of the longest. HPA*
(Botea, Mueller and Schaeffer, 2004) cuts a map into clusters, keeps
the costs between their entrances, and refines each abstract step by a
local search.

## Decision

- **An owner object per navmesh and filter:** `mnavHierarchy` is sized
  by named limits (tile slots, transitions, edges) and a cluster side
  in tiles. `mnavBuildHierarchy` builds it with a query context for the
  local searches and keeps a copy of the filter, whose costs it holds;
  a later commit to the navmesh makes it stale (`mnav_errorStale`).
- **Transitions:** each run of tile links along a cluster border, one
  way, is an entrance; its middle link is a transition, the search node
  of crossing it, standing at the portal's midpoint. Transitions are
  numbered by tile place, side and position, never by load order.
  Each off-mesh link, one way, that the filter crosses from one cluster
  into another is a transition too, standing at its landing; they come
  after the portals', in the attachments' order.
- **Edges:** from each transition, Dijkstra's search confined to the
  cluster it enters, opening nodes beyond but never expanding them,
  gives the cost to each transition leaving that cluster.
- **Queries:** `mnavFindHierarchicalPath` joins the end to the
  transitions entering its cluster by a confined search from the end,
  the walk's cost being symmetric: a portal through the cost found to
  cross it the other way, a link's landing through the cheapest node
  reached in its polygon and then straight on; and the start to those
  leaving its cluster; A* over the transitions, with the straight distance times
  the search's heuristic scale, ties to the lower index, picks them;
  then the navmesh search refines step by step, each step confined to
  one cluster, aimed at the next transition and ending on crossing any
  link of its run, starting over from the node the last step reached
  and keeping only the way so far.
  The result is an ordinary `mnavPath`, with its corridor and links.
- **Near ends:** ends within a cluster's side of each other try the
  plain search first and take the hierarchy only when it runs out of
  nodes, since a short way forced through entrances can be much longer.
- **Fallbacks:** a start and end in one cluster, an end the graph does
  not reach, or a step that finds no way take the plain search.

## Consequences

- Paths cost near the cheapest, not always the cheapest: on a test
  world of 64 tiles, forty random pairs came out at most 4.8% longer
  than the plain paths, most within 2%.
- Each query's searches hold one cluster's nodes and the way so far.
- Rebuilding only the clusters a commit changes is later work.
