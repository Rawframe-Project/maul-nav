# Hierarchy

Long paths on large worlds need many nodes: on a world of 24 by 24 tiles (768 m, about 28,000 polygons) with pillars and walls, the plain search needs up to 70,000 nodes for a long path, and the default 8,192 find none of the longest. `mnavHierarchy` (`src/hierarchy.c`, `src/hierarchy_path.c`) follows HPA* (Botea, Mueller and Schaeffer, 2004): it cuts the navmesh into clusters of tiles, keeps the costs between their entrances, and refines each abstract step by a local search.

## The graph

- **One per navmesh and filter.** A hierarchy is sized by named limits (tile slots, transitions, edges) and a cluster side in tiles. `mnavBuildHierarchy` builds it with a query context for its local searches and keeps a copy of the filter, since the edges hold the filter's costs. A commit to the navmesh makes it stale until it is updated.
- **Transitions.** Each run of tile links along a cluster border, one way, is an entrance, and its middle link is a transition, the search node of crossing it, standing at the portal's midpoint. Each off-mesh link that the filter crosses from one cluster into another is a transition too, standing at its landing. Transitions are numbered by tile place, side and position, never by load order, so the graph is the same however the tiles arrived.
- **Edges.** From each transition, Dijkstra's search confined to the cluster it enters, opening nodes beyond its sides but never expanding them, gives the cost to each transition leaving that cluster.

## A query

`mnavFindHierarchicalPath` joins the end to the transitions entering its cluster by a confined search from the end, run backward. Walks cost the same either way, and off-mesh links are followed from where they land to where they take off, so a one-way link inside the cluster serves only the ways it goes. It joins the start to the transitions leaving its cluster, runs A* over the transitions (the straight distance times the search's heuristic scale, ties to the lower index), then refines step by step with the navmesh search, each step confined to one cluster, aimed at the next transition, and ending on crossing any link of its run. Each step starts from the node the last one reached and keeps only the way so far, so a query holds one cluster's nodes at a time. The result is an ordinary `mnavPath`.

- Ends within a cluster's side of each other try the plain search first and take the hierarchy only when it runs out of nodes, since a short way forced through entrances can be much longer.
- A start and end in one cluster, an end the graph does not reach, or a step that finds no way fall back on the plain search.

Paths cost near the cheapest, not always the cheapest: on a test world of 64 tiles, forty random pairs came out at most 4.8% longer than the plain search's, most within 2%.

## Updates

`mnavUpdateHierarchy` brings a hierarchy up to the navmesh's last commit with the graph a build would make. The build keeps each tile slot's generation, a hash of its polygon areas, and a hash of the off-mesh links' attachments. When only areas changed and no link lands in a changed cluster, only the transitions entering the changed clusters are searched again; otherwise the hierarchy is built again. The report counts the searches run.

A transition's crossing is a polygon in the tile it enters, so the searches of the cluster it leaves end one polygon past its side. Those searches open the crossing polygon but never walk it, so its area's cost enters none of their costs; only whether the filter includes it does. Each transition keeps that, and the cluster it leaves is searched again only when it flips. Keeping those edges without the check once gave a graph different from a build's, which a fuzz target found; on the benchmark's flat world the check keeps an area change to about 20 of 588 transitions searched.
