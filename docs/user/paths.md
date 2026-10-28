# Paths and queries

Queries read a committed navmesh. A query context holds a search's scratch memory, sized by its limits (8,192 nodes and paths up to 1,000 m by default); each thread that queries uses a context of its own, and any number of contexts read one navmesh at once between commits.

```c
mnavQueryDef queryDef = mnavDefaultQueryDef();
mnavQuery* query = NULL;
mnavResult opened = mnavCreateQuery(&queryDef, &query);

mnavNearest a, b;
const mnavVec3 box = {1.0f, 2.0f, 1.0f};      // half extents of the search
mnavResult nearA = mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 8.0}, box, &a);
mnavResult nearB = mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 56.0}, box, &b);
// a.polygon.slot is 0 when no polygon lies in the box

mnavPath path;
mnavResult searched =
    mnavFindPath(query, navmesh, NULL, a.polygon, a.point, b.polygon, b.point, &path);
// path.end: found, partial (out of nodes or too long), no path, not loaded
// path.points: the straight path; path.polygons: the corridor; path.links
```

- **Ends.** Every search starts and ends on a polygon, so snap both ends with `mnavFindNearest` first. A point found with `incomplete` set may have a nearer polygon on a tile not yet loaded.
- **How a search ended.** `path.end` is `mnav_pathFound`, `mnav_pathOutOfNodes` or `mnav_pathTooLong` (a partial path toward the polygon nearest the end), `mnav_pathNotLoaded` (it ran into a place with no tile) or `mnav_pathNone`. A partial path is a useful start: walk it and search again from its end.
- **Memory.** The polygons, points and links a query returns live in the context until its next query. Copy what must outlive it.

## Filters

`mnavQueryFilter`, from `mnavDefaultQueryFilter`, is an agent's capabilities: a cost per meter for each area type, the area types it may enter, and the link kinds it may cross. `NULL` uses every walkable area at cost 1 and every link kind. Keep one filter per kind of agent; it is plain data and can be shared by any number of queries.

## The shortest way

`mnavFindPath` picks its corridor by costs through the midpoints of polygon edges, then pulls the path tight within it. Among pillars and clutter the corridor can weave, and the path come out a tenth or more longer than it needs to be. `mnavFindShortestPath` takes the same arguments and gives the same kind of result, but its path is the shortest the ground allows, and it often costs fewer instructions. It needs every area the filter includes at one cost (`mnav_errorInvalid` otherwise), so use it for agents with no area preferences.

## Spreading searches over frames

- **Sliced searches.** `mnavBeginPath`, then `mnavContinuePath` with a budget of nodes each frame, then `mnavFinishPath`. The result is the one `mnavFindPath` gives, bit for bit, for any budgets. A commit in between leaves the search out of date (`mnav_errorState`); begin it again.
- **Long paths.** On a large world a long search needs many nodes. A hierarchy groups tiles into clusters and keeps the costs between their entrances: `mnavCreateHierarchy` and `mnavBuildHierarchy` once, `mnavUpdateHierarchy` after commits (it redoes only what an area change touched), and `mnavFindHierarchicalPath` with a small node budget. Its paths are close to the cheapest, not always the cheapest; on test worlds most come within 2% of the plain search's.

## Spatial queries

| Question | Function |
|---|---|
| Is the way straight to a point clear? | `mnavRaycast`, which stops at a wall (with its normal), a place with no tile, or the end. |
| Where does a move along the surface end? | `mnavMoveAlongSurface`, which slides along the walls it meets. |
| How high is the ground here? | `mnavGetHeight`, on a polygon's detail surface. |
| How far is the nearest wall? | `mnavFindWallDistance`. |
| Which walls are near, for avoidance? | `mnavFindWalls`, nearest first, into a caller buffer. |
| Which polygons lie in a box? | `mnavFindPolygons`. |
| Where can something spawn? | `mnavFindRandomPoint` over the whole navmesh, `mnavFindRandomPointAround` within a circle, both from a seed. |
| Can a point be reached at all? | `mnavCheckReachable`, a path search without the straight path. |

## Grids

For tile maps, `mnavFindGridPath` searches an `mnavGrid` of area types in a query context, under the same filters: eight neighbors, no corner cut past a blocked cell. It runs jump point search when every included area costs the same and A* otherwise.

## Following a path

A path corridor keeps an agent's path valid as the agent and its target move. It is a plain struct over a polygon buffer the caller owns.

```c
mnavPolygonId buffer[256];
mnavCorridor corridor;
mnavResult reset = mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point);
mnavResult set = mnavSetCorridor(&corridor, &path);   // mnav_errorCapacity past 256

// Each tick:
mnavCorners corners;
mnavResult cornered = mnavCorridorCorners(query, navmesh, &corridor, &corners);
// head for corners.points[1]
mnavResult moved = mnavMoveCorridor(query, navmesh, NULL, &corridor, wanted, NULL);
```

- `mnavSteer` turns the corners into the velocity the agent wants: at the def's speed toward the next corner, swinging wide of a turn, slowing near the target, and stopped on arriving or at a link's takeoff (`mnav_steerAtLink`, with the link's index among the corners' links). It keeps no state; feed its velocity to [avoidance](crowds.md) as the agent's preferred one, and limit acceleration in the host.
- `mnavMoveCorridorTarget` moves the target. To give an agent a new target, find its polygon and load a new path with `mnavSetCorridor`, or move it with `mnavMoveCorridorTarget`: setting `corridor.target` by hand leaves the corridor's last polygon where it was, and a replan goes there.
- `mnavShortcutCorridor` straightens the corridor where a ray toward a point the host picks gets through.
- After a commit, `mnavCheckCorridor` counts the polygons still valid from the start, and `mnavReplanCorridor` searches again from where the corridor breaks.
- The corners carry the off-mesh links ahead with their kinds. At a link, play the traversal, move the agent to the landing point, and move the corridor on with `mnavMoveCorridor`.
- `mnavCorridorAreas` gives the corners with the runs of area along them: where the path enters mud, water or a road, and where it lands from a link, so the agent can change gait or sound there.

A buffer too small for a path is refused, never cut: `mnavSetCorridor` returns `mnav_errorCapacity` and leaves the corridor as it was.

The library never moves an agent. It returns where to go, and the host applies it.
