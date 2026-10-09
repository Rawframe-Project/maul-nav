# Maul Nav guide

This guide walks through the library in the order a program uses it:
bake tiles, load them into a navmesh, query it, follow paths, fly, move
crowds and draw what happened. Every function named here is described
in full in [the API reference](api.md); the design records behind each
part are in `adr/`. `samples/walk.c` and `samples/crowd.c` are complete
programs built from the steps below.

## The ground rules

- **Units and axes.** Meters and seconds; right-handed, +Y up, -Z
  forward. World positions are doubles (`mnavPos3`); positions inside a
  frame the library names (a bake's origin, a tile) are floats.
- **Results.** Every fallible function returns `mnavResult`: zero is
  success, negative values are errors (`mnav_errorInvalid`,
  `mnav_errorRange`, `mnav_errorLimit`, `mnav_errorStale`,
  `mnav_errorCapacity`), and `mnavResultName` names any of them. The
  results are marked `[[nodiscard]]` where the compiler supports it.
- **Defs and limits.** Each owner object (baker, navmesh, query
  context, flow field, hierarchy, avoidance set) is made from a def you
  get from its `mnavDefault...Def` function and adjust. A def carries
  named limits; the object takes the memory they need when it is made,
  and reaching a limit is a typed result, never unbounded work or a
  silent cut.
- **Memory.** Every def has an allocator; zeroed, the C library's is
  used. Queries allocate nothing: their scratch is the query context's.
- **Threads.** The library starts none. Each function's documentation
  says what may run at once: in short, any number of threads may query
  one navmesh while no commit runs, each with its own query context,
  and each object is used by one thread at a time.
- **Determinism.** The same inputs give the same bytes and the same
  results on every platform, compiler and worker count. Random queries
  take a seed from you.

## Baking tiles

A bake turns geometry into navmesh tiles. The world is cut into square
tiles of `tileCells` cells of `cellSize` meters (by default 128 cells
of 0.25 m: 32 m), counted from the def's `origin`; each tile bakes on
its own.

```c
mnavBakeDef def = mnavDefaultBakeDef();   // agent, cells, limits, tier
mnavBaker* baker = NULL;
mnavBakeDefResult made = mnavCreateBaker(&def, &baker);
// made.result; on mnav_errorInvalid, made.setting names the bad setting.

mnavTriangleMesh mesh = {vertices, vertexCount, indices, triangleCount, NULL};
mnavBakeReport report;
size_t size = 0;
mnavResult baked = mnavBakeTile(baker, &mesh, 1, tileX, tileZ, &report);
if (baked == mnav_success)
{
    baked = mnavCopyBakedTile(baker, bytes, capacity, &size);   // the tile's bytes
}
```

Every call returns a status, `mnavResult` or a result naming what was
refused; the snippets here keep each one, and the library's tests run
them as they are written.

- **Input.** `mnavBakeTile` takes triangle meshes, each triangle with an
  optional area type. `mnavBakeTileInput` also takes terrains (heights
  on a regular grid, `mnavTerrain`) and bake volumes (`mnavBakeVolume`:
  include, exclude or override an area inside a prism).
  `mnavBakeTile2D` bakes 2D outlines (`mnavOutline`) into a flat tile.
  Input is checked as hostile: NaNs, indices out of range and
  overflowing values are refused with the first element named in the
  report.
- **Checking first.** A tool can check a def or its input without
  baking, at import time for instance: `mnavValidateBakeDef` names a
  bad setting and gives the def's settings in cells,
  `mnavValidateTriangleMesh` and `mnavValidateOutline` name the first
  bad vertex, triangle or point with the checks a bake makes.
- **Large meshes.** A bake tests every triangle of its input against
  its tile. When many tiles bake from meshes much larger than a tile,
  make a tile index of the meshes once (`mnavCreateTileIndex`) and pass
  it as `mnavBakeInput.index`: each bake then reads only the triangles
  near its tile and makes the same bytes. Indexes are read-only, so
  workers share one. After changing a mesh, make a new index. Outlines
  have theirs too: `mnavCreateTileIndex2D`, passed to
  `mnavBakeTile2DInput` in an `mnavBake2DInput`.
- **The agent.** The def's `agent` (radius, height, step height,
  maximum slope) shapes what is walkable; a navmesh serves one agent
  profile.
- **The report.** `mnavBakeReport` gives the stage a bake ended in, the
  tile's counts, the memory peak, what was given up on (holes no bridge
  reached, islands dropped) and, when the def has a clock
  (`mnavClock`), each stage's time.
- **Workers.** Tiles are independent calls. Run them on your own task
  system, one baker per worker; the bytes are the same at any worker
  count.

### Obstacles

A crate or a barricade that stands still changes the tiles it stands
on (mnav-0016). Bake those tiles through a tile cache, which keeps
each tile's open space as it stands before bake volumes apply; then
rebuild a tile with obstacles, given as bake volumes, without its
triangles:

```c
mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();   // 4096 tiles in 64 MiB
mnavTileCache* cache = NULL;
mnavResult cached = mnavCreateTileCache(&cacheDef, &cache);
mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};
if (cached == mnav_success)
{
    cached = mnavBakeTileCached(baker, cache, &input, tileX, tileZ, &report);
}

// A crate on the tile: a ring on the ground and a height range.
const mnavVec2 ring[4] = {{36, 4}, {38, 4}, {38, 6}, {36, 6}};
const mnavBakeVolume crate = {ring, 4, -1.0f, 2.0f, mnav_volumeExclude, 0};
mnavResult rebuilt = mnavRebuildTile(baker, cache, tileX, tileZ, &crate, 1, &report);
```

- **The same bytes.** A rebuilt tile is, to the byte, the tile
  `mnavBakeTileInput` bakes from the same input with the obstacles
  appended to its volumes, in about a third of the time. Copy it out
  with `mnavCopyBakedTile` and replace the tile in the navmesh.
- **Every obstacle, each time.** A rebuild applies the cached volumes
  and the obstacles it is given, so pass every obstacle on the tile;
  rebuild with none to clear them. An obstacle reaches the tiles its
  ring touches, widened by the agent's radius.
- **Memory.** A cached tile is kept compressed: about 10 KB for a 32 m
  tile of hilly terrain at the default cells. Drop tiles
  (`mnavDropCachedTile`) as they stream out; `mnavGetTileCacheBytes`
  gives what the cache holds. Moving obstacles are for avoidance.

## The navmesh

A navmesh holds committed tiles and off-mesh links. Changes are staged,
then committed at a point you choose; queries see one committed state.

```c
mnavNavmesh* navmesh = NULL;
mnavBakeDefResult created = mnavCreateNavmesh(&def, &navmesh);   // the bake def
mnavTileResult staged = mnavStageTile(navmesh, bytes, size);     // copied
// staged.result; refused bytes are named by section and element
mnavResult committed = mnavCommit(navmesh);
```

- **Loading** checks every byte: a tile from another def, a damaged
  payload or a polygon that is not convex is refused with
  `mnavTileResult` naming the section and element.
- **Tiers.** The def's `tier` decides what may change after the first
  commit: `mnav_tierStatic` nothing, `mnav_tierModifiers` areas
  (`mnavStageArea`) and link toggles (`mnavStageLinkEnabled`),
  `mnav_tierDynamic` also tiles replaced or removed
  (`mnavStageTileRemoval`). A change the tier does not allow is
  refused.
- **Streaming.** Tiles load and unload at runtime. A search that runs
  into a tile not loaded ends with `mnav_pathNotLoaded`, never treating
  it as a wall.
- **Off-mesh links.** `mnavStageLink` adds a jump, drop, climb, ladder,
  door, teleport or host-declared kind, point to point or edge to edge
  (`width`), one way or two, with a cost, and `mnavStageLinkRemoval`
  takes one away. Links snap to the polygons at their ends at each
  commit; `mnavGetLink` tells whether a link is attached and enabled,
  and where its ends snapped. `mnavGenerateLinks` proposes drops and
  jumps along a range of tiles, with your clearance test if you give
  one.
- **Ids.** Polygons, tiles and links are named by ids with generations;
  an id of a replaced tile reads as `mnav_errorStale`.
- **Reading back.** `mnavGetTile` finds the tile loaded at a place,
  `mnavGetArea` a polygon's committed area and `mnavGetTier` the
  navmesh's tier.

## Queries

A query context holds a search's scratch, sized by its limits (nodes
and path length). Use one per thread.

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

- **Filters.** `mnavQueryFilter`, from `mnavDefaultQueryFilter`, sets
  each area's cost, the areas included and the link kinds crossed: an
  agent's capabilities. `NULL` uses every walkable area and kind.
- **The shortest way.** `mnavFindPath` picks its corridor by costs
  through portal midpoints, then pulls the path tight within it; among
  pillars or other clutter that corridor can weave, and the path come
  out a tenth or more longer than it need be.
  `mnavFindShortestPath` takes the same arguments and gives the same
  result, but its path is the shortest the navmesh allows on the
  ground, and it often costs fewer instructions. It needs every included
  area at one cost, so use it for agents with no area preferences.
- **Sliced searches.** `mnavBeginPath`, then `mnavContinuePath` with a
  node budget each tick, then `mnavFinishPath`: a server spreads many
  agents' searches over its ticks. A commit in between makes the search
  stale.
- **Long paths.** A hierarchy (`mnavCreateHierarchy`,
  `mnavBuildHierarchy`, `mnavUpdateHierarchy` after commits) groups
  tiles into clusters; `mnavFindHierarchicalPath` searches the graph
  between them and refines the path cluster by cluster, with a small
  node budget.
- **Spatial queries.** `mnavRaycast` along the surface,
  `mnavFindWallDistance`, `mnavGetHeight`, `mnavMoveAlongSurface`,
  `mnavFindPolygons` in a box, `mnavFindRandomPoint` and
  `mnavFindRandomPointAround` from a seed, `mnavCheckReachable`.
- **Grids.** For tile maps, `mnavFindGridPath` runs A* or jump point
  search over an `mnavGrid` of area types, under the same filters.

## Following a path

A path corridor keeps an agent's path valid as it and its target move.

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

- `mnavMoveCorridorTarget` moves the target; `mnavShortcutCorridor`
  straightens it where the way is clear.
- After a commit, `mnavCheckCorridor` counts the polygons still valid;
  `mnavReplanCorridor` searches again from where it breaks.
- The corners carry the off-mesh links ahead with their kinds, so you
  know when the agent reaches a jump or a ladder and can play the
  animation and move it yourself.

The library never moves an agent: it returns where to go, and you
apply it.

## Flow fields

When many agents share goals, one search serves them all.

- **Grids.** `mnavCreateFlowField`, then `mnavBuildFlowField` over an
  `mnavGrid` toward goal cells; `mnavFlowAt` gives each cell's cost and
  next cell. `mnavBeginFlowField` and `mnavContinueFlowField` work over a
  region in budgeted steps, and `mnavUpdateFlowField` repairs the field
  when goals move or cells change, giving the same field as a rebuild.
- **Navmeshes.** `mnavCreateNavFlow`, then `mnavBuildNavFlow` toward goal
  points; `mnavNavFlowAt` gives each polygon's cost, next polygon and the
  portal to head for (or the link to take). `mnavBeginNavFlow` and
  `mnavContinueNavFlow` work over a box of tiles in budgeted steps. A
  commit makes the field stale; build it again, in steps if you like,
  while agents read the last one.

## Flight volumes

Agents that fly or swim are not bound to the ground, so a navmesh does
not describe where they may go. A flight volume (`maul-nav/flight.h`)
does: each tile is a sparse voxel octree of the space a flier's center
may occupy.

- **The def.** `mnavDefaultFlightDef`, then set the voxel size, the
  tile's side in voxels (4 times a power of 2), the floor and ceiling
  in meters, and the flier's radius. Leave `groundBelow` on for worlds
  with ground, so that the space under a terrain is solid; turn it off
  for open space, where it would fill below every floating piece.
- **Baking.** `mnavCreateFlightBaker`, then `mnavBakeFlightTile` with
  the same `mnavBakeInput` a navmesh bake takes, then
  `mnavCopyFlightTile`. A voxel any triangle touches is solid, and so is
  every voxel within the flier's radius of one. Triangles are surfaces:
  a closed mesh's inside stays open space, which no path from outside
  reaches. Tiles are independent calls, as navmesh tiles are, and give
  the same bytes on every platform.
- **Loading.** `mnavCreateFlightVolume`, then `mnavStageFlightTile`
  and `mnavStageFlightTileRemoval`, then `mnavCommitFlight`, as with
  the navmesh. Every byte is checked; a tile baked with other settings
  is refused. `mnavGetFlightTile` reads a committed tile's
  fingerprint.
- **Queries.**
  - `mnavIsFlightOpen` tells whether a point is open to the flier.
  - `mnavFindNearestFlightPoint` finds the nearest open point within a
    radius; snap a path's ends with it first.
  - `mnavFindFlightPath` finds a path in an `mnavQuery` context, with
    its node budget and path length limit. Its points are each in sight
    of the next.
  - `mnavBeginFlightPath`, `mnavContinueFlightPath` and
    `mnavFinishFlightPath` run the same search in slices of a number of
    nodes, so that a long search spreads over frames; the path is the
    one `mnavFindFlightPath` gives. A commit to the volume between
    slices makes the search stale: begin again.
  - `mnavFlightRaycast` stops at the first blocked voxel or place with
    no tile.
  - `mnavCheckFlightPath` casts a path's steps in order and names the
    first one blocked or with no tile. After a commit, a host checks
    what is left of each path it follows, from where the agent is, and
    searches again when a step fails.

## Avoidance

Avoidance is a component of its own that needs no navmesh: it works on
the ground plane, steering agents round each other and round obstacles
by velocity obstacles (ORCA).

```c
mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
mnavAvoidance* avoidance = NULL;
mnavResult made = mnavCreateAvoidance(&def, &avoidance);

// Each step: positions, velocities and preferred velocities in, new
// velocities out.
mnavResult stepped =
    mnavAvoid(avoidance, agents, agentCount, obstacles, obstacleCount, 0.1, velocities);
```

The set keeps no state between steps: you pass every agent each time,
with the step's length. Obstacles are circles, segments or polygons,
still or moving. The same agents in any order get the same velocities.
Input has named ranges (`MNAV_MAX_AVOIDANCE_COORDINATE`,
`MNAV_MAX_AVOIDANCE_SPEED`, `MNAV_MAX_AVOIDANCE_RADIUS`,
`MNAV_MIN_AVOIDANCE_TIME`), wide enough for any scene, and an agent's
new velocity is never faster than its maximum speed.

Fliers sharing a flight volume avoid each other in space with
`mnavAvoid3D`, on the same set: `mnavAgent3D` has the same fields as
`mnavAgent` in three dimensions, and its obstacles are spheres
(`mnavSphere`), still or moving, which count against the set's
obstacle points. Walls and terrain are the flight volume's to keep
clear of: a flier follows its path and avoids what moves, and the host
keeps each step in open space with `mnavFlightRaycast`, sliding along
the axes when a step is blocked (`samples/flight.c`). A held-back
flier sidesteps to its right, +Y being up, and one rising or falling
straight sidesteps toward +X or -X, so that two meeting head on pass.

## Debug output

The library never draws. It fills an `mnavDebugBuffer` you own with
vertices (positions relative to the buffer's origin, a kind and a
value) and triangle and line indices, which any renderer can draw:

- `mnavDebugNavmesh`: polygons by area, edges, tile bounds;
- `mnavDebugLinks`, `mnavDebugPath`, `mnavDebugCorridor`;
- `mnavDebugFlowField` and `mnavDebugNavFlow`: arrows;
- `mnavDebugAvoidance`: agents and the neighbours each one sees;
  `mnavDebugAvoidance3D` the same for fliers, each as three rings, with
  the spheres they avoid;
- `mnavDebugFlight`: a flight volume's open blocks in a box, as wire
  boxes; `mnavDebugFlightPath`: a flight path's steps.

A full buffer returns `mnav_errorCapacity` with the counts the whole
needs, so you can grow it and draw again.

## Building and using it

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix "$PWD/package"
```

A program finds the installed package with
`find_package(maul-nav REQUIRED CONFIG)` and links `maul-nav::maul-nav`;
`samples/minimal` is such a program. `mnavGetVersion` gives the version
of the library linked, which a program can compare with the
`MNAV_VERSION_*` macros it was compiled with. The headers compile as
C17, C23 and C++17; the library itself is C23 (GCC 14, Clang 19 or
newer; clang-cl on Windows).
