# Baking

A bake turns geometry into navmesh tiles. The world is cut into square tiles of `tileCells` cells of `cellSize` meters, counted from the def's `origin` (by default 128 cells of 0.25 m, so 32 m tiles), and each tile bakes on its own.

```c
mnavBakeDef def = mnavDefaultBakeDef();   // agent, cells, limits, tier
mnavBaker* baker = NULL;
mnavBakeDefResult made = mnavCreateBaker(&def, &baker);
// made.result; on a refused def, made.setting names the bad setting.

mnavTriangleMesh mesh = {vertices, vertexCount, indices, triangleCount, NULL};
mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};   // meshes, terrains, volumes, index
mnavBakeReport report;
size_t size = 0;
mnavResult baked = mnavBakeTile(baker, &input, tileX, tileZ, &report);
if (baked == mnav_success)
{
    baked = mnavCopyBakedTile(baker, bytes, capacity, &size);   // the tile's bytes
}
```

The bytes are the tile: store them as cooked content, and load them into a [navmesh](navmesh.md) at run time. The same def and input give the same bytes on every platform.

## The def

- **The agent.** `agent` (radius, height, step height, steepest slope) decides what is walkable. A navmesh serves one agent profile; agents of very different sizes need a navmesh each.
- **Cells.** `cellSize` and `cellHeight` set the resolution. Smaller cells follow walls more closely and cost more memory and time. `mnavValidateBakeDef` checks a def without making anything and reports what its meters become in cells (`mnavBakeCells`), so a tool can show the effect of a setting.
- **The frame.** Input vertices are floats in meters relative to `origin`, a world position in doubles. A world much more than 1,000 km across at 0.25 m cells needs more than one origin.
- **Limits.** `limits` bounds the work of one bake and the size of the navmesh made from the same def: triangles, spans, polygons, vertices, links, tiles and memory. A limit reached is `mnav_errorCapacity`, with the report's `stage` saying where.
- **The tier.** `tier` is read by the navmesh, not the bake: it says what may change once tiles are loaded. See [the navmesh](navmesh.md#tiers).

## Input

- **Triangle meshes** (`mnavTriangleMesh`): vertices, three indices per triangle, and an optional area type per triangle. Area 0 (`mnav_areaNone`) is not walkable and blocks what lies under it; 1 to 63 are walkable kinds that query filters cost and include. Without areas, every triangle is `mnav_areaWalkable`.
- **Terrains** (`mnavTerrain`): heights on a regular grid with an optional area per cell; a cell of `mnav_areaNone` is a hole. A terrain is the compact form of the largest input and bakes to the same tile as the same triangles given as a mesh.
- **Bake volumes** (`mnavBakeVolume`): a prism on a ring of points with a height range. An exclude volume drops the ground inside, and agents then keep their radius from it as from a wall. When a bake has include volumes, only the ground inside them is kept. An area volume gives the walkable ground inside its area type without shrinking it.
- **2D outlines** (`mnavOutline`): closed rings of points with an area, for top-down games. `mnavBakeTile2D` fills the cells a walkable outline holds as a flat floor at height 0; an outline of `mnav_areaNone` cuts an obstruction. The result is an ordinary tile that every query serves.

`mnavBakeTile` takes meshes, terrains and volumes together in an `mnavBakeInput`, and `mnavBakeTile2D` takes outlines in an `mnavBake2DInput`.

Input is hostile: a NaN coordinate, an index out of range or an area past 63 is `mnav_errorInvalid`, a point past the extent, an infinity included, is `mnav_errorLimit`, and the report names the first mesh and the vertex, triangle or point refused. `mnavValidateTriangleMesh` and `mnavValidateOutline` run the same checks without baking, for an import step that wants to catch bad geometry early. Degenerate and duplicate triangles are accepted and contribute nothing.

## Many tiles

- **Workers.** Tiles are independent calls. Run them on the host's own task system with one baker per worker; bakers share nothing, and the bytes are the same at any worker count.
- **Large meshes.** A bake tests every input triangle against its tile. When many tiles bake from meshes much larger than a tile, make a tile index of the meshes once (`mnavCreateTileIndex`) and pass it as `mnavBakeInput.index`: each bake then reads only the triangles near its tile and makes the same bytes. An index is never changed after it is made, so workers share one. After editing a mesh, make a new index: a bake refuses an index whose counts no longer match, but a triangle moved into a tile it was not listed for is missed. Outlines have their own index, `mnavCreateTileIndex2D`, passed in the `mnavBake2DInput` of `mnavBakeTile2D`.
- **Skipping unchanged tiles.** The report's `fingerprint`, also written into the tile, hashes the bake's generation (`MNAV_BAKE_GENERATOR`, which rises only when a bake of the same input would differ), the def's settings, the tile's place and every input triangle that reaches it. A host can skip rebaking a tile whose fingerprint has not changed.

## The report

`mnavBakeReport` gives the stage a bake ended in, the tile's counts and size, the memory peak, and the work it gave up on rather than fail: holes no bridge reached, triangulations cut short (`partialRings`, usually from a `maxEdgeError` of a cell or more, whose walls can cross), detail samples with no height, and walkable islands smaller than `minRegionArea` dropped (`droppedRegions`), which usually point at geometry the host did not mean to be walkable. With a clock in the def (`mnavClock`), it also gives each stage's ticks. The clock never shapes the tile.

## Obstacles

A crate or a barricade that stands still changes the tiles it stands on. Bake those tiles through a tile cache, which keeps each tile's open space as it stands before bake volumes apply, and then rebuild a tile with obstacles, given as bake volumes, without its triangles:

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

- A rebuilt tile is, to the byte, the tile `mnavBakeTile` bakes from the same input with the obstacles added to its volumes, in about a third of the time. Copy it out with `mnavCopyBakedTile` and replace the tile in a navmesh of `mnav_tierDynamic`.
- A rebuild applies the cached volumes and the obstacles it is given, so pass every obstacle on the tile each time; rebuild with none to clear them.
- An obstacle reaches the tiles its ring touches, widened by a tile's border: the agent's radius rounded up to whole cells, and three cells more. Rebuild each of them.
- A cached tile is about 10 KB for a 32 m tile of hilly terrain at the default cells. Drop tiles with `mnavDropCachedTile` as they stream out; `mnavGetTileCacheBytes` gives what the cache holds. The cache lives in memory only.
- Obstacles that move are for [avoidance](crowds.md), not the tile cache. 2D bakes have no volumes and are not cached.
