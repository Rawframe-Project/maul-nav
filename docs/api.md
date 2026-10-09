# Maul Nav API reference

Generated from the public headers by `tools/gen_api.py`. The headers
are the source of truth; this file mirrors them.

## `base.h`

The base of the Maul Nav API: the library version, the export and attribute macros, the result codes every fallible function returns, the allocator and the vector types. Coordinates are meters in a right-handed frame with +Y up and -Z forward. World positions are doubles; positions within a frame the library names (a bake's origin, a tile) are binary32.

```c
mnavVersion mnavGetVersion(void);
```
Returns the version of the library that was linked, which may differ from the MNAV_VERSION macros a program was compiled with.  @return The library version. @par Thread safety Safe from any thread.

```c
uint64_t mnavHash64(uint64_t seed, const void* data, int32_t byteCount);
```
Hashes bytes into a 64-bit value, the hash every determinism check and navmesh fingerprint is built on: eight bytes a round, read in the host's byte order, xored in, multiplied by an odd constant and folded, the leftover bytes one at a time. Its constants are frozen.  @param seed       MNAV_HASH_INIT, or a hash to continue. @param data       The bytes. May be NULL when byteCount is 0. @param byteCount  The number of bytes, at least 0. @return The hash. @par Thread safety Safe from any thread.

```c
const char* mnavResultName(mnavResult result);
```
Returns the name of a result code, for diagnostics.  @param result  Any value; an unknown one is named as such. @return A static, NUL-terminated string such as "mnav_errorCapacity". @par Thread safety Safe from any thread.

## `avoidance.h`

Avoidance (mnav-0006): local steering among agents by velocity obstacles (ORCA), apart from the navmesh: nothing here uses a navmesh, and a host may use it without one. It works on the ground plane, a 3D world's (x, z) or a 2D world's (x, y), and for fliers in space, in meters and seconds.

```c
mnavAvoidanceDef mnavDefaultAvoidanceDef(void);
```
Returns the default avoidance def: up to 4096 agents, each avoiding its 10 nearest neighbours within 10 m, 2 s ahead, and up to 4096 obstacle points, each agent avoiding its 16 nearest obstacle edges or circles, 2 s ahead.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateAvoidance(const mnavAvoidanceDef* def, mnavAvoidance** avoidanceOut);
```
Makes an avoidance set with the memory its limits need.  @param def         The def, from mnavDefaultAvoidanceDef. @param avoidanceOut Receives the set, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def not from mnavDefaultAvoidanceDef; `mnav_errorRange` for a limit, distance or horizon out of its range; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyAvoidance(mnavAvoidance* avoidance);
```
Destroys an avoidance set.  @param avoidance The set, or NULL. @par Thread safety Safe from any thread; the set is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavAvoid(mnavAvoidance* avoidance, const mnavAgent* agents, int32_t agentCount, const mnavObstacle* obstacles, int32_t obstacleCount, double step, mnavPos2* velocitiesOut);
```
Finds each agent's new velocity: the one nearest its preferred velocity, no faster than its maximum speed, that avoids the obstacles within the obstacle horizon and colliding with its neighbours within the time horizon if they do their share (ORCA); when none does, the one that keeps clear of the obstacles and breaks the agents' constraints least. An agent sees an obstacle edge only from outside it. An agent held back from its preferred velocity aims 1% of its speed to the right of it, so that agents meeting in perfect symmetry pass rather than stop face to face. The same agents give the same velocities in any order, on every platform.  @param avoidance     The set. @param agents        The agents. @param agentCount    How many, at least 0. @param obstacles     The obstacles. @param obstacleCount How many, at least 0. @param step          The step the velocities are for, in seconds, at least MNAV_MIN_AVOIDANCE_TIME: agents already overlapping part within it. @param velocitiesOut Receives agentCount velocities, in the agents' order. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument with agents or obstacles, a negative count, a step not finite or shorter than MNAV_MIN_AVOIDANCE_TIME, an agent or obstacle with a value not finite or out of its range, a zero-length obstacle edge, a circle's radius not more than 0, another obstacle with a radius, or a polygon not counterclockwise; `mnav_errorLimit` for more agents or obstacle points than the set's limits. @par Thread safety Safe from any thread; the set is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavAvoid3D(mnavAvoidance* avoidance, const mnavAgent3D* agents, int32_t agentCount, const mnavSphere* spheres, int32_t sphereCount, double step, mnavPos3* velocitiesOut);
```
Finds each flier's new velocity, as mnavAvoid does on the ground: the one nearest its preferred velocity, no faster than its maximum speed, that avoids the spheres within the obstacle horizon and colliding with its neighbours within the time horizon if they do their share (ORCA in space); when none does, the one that keeps clear of the spheres and breaks the agents' constraints least. An agent held back from its preferred velocity aims 1% of its speed to the right of it, +Y being up; rising vertically it turns toward +X, falling toward -X. The same agents give the same velocities in any order, on every platform.  @param avoidance     The set. @param agents        The agents. @param agentCount    How many, at least 0. @param spheres       The spheres. @param sphereCount   How many, at least 0. @param step          The step the velocities are for, in seconds, at least MNAV_MIN_AVOIDANCE_TIME: agents already overlapping part within it. @param velocitiesOut Receives agentCount velocities, in the agents' order. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument with agents or spheres, a negative count, a step not finite or shorter than MNAV_MIN_AVOIDANCE_TIME, or an agent or sphere with a value not finite or out of its range; `mnav_errorLimit` for more agents than the set's limit or more spheres than its obstacle points. @par Thread safety Safe from any thread; the set is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugAvoidance(mnavAvoidance* avoidance, const mnavAgent* agents, int32_t agentCount, double height, mnavDebugBuffer* buffer);
```
Appends each agent's outline, a 16-gon (mnav_debugAgent), and a line to each neighbour mnavAvoid would give it (mnav_debugNeighbor), at a height, agent X and Y lying at ground X and Z.  @param avoidance  The set; its scratch is used. @param agents     The agents. @param agentCount How many, at least 0. @param height     The lines' height. @param buffer     The buffer appended to. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument with agents, an agent as mnavAvoid refuses it, a height not finite, or a buffer with a count out of range, an array missing or an origin not finite; `mnav_errorLimit` for more agents than the set's limit; `mnav_errorCapacity` when the buffer filled, its counts saying what the whole needs. @par Thread safety Safe from any thread; the set and the buffer are used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugAvoidance3D(mnavAvoidance* avoidance, const mnavAgent3D* agents, int32_t agentCount, const mnavSphere* spheres, int32_t sphereCount, mnavDebugBuffer* buffer);
```
Appends each flier's outline, three 16-gons on its radius, one in each axis plane (mnav_debugAgent), a line to each neighbour mnavAvoid3D would give it (mnav_debugNeighbor), and each sphere's three rings (mnav_debugObstacle).  @param avoidance   The set; its scratch is used. @param agents      The agents. @param agentCount  How many, at least 0. @param spheres     The spheres. @param sphereCount How many, at least 0. @param buffer      The buffer appended to. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument with agents or spheres, a negative count, an agent or sphere as mnavAvoid3D refuses it, or a buffer with a count out of range, an array missing or an origin not finite; `mnav_errorLimit` for more agents or spheres than mnavAvoid3D takes; `mnav_errorCapacity` when the buffer filled, its counts saying what the whole needs. @par Thread safety Safe from any thread; the set and the buffer are used by one thread at a time.

## `bake.h`

The bake's settings and its input surface: the def a bake runs with, the agent profile it bakes for, the named limits that bound its work, and the triangle meshes a host hands it, each checked before any work (mnav-0002).

```c
MNAV_NODISCARD MNAV_API mnavInputResult mnavValidateOutline(const mnavBakeDef* def, const mnavOutline* outline);
```
Checks a 2D outline as hostile input to a bake with a def.  @param def     The bake def the outline is for. @param outline The outline. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, an invalid def, fewer than 3 points, an area of MNAV_AREA_TYPES or more, or a point that is not finite (the point); `mnav_errorRange` for a point past the extent input may have (the point); `mnav_errorLimit` for more points than the inputTriangles limit. @par Thread safety Safe from any thread.

```c
mnavBakeDef mnavDefaultBakeDef(void);
```
Returns the default bake def: cells of 0.25 m by 0.125 m, tiles of 128 cells, an agent 0.5 m in radius and 2 m tall that steps 0.75 m and walks slopes up to 45 degrees, regions of at least 2 square meters, walls within 0.3 m of the cells and at most 12 m long, detail samples every 1.5 m within 0.125 m, and limits sized for a large level.  @return The def, with a valid cookie. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavBakeDefResult mnavValidateBakeDef(const mnavBakeDef* def, mnavBakeCells* cellsOut);
```
Checks a bake def and converts its meters to cells.  @param def       The def to check. @param cellsOut  Receives the settings in cells when the def is valid. May be NULL. @return `mnav_success`; `mnav_errorInvalid` with the setting for a NULL def, a def without its cookie, an allocator with one function, a value that is not finite or lies outside its range, an agent radius whose border is wider than a tile, or an agent height or step height past MNAV_MAX_HEIGHT_CELLS. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavInputResult mnavValidateTriangleMesh(const mnavBakeDef* def, const mnavTriangleMesh* mesh);
```
Checks a triangle mesh as hostile input to a bake with a def.  @param def   The bake def the mesh is for. @param mesh  The mesh. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, an invalid def, a negative count, a NULL array its count needs, a vertex coordinate that is not finite, a vertex index outside the vertices or an area type of MNAV_AREA_TYPES or more; `mnav_errorLimit` for more triangles than the def's inputTriangles limit; `mnav_errorRange` for a vertex more than MNAV_MAX_EXTENT_CELLS cells from the origin on the ground or MNAV_MAX_HEIGHT_CELLS cell heights above or below it. The element and index name the first offending vertex or triangle. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavBakeDefResult mnavCreateBaker(const mnavBakeDef* def, mnavBaker** bakerOut);
```
Makes a baker from a def: checks the def and keeps a copy, its allocator and its memory limit.  @param def       The def. @param bakerOut  Receives the baker, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` with the setting for an invalid def or a NULL argument; `mnav_errorLimit` when the baker does not fit the def's memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyBaker(mnavBaker* baker);
```
Destroys a baker and the tile it holds.  @param baker  The baker, or NULL. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBakeTileInput(mnavBaker* baker, const mnavBakeInput* input, int32_t tileX, int32_t tileZ, mnavBakeReport* reportOut);
```
Bakes one tile from triangle meshes and terrains, shaped by volumes, as mnavBakeTile does from meshes alone; a terrain's triangles count as input triangles, after the meshes', toward the limits and the fingerprint, and a volume's points count as input triangles too. A refused terrain is reported as the mesh at its index after the meshes, a refused volume as the one at its index after the terrains. Given a tile index, the bake checks each mesh's counts and pointers and then only the triangles the index lists for the tile, with their vertices, before reading them.  @param baker     The baker; the tile it held before is dropped. @param input     The meshes and terrains. @param tileX     The tile's column. @param tileZ     The tile's row. @param reportOut Receives the report. May be NULL. @return As mnavBakeTile; a terrain with a side out of range, a spacing not more than 0 or not finite, a height not finite (the sample) or an area of MNAV_AREA_TYPES or more (the cell) is `mnav_errorInvalid`, a sample past the extent input may have `mnav_errorRange`; a volume with fewer than 3 points, a kind out of range, an area volume's area not walkable, heights not finite or backward, or a point not finite (the point) is `mnav_errorInvalid`, a point past the extent (the point) `mnav_errorRange`; a tile index made for another grid, another number of meshes or another number of triangles in a mesh is `mnav_errorInvalid`. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateTileIndex(const mnavBakeDef* def, const mnavTriangleMesh* meshes, int32_t meshCount, mnavTileIndex** indexOut, mnavBakeReport* reportOut);
```
Makes a tile index of meshes for a def's tile grid (mnav-0014): checks every mesh as a bake does, then lists for each tile the triangles whose ground bounds, widened by the tile's border and two cells, reach it, in input order. A bake given the index through mnavBakeInput reads only the tile's list and makes the same tile, to the byte, as a bake that reads every triangle. Changing a mesh's vertices or triangles calls for a new index.  @param def       The def; the index fits defs with its cell size, tile cells and agent radius. @param meshes    The meshes, in the def's frame. Only read during the call. @param meshCount The number of meshes, at least 0. @param indexOut  Receives the index, or NULL on failure. @param reportOut Receives the result, the first mesh refused and what its check found, and the memory peak; the other fields are 0. May be NULL. @return `mnav_success`; `mnav_errorInvalid` for an invalid def, a NULL indexOut, a negative count, NULL meshes with a positive count, or a mesh its check refuses; `mnav_errorRange` for a mesh past the extent; `mnav_errorLimit` past the def's memory limit or past 2^31 - 1 listed triangles; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread. An index is never changed after it is made, so any number of bakes on any threads may read it at once.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateTileIndex2D(const mnavBakeDef* def, const mnavOutline* outlines, int32_t outlineCount, mnavTileIndex** indexOut, mnavBakeReport* reportOut);
```
Makes a tile index of 2D outlines for a def's tile grid (mnav-0014), as mnavCreateTileIndex does of meshes: checks every outline as a 2D bake does, then lists for each tile the outlines whose bounds, widened by the tile's border and two cells, reach it, in input order. A bake given the index through mnavBake2DInput reads only the tile's list and makes the same tile, to the byte.  @param def          The def; the index fits defs with its cell size, tile cells and agent radius. @param outlines     The outlines, in the def's frame. Only read during the call. @param outlineCount The number of outlines, at least 0. @param indexOut     Receives the index, or NULL on failure. @param reportOut    Receives the result, the first outline refused (in the report's mesh) and what its check found, and the memory peak; the other fields are 0. May be NULL. @return As mnavCreateTileIndex, for outlines. @par Thread safety Safe from any thread. An index is never changed after it is made, so any number of bakes on any threads may read it at once.

```c
void mnavDestroyTileIndex(mnavTileIndex* index);
```
Destroys a tile index.  @param index  The index, or NULL. @par Thread safety Safe from any thread; the index is used by one thread at a time. No bake may be reading it.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBakeTile(mnavBaker* baker, const mnavTriangleMesh* meshes, int32_t meshCount, int32_t tileX, int32_t tileZ, mnavBakeReport* reportOut);
```
Bakes tile (tileX, tileZ) of the meshes: checks every mesh, then rasterizes, filters, partitions, traces and triangulates the tile and lays its detail, and keeps the tile's bytes for mnavCopyBakedTile, in place of any tile baked before. The same meshes and def give the same bytes on every platform.  @param baker      The baker. @param meshes     The input meshes, in the def's frame. @param meshCount  The number of meshes, at least 0. @param tileX      The tile's column in the grid from the origin. @param tileZ      The tile's row. @param reportOut  Receives what the bake did. May be NULL. @return `mnav_success`; `mnav_errorInvalid` for a NULL baker, a negative count, NULL meshes with a positive count, or a mesh its check refuses (the report names it); `mnav_errorRange` for a mesh past the extent or a tile past it; `mnav_errorLimit` when a named limit or the memory limit is reached (the report's stage says where); `mnav_errorCapacity` when the allocator fails. On failure the baker holds no tile. @par Thread safety Safe from any thread; the baker is used by one thread at a time. Bakers share nothing, so one per thread bakes tiles in parallel.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBakeTile2D(mnavBaker* baker, const mnavOutline* outlines, int32_t outlineCount, int32_t tileX, int32_t tileZ, mnavBakeReport* reportOut);
```
Bakes tile (tileX, tileZ) of 2D outlines (mnav-0002): checks every outline, fills the cells whose centers a walkable outline holds and no obstruction does as a flat floor at height 0, with the highest area among the outlines holding them, then erodes, partitions, traces and triangulates as mnavBakeTile does; the walkable filters, which judge heights, do not run. The report's mesh names a refused outline and its triangles count the outlines reaching the tile.  @param baker        The baker. @param outlines     The input outlines, in the def's frame. @param outlineCount The number of outlines, at least 0. @param tileX        The tile's column in the grid from the origin. @param tileZ        The tile's row. @param reportOut    Receives what the bake did. May be NULL. @return As mnavBakeTile, for outlines; also `mnav_errorLimit` for more points in all than the inputTriangles limit, or more outlines reaching the tile than the tileTriangles limit. @par Thread safety Safe from any thread; the baker is used by one thread at a time. Bakers share nothing, so one per thread bakes tiles in parallel.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBakeTile2DInput(mnavBaker* baker, const mnavBake2DInput* input, int32_t tileX, int32_t tileZ, mnavBakeReport* reportOut);
```
Bakes tile (tileX, tileZ) of 2D outlines as mnavBakeTile2D does, from an input record. Given a tile index of the outlines, the bake checks only the outlines it lists for the tile, which hold every outline reaching it, and makes the same tile to the byte; the total of the points still counts every outline.  @param baker     The baker; the tile it held before is dropped. @param input     The outlines and the index, or NULL. @param tileX     The tile's column. @param tileZ     The tile's row. @param reportOut Receives the report. May be NULL. @return As mnavBakeTile2D; a NULL input is `mnav_errorInvalid`, and so is a tile index made for another grid, for meshes, or for another number of outlines or of points in one. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCopyBakedTile(const mnavBaker* baker, uint8_t* buffer, size_t capacity, size_t* sizeOut);
```
Copies the last baked tile's bytes into caller memory.  @param baker     The baker. @param buffer    The memory, at least capacity bytes; may be NULL when capacity is 0. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the tile's size in bytes. May be NULL. @return `mnav_success`; `mnav_errorCapacity` when the buffer is smaller than the tile, with the size still written; `mnav_errorInvalid` for a NULL baker, a NULL buffer with a positive capacity, or a baker holding no tile. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

## `debug.h`

Debug geometry of navmeshes, links, paths and corridors (mnav-0010), as plain vertex and index data in a caller's buffer.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugNavmesh(const mnavNavmesh* navmesh, int32_t tileX0, int32_t tileZ0, int32_t tileX1, int32_t tileZ1, mnavDebugBuffer* buffer);
```
Appends the committed tiles on a range of places: each polygon's detail triangles (mnav_debugPolygon, its area), its edges as lines (inner edges once, walls, tile sides) and each tile's bounds on the ground at its lowest detail height.  @param navmesh The navmesh. @param tileX0  The first tile column. @param tileZ0  The first tile row. @param tileX1  The last tile column, at least tileX0. @param tileZ1  The last tile row, at least tileZ0. @param buffer  The buffer appended to. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a range backward, or a buffer with a count out of range, an array missing or an origin not finite; `mnav_errorCapacity` when the buffer filled, its counts saying what the whole needs. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the buffer is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugLinks(const mnavNavmesh* navmesh, mnavDebugBuffer* buffer);
```
Appends every committed off-mesh link as an arc of eight lines from its start to its end, rising a quarter of its length: at its snapped points when attached, its defined ones otherwise.  @param navmesh The navmesh. @param buffer  The buffer appended to. @return As mnavDebugNavmesh. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the buffer is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugPath(const mnavPath* path, mnavDebugBuffer* buffer);
```
Appends a path's straight line, point to point.  @param path   The path. @param buffer The buffer appended to. @return As mnavDebugNavmesh. @par Thread safety Safe from any thread; the buffer is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugCorridor(const mnavNavmesh* navmesh, const mnavPolygonId* polygons, int32_t count, mnavDebugBuffer* buffer);
```
Appends the detail triangles of a run of polygons, a path's or a corridor's, each tagged with its place in the run; polygons whose tile has gone are skipped.  @param navmesh  The navmesh. @param polygons The polygons. @param count    How many, at least 0. @param buffer   The buffer appended to. @return As mnavDebugNavmesh, and `mnav_errorInvalid` for a polygon id never handed out. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the buffer is used by one thread at a time.

## `flight.h`

Flight volumes (mnav-0015): where agents that fly or swim may go, baked per tile from the navmesh bake's input into a compact sparse voxel octree, loaded from bytes, staged and committed together as navmesh tiles are.

```c
mnavFlightDef mnavDefaultFlightDef(void);
```
Makes a flight def with every setting at its default: 1 m voxels, tiles of 32 voxels, a volume from 0 to 64 m, a flier of 0.5 m, the ground below solid, and limits for a medium world.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavFlightDefResult mnavValidateFlightDef(const mnavFlightDef* def);
```
Checks a flight def.  @param def  The def. @return `mnav_success`; `mnav_errorInvalid` with the first setting refused, or for a NULL def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavFlightDefResult mnavCreateFlightBaker(const mnavFlightDef* def, mnavFlightBaker** bakerOut);
```
Makes a flight baker: checks a def and keeps a copy, its allocator and its memory limit.  @param def       The def. @param bakerOut  Receives the baker, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` with the setting for an invalid def or a NULL argument; `mnav_errorLimit` when the baker does not fit the def's memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyFlightBaker(mnavFlightBaker* baker);
```
Destroys a flight baker and the tile it holds.  @param baker  The baker, or NULL. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBakeFlightTile(mnavFlightBaker* baker, const mnavBakeInput* input, int32_t tileX, int32_t tileZ, mnavFlightBakeReport* reportOut);
```
Bakes one flight tile from the input a navmesh bake reads (mnav-0015): every triangle is rasterized at the voxel size, and a voxel any triangle touches is solid; with groundBelow, so is each column below its lowest solid voxel; then every voxel within the flier's radius of a solid one, by whole voxels, is blocked too. A tile index in the input is not used: it lists triangles for a navmesh's tiles. The tile's bytes are the same on every platform.  @param baker     The baker; the tile it held before is dropped. @param input     The meshes, terrains and volumes, as mnavBakeTileInput reads them. @param tileX     The tile's column. @param tileZ     The tile's row. @param reportOut Receives the report. May be NULL. @return `mnav_success`; `mnav_errorInvalid` for a NULL baker or input or input mnavBakeTileInput refuses, the mesh named in the report; `mnav_errorRange` for input or a tile past the extent; `mnav_errorLimit` past a limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCopyFlightTile(const mnavFlightBaker* baker, uint8_t* buffer, size_t capacity, size_t* sizeOut);
```
Copies the last baked flight tile's bytes into caller memory.  @param baker     The baker. @param buffer    The memory, at least capacity bytes; may be NULL when capacity is 0. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the tile's size in bytes. May be NULL. @return `mnav_success`; `mnav_errorCapacity` when the buffer is smaller than the tile, with the size still written; `mnav_errorInvalid` for a NULL baker, a NULL buffer with a positive capacity, or a baker holding no tile. @par Thread safety Safe from any thread; the baker is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavFlightDefResult mnavCreateFlightVolume(const mnavFlightDef* def, mnavFlightVolume** volumeOut);
```
Makes an empty flight volume for tiles baked with a def: checks the def and keeps a copy, its allocator and its limits.  @param def        The def the tiles are baked with. @param volumeOut  Receives the volume, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` with the setting for an invalid def or a NULL argument; `mnav_errorLimit` when the volume does not fit the def's memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyFlightVolume(mnavFlightVolume* volume);
```
Destroys a flight volume, its tiles and everything staged.  @param volume  The volume, or NULL. @par Thread safety Safe from any thread; the volume is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavTileResult mnavStageFlightTile(mnavFlightVolume* volume, const uint8_t* bytes, size_t size);
```
Loads flight tile bytes, checking every field as hostile input and the tile's settings against the volume's def, and stages the tile for the next commit, in place of anything staged at its place before. Queries do not see it until the commit.  @param volume  The volume. @param bytes   The tile's bytes, as mnavCopyFlightTile gives them. @param size    Their size in bytes. @return `mnav_success`; `mnav_errorInvalid` naming the section and element for malformed bytes, a tile baked with other settings (the header) or a NULL argument; `mnav_errorVersion` for another flight tile format version; `mnav_errorLimit` for a tile past the def's node or leaf limit (the header) or past the memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the volume is used by one thread at a time. Staging changes nothing queries read, but it may not run beside another call that stages or commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageFlightTileRemoval(mnavFlightVolume* volume, int32_t tileX, int32_t tileZ);
```
Stages the removal of the flight tile at a place for the next commit, in place of anything staged there before.  @param volume  The volume. @param tileX   The tile's column. @param tileZ   The tile's row. @return `mnav_success`; `mnav_errorInvalid` for a NULL volume; `mnav_errorLimit` past the memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the volume is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCommitFlight(mnavFlightVolume* volume);
```
Applies everything staged at once: installs and removes the tiles. Either all of it applies or, on failure, nothing does and the staged changes stay.  @param volume  The volume. @return `mnav_success`; `mnav_errorInvalid` for a NULL volume; `mnav_errorLimit` when the tiles would pass the tiles limit or memory its limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the volume is used by one thread at a time, so no query may run during a commit.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGetFlightTile(const mnavFlightVolume* volume, int32_t tileX, int32_t tileZ, uint64_t* fingerprintOut);
```
Reads the fingerprint of the flight tile committed at a place, as its bake reported it, so that a host can tell whether its input has changed since.  @param volume          The volume. @param tileX           The tile's column. @param tileZ           The tile's row. @param fingerprintOut  Receives the fingerprint, or 0. @return `mnav_success`; `mnav_errorNotLoaded` when no tile is committed there; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread. Any number of threads may read the volume at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavIsFlightOpen(const mnavFlightVolume* volume, mnavPos3 point, bool* openOut);
```
Tells whether the flier's center may be at a point: whether the voxel holding it is neither solid nor within the flier's radius of a solid voxel.  @param volume   The volume. @param point    The point, in world coordinates. @param openOut  Receives whether the point is open; false on failure. @return `mnav_success`; `mnav_errorNotLoaded` when no tile is committed under the point; `mnav_errorRange` for a point not finite or below or above the volume; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread. Any number of threads may read the volume at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindFlightPath(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 start, mnavPos3 end, mnavFlightPath* pathOut);
```
Finds a flier's path between two points (mnav-0015): Lazy Theta* (Nash, Koenig and Tovey, 2010) over the open blocks of the committed tiles, linked through shared faces, each block entered at the point of its face nearest the point the path comes from. A point is in sight of another when the segment between them crosses no blocked voxel, nor an edge or corner of one. The search keeps to the query context's node budget and path length limit; short of the end, the path runs to the point searched nearest it.  @param query    The query context; its last path is dropped. @param volume   The flight volume. @param start    The start, in world coordinates. @param end      The end, in world coordinates. @param pathOut  Receives the path. A start or end not open ends the search at once: `mnav_pathNone` for a blocked voxel, `mnav_pathNotLoaded` for a place with no tile, with the start alone as its path. @return `mnav_success`; `mnav_errorRange` for a point not finite, below or above the volume or past the extent; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread; the context is used by one thread at a time, and the volume is not changed during the search.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBeginFlightPath(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 start, mnavPos3 end);
```
Begins a flier's path search to run in slices: the same search as mnavFindFlightPath, continued by mnavContinueFlightPath and ended by mnavFinishFlightPath. Whatever the slices, the path is the one mnavFindFlightPath gives.  @param query   The query context; it holds the search until the next search of any kind begins. @param volume  The flight volume. @param start   The start, in world coordinates. @param end     The end, in world coordinates. @return `mnav_success`; `mnav_errorRange` for a point not finite, below or above the volume or past the extent; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavContinueFlightPath(mnavQuery* query, const mnavFlightVolume* volume, int32_t nodes, bool* endedOut);
```
Continues a search begun by mnavBeginFlightPath: closes up to a number of nodes, fewer when the search ends first.  @param query    The query context. @param volume   The volume the search began on. @param nodes    The most nodes to close, at least 1. @param endedOut Receives whether the search has ended. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, no flight search begun, or fewer than 1 node; `mnav_errorStale` for another volume, or one committed to since the search began: begin again. @par Thread safety Safe from any thread; the context is used by one thread at a time, and the volume is not changed during the call.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFinishFlightPath(mnavQuery* query, const mnavFlightVolume* volume, mnavFlightPath* pathOut);
```
Ends a search begun by mnavBeginFlightPath and writes its path: as mnavFindFlightPath's when it has ended; otherwise to the point searched nearest the end, its end mnav_pathUnfinished.  @param query    The query context; its memory holds the path. @param volume   The volume the search began on. @param pathOut  Receives the path. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or no flight search begun; `mnav_errorStale` for another volume, or one committed to since the search began. @par Thread safety Safe from any thread; the context is used by one thread at a time, and the volume is not changed during the call.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFlightRaycast(const mnavFlightVolume* volume, mnavPos3 from, mnavPos3 to, mnavFlightHit* hitOut);
```
Casts a flier's ray: walks the voxels the segment crosses, and where it passes an edge or a corner every voxel it touches, and stops at the first one the flier may not enter or with no tile.  @param volume  The flight volume. @param from    The ray's start, in world coordinates. @param to      Its end. @param hitOut  Receives what stopped it, where and at what fraction. @return `mnav_success`; `mnav_errorRange` for a point not finite or past the extent; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread. Any number of threads may read the volume at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindNearestFlightPoint(const mnavFlightVolume* volume, mnavPos3 point, float radius, mnavPos3* nearestOut, bool* foundOut);
```
Finds the nearest point open to the flier's center within a radius of a point: the point itself when it is open, or else a point just inside the nearest open voxel, at most a thousandth of a voxel in from its face. Tiles not loaded are not searched.  @param volume      The flight volume. @param point       The point, in world coordinates. @param radius      How far to look, in meters, at least 0. @param nearestOut  Receives the nearest open point. @param foundOut    Receives whether one lies within the radius. @return `mnav_success`; `mnav_errorRange` for a point not finite or past the extent, or a radius below 0 or not finite; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread. Any number of threads may read the volume at once between commits; none may while a stage or commit call runs.

## `flow.h`

Flow fields (mnav-0007): for every cell of a grid, the cost of its cheapest way to the nearest of a set of goals and the next cell on it, so that any number of agents sharing the goals find their way at the cost of one search.

```c
mnavFlowFieldDef mnavDefaultFlowFieldDef(void);
```
Returns the default flow field def: up to 65536 cells.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateFlowField(const mnavFlowFieldDef* def, mnavFlowField** fieldOut);
```
Makes a flow field with the memory its cell limit needs.  @param def      The def, from mnavDefaultFlowFieldDef. @param fieldOut Receives the field, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def not from mnavDefaultFlowFieldDef; `mnav_errorRange` for a cell limit out of its range; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyFlowField(mnavFlowField* field);
```
Destroys a flow field.  @param field The field, or NULL. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBeginFlowField(mnavFlowField* field, const mnavGrid* grid, const mnavQueryFilter* filter, const mnavFlowRegion* region, const mnavCell* goals, int32_t goalCount);
```
Begins building the field for a region of a grid and a set of goal cells (mnav-0007): one search from all the goals, with the steps of mnavFindGridPath, to the 8 neighbours, never across a blocked corner, a step costing its length times the mean of its two cells' area costs. Cells outside the region are as blocked. A cell's way to the goals is a cheapest one, ties going to the neighbour found first by cost, then by index; the same grid, region and goals give the same field on every platform, however the work is divided. Blocked goals and goals outside the region are left out. mnavContinueFlowField does the work.  @param field     The field; its last field is dropped. @param grid      The grid; its areas are read until the work ends. @param filter    The areas usable and their costs, or NULL; copied. @param region    The region, or NULL for the whole grid. @param goals     The goal cells, in grid places. @param goalCount How many, at least 0. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a negative count, a grid with no areas, a side out of range or a cell size not more than 0 or not finite, a region empty or not within the grid, a goal outside the grid, or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorLimit` for a region of more cells than the field's limit. On an error the field holds nothing. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavContinueFlowField(mnavFlowField* field, const mnavGrid* grid, int32_t cells, bool* endedOut);
```
Continues the work begun on a field: settles up to a number of cells, fewer when the work ends first.  @param field    The field. @param grid     The grid the work began on, its areas unchanged; it may lie elsewhere in memory. @param cells    The most cells to settle, at least 1. @param endedOut Receives whether the work has ended. May be NULL. @return `mnav_success`, also when the work had already ended; `mnav_errorInvalid` for a NULL field or grid, no work begun, fewer than 1 cell, or a grid of another size or cell size. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavUpdateFlowField(mnavFlowField* field, const mnavGrid* grid, const mnavCell* goals, int32_t goalCount, const mnavCell* changed, int32_t changedCount);
```
Begins repairing the field after its goals or the areas of some of its grid's cells changed (mnav-0007): only the cells whose ways change are searched again, and when the work ends the field is the one mnavBeginFlowField would make with the new goals and areas, bit for bit. The cells whose ways ran through what changed are reset in this call; mnavContinueFlowField does the rest by its budget.  @param field        The field; its work must have ended. @param grid         The grid the field was begun on, of the same size and cell size, its areas already changed; read until the work ends. @param goals        The goal cells now, in grid places. @param goalCount    How many, at least 0. @param changed      The cells whose areas changed since the field was last begun or repaired; each once or more, those outside the region ignored. @param changedCount How many, at least 0. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument with a count, a negative count, nothing begun, a grid of another size or cell size or with no areas, or a goal or changed cell outside the grid; `mnav_errorStale` while work on the field has not ended. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBuildFlowField(mnavFlowField* field, const mnavGrid* grid, const mnavQueryFilter* filter, const mnavCell* goals, int32_t goalCount);
```
Builds the field for a whole grid and a set of goal cells: begins as mnavBeginFlowField and continues to the end.  @param field     The field; its last field is replaced. @param grid      The grid, read only during the call. @param filter    The areas usable and their costs, or NULL. @param goals     The goal cells. @param goalCount How many, at least 0. @return As mnavBeginFlowField. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFlowAt(const mnavFlowField* field, mnavCell cell, mnavFlow* flowOut);
```
Reads a cell's way to the goals from the field.  @param field   The field. @param cell    The cell, in grid places. @param flowOut Receives the cell's way, its next cell in grid places. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a cell outside the field's region, or nothing begun; `mnav_errorStale` while work begun on the field has not ended. @par Thread safety Safe from any thread. Any number of threads may read a field at once while no work runs on it.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugFlowField(const mnavFlowField* field, double height, mnavDebugBuffer* buffer);
```
Appends an arrow for each cell of the field that has a next cell (mnav_debugFlow): from the cell's center toward the next one's, 0.8 of a cell long, with two barbs, at a height, grid cell (x, y) lying at ground X and Z by the grid's cell size.  @param field  The field. @param height The arrows' height. @param buffer The buffer appended to. @return `mnav_success`, appending nothing when nothing was begun; `mnav_errorInvalid` for a NULL argument, a height not finite, or a buffer with a count out of range, an array missing or an origin not finite; `mnav_errorStale` while work begun on the field has not ended; `mnav_errorCapacity` when the buffer filled, its counts saying what the whole needs. @par Thread safety Safe from any thread. Any number of threads may read a field at once while no work runs on it; the buffer is used by one thread at a time.

## `hierarchy.h`

Hierarchical paths (mnav-0008): an abstract graph over clusters of tiles, whose transitions stand at the portals between clusters, for long paths on large navmeshes; a path found on it is refined by the navmesh search confined to the clusters it crosses.

```c
mnavHierarchyDef mnavDefaultHierarchyDef(void);
```
Returns the default hierarchy def: clusters of 4 by 4 tiles, up to 4,096 tile slots, 16,384 transitions and 262,144 edges.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateHierarchy(const mnavHierarchyDef* def, mnavHierarchy** hierarchyOut);
```
Makes a hierarchy with the memory its limits need.  @param def          The def, from mnavDefaultHierarchyDef. @param hierarchyOut Receives the hierarchy, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def not from mnavDefaultHierarchyDef; `mnav_errorRange` for a limit or cluster side out of its range; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyHierarchy(mnavHierarchy* hierarchy);
```
Destroys a hierarchy.  @param hierarchy The hierarchy, or NULL. @par Thread safety Safe from any thread; the hierarchy is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBuildHierarchy(mnavHierarchy* hierarchy, mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavHierarchyReport* reportOut);
```
Builds the hierarchy for a navmesh as committed and a filter. Each run of tile links along a cluster border, one way, is an entrance whose middle link is a transition; each transition's edges to the transitions leaving the cluster it enters cost what the navmesh search finds within that cluster. Transitions are numbered by tile place, side and position, so the graph does not depend on the order tiles were loaded in. Each off-mesh link, one way, that the filter crosses from one cluster into another is a transition too. A later commit to the navmesh makes the hierarchy stale until it is built again.  @param hierarchy The hierarchy; its last graph is replaced. @param query     A context for the searches within clusters; its last search ends. @param navmesh   The navmesh. @param filter    The areas usable and their costs, or NULL; the hierarchy keeps a copy. @param reportOut Receives what was made. May be NULL. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorLimit` for more tile slots, transitions or edges than the limits, or a cluster needing more nodes than the context's limit. On an error the hierarchy holds no graph. @par Thread safety Safe from any thread; the hierarchy and the context are used by one thread at a time, and no commit runs on the navmesh.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavUpdateHierarchy(mnavHierarchy* hierarchy, mnavQuery* query, const mnavNavmesh* navmesh, mnavHierarchyReport* reportOut);
```
Brings the hierarchy up to the navmesh's last commit. When only polygon areas changed, only the edges of transitions entering the clusters whose tiles changed, and the clusters with transitions out into those tiles, are searched again; when tiles or off-mesh links changed, the hierarchy is built again. The graph is the one a build would make.  @param hierarchy The hierarchy, built for this navmesh. @param query     A context for the searches within clusters; its last search ends. @param navmesh   The navmesh. @param reportOut Receives what the graph holds and the searches run. May be NULL. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a hierarchy with no graph, or one built for another navmesh; `mnav_errorLimit` as mnavBuildHierarchy. On an error the hierarchy holds no graph. @par Thread safety Safe from any thread; the hierarchy and the context are used by one thread at a time, and no commit runs on the navmesh.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindHierarchicalPath(mnavQuery* query, mnavHierarchy* hierarchy, const mnavNavmesh* navmesh, mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end, mnavPath* pathOut);
```
Finds a path as mnavFindPath does, with the hierarchy's filter, through the hierarchy: the start and the end join their clusters' transitions by searches within those clusters, A* over the transitions picks the clusters to cross, and the navmesh search confined to them gives the path. Paths within one cluster, and ends the graph does not reach, take the plain search. Paths are near the cheapest, not always the cheapest. Where routes through the clusters cost the same in the graph, it takes one, the same on every platform, but which one is not promised; the paths they refine to may differ a little in cost.  @param query        The context; its memory holds the path. @param hierarchy    The hierarchy, built for this navmesh. @param navmesh      The navmesh. @param startPolygon The polygon the start lies in. @param start        The start point. @param endPolygon   The polygon the end lies in. @param end          The end point. @param pathOut      Receives the path. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a point not finite, a hierarchy with no graph, or one built for another navmesh; `mnav_errorStale` for a navmesh committed since the build; the errors of mnavFindPath otherwise. @par Thread safety Safe from any thread; the context and the hierarchy are used by one thread at a time.

## `linkgen.h`

Link generation (mnav-0009): off-mesh links for drops off ledges and jumps across gaps, made from a committed navmesh for the host to stage.

```c
mnavLinkGenDef mnavDefaultLinkGenDef(void);
```
Returns the default generation def: samples every 1 m, drops up to 3 m going both ways up to 0.5 m, jumps up to 2 m, links dropped when walking is within 3 times as far or their ends within 1 m of another's, a snap radius of 0.5 m, costs 2 and 4, kinds mnav_linkDrop and mnav_linkJump, and no clearance test.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGenerateLinks(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, const mnavLinkGenDef* def, int32_t tileX0, int32_t tileZ0, int32_t tileX1, int32_t tileZ1, mnavLinkDef* linksOut, int32_t capacity, int32_t* countOut);
```
Generates links from the edges of the polygons on a range of tile places that have nothing across them and are not tile sides. Each edge is sampled every `spacing` meters; a sample tries a drop, landing `2 * radius + 4 * cellSize` out from the edge on the highest surface between the agent's step height and `dropMax` below, and a jump, landing from `2 * radius` to `jumpMax` out at the nearest distance where a surface lies within the step height of the start, the radius, step height and cell size being the navmesh's. A link is kept when the clearance test, if any, passes it, when the navmesh does not already walk from its start to its landing within `detour` times the straight distance, and when no earlier link has both ends within `filterDistance` of its own. The order is tile place, polygon, edge, sample; the same navmesh gives the same links.  @param query     A context for the walking searches; its last search ends. @param navmesh   The navmesh. @param filter    The areas links may start and land on, and walk through, or NULL. @param def       The def, from mnavDefaultLinkGenDef. @param tileX0    The first tile column. @param tileZ0    The first tile row. @param tileX1    The last tile column, at least tileX0. @param tileZ1    The last tile row, at least tileZ0. @param linksOut  Receives the links, up to capacity. @param capacity  The buffer's size in links, at least 0. @param countOut  Receives the number of links generated, also beyond the capacity. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a def not from mnavDefaultLinkGenDef, a tile range backward, or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a def value or filter cost out of its range; `mnav_errorCapacity` when more links were generated than the buffer holds, the first capacity written. @par Thread safety Safe from any thread; the context is used by one thread at a time, and no commit runs on the navmesh. The def's clearance test runs on the calling thread, within the call.

## `navflow.h`

Flow fields over the navmesh (mnav-0013): for every polygon, the cost of its cheapest way to the nearest of a set of goal points and the polygon to go to next, so that any number of agents sharing the goals find their way at the cost of one search.

```c
mnavNavFlowDef mnavDefaultNavFlowDef(void);
```
Returns the default navmesh flow field def: up to 65,536 polygons, 4,096 tile slots and 4,096 off-mesh links.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateNavFlow(const mnavNavFlowDef* def, mnavNavFlow** fieldOut);
```
Makes a navmesh flow field with the memory its limits need.  @param def      The def, from mnavDefaultNavFlowDef. @param fieldOut Receives the field, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def not from mnavDefaultNavFlowDef; `mnav_errorRange` for a limit out of its range; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyNavFlow(mnavNavFlow* field);
```
Destroys a navmesh flow field.  @param field The field, or NULL. @par Thread safety Safe from any thread; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBeginNavFlow( mnavNavFlow* field, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, const mnavNavFlowRegion* region, const mnavNavFlowGoal* goals, int32_t goalCount);
```
Begins building the field over the tiles of a region, as mnavBuildNavFlow builds it over all; polygons outside the region are as left out, and only those inside count toward the field's limit. mnavContinueNavFlow does the work; however it is divided, the field is the same.  @param field     The field; its last field is dropped. @param navmesh   The navmesh; the work needs it unchanged. @param filter    The areas usable and their costs, and the link kinds, or NULL; copied. @param region    The tiles searched, or NULL for all. @param goals     The goals; those outside the region are left out. @param goalCount How many, at least 0. @return As mnavBuildNavFlow; also `mnav_errorInvalid` for a region with x0 past x1 or z0 past z1. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavContinueNavFlow(mnavNavFlow* field, const mnavNavmesh* navmesh, int32_t polygons, bool* endedOut);
```
Continues the work begun on a field: settles up to a number of polygons, fewer when the work ends first.  @param field    The field. @param navmesh  The navmesh the work began on. @param polygons The most polygons to settle, at least 1. @param endedOut Receives whether the work has ended. May be NULL. @return `mnav_success`, also when the work had already ended; `mnav_errorInvalid` for a NULL field or navmesh, nothing begun, or fewer than 1 polygon; `mnav_errorStale` for another navmesh or one committed to since the work began: begin again. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBuildNavFlow(mnavNavFlow* field, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, const mnavNavFlowGoal* goals, int32_t goalCount);
```
Builds the field for a navmesh and a set of goal points by one search backward from all of them, as path searches price their ways (mnav-0005): a polygon stands at the midpoint of the portal it is left by toward the goals, and its cost is the next polygon's plus the walk between them times the next polygon's area cost, with an off-mesh link's cost where the way crosses one. Ties go to the lower cost, then the polygon of the lower slot and index; the same navmesh and goals give the same field on every platform. A polygon with two ways on of equal cost keeps one of them, the same on every platform, but which one is not promised. Goals on polygons the filter leaves out are left out.  @param field     The field; its last field is replaced. @param navmesh   The navmesh; reads need it unchanged since. @param filter    The areas usable and their costs, and the link kinds, or NULL; copied. @param goals     The goals. @param goalCount How many, at least 0. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a negative count, a goal polygon id never handed out or a goal point not finite, or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a goal polygon whose tile was replaced or removed; `mnav_errorLimit` for a navmesh of more polygons (in the tiles searched), tile slots or off-mesh links than the field's limits. A build allocates nothing. On an error the field holds nothing. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it; the field is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavNavFlowAt(const mnavNavFlow* field, const mnavNavmesh* navmesh, mnavPolygonId polygon, mnavPolygonFlow* flowOut);
```
Reads a polygon's way to the goals from the field.  @param field   The field. @param navmesh The navmesh the field was built on. @param polygon The polygon. @param flowOut Receives its way. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, nothing begun, a polygon id never handed out or a polygon outside the region; `mnav_errorStale` for another navmesh, one committed to since the work began, a polygon whose tile was replaced or removed, or work not yet ended. @par Thread safety Safe from any thread. Any number of threads may read a field at once while no build runs on it.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavDebugNavFlow(const mnavNavFlow* field, const mnavNavmesh* navmesh, mnavDebugBuffer* buffer);
```
Appends an arrow for each polygon of the field that has a way on (mnav_debugFlow): from the mean of the polygon's corners to the midpoint of its portal, or to the takeoff point where the way leaves by an off-mesh link, with two barbs a quarter of its length (at most half a meter) on the ground plane.  @param field   The field. @param navmesh The navmesh the field was built on. @param buffer  The buffer appended to. @return `mnav_success`, appending nothing when nothing was begun; `mnav_errorInvalid` for a NULL argument or a buffer with a count out of range, an array missing or an origin not finite; `mnav_errorStale` for another navmesh, one committed to since the work began, or work not yet ended; `mnav_errorCapacity` when the buffer filled, its counts saying what the whole needs. @par Thread safety Safe from any thread. Any number of threads may read a field at once while no build runs on it; the buffer is used by one thread at a time.

## `navmesh.h`

The navmesh queries read: tiles loaded from baked bytes, staged and committed together, their polygons named by generation-checked ids and linked across the tiles' shared sides (mnav-0004).

```c
MNAV_NODISCARD MNAV_API mnavBakeDefResult mnavCreateNavmesh(const mnavBakeDef* def, mnavNavmesh** navmeshOut);
```
Makes an empty navmesh for tiles baked with a def: checks the def and keeps a copy, its allocator and its limits.  @param def          The def the tiles are baked with. @param navmeshOut   Receives the navmesh, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` with the setting for an invalid def or a NULL argument; `mnav_errorLimit` when the navmesh does not fit the def's memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyNavmesh(mnavNavmesh* navmesh);
```
Destroys a navmesh, its tiles and everything staged.  @param navmesh  The navmesh, or NULL. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavTileResult mnavStageTile(mnavNavmesh* navmesh, const uint8_t* bytes, size_t size);
```
Loads tile bytes, checking every field as hostile input and the tile's settings against the navmesh's def, and stages the tile for the next commit, in place of anything staged at its place before. Queries do not see it until the commit.  @param navmesh  The navmesh. @param bytes    The tile's bytes, as mnavCopyBakedTile gives them. @param size     Their size in bytes. @return `mnav_success`; `mnav_errorInvalid` naming the section and element for malformed bytes, a tile baked with other settings (the header) or a NULL argument; `mnav_errorVersion` for another tile format version; `mnav_errorLimit` past the memory limit; `mnav_errorCapacity` when the allocator fails; `mnav_errorTier` for a tile at a place where one is committed, below mnav_tierDynamic. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time. Staging changes nothing queries read, but it may not run beside another call that stages or commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageTileRemoval(mnavNavmesh* navmesh, int32_t tileX, int32_t tileZ);
```
Stages the removal of the tile at a place for the next commit, in place of anything staged there before.  @param navmesh  The navmesh. @param tileX    The tile's column. @param tileZ    The tile's row. @return `mnav_success`; `mnav_errorInvalid` for a NULL navmesh; `mnav_errorLimit` past the memory limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCommit(mnavNavmesh* navmesh);
```
Applies everything staged at once: installs and removes the tiles and links every polygon on a changed tile's sides to the polygons across them, adds and removes off-mesh links, and snaps every off-mesh link's ends again. Either all of it applies or, on failure, nothing does and the staged changes stay. A replaced or removed tile's ids become stale, and so do removed links' ids.  @param navmesh  The navmesh. @return `mnav_success`; `mnav_errorInvalid` for a NULL navmesh; `mnav_errorLimit` when the tiles would pass the tiles limit, a tile its tileLinks limit, or memory its limit; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time, so no query may run during a commit.

```c
mnavTier mnavGetTier(const mnavNavmesh* navmesh);
```
Reads a navmesh's runtime tier.  @param navmesh The navmesh. @return Its tier; mnav_tierStatic for NULL. @par Thread safety Safe from any thread. Any number of threads may read it and query at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageArea(mnavNavmesh* navmesh, mnavPolygonId polygon, mnavAreaType area);
```
Stages a change of a polygon's area, applied at the next commit; mnav_areaNone blocks it. The change lasts while the polygon's tile is loaded, and is dropped when the commit replaces or removes it. Staged again before the commit, the last area counts.  @param navmesh The navmesh, of mnav_tierModifiers or above. @param polygon The polygon. @param area    Its new area, below MNAV_AREA_TYPES. @return `mnav_success`; `mnav_errorInvalid` for a NULL navmesh, an area out of range or an id never handed out; `mnav_errorStale` for a polygon whose tile has been replaced or removed; `mnav_errorTier` for a static navmesh; `mnav_errorCapacity` when memory runs out. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time, so no query may run during a stage call.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGetArea(const mnavNavmesh* navmesh, mnavPolygonId polygon, mnavAreaType* areaOut);
```
Reads a polygon's area as committed.  @param navmesh The navmesh. @param polygon The polygon. @param areaOut Receives its area. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or an id never handed out; `mnav_errorStale` for a polygon whose tile has been replaced or removed. @par Thread safety Safe from any thread. Any number of threads may read areas and query at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGetTile(const mnavNavmesh* navmesh, int32_t tileX, int32_t tileZ, mnavTileId* tileOut);
```
Finds the tile at a place.  @param navmesh  The navmesh. @param tileX    The tile's column. @param tileZ    The tile's row. @param tileOut  Receives the tile's id, or a zeroed id. @return `mnav_success`; `mnav_errorNotLoaded` when no tile is committed there; `mnav_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread. Any number of threads may find tiles and query at once between commits; none may while a stage or commit call runs.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageLink(mnavNavmesh* navmesh, const mnavLinkDef* def, mnavLinkId* linkOut);
```
Stages an off-mesh link; it is added at the next commit. Its ends snap, at every commit, to the nearest polygon within its radius on the ground and the agent's step in height.  @param navmesh  The navmesh. @param def      The link. @param linkOut  Receives its id, usable once it is committed. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a point that is not finite, or an edge link whose ends lie at one place on the ground or so near that the square of their distance rounds to 0; `mnav_errorRange` for a point farther from the def's origin than bake input may lie (MNAV_MAX_EXTENT_CELLS cells on the ground, MNAV_MAX_HEIGHT_CELLS cell heights up or down), or a radius, cost, kind or width out of its range; `mnav_errorLimit` when the links' crossings, staged and committed, would pass the links limit, or memory its limit: the link is refused whole, none of its crossings taken; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time, so no query may run while a link is staged.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageLinkRemoval(mnavNavmesh* navmesh, mnavLinkId link);
```
Stages an off-mesh link's removal; it goes at the next commit. A link staged and not yet committed goes at once.  @param navmesh  The navmesh. @param link     The link. @return `mnav_success`; `mnav_errorInvalid` for a NULL navmesh or an id never handed out; `mnav_errorStale` for a link already removed. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time, so no query may run while a removal is staged.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavStageLinkEnabled(mnavNavmesh* navmesh, mnavLinkId link, bool enabled);
```
Stages enabling or disabling an off-mesh link, applied at the next commit; a link is enabled when added. A disabled link stays snapped but is not crossed.  @param navmesh The navmesh, of mnav_tierModifiers or above. @param link    The link, committed or staged. @param enabled Whether it may be crossed. @return `mnav_success`; `mnav_errorInvalid` for a NULL navmesh or an id never handed out; `mnav_errorStale` for a link removed or staged for removal; `mnav_errorTier` for a static navmesh. @par Thread safety Safe from any thread; the navmesh is used by one thread at a time, so no query may run during a stage call.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGetLink(const mnavNavmesh* navmesh, mnavLinkId link, mnavLinkState* stateOut);
```
Reads an off-mesh link as committed.  @param navmesh  The navmesh. @param link     The link. @param stateOut Receives its state. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or an id never handed out; `mnav_errorNotLoaded` for a link staged and not yet committed; `mnav_errorStale` for a link removed. @par Thread safety Safe from any thread. Any number of threads may read links and query at once between commits; none may while a stage or commit call runs.

## `query.h`

Queries over a navmesh's committed tiles (mnav-0005). Every query reads and never changes the navmesh, and works in world coordinates: meters, right-handed, +Y up.

```c
mnavQueryFilter mnavDefaultQueryFilter(void);
```
Returns the filter that uses every walkable area at a cost of 1 and every kind of off-mesh link.  @return The filter. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindPolygons(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPos3 center, mnavVec3 halfExtents, mnavPolygonId* polygons, int32_t capacity, mnavFound* foundOut);
```
Lists the polygons the filter includes whose bounds meet a box: their vertices on the ground and their detail's heights. They come by tile, in place order (x, then z), then by polygon index.  @param navmesh     The navmesh. @param filter      The areas wanted, or NULL for every walkable one; one including mnav_areaNone finds blocked polygons. @param center      The box's center. @param halfExtents The box's half sizes, in meters, at least 0. @param polygons    Room for capacity ids, or NULL when capacity is 0. @param capacity    The room, at least 0. @param foundOut    Receives the count and whether the box was all loaded. @return `mnav_success`; `mnav_errorCapacity` when more polygons meet the box than the buffer holds: it holds the first ones and foundOut counts all; `mnav_errorInvalid` for a NULL argument, a negative capacity, a center or extent that is not finite or a negative extent, or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread. Any number of queries may run at once between commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindNearest(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPos3 point, mnavVec3 halfExtents, mnavNearest* nearestOut);
```
Finds the polygon, among those the filter includes, nearest a point whose nearest point lies within a box round it, and that point. A point over a polygon scores the height it lies beyond the agent's step, any other the distance to the polygon; ties go to the shorter distance, then the tile first by place (x, then z), then the lower polygon index.  @param navmesh      The navmesh. @param filter       The areas usable, or NULL for every walkable one. @param point        The query point. @param halfExtents  The box's half sizes, in meters, at least 0. @param nearestOut   Receives the result. @return `mnav_success`, also when no polygon's nearest point lies in the box (the polygon's slot is then 0); `mnav_errorInvalid` for a NULL argument, a point or extent that is not finite, or a negative extent; `mnav_errorInvalid` for a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread. Any number of queries may run at once between commits.

```c
mnavQueryDef mnavDefaultQueryDef(void);
```
Returns a query def with 8,192 nodes per search and paths up to 1,000 m.  @return The def. @par Thread safety Safe from any thread.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCreateQuery(const mnavQueryDef* def, mnavQuery** queryOut);
```
Makes a query context with the memory its limits need.  @param def      The def, from mnavDefaultQueryDef. @param queryOut Receives the context, or NULL on failure. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def not from mnavDefaultQueryDef; `mnav_errorRange` for a limit out of its range; `mnav_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void mnavDestroyQuery(mnavQuery* query);
```
Destroys a query context. NULL is ignored.  @param query    The context. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindPath(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end, mnavPath* pathOut);
```
Searches for the shortest way from a point on one polygon to a point on another (mnav-0005): A* over the edges between polygons, its heuristic the straight distance to the end point; ties go to the node made first. A step costs its length times the cost of the area it crosses, and the heuristic is scaled by the cheapest included area's cost; polygons of excluded areas other than the start polygon are not entered. Attached off-mesh links of included kinds are crossed at their declared cost, the heuristic scaled down to the lowest cost per meter among them (mnav-0005). The corridor found is pulled tight into a straight path with the funnel algorithm, stretch by stretch between links.  @param query        The context; its memory holds the result. @param navmesh      The navmesh. @param filter       The areas usable and their costs, or NULL for every walkable area at a cost of 1. @param startPolygon The polygon the start point lies on, as mnavFindNearest gives it. @param start        The start point. @param endPolygon   The polygon the end point lies on. @param end          The end point. @param pathOut      Receives the result. @return `mnav_success` whenever a search ran, however it ended; `mnav_errorInvalid` for a NULL argument, a point that is not finite or a polygon id that never existed; `mnav_errorInvalid` for a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a polygon id whose tile has been replaced or removed. @par Thread safety Safe from any thread; the context is used by one thread at a time. Any number of contexts may search one navmesh at once between commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindShortestPath(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end, mnavPath* pathOut);
```
Searches for the shortest way on the ground from a point on one polygon to a point on another (mnav-0005): Polyanya (Cui, Harabor and Grastien, 2017), whose nodes are intervals of portals seen from a root, the start, a corner turned at or an off-mesh link's landing point. Every included area must have the same cost: a step costs its length times that cost, so the way found is the shortest the navmesh allows, measured in x and z, not a way through portal midpoints. Polygons of excluded areas other than the start polygon are not entered. Attached off-mesh links of included kinds are crossed at their declared cost, from the takeoff point to the landing point. Ties go to the node made first. When the nodes run out, the A* search tells whether the end can be reached at all, and an end it cannot reach ends the search as no path. The result is that of mnavFindPath: the polygons crossed, the turning points, which are the path, and the links crossed; cost and length are those of the points, in three dimensions. A sliced path search in the same context ends.  @param query        The context; its memory holds the result. @param navmesh      The navmesh. @param filter       The areas usable and their costs, or NULL for every walkable area at a cost of 1. @param startPolygon The polygon the start point lies on, as mnavFindNearest gives it. @param start        The start point. @param endPolygon   The polygon the end point lies on. @param end          The end point. @param pathOut      Receives the result. @return `mnav_success` whenever a search ran, however it ended; `mnav_errorInvalid` for a NULL argument, a point that is not finite, a polygon id that never existed, a filter not built from mnavDefaultQueryFilter or one whose included areas differ in cost; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a polygon id whose tile has been replaced or removed. @par Thread safety Safe from any thread; the context is used by one thread at a time. Any number of contexts may search one navmesh at once between commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavBeginPath(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end);
```
Begins a path search to run in slices (mnav-0005): the same search as mnavFindPath, with its own copy of the filter, continued by mnavContinuePath and ended by mnavFinishPath. Whatever the slices, the result is the one mnavFindPath gives.  @param query        The context; it holds the search until the next begins. @param navmesh      The navmesh. @param filter       The areas usable and their costs, or NULL for every walkable area at a cost of 1; copied. @param startPolygon The polygon the start point lies on. @param start        The start point. @param endPolygon   The polygon the end point lies on. @param end          The end point. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a point that is not finite, a polygon id that never existed or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a polygon id whose tile has been replaced or removed. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavContinuePath(mnavQuery* query, const mnavNavmesh* navmesh, int32_t nodes, bool* endedOut);
```
Continues a search begun by mnavBeginPath: closes up to a number of nodes, fewer when the search ends first.  @param query    The context. @param navmesh  The navmesh the search began on. @param nodes    The most nodes to close, at least 1. @param endedOut Receives whether the search has ended. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, no search begun, or fewer than 1 node; `mnav_errorStale` for another navmesh, or one committed to since the search began: begin again. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFinishPath(mnavQuery* query, const mnavNavmesh* navmesh, mnavPath* pathOut);
```
Ends a search begun by mnavBeginPath and writes its result: as mnavFindPath's when it has ended; otherwise toward the node nearest the end, its end mnav_pathUnfinished.  @param query    The context; its memory holds the result. @param navmesh  The navmesh the search began on. @param pathOut  Receives the result. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or no search begun; `mnav_errorStale` for another navmesh, or one committed to since the search began. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavMoveAlongSurface( mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPos3 end, mnavMove* moveOut);
```
Moves from a point on a polygon toward a wanted point along the navmesh, as far as the walls allow (mnav-0005): a breadth-first walk over the polygons the filter includes whose shared edges meet the circle round the move, in edge order. In the polygon holding the wanted point the move reaches it; otherwise it ends at the point nearest it on the walls met, ties to the wall met first.  @param query        The context; its memory holds the polygons. @param navmesh      The navmesh. @param filter       The areas usable, or NULL for every walkable one. @param startPolygon The polygon the start point lies on. @param start        The start point. @param end          The wanted point; only its ground position counts. @param moveOut      Receives the result. @return `mnav_success` whenever the move ran, however it ended; `mnav_errorInvalid` for a NULL argument, a point that is not finite, a polygon id that never existed or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a polygon id whose tile has been replaced or removed. @par Thread safety Safe from any thread; the context is used by one thread at a time. Any number of contexts may move on one navmesh at once between commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavResetCorridor(mnavCorridor* corridor, mnavPolygonId* buffer, int32_t capacity, mnavPolygonId polygon, mnavPos3 position);
```
Starts a corridor over a buffer: one polygon, the position and the target both at a point on it.  @param corridor The corridor. @param buffer   Room for capacity polygons, kept by the corridor. @param capacity The buffer's polygons, at least 1. @param polygon  The polygon the point lies on. @param position The point. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a capacity below 1 or a point that is not finite. @par Thread safety Safe from any thread; the corridor is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavSetCorridor(mnavCorridor* corridor, const mnavPath* path);
```
Loads a path into a corridor: its polygons, its first point as the position and its last as the target.  @param corridor The corridor. @param path     A path from mnavFindPath or mnavFinishPath. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a path with no polygon or point; `mnav_errorCapacity` when the path has more polygons than the corridor's buffer, which is unchanged. @par Thread safety Safe from any thread; the corridor is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCheckCorridor(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, const mnavCorridor* corridor, int32_t* validOut);
```
Counts a corridor's leading polygons that are still current, of areas the filter includes, and joined to the one before by an edge, a tile link or an off-mesh link of a kind it includes. Fewer than all means the corridor needs trimming there and replanning.  @param navmesh  The navmesh. @param filter   The areas and kinds usable, or NULL for all. @param corridor The corridor. @param validOut Receives the count. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread. Any number of threads may check corridors at once between commits.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCorridorCorners(mnavQuery* query, const mnavNavmesh* navmesh, const mnavCorridor* corridor, mnavCorners* cornersOut);
```
Finds a corridor's straight path from its position to its target: the funnel over the portals between its polygons.  @param query    The context; its memory holds the corners. @param navmesh  The navmesh. @param corridor The corridor. @param cornersOut Receives the straight path. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or polygons not joined; `mnav_errorStale` for a polygon whose tile has been replaced or removed; `mnav_errorLimit` for a corridor of more polygons than the context's node limit. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavMoveCorridor(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavCorridor* corridor, mnavPos3 wanted, mnavMove* moveOut);
```
Moves a corridor's position along the surface toward a wanted point (mnavMoveAlongSurface from its first polygon) and merges the polygons walked into its start: it keeps its polygons from the farthest one the walk passed, led to by the walk.  @param query    The context, for the walk. @param navmesh  The navmesh. @param filter   The areas usable, or NULL for every walkable one. @param corridor The corridor. @param wanted   Where the agent would be; only its ground position counts. @param moveOut  Receives the walk, or NULL. @return As mnavMoveAlongSurface; also `mnav_errorInvalid` for a corridor with no buffer, and `mnav_errorCapacity` when the merged corridor would outgrow its buffer, which leaves it unchanged. @par Thread safety Safe from any thread; the context is used by one thread at a time. So is the corridor.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavMoveCorridorTarget(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavCorridor* corridor, mnavPos3 wanted, mnavMove* moveOut);
```
Moves a corridor's target along the surface toward a wanted point (mnavMoveAlongSurface from its last polygon) and merges the polygons walked into its end: it keeps its polygons up to the first one the walk passed, then the rest of the walk.  @param query    The context, for the walk. @param navmesh  The navmesh. @param filter   The areas usable, or NULL for every walkable one. @param corridor The corridor. @param wanted   Where the target would be; only its ground position counts. @param moveOut  Receives the walk, or NULL. @return As mnavMoveCorridor. @par Thread safety Safe from any thread; the context is used by one thread at a time. So is the corridor.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavShortcutCorridor(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavCorridor* corridor, mnavPos3 toward, bool* shortenedOut);
```
Shortens a corridor where the agent can see ahead: casts a ray from its position toward a point, usually a corner a few ahead; when the ray reaches it, the polygons it crossed replace the corridor's start up to the last corridor polygon it passed, if that is fewer.  @param query        The context, for the ray. @param navmesh      The navmesh. @param filter       The areas usable, or NULL for every walkable one. @param corridor     The corridor. @param toward       The point to look toward. @param shortenedOut Receives whether the corridor changed. @return As mnavRaycast; also `mnav_errorInvalid` for a corridor with no buffer or a NULL shortenedOut. @par Thread safety Safe from any thread; the context is used by one thread at a time. So is the corridor.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavReplanCorridor(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavCorridor* corridor, mnavVec3 halfExtents, mnavPath* pathOut);
```
Plans a corridor again from its position to its target and loads the path found: from its first and last polygons while they are current, else from the polygons nearest the position and the target within a box round each. A corridor whose position or target finds no polygon is left as it was, the path's end saying why.  @param query       The context, for the search; it holds the path. @param navmesh     The navmesh. @param filter      The areas usable and their costs, or NULL. @param corridor    The corridor. @param halfExtents The box's half sizes for finding polygons again. @param pathOut     Receives the path, or an empty one ended mnav_pathNotLoaded or mnav_pathNone when an end found no polygon. @return `mnav_success` whenever the corridor was looked at; `mnav_errorInvalid` for a NULL argument or a corridor with no buffer; `mnav_errorCapacity` when the path has more polygons than the buffer, which leaves the corridor as it was; the errors of mnavFindNearest and mnavFindPath otherwise. @par Thread safety Safe from any thread; the context is used by one thread at a time. So is the corridor.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindGridPath(mnavQuery* query, const mnavGrid* grid, const mnavQueryFilter* filter, mnavCell start, mnavCell end, mnavGridPath* pathOut);
```
Finds the cheapest path between two cells of a grid, moving to the 8 neighbours and never across a blocked corner: a diagonal step needs both cells beside it open. A step costs its length times the mean of its two cells' area costs. When every area the filter includes costs the same, the search is jump point search, else A*; both keep the context's node limit and path length, and end as mnavFindPath does, with the path to the cell nearest the end when the end is not reached. A grid search ends any sliced search.  @param query   The context; its memory holds the path. @param grid    The grid. @param filter  The areas usable and their costs, or NULL. @param start   The start cell. @param end     The end cell. @param pathOut Receives the path; mnav_pathNone with the start alone when the start is blocked. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a grid with no areas, a side out of range or a cell size not more than 0 or not finite, a cell outside the grid, or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavGetHeight(const mnavNavmesh* navmesh, mnavPolygonId polygon, double x, double z, double* heightOut);
```
Reads the height of a polygon's detail surface at a point on the ground.  @param navmesh   The navmesh. @param polygon   The polygon. @param x         The point's X. @param z         The point's Z. @param heightOut Receives the height. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a coordinate not finite, or a polygon id never handed out; `mnav_errorStale` for a polygon whose tile has gone; `mnav_errorRange` for a point outside the polygon on the ground. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindWallDistance(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId polygon, mnavPos3 center, double radius, mnavWall* wallOut);
```
Finds the nearest wall to a point within a radius on the ground, searching from its polygon across the edges that come within the radius, off-mesh links aside. An edge is a wall when no polygon the filter includes lies across it, a tile side with no tile loaded beyond included. A wall exactly the radius away lies within it; of walls at the same distance it gives one, the same on every platform, but which one is not promised.  @param query   The context; its last search ends. @param navmesh The navmesh. @param filter  The areas usable, or NULL. @param polygon The polygon the center lies in. @param center  The point. @param radius  How far to look, in meters, at least 0 and finite. @param wallOut Receives the wall. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a point or radius not finite, a negative radius, a polygon id never handed out, or a filter not built from mnavDefaultQueryFilter; `mnav_errorStale` for a polygon whose tile has gone; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindRandomPoint(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, uint64_t seed, mnavRandomPoint* pointOut);
```
Picks a point uniformly over the ground area of every polygon the filter includes, from a seed: the same seed and navmesh give the same point on every platform. The point lies on the polygon's detail surface. The call reads every polygon twice.  @param navmesh  The navmesh. @param filter   The areas usable, or NULL. @param seed     Any value. @param pointOut Receives the point. @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range. @par Thread safety Safe from any thread. Any number of threads may read the navmesh at once while no commit runs on it.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavFindRandomPointAround(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId polygon, mnavPos3 center, double radius, uint64_t seed, mnavRandomPoint* pointOut);
```
Picks a point reachable from a center: a polygon the walls search reaches within the radius, by ground area, then a point uniformly on it, which may lie beyond the radius by up to the polygon's size.  @param query    The context; its last search ends. @param navmesh  The navmesh. @param filter   The areas usable, or NULL. @param polygon  The polygon the center lies in. @param center   The center. @param radius   How far to search, in meters, at least 0 and finite. @param seed     Any value. @param pointOut Receives the point. @return As mnavFindWallDistance. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavCheckReachable(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end, mnavPathEnd* endOut);
```
Tells whether the end can be reached from the start: the path search with its limits, without a path made. mnav_pathFound when it can, mnav_pathNone when it cannot, or the limit that stopped the search before it could tell.  @param query        The context; its last search ends. @param navmesh      The navmesh. @param filter       The areas usable and their costs, or NULL. @param startPolygon The polygon the start lies in. @param start        The start point. @param endPolygon   The polygon the end lies in. @param end          The end point. @param endOut       Receives how the search ended. @return As mnavFindPath. @par Thread safety Safe from any thread; the context is used by one thread at a time.

```c
MNAV_NODISCARD MNAV_API mnavResult mnavRaycast(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavPolygonId startPolygon, mnavPos3 start, mnavPos3 end, mnavRay* rayOut);
```
Casts a ray along the navmesh on the ground from a point on a polygon toward an end point (mnav-0005): polygon to polygon through the first edge it crosses, until it reaches the end point, meets a wall (an edge into a polygon the filter excludes is a wall too) or a tile side with no tile loaded, or crosses as many polygons as the context's node limit. Where it leaves through a corner, it goes on through an edge that leads on, the lowest-numbered first; at a corner where two walls meet, the normal is one of theirs, the same on every platform, but which one is not promised.  @param query        The context; its memory holds the polygons. @param navmesh      The navmesh. @param filter       The areas usable, or NULL for every walkable one. @param startPolygon The polygon the start point lies on, as mnavFindNearest gives it. @param start        The start point. @param end          The end point; only its ground position counts. @param rayOut       Receives the result. @return `mnav_success` whenever the ray was cast, however it ended; `mnav_errorInvalid` for a NULL argument, a point that is not finite or a polygon id that never existed; `mnav_errorInvalid` for a filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its range; `mnav_errorStale` for a polygon id whose tile has been replaced or removed. @par Thread safety Safe from any thread; the context is used by one thread at a time. Any number of contexts may cast on one navmesh at once between commits.

---

112 functions across 12 headers.
