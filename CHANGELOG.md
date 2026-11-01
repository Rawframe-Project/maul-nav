# Changelog

All notable changes to this project are recorded here, grouped as
Added, Changed, Fixed, Removed and Security. The project uses
[Semantic Versioning](https://semver.org/). Before 1.0.0, any release
may change the API, the ABI and the tile formats.

## [0.7.0](https://github.com/Rawframe-Project/maul-nav/compare/v0.6.0...v0.7.0) (2026-11-01)

This release fixes what an audit of the status codes and a differential
check of tile sides found. A walkable region one cell thin is no longer
dropped by the bake, so neighboring tiles agree on their shared side;
this raises the bake generation to 5. A debug buffer counts triangles
and lines, not indices, and link generation gives a measuring call room
at the link limit instead of a refusal.

No tile format rises, so tiles baked by 0.6.0 load as they are. Every
tile's fingerprint differs from a new bake's, so a host that keeps
tiles by fingerprint rebakes them once. Paths, corridors, flow fields,
links and velocities are unchanged, except on tiles with a walkable
region one cell thin, which now gets a polygon where it had none.

### Fixed

- **Breaking:** a walkable region one cell thin, which simplification
  within `maxEdgeError` left with two vertices, was dropped by the
  contour stage without a count. On a tile side, the neighboring tile
  kept the same cells in a wider region, so its side edge faced nothing
  and acted as a wall. Now such a region keeps the outline of its
  cells, and the walls round it are simplified within 0 where they meet
  it. `MNAV_BAKE_GENERATOR` is 5, so every tile's bytes and fingerprint
  change.
  ([#254](https://github.com/Rawframe-Project/maul-nav/pull/254))
- **Breaking:** `mnavDebugBuffer`'s `triangleCapacity` and
  `triangleCount` counted indices, three a triangle, and `lineCapacity`
  and `lineCount` two a line, while their names and
  `mnavTriangleMesh.triangleCount` count triangles. Now they count
  triangles and lines: a host gives room for `3 * triangleCapacity` and
  `2 * lineCapacity` indices and draws `3 * triangleCount` and
  `2 * lineCount`. The geometry drawn is unchanged.
  ([#237](https://github.com/Rawframe-Project/maul-nav/pull/237))
- **Breaking:** `mnavQuery_GenerateLinks` with a buffer too short could
  stop with `mnav_errorLimit` at `MNAV_MAX_LINKS` while a call with room
  for every link succeeds, since the count past a short buffer also
  holds near duplicates. Now such a call gives `mnav_errorCapacity` with
  a count of `MNAV_MAX_LINKS`, and `mnav_errorLimit` comes only with a
  buffer that holds every link counted.
  ([#248](https://github.com/Rawframe-Project/maul-nav/pull/248))
- `mnavQuery_GenerateLinks` promised its count past a short buffer was
  the number of links generated. The `filterDistance` rule compares a
  link only with those held in the buffer, so that count may be more
  than a long enough buffer keeps. `linkgen.h` and the navmesh guide
  now say the count is room enough for a second call, which then holds
  every link a long buffer keeps.
  ([#239](https://github.com/Rawframe-Project/maul-nav/pull/239))

## [0.6.0](https://github.com/Rawframe-Project/maul-nav/compare/v0.5.0...v0.6.0) (2026-11-01)

This release brings maul-nav in line with the family rulings of the
1.0.0 reviews. Every enumeration typedef is a 32-bit integer, and every
counter is named for what it counts. A bake now keeps a very short edge
length or sample distance in force, which raises the bake generation to
4. A commit or destroy no longer frees a tile while its ids are still
live.

No tile format rises, so tiles baked by 0.5.0 load as they are. Their
fingerprints differ from a new bake's, so a host that keeps tiles by
fingerprint rebakes them once. Paths, corridors, flow fields and
velocities are unchanged, except on tiles baked with a
`maxEdgeLength` below one cell or a `detailSampleDistance` below a
sixteenth of a cell, whose polygons or detail now differ.

### Changed

- **Breaking:** every enumeration typedef but `mnavAreaType` is
  `int32_t`, where it was `uint8_t` (`mnavDebugKind` was `uint16_t`).
  This changes the layout of every struct that holds one.
  ([#230](https://github.com/Rawframe-Project/maul-nav/pull/230))
  - The 17 typedefs are `mnavDebugKind`; `mnavBakeSetting`,
    `mnavVolumeKind`, `mnavInputElement`, `mnavInputKind` and
    `mnavBakeStage`; `mnavFlightSetting`, `mnavFlightVolumeSetting` and
    `mnavFlightStop`; `mnavTier`, `mnavNavmeshSetting`,
    `mnavTileSection` and `mnavLinkKind`; and `mnavPathEnd`,
    `mnavMoveEnd`, `mnavSteerState` and `mnavRayEnd`.
  - A negative kind or tier is refused with `mnav_errorInvalid`, like
    one past the last: the navmesh def's `tier`, `mnavLinkDef.kind`, the
    link generation def's `dropKind` and `jumpKind`, and
    `mnavBakeVolume.kind`. A query filter never lets a negative link
    kind through.
  - `mnavDebugVertex.value` is `uint32_t`, and the vertex is 20 bytes. A
    corridor polygon's value is its place, no longer modulo 65536, and a
    flight block's value is its side, no longer capped at 65535.
  - `mnavAreaType` stays `uint8_t`: it is a number the host chooses,
    from 0 to `MNAV_AREA_TYPES - 1`.
- **Breaking:** the counters of the reports and results are named
  `thingCount` or `thingDropped`. Measures (`tileBytes`, `memoryPeak`,
  `stageTicks`, `stageMemory`) and the def limits keep their names.
  ([#231](https://github.com/Rawframe-Project/maul-nav/pull/231))
  - `mnavBakeReport`: `triangleCount`, `spanCount`, `regionCount`,
    `polygonCount`, `vertexCount`, `detailVertexCount`,
    `detailTriangleCount`, `holesDropped`, `regionsDropped`,
    `partialRingCount`, `fallbackHeightCount` and `cappedDetailCount`.
  - `mnavFlightBakeReport`: `triangleCount`, `spanCount`, `cubeCount`,
    `nodeCount` and `leafCount`.
  - `mnavHierarchyReport`: `clusterCount`, `transitionCount`,
    `edgeCount` and `searchCount`.
  - `mnavLinkState.crossings` is `crossingCount`.
  - `mnavFound.count`, `mnavWallsFound.count` and `mnavCorridor.count`
    are `polygonCount`, `wallCount` and `polygonCount`.

### Fixed

- **Breaking:** a positive `maxEdgeLength` below one cell, or a positive
  `detailSampleDistance` below a sixteenth of a cell, rounded down to 0
  and turned the setting off. Now such a length limits walls to one
  cell, and such a distance samples every cell. `MNAV_BAKE_GENERATOR` is
  4, so every tile's fingerprint changes.
  ([#218](https://github.com/Rawframe-Project/maul-nav/pull/218))
  - `MNAV_MAX_EDGE_ERROR` names the bound on `maxEdgeError`, which was
    checked without a name.
  - Each field of `mnavBakeDef` states its range. `limits.tileTriangles`
    names `MNAV_MAX_INPUT_TRIANGLES` as its bound.
  - Apart from the generator byte, tile bytes change only for a def
    with such a setting.
- A navmesh commit freed the tiles it removed or replaced while their
  ids were still live, so an allocator's free hook that read one back
  read released memory. `mnavDestroyNavmesh`, and a flight volume's
  commit and destroy, did the same. Now each makes the ids stale first:
  a free hook reads `mnav_errorStale` for a navmesh polygon, and
  `mnav_errorNotLoaded` for a flight tile that is gone.
  ([#227](https://github.com/Rawframe-Project/maul-nav/pull/227))

## [0.5.0](https://github.com/Rawframe-Project/maul-nav/compare/v0.4.0...v0.5.0) (2026-10-30)

This release finishes the API review for 1.0.0. A call that fills a
caller's array now zeroes it over its capacity when it fails, and a
refused debug call leaves its buffer as it was. The bake def's setting
values carry the def's name, and the family's hash takes a 64-bit byte
count.

On success nothing changes: tile bytes, fingerprints, paths, debug
geometry and velocities are the same, and no tile format rises, so tiles
baked by 0.4.0 load as they are.

### Changed

- **Breaking:** a call that fills a caller's array zeroes it over its
  capacity on any failure that does not define its items. Before, it
  left the array as it was, so a failure could leave an earlier call's
  walls, links or tile bytes in it.
  ([#208](https://github.com/Rawframe-Project/maul-nav/pull/208))
  - The calls are `mnavNavmesh_FindPolygons`, `mnavQuery_FindWalls`,
    `mnavQuery_FindCorridorAreas`, `mnavQuery_GenerateLinks`,
    `mnavBaker_CopyTile` and `mnavFlightBaker_CopyTile`.
  - A buffer too small still keeps the first capacity items, and link
    generation that stops at a bound keeps the links found.
  - A call refused as busy, called back in from host code, still writes
    nothing.
  - A tile copy into a buffer too small for the tile now gets zeros, not
    the bytes that were there.
  - Each array's documentation says what a failure leaves in it. No
    output may overlap an input.
- **Breaking:** a debug call that fails leaves its buffer as it was,
  except when the buffer fills (`mnav_errorCapacity`), which writes what
  fits and counts what the whole needs.
  ([#209](https://github.com/Rawframe-Project/maul-nav/pull/209))
  - `mnavNavmesh_DebugCorridor` used to append the polygons before a
    malformed id and then refuse the run. It now checks every id before
    it draws.
  - `mnavNavmesh_CheckCorridor`'s count is named `validCountOut`; no
    call changes.
- **Breaking:** `mnavBakeSetting`'s values are named for the bake def,
  beside the navmesh's, flight's and flight volume's settings. Their
  numbers do not change.
  ([#211](https://github.com/Rawframe-Project/maul-nav/pull/211))
  - `mnav_settingCellSize` is `mnav_bakeSettingCellSize`, and so on for
    all 22: `mnav_bakeSettingNone`, `Cookie`, `Allocator`, `Origin`,
    `CellSize`, `CellHeight`, `TileCells`, `AgentRadius`, `AgentHeight`,
    `AgentStepHeight`, `AgentMaxSlope`, `InputTriangles`,
    `TileTriangles`, `TileSpans`, `TilePolygons`, `TileVertices`,
    `MemoryBytes`, `MinRegionArea`, `MaxEdgeError`, `MaxEdgeLength`,
    `DetailSampleDistance` and `DetailMaxError`.
- **Breaking:** `mnavHash64` takes its byte count as an `int64_t`, where
  it took an `int32_t`, so it hashes 2 GiB or more in one call. Its
  result for every count that fit in an `int32_t` is unchanged.
  ([#214](https://github.com/Rawframe-Project/maul-nav/pull/214))

## [0.4.0](https://github.com/Rawframe-Project/maul-nav/compare/v0.3.0...v0.4.0) (2026-10-30)

This release settles the last API changes planned before 1.0.0.
- Navmeshes and flight volumes are made from defs of their own.
- Any number of query contexts can search one hierarchy at once.
- A refused bake input is named by its kind and index.
- The grid flow field is `mnavGridFlow`.
- A call refused as busy from host code writes none of its outputs.

Tile bytes, fingerprints, paths, flow fields and velocities do not
change, and no tile format rises: tiles baked by 0.3.0 load as they are.

### Changed

- **Breaking:** a navmesh is made from an `mnavNavmeshDef`, from
  `mnavDefaultNavmeshDef()`, and a flight volume from an
  `mnavFlightVolumeDef`, from `mnavDefaultFlightVolumeDef()`. Before,
  each used its baker's def, whose allocator, memory budget, tier and
  limits it shared with the bakes.
  ([#191](https://github.com/Rawframe-Project/maul-nav/pull/191))
  - The navmesh def holds:
    - its own allocator;
    - `bake`, the def its tiles are baked with;
    - `tier`;
    - `limits` (`tiles`, `tileLinks`, `links` and `memoryBytes`).
  - Move these fields there:
    - `mnavBakeDef.tier`;
    - `tiles`, `tileLinks` and `links` from `mnavBakeLimits`.
  - Give the navmesh its own `memoryBytes`.
  - The flight volume def holds its own allocator, `flight` (the flight
    def), and `limits` (`tiles` and `memoryBytes`).
    `mnavFlightLimits.tiles` moves there.
  - `mnavCreateNavmesh` returns an `mnavNavmeshDefResult`: the status,
    the refused `mnav_navmeshSetting*`, and for
    `mnav_navmeshSettingBake` the bake def's setting.
    `mnavCreateFlightVolume` returns an `mnavFlightVolumeDefResult` the
    same way.
  - `mnavBakeSetting` loses the tier and the three limits, and the
    settings after them are renumbered. `mnavFlightSetting` loses
    `mnav_flightSettingTiles`.
  - `MNAV_MAX_TILES`, `MNAV_MAX_TILE_LINKS`, `MNAV_MAX_LINKS` and
    `mnavTier` move to `navmesh.h`.
- **Breaking:** a bake report names the refused input by its kind and
  its index in that kind's array, not by one index counted across
  meshes, terrains and volumes.
  ([#185](https://github.com/Rawframe-Project/maul-nav/pull/185))
  - `mnavInputResult` gains `kind`, an `mnavInputKind`
    (`mnav_inputMesh`, `_Terrain`, `_Volume`, `_Outline` or
    `_Obstacle`).
  - Its `index` is now the input's index, and the old `index`, the
    element within the input, is `elementIndex`.
  - `mnavBakeReport` and `mnavFlightBakeReport` lose `mesh`; read
    `input.kind` and `input.index`.
  - `mnavValidateTriangleMesh` and `mnavValidateOutline` are replaced by
    `mnavValidateBakeInput` and `mnavValidateBake2DInput`, which check a
    whole bake input as a bake does.
- **Breaking:** `mnavQuery_FindHierarchicalPath` takes a `const
  mnavHierarchy*`. Any number of contexts may search one hierarchy at
  once, as they search a navmesh, while no build or update runs on it
  and no commit on the navmesh.
  ([#198](https://github.com/Rawframe-Project/maul-nav/pull/198))
  - The search keeps its state in the query context, sized by the new
    `mnavQueryLimits.transitions`: 16,384 by default, 0 to
    `MNAV_MAX_HIERARCHY_TRANSITIONS`.
  - The default context grows by about 459 KB, and each hierarchy
    shrinks by as much.
  - A hierarchy of more transitions than the context's limit is
    `mnav_errorCapacity`. A host whose hierarchies have more transitions
    raises the limit.
- **Breaking:** the grid flow field is named for its grid, beside the
  navmesh flow field `mnavNavFlow`.
  ([#199](https://github.com/Rawframe-Project/maul-nav/pull/199))

| Old | New |
|---|---|
| `mnavFlowField` | `mnavGridFlow` |
| `mnavFlowFieldDef`, `mnavDefaultFlowFieldDef` | `mnavGridFlowDef`, `mnavDefaultGridFlowDef` |
| `MNAV_FLOW_FIELD_DEF_COOKIE` | `MNAV_GRID_FLOW_DEF_COOKIE` |
| `mnavCreateFlowField`, `mnavDestroyFlowField` | `mnavCreateGridFlow`, `mnavDestroyGridFlow` |
| `mnavFlowField_Begin`, `_Continue`, `_Update`, `_Build`, `_GetFlow`, `_Debug`, `_GetMisuse` | `mnavGridFlow_Begin`, `_Continue`, `_Update`, `_Build`, `_GetFlow`, `_Debug`, `_GetMisuse` |
| `mnavFlow` | `mnavCellFlow` |
| `MNAV_MAX_FLOW_CELLS` | `MNAV_MAX_GRID_FLOW_CELLS` |
| `MNAV_MAX_NAVFLOW_POLYGONS`, `_TILES`, `_LINKS` | `MNAV_MAX_NAV_FLOW_POLYGONS`, `_TILES`, `_LINKS` |

- **Breaking:** a call refused with `mnav_errorBusy` writes none of its
  outputs: they stay as they were, where 0.3.0 zeroed them. Only a call
  back in from host code the object runs can be refused as busy: an
  allocator, or a link generation's clearance test. An output may be one
  the call in progress is filling.
  ([#197](https://github.com/Rawframe-Project/maul-nav/pull/197),
  [#196](https://github.com/Rawframe-Project/maul-nav/pull/196))
  - This holds for:
    - the path, corridor, surface, wall, random point, ray, grid path
      and flight path searches of a query context;
    - `mnavQuery_CheckReachable` and `mnavQuery_GenerateLinks`;
    - `mnavGridFlow_Continue` and `mnavNavFlow_Continue`;
    - `mnavHierarchy_Build` and `mnavHierarchy_Update`;
    - `mnavAvoidance_Avoid` and `mnavAvoidance_Avoid3D`, whose
      velocities stay as given.
  - The link id `mnavStageLink` gives is a creation's: the null id on
    any failure, busy included.
- **Breaking:** `mnavFlightVolume_FindNearestPoint` zeroes `nearestOut`
  when no open point lies within the radius, as the other "not found"
  answers do. Before, it left the query point there.
  ([#190](https://github.com/Rawframe-Project/maul-nav/pull/190))
- The user guide (`docs/user/`) uses the new defs, names and report
  fields.

## [0.3.0](https://github.com/Rawframe-Project/maul-nav/compare/v0.2.0...v0.3.0) (2026-10-30)

The whole API takes the Maul family's naming rule, so most functions
have new names and 25 take their arguments in a new order (the table
under Changed). Avoidance velocities change: they are binary32 now, a
velocity held to the maximum speed no longer passes it, and an agent
that only its own speed holds back no longer steps aside. Tile bytes,
fingerprints and paths do not change, and no tile format rises: tiles
baked by 0.2.0 load as they are.

### Added

- `mnavLink_IsEnabled(navmesh, link, enabledOut)` reads whether a link
  is switched on, as committed: right after `mnavLink_StageEnable` it
  still gives the old state, and the new one after
  `mnavNavmesh_Commit`. A link staged and not yet committed is
  `mnav_errorState`, as with `mnavLink_GetState`.
  ([#181](https://github.com/Rawframe-Project/maul-nav/pull/181))

### Changed

- **Breaking:** functions are named by the family's naming rule.
  ([#181](https://github.com/Rawframe-Project/maul-nav/pull/181))
  - A call on one object is `mnavObject_Verb`: the object is the kind
    of the first id the call acts on (`mnavPolygon_GetArea`,
    `mnavLink_StageEnable`), or else the object it takes first
    (`mnavNavmesh_Commit`, `mnavQuery_FindPath`). Each object's misuse
    count is `mnav<Object>_GetMisuse`.
  - Creation, destruction, defaults, def validation, calls on a value
    such as a corridor (`mnavResetCorridor`, `mnavSetCorridor`,
    `mnavSteer`), `mnavGetVersion` and `mnavHash64` keep their names.
    `mnavResultName` is `mnavGetResultName`.
  - Every call that takes a query filter takes it after its inputs,
    just before its outputs, and the corridor calls take the corridor
    right after the query or navmesh. The table gives the new
    parameter list of each call that moves.
  - `mnavClearanceFn` is `mnavClearanceFunction`, a function type, and
    `mnavLinkGenDef.clear` is `mnavClearanceFunction* clearance`.
  - `mnavSteerDef.anticipateTurns` is `enableTurnAnticipation`, and
    `mnavFlightDef.groundBelow` is `enableGroundFill`.

| Old | New | Parameters |
|---|---|---|
| `mnavResultName` | `mnavGetResultName` |  |
| `mnavGetAvoidanceMisuse` | `mnavAvoidance_GetMisuse` |  |
| `mnavAvoid` | `mnavAvoidance_Avoid` |  |
| `mnavAvoid3D` | `mnavAvoidance_Avoid3D` |  |
| `mnavDebugAvoidance` | `mnavAvoidance_Debug` |  |
| `mnavDebugAvoidance3D` | `mnavAvoidance_Debug3D` |  |
| `mnavGetBakerMisuse` | `mnavBaker_GetMisuse` |  |
| `mnavBakeTile` | `mnavBaker_BakeTile` |  |
| `mnavBakeTile2D` | `mnavBaker_BakeTile2D` |  |
| `mnavCopyBakedTile` | `mnavBaker_CopyTile` |  |
| `mnavBakeTileCached` | `mnavBaker_BakeTileCached` |  |
| `mnavRebuildTile` | `mnavBaker_RebuildTile` |  |
| `mnavGetTileIndexMisuse` | `mnavTileIndex_GetMisuse` |  |
| `mnavGetTileCacheMisuse` | `mnavTileCache_GetMisuse` |  |
| `mnavDropCachedTile` | `mnavTileCache_DropTile` |  |
| `mnavGetTileCacheBytes` | `mnavTileCache_GetBytes` |  |
| `mnavDebugNavmesh` | `mnavNavmesh_Debug` |  |
| `mnavDebugLinks` | `mnavNavmesh_DebugLinks` |  |
| `mnavDebugCorridor` | `mnavNavmesh_DebugCorridor` |  |
| `mnavGetFlightBakerMisuse` | `mnavFlightBaker_GetMisuse` |  |
| `mnavBakeFlightTile` | `mnavFlightBaker_BakeTile` |  |
| `mnavCopyFlightTile` | `mnavFlightBaker_CopyTile` |  |
| `mnavGetFlightVolumeMisuse` | `mnavFlightVolume_GetMisuse` |  |
| `mnavStageFlightTile` | `mnavFlightVolume_StageTile` |  |
| `mnavStageFlightTileRemoval` | `mnavFlightVolume_StageTileRemoval` |  |
| `mnavCommitFlight` | `mnavFlightVolume_Commit` |  |
| `mnavGetFlightTile` | `mnavFlightVolume_GetTile` |  |
| `mnavIsFlightOpen` | `mnavFlightVolume_IsOpen` |  |
| `mnavFlightRaycast` | `mnavFlightVolume_Raycast` |  |
| `mnavCheckFlightPath` | `mnavFlightVolume_CheckPath` |  |
| `mnavFindNearestFlightPoint` | `mnavFlightVolume_FindNearestPoint` |  |
| `mnavDebugFlight` | `mnavFlightVolume_Debug` |  |
| `mnavFindFlightPath` | `mnavQuery_FindFlightPath` |  |
| `mnavBeginFlightPath` | `mnavQuery_BeginFlightPath` |  |
| `mnavContinueFlightPath` | `mnavQuery_ContinueFlightPath` |  |
| `mnavFinishFlightPath` | `mnavQuery_FinishFlightPath` |  |
| `mnavGetFlowFieldMisuse` | `mnavFlowField_GetMisuse` |  |
| `mnavBeginFlowField` | `mnavFlowField_Begin` | `(field, grid, region, goals, goalCount, filter)` |
| `mnavContinueFlowField` | `mnavFlowField_Continue` |  |
| `mnavUpdateFlowField` | `mnavFlowField_Update` |  |
| `mnavBuildFlowField` | `mnavFlowField_Build` | `(field, grid, goals, goalCount, filter)` |
| `mnavFlowAt` | `mnavFlowField_GetFlow` |  |
| `mnavDebugFlowField` | `mnavFlowField_Debug` |  |
| `mnavGetHierarchyMisuse` | `mnavHierarchy_GetMisuse` |  |
| `mnavBuildHierarchy` | `mnavHierarchy_Build` |  |
| `mnavUpdateHierarchy` | `mnavHierarchy_Update` |  |
| `mnavFindHierarchicalPath` | `mnavQuery_FindHierarchicalPath` |  |
| `mnavGenerateLinks` | `mnavQuery_GenerateLinks` | `(query, navmesh, def, tiles, filter, linksOut, capacity, countOut)` |
| `mnavClearanceFn` | `mnavClearanceFunction` | a function type (see above) |
| `mnavGetNavFlowMisuse` | `mnavNavFlow_GetMisuse` |  |
| `mnavBeginNavFlow` | `mnavNavFlow_Begin` | `(field, navmesh, region, goals, goalCount, filter)` |
| `mnavContinueNavFlow` | `mnavNavFlow_Continue` |  |
| `mnavBuildNavFlow` | `mnavNavFlow_Build` | `(field, navmesh, goals, goalCount, filter)` |
| `mnavNavFlowAt` | `mnavNavFlow_GetFlow` |  |
| `mnavDebugNavFlow` | `mnavNavFlow_Debug` |  |
| `mnavGetNavmeshMisuse` | `mnavNavmesh_GetMisuse` |  |
| `mnavStageTile` | `mnavNavmesh_StageTile` |  |
| `mnavStageTileRemoval` | `mnavNavmesh_StageTileRemoval` |  |
| `mnavCommit` | `mnavNavmesh_Commit` |  |
| `mnavGetTier` | `mnavNavmesh_GetTier` |  |
| `mnavGetTile` | `mnavNavmesh_GetTile` |  |
| `mnavStageArea` | `mnavPolygon_StageArea` |  |
| `mnavGetArea` | `mnavPolygon_GetArea` |  |
| `mnavStageLinkEnabled` | `mnavLink_StageEnable` |  |
| `mnavGetLink` | `mnavLink_GetState` |  |
| `mnavGetQueryMisuse` | `mnavQuery_GetMisuse` |  |
| `mnavFindPolygons` | `mnavNavmesh_FindPolygons` | `(navmesh, center, halfExtents, filter, polygonsOut, capacity, foundOut)` |
| `mnavFindNearest` | `mnavNavmesh_FindNearest` | `(navmesh, point, halfExtents, filter, nearestOut)` |
| `mnavFindRandomPoint` | `mnavNavmesh_FindRandomPoint` | `(navmesh, seed, filter, pointOut)` |
| `mnavCheckCorridor` | `mnavNavmesh_CheckCorridor` | `(navmesh, corridor, filter, validOut)` |
| `mnavGetHeight` | `mnavPolygon_GetHeight` |  |
| `mnavFindPath` | `mnavQuery_FindPath` | `(query, navmesh, startPolygon, start, endPolygon, end, filter, pathOut)` |
| `mnavFindShortestPath` | `mnavQuery_FindShortestPath` | `(query, navmesh, startPolygon, start, endPolygon, end, filter, pathOut)` |
| `mnavBeginPath` | `mnavQuery_BeginPath` | `(query, navmesh, startPolygon, start, endPolygon, end, filter)` |
| `mnavContinuePath` | `mnavQuery_ContinuePath` |  |
| `mnavFinishPath` | `mnavQuery_FinishPath` |  |
| `mnavMoveAlongSurface` | `mnavQuery_MoveAlongSurface` | `(query, navmesh, startPolygon, start, end, filter, moveOut)` |
| `mnavCorridorCorners` | `mnavQuery_FindCorridorCorners` | `(query, corridor, navmesh, cornersOut)` |
| `mnavCorridorAreas` | `mnavQuery_FindCorridorAreas` | `(query, corridor, navmesh, cornersOut, runsOut, capacity, countOut)` |
| `mnavMoveCorridor` | `mnavQuery_MoveCorridor` | `(query, corridor, navmesh, wanted, filter, moveOut)` |
| `mnavMoveCorridorTarget` | `mnavQuery_MoveCorridorTarget` | `(query, corridor, navmesh, wanted, filter, moveOut)` |
| `mnavShortcutCorridor` | `mnavQuery_ShortcutCorridor` | `(query, corridor, navmesh, toward, filter, shortenedOut)` |
| `mnavReplanCorridor` | `mnavQuery_ReplanCorridor` | `(query, corridor, navmesh, halfExtents, filter, pathOut)` |
| `mnavFindGridPath` | `mnavQuery_FindGridPath` | `(query, grid, start, end, filter, pathOut)` |
| `mnavFindWallDistance` | `mnavQuery_FindWallDistance` | `(query, navmesh, polygon, center, radius, filter, nearestOut)` |
| `mnavFindWalls` | `mnavQuery_FindWalls` | `(query, navmesh, polygon, center, radius, filter, wallsOut, capacity, foundOut)` |
| `mnavFindRandomPointAround` | `mnavQuery_FindRandomPointAround` | `(query, navmesh, polygon, center, radius, seed, filter, pointOut)` |
| `mnavCheckReachable` | `mnavQuery_CheckReachable` | `(query, navmesh, startPolygon, start, endPolygon, end, filter, endOut)` |
| `mnavRaycast` | `mnavQuery_Raycast` | `(query, navmesh, startPolygon, start, end, filter, rayOut)` |

- **Breaking:** velocities and directions are binary32 vectors, and
  positions stay binary64.
  ([#179](https://github.com/Rawframe-Project/maul-nav/pull/179))
  - `mnavAgent` and `mnavObstacle` take `velocity` and `preferred` as
    `mnavVec2`, and `mnavAvoidance_Avoid` writes `mnavVec2`;
    `mnavAgent3D` and `mnavSphere` take `mnavVec3`, and
    `mnavAvoidance_Avoid3D` writes `mnavVec3`.
  - `mnavSteering.velocity` and `mnavWall.normal` are `mnavVec3`, and
    `mnavRay`'s `normalX` and `normalZ` are one `mnavVec3 normal`.
  - `mnavVec2`, `mnavVec3`, `mnavPos2` and `mnavPos3` are all in
    `base.h`.
- **Breaking:** avoidance velocities change, so runs that feed them
  back differ from 0.2.0's.
  - Avoidance still computes in binary64 and rounds each velocity it
    gives back to binary32 once, never past the agent's maximum speed.
    `mnavSteer` rounds its velocity to binary32 too.
    ([#179](https://github.com/Rawframe-Project/maul-nav/pull/179))
  - A velocity scaled back to the maximum speed could pass it by a
    unit in the last place; it now stays within it, as documented.
    ([#177](https://github.com/Rawframe-Project/maul-nav/pull/177))
  - An agent whose preferred velocity is faster than its maximum speed,
    and that no neighbour, obstacle or sphere holds back, goes along its
    preferred direction. It used to aim 1% of its speed to the right of
    it, about 0.57 degrees.
    ([#178](https://github.com/Rawframe-Project/maul-nav/pull/178))
- **Breaking:** a call whose first parameter is an object zeroes its
  outputs on any failure. It keeps only what the failure defines: the
  size or count a buffer too small needed, the links counted when link
  generation stops at a bound, and bake reports. The outputs that
  `mnavPolygon_GetHeight`, `mnavFlowField_GetFlow`,
  `mnavNavFlow_GetFlow` and `mnavQuery_FindFlightPath` left as they
  were on `mnav_errorOutside` are now zeroed, and so are
  `mnavAvoidance_Avoid`'s and `mnavAvoidance_Avoid3D`'s velocities when
  the set's limit holds the agents.
  ([#168](https://github.com/Rawframe-Project/maul-nav/pull/168))
- **Breaking:** one type describes a wall.
  ([#169](https://github.com/Rawframe-Project/maul-nav/pull/169))
  - `mnavWallSegment` is removed. `mnavQuery_FindWalls` fills
    `mnavWall { start, end, point, normal, distance }`, which gains the
    point nearest the center.
  - `mnavQuery_FindWallDistance` fills an
    `mnavNearestWall { found, limited, wall }`, whose wall is, field
    for field, the first one `mnavQuery_FindWalls` lists. Its normal is
    the wall edge's normal toward the walkable side; it used to be the
    direction from the wall's point to the center, which differs at a
    corner and is `(center - point) / distance`. With no wall found,
    the wall is zeroed instead of holding the radius and the center.
- **Breaking:** one type gives a box of tiles, and one a box of cells.
  ([#170](https://github.com/Rawframe-Project/maul-nav/pull/170))
  - `mnavNavmesh_Debug` and `mnavQuery_GenerateLinks` take a
    `const mnavTileRange*` (NULL for every tile) instead of four tile
    coordinates, and `mnavTileRange { x0, z0, x1, z1 }` replaces
    `mnavNavFlowRegion`.
  - `mnavCellRange { x0, y0, x1, y1 }` replaces
    `mnavFlowRegion { x, y, width, height }`. Both corners of either
    range are included, so a region of no cells is a backward range.
- **Breaking:** `mnavBaker_CopyTile` and `mnavFlightBaker_CopyTile`
  refuse a baker that holds no tile (before its first bake, or after a
  failed one) with `mnav_errorState`, not `mnav_errorInvalid`, and
  count no misuse for it.
  ([#167](https://github.com/Rawframe-Project/maul-nav/pull/167))
- The user guide (`docs/user/`) uses the new names, types and argument
  orders, and the troubleshooting page says what a failed call leaves
  in its outputs.

### Fixed

- Removing a tile-border vertex could leave a triangle with no area
  where two vertices share a ground place at different heights. A
  Debug build then trapped, and a Release build could divide by zero
  in the detail stage. No known bake input reaches it, and an input
  that does not keeps its tile bytes.
  ([#154](https://github.com/Rawframe-Project/maul-nav/pull/154))
- The flight sample (`samples/flight.c`) no longer leaves fliers
  pressed against a pillar when their speed changes slightly: a flier
  moves on to the next route point only once it can see it, and
  searches its route again when it loses sight of it.
  ([#175](https://github.com/Rawframe-Project/maul-nav/pull/175))

## [0.2.0](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.2.0) (2026-10-28)

Tile bytes, fingerprints, paths and velocities do not change, and no
tile format rises: tiles baked by 0.1.0 load as they are. Flow fields
change for one call sequence (see Fixed).

### Added

- A misuse count on every object, read with `mnavGetNavmeshMisuse`,
  `mnavGetQueryMisuse`, `mnavGetBakerMisuse`, `mnavGetTileCacheMisuse`,
  `mnavGetTileIndexMisuse`, `mnavGetHierarchyMisuse`,
  `mnavGetFlowFieldMisuse`, `mnavGetNavFlowMisuse`,
  `mnavGetAvoidanceMisuse`, `mnavGetFlightBakerMisuse` and
  `mnavGetFlightVolumeMisuse`. A count rises for each call refused with
  `mnav_errorInvalid` or `mnav_errorBusy`, so a release build can catch
  a host's bugs.
  ([#141](https://github.com/Rawframe-Project/maul-nav/pull/141),
  [#145](https://github.com/Rawframe-Project/maul-nav/pull/145),
  [#148](https://github.com/Rawframe-Project/maul-nav/pull/148))

### Changed

- **Breaking:** the results take the Maul family's numbering:
  `mnav_errorInvalid` -1, `mnav_errorCapacity` -2, `mnav_errorStale` -3,
  `mnav_errorUnsupported` -4, `mnav_errorState` -6, `mnav_errorVersion`
  -7, `mnav_errorFormat` -8, `mnav_errorLimit` -9, `mnav_errorBusy` -10,
  and maul-nav's own `mnav_errorNotLoaded` -32 and `mnav_errorOutside`
  -33. A host that stored or compared the numbers compares the names.
  Some calls refuse with a different result:
  - A limit the caller sets (a def's limits and memory, a query's
    nodes), `mnavBakeLimits` and `mnavFlightLimits` included, is
    `mnav_errorCapacity`; it was `mnav_errorLimit`.
  - `mnav_errorRange` is removed. A coordinate past the extent is
    `mnav_errorLimit`, as is any value past a published `MNAV_MAX_*` or
    `MNAV_MIN_*` bound. `MNAV_MAX_PIECE_CORNERS`,
    `MNAV_MAX_CLIP_CORNERS`, `MNAV_MAX_INDEX_ENTRIES` and
    `MNAV_MAX_FLIGHT_VOXELS` are new public bounds.
  - Every check on tile and flight tile bytes is `mnav_errorFormat`.
  - A search or field not begun, not ended or out of date after a
    commit, and a link read before its commit, is `mnav_errorState`.
  - A NaN is `mnav_errorInvalid`. An infinity is `mnav_errorLimit` past
    a published bound, and `mnav_errorInvalid` elsewhere.
  - `mnavIsFlightOpen` refuses a point more than 2^28 voxels from the
    origin, as the other flight calls do; it took up to 2^30.

  ([#135](https://github.com/Rawframe-Project/maul-nav/pull/135))
- **Breaking:** `mnav_errorTier` is removed. A staged tile, area change
  or link toggle that the navmesh's tier does not allow is refused with
  `mnav_errorUnsupported`.
  ([#138](https://github.com/Rawframe-Project/maul-nav/pull/138))
- **Breaking:** host code may not call into the objects of a running
  call. That is an allocator, a baker's clock or a link generation's
  clearance test. Such a call is refused with `mnav_errorBusy` when it
  returns a status, does nothing when it returns nothing, and counts one
  misuse. This holds for the object the host code serves and for the
  other objects the running call uses: the baker, tile index and tile
  cache of a bake or rebuild, and the context and navmesh of a
  hierarchy's build or update. It also holds for the release of an
  object's own memory. A clearance test or a hierarchy's allocator may
  still call `mnavStageTile` and `mnavStageTileRemoval`. Before, such a
  call could read freed memory.
  ([#141](https://github.com/Rawframe-Project/maul-nav/pull/141),
  [#145](https://github.com/Rawframe-Project/maul-nav/pull/145),
  [#147](https://github.com/Rawframe-Project/maul-nav/pull/147))
- **Breaking:** a call refused with `mnav_errorInvalid` counts one
  misuse on the first object it is given. Input geometry a bake refuses
  and its report names counts none.
  ([#148](https://github.com/Rawframe-Project/maul-nav/pull/148))
- **Breaking:** a place outside what an object covers is
  `mnav_errorOutside`, not `mnav_errorInvalid`, and counts no misuse.
  This covers:
  - `mnavGetHeight` for a point off the polygon
  - `mnavFlowAt` and `mnavNavFlowAt` for a cell or polygon outside the
    field's region
  - `mnavIsFlightOpen`, `mnavFindFlightPath` and `mnavBeginFlightPath`
    for a point below or above the flight volume

  A host that handled these as `mnav_errorInvalid` handles
  `mnav_errorOutside`. The outputs stay as they were, but
  `mnavIsFlightOpen` sets `openOut` to false.
  ([#148](https://github.com/Rawframe-Project/maul-nav/pull/148),
  [#150](https://github.com/Rawframe-Project/maul-nav/pull/150))
- The troubleshooting table lists the new numbering and results. The
  getting-started and baking pages say what host code may call.

### Fixed

- **Breaking:** a flow field repaired after `mnavBeginFlowField` cut an
  earlier repair short now matches a fresh build, as
  `mnavUpdateFlowField` promises. Before, cells the earlier repair had
  marked could keep their old costs and next cells. Flow fields change
  for that sequence only.
  ([#130](https://github.com/Rawframe-Project/maul-nav/pull/130))
- `mnavStageLink` refuses a link with `mnav_errorLimit` when a crossing
  would take a link slot its attachment key cannot hold. This can happen
  at `MNAV_MAX_LINKS` links once a slot has worn out. Before, the link
  took a slot that wrapped to slot 0, and a search crossed the wrong
  link or read out of bounds.
  ([#122](https://github.com/Rawframe-Project/maul-nav/pull/122))

## [0.1.0](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.1.0) (2026-10-27)

### Changed

- **Breaking:** a navmesh tile is a 32-byte header (magic `MNAV`, format
  version, generation, fingerprint, the payload's hash and size, and the
  field count) followed by tagged fields in name order: `DPRT`, `DTRI`,
  `DVRT`, `POLY`, `TILE` and `VERT`. Bytes nothing read are gone: the
  stored header size, reserved and pad words, the element counts, which
  follow from the fields' lengths, and each detail triangle's outline
  bits, with `mnavDetailTriangle.outline`. The loader refuses a field
  that is repeated, out of order, empty, not a whole number of records,
  or padded with non-zero bytes, and bytes after the last field, with
  `mnav_errorInvalid` naming the field's `mnavTileSection`; an unknown
  tag is `mnav_errorVersion`. `MNAV_TILE_FORMAT` is 3. Every tile's
  bytes and fingerprint change, so a host rebakes its tiles; the meshes
  they hold, and so paths and queries, do not change.
  ([#97](https://github.com/Rawframe-Project/maul-nav/pull/97))
- **Breaking:** a flight tile takes the same form: a 32-byte header with
  the magic `MNVF`, then the fields `FLHT`, `LEAF`, `NODE` and `ROOT`,
  refused as tiles are. `ROOT` must hold the def's cube count and `FLHT`
  exactly 56 bytes, and `NODE` and `LEAF` are checked against the def's
  limits before anything is allocated. `MNAV_FLIGHT_FORMAT` is 3 and
  `MNAV_FLIGHT_GENERATOR` 2. Every flight tile's bytes and fingerprint
  change; flight queries do not.
  ([#98](https://github.com/Rawframe-Project/maul-nav/pull/98))
- **Breaking:** one bake entry point per input kind. `mnavBakeTile` and
  `mnavBakeTile2D` now take an input record, `const mnavBakeInput*` and
  `const mnavBake2DInput*`, and the forms that took meshes or outlines
  alone are removed, as are the names `mnavBakeTileInput` and
  `mnavBakeTile2DInput`. A call that passed meshes alone passes
  `&(mnavBakeInput){meshes, count, NULL, 0, NULL, 0, NULL}`. Results do
  not change.
  ([#95](https://github.com/Rawframe-Project/maul-nav/pull/95))
- `mnavBakeDef.maxEdgeError`'s documentation says that walls simplified
  a cell or more can cross, that a ring which then does not triangulate
  whole loses the area it did not triangulate, and that
  `mnavBakeReport.partialRings` counts it.
  ([#91](https://github.com/Rawframe-Project/maul-nav/pull/91))
- `mnavBakeReport.partialRings` counts the polygon stage's rings alone:
  a detail outline, a simple ring since #109, always triangulates whole.
  ([#112](https://github.com/Rawframe-Project/maul-nav/pull/112))

### Fixed

- **Breaking:** `mnavGenerateLinks` refuses a spacing below the
  navmesh's cell size with `mnav_errorInvalid`. A spacing small enough
  used to make an out-of-range conversion of each edge's sample count.
  Generation now stops with `mnav_errorLimit` when a landing's search
  box meets more than `MNAV_MAX_LINK_LANDINGS` (32) polygons, which used
  to drop that link without a word, or at a link past `MNAV_MAX_LINKS`.
  On a stop, `countOut` gives the links found so far and the buffer
  holds the first of them. Links for a spacing of a cell or more do not
  change.
  ([#116](https://github.com/Rawframe-Project/maul-nav/pull/116))
- **Breaking:** `mnavFindRandomPointAround` returns `mnav_errorLimit`
  when a polygon clipped to its circle would have more corners than the
  clip holds. Rounding near a cut could add one corner too many, and the
  clip wrote past its buffer. Results otherwise do not change.
  ([#113](https://github.com/Rawframe-Project/maul-nav/pull/113))
- **Breaking:** the detail stage places each edge sample exactly on its
  edge, so a sliver polygon's outline is a simple ring and triangulates
  whole, where before a sample rounded off its edge could leave the ring
  cut short and counted in `partialRings`. Polygons sharing an edge
  still place the same points. Tile bytes and fingerprints change for
  polygons with sloped edges, and `MNAV_BAKE_GENERATOR` is 3, so a host
  rebakes its tiles.
  ([#109](https://github.com/Rawframe-Project/maul-nav/pull/109))
- A raster cut stops at its piece's room of 12 corners: the bake fails
  at rasterizing with `mnav_errorLimit` instead of writing past the
  piece. Rounding of an interpolated corner near a cell line could make
  a cut add more corners than exact arithmetic allows. No known bake
  reaches it, and every tile is as it was.
  ([#115](https://github.com/Rawframe-Project/maul-nav/pull/115))
- Every call that reads an `mnavCorridor` refuses one that is not well
  formed, with `mnav_errorInvalid`, before reading its buffer: a
  corridor has a buffer, a capacity of at least 1, and from 1 to
  capacity polygons. A count past the capacity used to make the calls
  read past the caller's buffer. `mnavSetCorridor` refuses only a
  capacity below 1.
  ([#114](https://github.com/Rawframe-Project/maul-nav/pull/114))
- `mnavRebuildTile` compares a cached tile's bake settings in full, not
  by a 64-bit hash, so a baker of another def is always refused with
  `mnav_errorInvalid`. A cache's tile slots grow by 80 bytes each.
  ([#90](https://github.com/Rawframe-Project/maul-nav/pull/90))

## [0.0.4](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.4) (2026-10-24)

### Changed

- A navmesh tile's rasterize stage takes about two thirds of the
  instructions it did: about 31% fewer at a 2 m cell and 33% fewer at
  a 0.25 m cell. A whole tile's bake takes about 14% and 19% fewer.
  A flight volume bake shares the rasterizer and gains too. Tile bytes
  are the same as before, and `MNAV_BAKE_GENERATOR` does not change.
  ([#25](https://github.com/Rawframe-Project/maul-nav/pull/25))

## [0.0.3](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.3) (2026-10-24)

### Changed

- A flight path search takes about a third of the instructions it
  did: the line-of-sight walk skips open blocks whole and steps through
  mixed leaves by bit tests. Found paths and raycasts are the same bits
  as before.
- A flight path whose end (or start) lies in a closed space the other
  cannot reach, such as the inside of a closed mesh, now ends at once
  with `mnav_pathNone` and the start alone as its path. This applies
  when the space is found closed within 512 blocks and the node budget.
  Before, the search flooded its whole node budget and returned
  `mnav_pathOutOfNodes` with a partial path toward the outside, which
  could never reach the end.
  ([#22](https://github.com/Rawframe-Project/maul-nav/pull/22))

### Fixed

- A host that runs its threads with flush-to-zero, denormals-are-zero
  or another rounding mode now gets the same bakes, paths and query
  results as every other host. Every public call that computes switches
  to the library's floating-point setting and restores the host's on
  return. The allocator and the bake clock run in the host's own
  setting. Results in the default setting do not change.
  ([#24](https://github.com/Rawframe-Project/maul-nav/pull/24))

## [0.0.2](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.2) (2026-10-24)

### Fixed

- A navmesh slot whose tile is removed or replaced at the last
  generation is now retired. Before, the next tile staged there wrapped
  to generation 0, and its polygon ids were refused by every query.

## [0.0.1](https://github.com/Rawframe-Project/maul-nav/releases/tag/v0.0.1) (2026-10-24)

The first release: navmesh and flight volume baking, queries, corridors,
flow fields and avoidance, with results that are bit-identical on every
platform, compiler and worker count.

### Added

- Navmesh baking, tile by tile, from triangle meshes, terrains and 2D
  outlines, with area types per triangle or cell and include, exclude
  and area volumes. Bakes check their settings and input as hostile,
  name the first setting or element refused, and report each tile's
  counts, memory, the work they gave up on and, with a clock, each
  stage's time. A tile index lets many bakes of one large mesh read only
  the triangles near each tile.
- A tile format, versioned and fingerprinted, whose loader checks every
  byte and names the section and element of a damaged tile.
- A navmesh that stages tiles and changes and commits them all at once,
  with ids that refuse a replaced tile, streaming that reports places
  with no tile instead of treating them as walls, and runtime tiers for
  area changes, link toggles and tile replacement.
- Off-mesh links point to point or edge to edge, one way or both, of six
  built-in kinds and the host's own, snapped at every commit, and
  generated drops and jumps with an optional clearance test.
- A tile cache that rebuilds tiles with obstacles to the same bytes as a
  full bake, in about a third of the time.
- Nearest points, A* paths and exact shortest paths, sliced searches,
  hierarchical paths for large worlds, rays, surface moves, heights,
  walls, polygons in a box, random points from a seed, reachability and
  grid paths, all with query filters for area costs and link kinds.
- Path corridors that follow moving agents and targets, with corners,
  area runs, shortcuts, checks and replanning after commits, and
  stateless steering toward the corners.
- Flow fields over grids, built and repaired in steps, and over
  navmeshes, built in steps.
- Flight volumes for agents that fly or swim: sparse voxel octree tiles
  baked from the same input, any-angle paths, sliced searches, rays,
  path checks and nearest open points.
- ORCA avoidance on the ground and in space, with obstacles, priorities
  and layers, the same velocities for the same agents in any order.
- Debug geometry for navmeshes, links, paths, corridors, flow fields,
  avoidance and flight volumes as plain vertices and indices.
- A shared library option, a CMake package and a pkg-config module, and
  support for Linux on x86-64 and arm64, macOS, Windows with `clang-cl`,
  and WebAssembly.

### Changed

- Tiles and flight tiles baked by earlier releases must be rebaked: they
  are refused with `mnav_errorVersion`.
- Against the withdrawn 1.x releases, `mnavQueryFilter` and
  `mnavLinkGenDef` lose their `version` field, and only the root defs
  carry `MNAV_ABI_VERSION`, which holds the whole release before 1.0. A
  program built against 1.x headers is refused with `mnav_errorVersion`
  and must be rebuilt.
