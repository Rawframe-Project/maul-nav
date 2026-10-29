# The navmesh

A navmesh holds the committed tiles and off-mesh links that queries read. Changes are staged, then committed together at a point the host chooses, so a query never sees half of a change.

```c
mnavNavmesh* navmesh = NULL;
mnavBakeDefResult created = mnavCreateNavmesh(&def, &navmesh);   // the bake def
mnavTileResult staged = mnavNavmesh_StageTile(navmesh, bytes, size);     // copied
// staged.result; refused bytes are named by section and element
mnavResult committed = mnavNavmesh_Commit(navmesh);
```

- **The bake's def.** A navmesh is made from the def its tiles were baked with. A tile baked with another cell size, tile size, origin or agent is refused when staged.
- **Loading.** Tile bytes are hostile input: every field is checked before anything is allocated, and a damaged payload, a polygon that is not convex or a neighbor that does not lead back is refused with `mnavTileResult` naming the section and element. Bytes of another tile format version are `mnav_errorVersion`. The navmesh copies the bytes, so the caller's buffer can be reused at once.
- **Commits.** `mnavNavmesh_Commit` applies everything staged: tiles in and out, links across the changed tiles' sides, off-mesh links added and removed and snapped again. Either all of it applies, or nothing does and the staged changes stay. Stage from a loading thread while queries run, as long as one thread stages at a time; commit when no query runs.

## Streaming

Tiles load and unload at run time (`mnavNavmesh_StageTile`, `mnavNavmesh_StageTileRemoval`). A place with no committed tile is not a wall: a search that runs into one ends with `mnav_pathNotLoaded`, a ray with `mnav_rayNotLoaded`, and `mnavNavmesh_GetTile` there returns `mnav_errorNotLoaded`. An agent waiting on such a path waits for the tile, then searches again.

## Ids

Tiles, polygons and off-mesh links are named by ids that carry a generation. Replacing or removing a tile moves its slot to the next generation, so an id handed out before is refused with `mnav_errorStale` instead of naming whatever now stands there. A host that keeps polygon ids (in corridors, in its own maps) checks them after each commit; corridors do it with `mnavNavmesh_CheckCorridor`.

## Tiers

The def's `tier` declares what may change once tiles are loaded. Tiles and off-mesh links load and unload in every tier.

| Tier | Also allows |
|---|---|
| `mnav_tierStatic` | Nothing more. |
| `mnav_tierModifiers` | A polygon's area changed (`mnavPolygon_StageArea`, `mnav_areaNone` to block it) and links enabled or disabled (`mnavLink_StageEnable`). |
| `mnav_tierDynamic` | A loaded tile replaced in one commit, as after a rebuild with obstacles. |

A change the tier does not allow is `mnav_errorUnsupported`. An area change lasts while its tile is loaded and goes when the tile is replaced or removed. To change the area of part of a polygon, mark the shape on the input at bake time with an area volume, so that polygons end where the change does.

`mnavPolygon_GetArea` reads a polygon's committed area, `mnavNavmesh_GetTier` the navmesh's tier.

## Off-mesh links

An off-mesh link joins two points the ground does not join: a jump, a drop, a climb, a ladder, a door, a teleport, or one of the host's own kinds (6 to 63).

- `mnavStageLink` takes an `mnavLinkDef`: the two ends, how far each may snap, a cost, a kind, and whether it goes both ways. The id comes back at once; the link is added at the next commit.
- At every commit each link's ends snap to the nearest polygon within its radius, so a link attaches when both ends' tiles are loaded and detaches when one leaves. `mnavLink_GetState` says whether it is attached and enabled and where its ends snapped.
- An edge link (`width` above 0) joins two edges rather than two points, such as a long ledge to the ground below it. It is crossed at points spaced no farther apart than the agent's radius, and each crossing takes one of the def's `limits.links`: a link 16 m wide for agents of 0.5 m takes 33. Size the limit by crossings, not by links.
- `mnavStageLinkRemoval` removes a link; a link staged and not yet committed goes at once.
- Paths list the links they cross, with their kinds, so the host can play the traversal and move the agent itself.

## Generating links

`mnavQuery_GenerateLinks` proposes drops and jumps along the open edges of an `mnavTileRange` of tiles (NULL for every tile), sampled every `spacing` meters, into a caller buffer. A drop lands below the edge, down to `dropMax`, and goes both ways when no deeper than `climbMax`; a jump lands across a gap, up to `jumpMax`. A link is kept only when the navmesh does not already walk there within `detour` times the straight distance, when it is not within `filterDistance` of an earlier one, and when the host's clearance test, if the def has one, passes it.

The navmesh holds no geometry, so without a clearance test a jump over a wall between two areas the navmesh does not join is proposed too. Give the generator the host's collision test to rule those out, and stage the links worth keeping once generation returns. The test may read the navmesh, such as a ray or a nearest point, with a query context of its own, and may stage a tile or a tile's removal, which changes nothing generation reads. Any other stage call, a commit or destroy call on the navmesh, or any call on the generating context, made from the test is refused with `mnav_errorBusy` and counted as misuse (`mnavNavmesh_GetMisuse`, `mnavQuery_GetMisuse`).
