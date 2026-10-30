# Flight volumes

Agents that fly or swim are not bound to the ground, so a navmesh does not describe where they may go. A flight volume (`maul-nav/flight.h`) does: each tile is a sparse voxel octree of the space a flier's center may occupy. It is baked from the same input as the navmesh, and streamed and committed the same way.

## The def

`mnavDefaultFlightDef` gives 1 m voxels, tiles of 32 voxels, a volume from 0 to 64 m and a flier 0.5 m in radius. Then set:

- **The voxel size and tile side.** A tile's side is 4 times a power of 2, from 16 to 512 voxels. Smaller voxels thread narrower gaps and cost more memory.
- **The floor and ceiling,** in meters from the origin. Fliers never leave this range: a point below or above it given to `mnavFlightVolume_IsOpen` or a flight path search is `mnav_errorOutside`.
- **The radius.** The flier is a sphere: every voxel within its radius of something solid is closed to its center. A long, flat flier takes the sphere that holds it.
- **`enableGroundFill`.** On, the space under each column's lowest solid voxel is solid too, so the space beneath a terrain is not open. Turn it off for open space, such as a field of floating debris, where it would fill below every floating piece.

`mnavValidateFlightDef` checks a def without making anything and names the first bad setting.

## Baking and loading

- `mnavCreateFlightBaker`, then `mnavFlightBaker_BakeTile` with the same `mnavBakeInput` a navmesh bake takes, then `mnavFlightBaker_CopyTile`. A voxel any triangle touches is solid. Triangles are surfaces: the inside of a closed mesh stays open space, which no path from outside reaches. A search for an end in such a space gives `mnav_pathNone` with the start alone at once when the space holds at most 512 blocks and no more than the query's node limit (`mnavQueryLimits.nodes`); a larger one is searched until the node budget runs out, so keep path ends out of closed meshes. Tiles are independent calls, give the same bytes on every platform, and carry a fingerprint as navmesh tiles do.
- `mnavCreateFlightVolume` from a flight volume def (`mnavDefaultFlightVolumeDef`): its allocator, its limits on tiles and memory, and in `flight` the def its tiles are baked with. Then `mnavFlightVolume_StageTile` and `mnavFlightVolume_StageTileRemoval`, then `mnavFlightVolume_Commit`. Every byte is checked as hostile input, and a tile baked with other settings is refused. `mnavFlightVolume_GetTile` reads a committed tile's fingerprint.
- A world with fliers bakes each tile twice: once at the navmesh's cells, once at the flight volume's voxels.

## Queries

- `mnavFlightVolume_IsOpen` tells whether a point is open to the flier.
- `mnavFlightVolume_FindNearestPoint` finds the nearest open point within a radius. Snap a path's ends with it first.
- `mnavQuery_FindFlightPath` finds a path in an `mnavQuery` context, under its node budget and path length limit. Its points are each in sight of the next, so the flier flies straight between them.
- `mnavQuery_BeginFlightPath`, `mnavQuery_ContinueFlightPath` and `mnavQuery_FinishFlightPath` run the same search a number of nodes at a time, to spread a long search over frames; the path is the one `mnavQuery_FindFlightPath` gives. A commit between slices makes the search stale: begin again.
- `mnavFlightVolume_Raycast` stops at the first blocked voxel, the floor or ceiling, or a place with no tile.
- `mnavFlightVolume_CheckPath` casts a path's steps in order and names the first one blocked or with no tile. After a commit, check what is left of each path being followed, from where the flier is, and search again when a step fails.

Fliers avoid each other with `mnavAvoidance_Avoid3D`; see [crowds](crowds.md#fliers).
