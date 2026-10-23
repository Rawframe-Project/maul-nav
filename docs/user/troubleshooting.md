# Troubleshooting

## What a result means

| Result | Meaning | What to do |
|---|---|---|
| `mnav_errorInvalid` | An argument is wrong: a NULL pointer, a def not made by its default, a value outside its range, malformed input or tile bytes. | The result struct, where there is one, names the setting, the input element or the tile section refused. |
| `mnav_errorCapacity` | A caller buffer is too small, or the allocator failed. | A buffer call reports the size it needed; grow the buffer and call again. |
| `mnav_errorLimit` | A named limit in a def was reached, and the work it bounds was not done. | Raise the limit in the def, or split the work. A bake's report says which stage hit it. |
| `mnav_errorRange` | A coordinate is finite but too far from the frame's origin for the library to represent. | Move the origin nearer the input, or split the world into several origins. |
| `mnav_errorVersion` | Tile bytes of another format version, or a def built against headers of another library version. | Rebake the tiles, or rebuild the program against the linked library. |
| `mnav_errorNotLoaded` | The place has no tile loaded. It is not a wall: nothing is known there. | Wait for the tile, then ask again. |
| `mnav_errorStale` | An id names a tile, polygon or link replaced or removed since it was handed out, or a search or field outlived a commit. | Find the polygon again, or begin the search or field again. |
| `mnav_errorTier` | The navmesh's tier does not allow the change. | Make the navmesh with the tier the change needs. |

`mnavResultName` gives any result's name for a log.

## Common surprises

- **No polygon found.** `mnavFindNearest` returns slot 0 when no polygon's nearest point lies in its box. Widen the box upward and downward first: a point on a character's head or at its feet can lie outside a box sized for the ground.
- **A path ends short.** A path that ends `mnav_pathOutOfNodes` or `mnav_pathTooLong` is partial: raise the query def's limits, slice the search, or use a [hierarchy](paths.md#spreading-searches-over-frames) for long paths. A path that ends `mnav_pathNotLoaded` met a tile that is not loaded.
- **Agents stop at walls.** Avoidance does not know the navmesh. Feed it the walls near each agent, moved out by the agent's radius ([crowds](crowds.md#a-crowd-on-a-navmesh)).
- **An agent goes to its old target.** Setting `corridor.target` by hand leaves the corridor's last polygon where it was. Use `mnavMoveCorridorTarget` or set a new path.
- **A tile is refused when staged.** A tile is refused when it was baked with another def than the navmesh's: another cell size, tile size, origin or agent.
- **Missing walkable areas.** The bake report counts islands dropped as smaller than `minRegionArea` and away from the tile's border. A large count often means stray walkable geometry, such as the tops of walls.
- **Jumps through walls.** Link generation without a clearance test proposes jumps the host's world blocks. Pass the host's collision test in the def.
