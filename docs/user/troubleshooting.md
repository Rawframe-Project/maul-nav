# Troubleshooting

## What a result means

| Result | Meaning | What to do |
|---|---|---|
| `mnav_errorInvalid` | An argument a correct program never passes: a NULL pointer, a def not made by its default, a NaN, a value outside a range its documentation gives without a named constant, an object other than the one the call needs, malformed input geometry. | The result struct, where there is one, names the setting or the input element refused. |
| `mnav_errorCapacity` | Room is too small: a caller buffer, the allocator, or a limit the def sets (memory, polygons per tile, nodes per search, agents per step) was reached, and the work it bounds was not done. | A buffer call reports the size it needed; grow the buffer, raise the limit in the def, or split the work. A bake's report says which stage hit it. |
| `mnav_errorStale` | An id names a tile, polygon or link replaced or removed since it was handed out. | Find the polygon again. |
| `mnav_errorUnsupported` | The navmesh was made without what the change needs: its tier does not allow it, and no later call can add it. | Make the navmesh with the tier the change needs. |
| `mnav_errorState` | The object is not in a state the call takes: a search or field not begun or not ended, one that outlived a commit to its navmesh or volume, a link staged and not yet committed. | Begin the search or field again, or finish it first. |
| `mnav_errorVersion` | Tile bytes of another format version, or a def built against headers of another library version. | Rebake the tiles, or rebuild the program against the linked library. |
| `mnav_errorFormat` | Tile bytes are malformed, or were baked with other settings. | The tile result names the section and element refused. Rebake the tile. |
| `mnav_errorLimit` | An argument passes a fixed bound the library publishes as an `MNAV_MAX_` or `MNAV_MIN_` constant, an infinity included: a coordinate past the extent, a def value past its bound. More room would not help. | Keep values within the bound; for coordinates, move the origin nearer the input, or split the world into several origins. |
| `mnav_errorBusy` | The call came back in from host code the object is running: its allocator, a baker's clock, or a link generation's clearance test. One misuse is counted, which the object's `mnavGet...Misuse` call reads. | Make the call after the host code returns. An allocator or a clock may call nothing of the object it serves, or of another object the running call uses, such as the tile cache a rebuild reads. A clearance test may read the navmesh with a query context of its own and stage a tile or a tile's removal; it may not otherwise change the navmesh, or use the generating context. |
| `mnav_errorNotLoaded` | The place has no tile loaded. It is not a wall: nothing is known there. | Wait for the tile, then ask again. |

`mnavResultName` gives any result's name for a log.

## Common surprises

- **No polygon found.** `mnavFindNearest` returns slot 0 when no polygon's nearest point lies in its box. Widen the box upward and downward first: a point on a character's head or at its feet can lie outside a box sized for the ground.
- **A path ends short.** A path that ends `mnav_pathOutOfNodes` or `mnav_pathTooLong` is partial: raise the query def's limits, slice the search, or use a [hierarchy](paths.md#spreading-searches-over-frames) for long paths. A path that ends `mnav_pathNotLoaded` met a tile that is not loaded.
- **Agents stop at walls.** Avoidance does not know the navmesh. Feed it the walls near each agent, moved out by the agent's radius ([crowds](crowds.md#a-crowd-on-a-navmesh)).
- **An agent goes to its old target.** Setting `corridor.target` by hand leaves the corridor's last polygon where it was. Use `mnavMoveCorridorTarget` or set a new path.
- **A tile is refused when staged.** A tile is refused when it was baked with another def than the navmesh's: another cell size, tile size, origin or agent.
- **Missing walkable areas.** The bake report counts islands dropped as smaller than `minRegionArea` and away from the tile's border. A large count often means stray walkable geometry, such as the tops of walls.
- **Jumps through walls.** Link generation without a clearance test proposes jumps the host's world blocks. Pass the host's collision test in the def.
