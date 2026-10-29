# Navmesh

How the navmesh holds tiles while they stream in and out, how it names polygons safely across changes, and how off-mesh links and runtime changes ride the same commit. The container is `src/navmesh.c`; commits are `src/commit.c`, area changes `src/modifiers.c` and off-mesh links `src/offmesh.c`.

## Staged, then committed

Queries run on many threads and must never see a set of changes half applied. So nothing a query reads changes outside `mnavNavmesh_Commit`. `mnavNavmesh_StageTile` loads and checks a tile's bytes ([bake.md](bake.md#the-tile-format)) into staging, where the last change staged at a place wins. `mnavNavmesh_Commit` works out the whole result beside the live navmesh (tiles, places, cross-tile links, off-mesh links and their snaps) and swaps it in only when all of it fits within the limits; on failure nothing changes and the staged changes stay, so the host can raise a limit and commit again. No cross-tile link is ever dropped to make a commit fit.

A commit costs work in proportion to the tiles next to its changes, plus copies of the slot and place arrays.

## Ids

A tile is a 1-based slot and the slot's generation; a polygon id adds the polygon's index. Replacing or removing a tile moves its slot to the next generation, so an old id is refused with `mnav_errorStale` instead of naming whatever stands there now. Freed slots are reused lowest first, and a slot whose generation would wrap is retired rather than reused. Within one commit, slots go to staged tiles in order of their places, so the staging order cannot change a single id.

Off-mesh links have ids of the same shape, slot and generation.

## Links across tiles

At a commit, each polygon edge on a tile side links to every polygon edge on the facing side of the neighboring tile that overlaps it along the side, when the heights at both ends of the overlap are within the agent's step. The overlap is computed in integer cells, so two tiles baked separately link the same way on every machine. A link records the exact cell range it covers, which rays and the funnel use to cross tile sides at the right place. Tiles baked from different geometry (one of two rebuilt by the tile cache, say) may link along part of a side only; the parts no polygon beyond covers are walls.

## Not loaded is not a wall

A place with no committed tile is `mnav_errorNotLoaded`. Searches, rays and moves that reach one end with their own not-loaded result, and nearest-point and box queries report an `incomplete` box. Treating it as a wall would send agents round a gap that will be floor in a moment, and would cache wrong answers in the host.

## Off-mesh links

Links are runtime data, never baked, so they stream with the tiles they join and the host can add them from its own level data. `mnavStageLink` returns an id at once; the link is added at the next commit. At every commit, each link's ends snap to the nearest polygon within its radius on the ground and the agent's step in height, so a link attaches when both ends' tiles are loaded and detaches when one leaves. Only the links a commit adds and those whose snap boxes cover a changed place snap again; the result equals snapping every link, and the commit's cost follows the places changed.

An edge link is staged as crossings spaced evenly along its width, no farther apart than the agent's radius, at most `MNAV_MAX_LINK_CROSSINGS`, each in a link slot of its own that counts against `limits.links`. The id names the first; removal and toggles reach all of them. A link's ends must lie within the extent bake input may have, since merely finite ends let an edge link's direction overflow to NaN.

Searches keep each navmesh's attachments sorted by takeoff polygon and by landing polygon, so the forward searches and the backward ones (the hierarchy's, the navmesh flow field's) each find a polygon's links in one lookup.

## Tiers

The def's tier declares what may change once tiles are loaded, so a host that declares less pays for less. `mnav_tierModifiers` adds area changes (a whole polygon's area; a shape is marked on the input at bake time so polygons end where it does) and link toggles (a disabled link stays snapped but is not crossed). `mnav_tierDynamic` adds replacing a loaded tile in one commit. A changed area lasts while its tile is loaded, since a replaced tile's polygons are new polygons.

## Threads

Any number of threads may query between commits. Staging touches only staging, so one thread may stage while queries run, but no two stage or commit calls run at once, and no query runs during a commit. The navmesh counts its commits; sliced searches, hierarchies and navmesh flow fields keep the count they started from and refuse to go on after a commit, so no answer mixes two versions of the navmesh.
