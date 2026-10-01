# mnav-0004. The navmesh and its commits

Status: Accepted

## Context

Queries read a navmesh made of baked tiles (mnav-0003). Tiles arrive,
are replaced and leave while the host runs: streaming, rebakes after
the world changes. Queries run on many threads at once and must never
see a set of changes half applied, nor follow a name to a polygon that
is gone (record 0016).

## Decision

- **Made from the bake's def:** `mnavCreateNavmesh` takes the def the
  tiles were baked with; a tile baked with other cell sizes, tile size,
  origin or agent is refused when staged.
- **Staged, then committed:** `mnavStageTile` loads and checks a tile's
  bytes (mnav-0003) and stages it; `mnavStageTileRemoval` stages a
  removal; the last change staged at a place wins. `mnavCommit` works
  out the whole result beside the navmesh, tiles, places and links, and
  swaps it in only when all of it fits; on failure nothing changes and
  the staged changes stay. Queries run between commits, never during
  one.
- **Ids:** a tile is a 1-based slot and the slot's generation; a polygon
  adds its index. Replacing or removing a tile moves its slot to the
  next generation, so old ids are refused; a freed slot is reused,
  lowest first, and a slot whose generation would wrap is retired.
  Within one commit, slots go to staged tiles in order of place, so the
  staging order does not matter.
- **Links across tiles:** a polygon edge on a tile side links to every
  polygon edge on the facing side of the tile next to it that overlaps
  it along the side, with heights within the agent's step at both ends
  of the overlap, computed in integers. A tile's links beyond its
  tileLinks limit fail the commit; no link is dropped.
- **Not loaded:** a place with no committed tile is
  `mnav_errorNotLoaded`, not a wall.

- **Off-mesh links:** points joined where the ground does not join
  them (jump, drop, climb, ladder, door, teleport, and host kinds up to
  64), one-way or two-way, with a cost. They are runtime data, never
  baked: `mnavStageLink` returns an id at once and the link is added at
  the next commit, `mnavStageLinkRemoval` removes it there, and both
  ride the same all-or-nothing commit as tiles. At every commit each
  link's ends snap again to the nearest polygon within its radius on
  the ground and the agent's step in height, so a link attaches when
  both ends' tiles are loaded and detaches when one leaves;
  `mnavGetLink` reads which. A navmesh holds at most `limits.links`.

## Consequences

- A commit costs work proportional to the tiles next to its changes,
  plus copies of the slot and place arrays.
- Hosts commit at a point of their choosing, typically between their
  own frame phases, and stage from loading threads in the meantime as
  long as one thread stages at a time.
