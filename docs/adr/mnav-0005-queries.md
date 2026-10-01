# mnav-0005. Queries

Status: Accepted

## Context

Queries read a committed navmesh (mnav-0004) from any number of
threads. Their answers must be the same on every platform and must not
depend on how the navmesh stores its tiles, and an answer near tiles
that are not loaded must say so rather than treat them as walls.

## Decision

- **Read-only, in world coordinates:** a query takes the navmesh as
  const, positions as binary64 meters (right-handed, +Y up), and
  extents as binary32 meters; inputs that are not finite and negative
  extents are `mnav_errorInvalid`. Internally each tile's integer data
  is read in its own frame in binary64, with no contraction and no
  library transcendentals, so results are bit-identical everywhere.
- **Ties are part of the contract:** every query states how it breaks
  ties, by measured values first and then by tile place and polygon
  index, never by storage order.
- **Unloaded places are reported:** a result that may have been
  different with more tiles loaded says so.
- **Nearest point:** `mnavFindNearest` returns the polygon whose nearest
  point lies in a box round the query point and scores best: over a
  polygon, the height beyond the agent's step; otherwise the distance.
  Ties go to the shorter distance, then the tile first by place, then
  the lower polygon index. The result tells whether the point lies over
  the polygon and whether part of the box is not loaded.

## Consequences

- Queries scan the polygons of each loaded tile in their box; a search
  tree per tile is added when measurements call for it, without
  changing any answer.
