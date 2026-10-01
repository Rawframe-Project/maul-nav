# mnav-0002. Bake settings and input

Status: Accepted

## Context

A bake turns geometry a host extracts from its own world into a
navmesh. Its settings decide every cell of the result, and its input
comes from outside the library, so both are checked before any work,
and the conversion from what a person picks to what the bake uses must
be the same on every platform (mnav-0001).

## Decision

- **Units:** every setting is in meters (the maximum slope in
  degrees). The library converts to cells in binary32: a quotient
  within 2^-10 of an integer counts as that integer; then heights and
  the radius round up and the step height rounds down.
  `mnavValidateBakeDef` reports the result in `mnavBakeCells`. The
  slope's cosine is the library's own function.
- **Ranges:** cell size and height from 1 mm to 10 m; tiles of 16 to
  1024 cells; the agent's radius plus a 3-cell margin no wider than a
  tile; its height and step height within 32,767 cell heights; its
  slope from 0 up to but not including 90 degrees. A value outside its
  range, or not finite, is `mnav_errorInvalid` naming the setting.
- **Limits:** the bake's named limits count work (input triangles per
  bake and per tile, spans, polygons, vertices and links per tile,
  tiles, and memory in bytes), each at least 1 and at most a published
  cap. None is in seconds.
- **Frame:** the def names an origin in world doubles; input vertices
  are binary32 meters relative to it. A vertex more than 2^22 cells
  from the origin on the ground, or more than 32,767 cell heights above
  or below it, is `mnav_errorRange`.
- **Areas:** each triangle has an area type, 0 unwalkable and 1 to 63
  walkable kinds; a mesh without areas is walkable throughout. There
  are no separate polygon flags: query filters cost and include by
  area.
- **Hostile input:** `mnavValidateTriangleMesh` refuses non-finite
  coordinates, indices outside the vertices and area types past 63,
  naming the first offending vertex, then triangle. Degenerate and
  duplicate triangles are accepted and contribute nothing.

## Consequences

A host picks settings in the units of its world and sees what cells
they became; no setting can silently lose a cell to rounding. Input
problems are found with the element that caused them before a bake
allocates anything. A world more than about 1,000 km across at 0.25 m
cells needs more than one origin.
