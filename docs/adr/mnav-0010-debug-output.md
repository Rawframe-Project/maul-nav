# mnav-0010. Debug output

Status: Accepted

## Context

The requirements ask for debug geometry (navmesh polygons, tiles,
links, paths, corridors, flow-field arrows, avoidance neighbours) as
plain vertex and index data any renderer can draw, the library never
drawing. Detour's DebugUtils calls a host's immediate-mode interface
and picks colours itself.

## Decision

- **Retained buffers:** `mnavDebugBuffer` (draw.h) is a caller's vertex
  array and two index lists, triangles and lines, with capacities and
  counts. Debug functions append, each primitive with vertices of its
  own, and count past a capacity without writing there, returning
  `mnav_errorCapacity` so the host can size the buffer from the counts.
- **Kinds, not colours:** a vertex is three floats relative to the
  buffer's origin, what it shows (`mnavDebugKind`) and a value: area
  type for polygon fills, whether tile links join a tile side, link
  kind with 256 added when detached or disabled, place in a corridor.
- **Functions:** `mnavDebugNavmesh` (detail triangles, inner edges once,
  walls, tile sides, tile bounds, for a range of tile places),
  `mnavDebugLinks` (an arc of eight lines rising a quarter of the
  link's length), `mnavDebugPath`, `mnavDebugCorridor` (debug.h);
  `mnavDebugFlowField` (an arrow with two barbs per cell with a next
  one, in flow.h); `mnavDebugAvoidance` (a 16-gon per agent and a line
  to each neighbour mnavAvoid would give it, in avoidance.h).
- **Modules:** the appending helper uses the base alone, so avoidance
  still uses no navmesh module; circles are drawn from corners written
  out as constants, the same on every platform.
- **Bake reports** keep counts, the memory peak and the work given up
  on, and for each stage the most memory held while it ran and its
  time. The library keeps no clock: a host that wants times passes one
  in the bake def (`mnavClock`), read between stages; neither the clock
  nor the times reach the tile or its fingerprint. A host timing the
  call alone could not see the stages inside it.

## Consequences

- Positions are floats near the origin the host picks, so large worlds
  keep their precision where the camera is.
- Primitives share no vertices: simpler, at the cost of repeated
  corners.
