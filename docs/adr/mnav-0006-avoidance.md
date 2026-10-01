# mnav-0006. Avoidance

Status: Accepted

## Context

Hosts steer agents round each other locally while they follow paths.
The requirements ask for one velocity-space method, apart from the
navmesh, deterministic on every platform, with neighbours found in an
order that does not depend on how agents were added, agents with a
priority, and no time kept between calls. RVO2, the reference ORCA
library, computes in binary32, orders equal neighbours by its tree's
visiting order, has no priority, and stops agents meeting in perfect
symmetry face to face.

## Decision

- **Apart from the navmesh:** `avoidance.h` and its modules use only
  the base (results, allocator); the module graph gives them no edge to
  a navmesh module. It is a component of the one library, not a second
  library, since the family's install files export one.
- **ORCA in binary64:** `mnavAvoid` takes the agents (`mnavAgent`:
  position, velocity, preferred velocity, radius, maximum speed,
  priority, the host's id) and a step, and writes each agent's new
  velocity: the 2D linear program over its ORCA lines, the 3D one when
  they leave nothing. Lines count as parallel within 1e-9.
- **Order:** neighbours come from a grid of cells as wide as the
  neighbour range, sorted by cell, then id; each agent keeps the
  nearest by distance, then id, at most `limits.neighbors`. The same
  agents in any order give the same velocities, bit for bit.
- **Priority:** an agent takes the other's priority over the sum of
  both of each avoidance; equal priorities are plain ORCA.
- **Symmetry:** an agent held back from its preferred velocity aims 1%
  of its speed to its right; an agent left free keeps it exactly.
- **Obstacles:** `mnavObstacle` is a circle (one point and a radius),
  a segment (two points) or a counterclockwise polygon, with a velocity
  and an id. A circle gives the line of an agent that never gives way;
  edges give RVO2's obstacle lines, seen from outside only, the nearest
  first by distance, then id and vertex, at most
  `limits.obstacleNeighbors`. A moving obstacle's lines are its static
  lines, built from the agent's velocity relative to it, moved by its
  velocity. Obstacle lines are kept by the 3D program; their points
  count against `limits.obstacleVertices`.
- **Memory and limits:** an `mnavAvoidance` owns the memory for
  `limits.agents` and `limits.neighbors`; more agents is
  `mnav_errorLimit`.

## Consequences

- The host owns time: it passes the step and applies the velocities.
- Dense symmetric crowds can still jam with a short time horizon, as
  in RVO2; the horizon is the host's to set.
- Avoidance is local: an agent whose preferred velocity points into a
  block waits at its face, as in RVO2; going round is a path's work.
- Each agent tests every obstacle edge; a grid for edges waits for a
  measured need.
