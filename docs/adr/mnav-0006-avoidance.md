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
- **Order:** neighbours come from a grid of cells half the neighbour
  range wide, sorted by cell, then id, each occupied cell found through
  a hash table built once a step; an agent searches the rings of cells
  round its own, nearest first, and stops at a ring that cannot hold a
  neighbour nearer than the worst it keeps. It keeps the nearest by
  distance, then id, at most `limits.neighbors`, the same set a search
  of every cell in range finds. The same agents in any order give the
  same velocities, bit for bit. (The first grid had cells the whole
  range wide and found cells by binary search; the doorway benchmark of
  1000 agents took 26.3 billion instructions with it and 19.7 billion
  with this, the velocities unchanged.)
- **Priority:** an agent takes the other's priority over the sum of
  both of each avoidance; equal priorities are plain ORCA.
- **Symmetry:** an agent held back from its preferred velocity aims 1%
  of its speed to its right; an agent left free keeps it exactly.
- **Obstacles:** `mnavObstacle` is a circle (one point and a radius),
  a segment (two points) or a counterclockwise polygon, with a velocity
  and an id. A circle gives the line of an agent that never gives way;
  an agent already overlapping one must leave it within the step, but
  no faster than its maximum speed, so that the 3D program, which keeps
  obstacle lines, always can (asked to leave faster, it once kept the
  agent's preferred velocity and walked it further in);
  edges give RVO2's obstacle lines, seen from outside only, the nearest
  first by distance, then id and vertex, at most
  `limits.obstacleNeighbors`. A moving obstacle's lines are its static
  lines, built from the agent's velocity relative to it, moved by its
  velocity. Obstacle lines are kept by the 3D program; their points
  count against `limits.obstacleVertices`.
- **Memory and limits:** an `mnavAvoidance` owns the memory for
  `limits.agents` and `limits.neighbors`; more agents is
  `mnav_errorLimit`.
- **Input ranges:** coordinates within `MNAV_MAX_AVOIDANCE_COORDINATE`
  (1e12 m), velocity components and maximum speeds within
  `MNAV_MAX_AVOIDANCE_SPEED` (1e6 m/s), radii up to
  `MNAV_MAX_AVOIDANCE_RADIUS` (1e6 m), the step and horizons at least
  `MNAV_MIN_AVOIDANCE_TIME` (1e-6 s); other input is refused. Values
  merely finite let an ORCA line lie so far out that its program
  cancelled catastrophically (a velocity of 3.7e210 m/s for a limit of
  0.3, found by the fuzz target), and the step and horizons scale the
  lines by their inverses. Grid cells saturate at 2^52.
- **Speed:** a velocity the programs leave past the maximum speed,
  which rounding does when lines meet at a narrow angle (1.2e-8 of it
  in a fuzz case), is scaled back onto the speed circle, so no agent
  is ever given more than its maximum speed.
- **Short vectors:** a unit vector is taken of vectors so short that
  their squared length is subnormal by first scaling them by 2^600,
  exactly. The fuzz target found an agent a hair from an obstacle
  corner given a NaN velocity: the vector to the corner squared to 0.
  A vector of no length at all gives {0, 0}, and the obstacle lines
  then fall back on the edge's own constraint; the callers' conditions
  keep exact zeros away, so that only guards rounding.
- **In space (amended 2026-10-09):** `mnavAvoid3D` steers fliers on
  the same set, for agents sharing a flight volume (mnav-0015):
  - each neighbor gives a half-space of velocities bounded by a plane,
    built as RVO2-3D builds it (a cut-off sphere, the cone, or parting
    within the step), in binary64. Exactly head on, where every side of
    the cone is as near, the plane's normal is square to the line
    between them, across world up or else world +X, so that each of
    the pair takes the other's opposite;
  - the new velocity is RVO2-3D's 3D linear program within the speed
    sphere, with its 4D fallback breaking the planes least; planes are
    parallel within 1e-9, as lines are. A plane touching the speed
    sphere leaves a disc of no size; the projection onto its rim goes
    by the unit offset and falls back on the disc's center, which the
    fuzz target found giving NaN when rounding left no offset;
  - neighbors come from the same sorted grid, its cells cubes searched
    by shells; the ground call's results stay the same to the bit;
  - priorities split each avoidance as on the ground. The sidestep goes
    to the preferred velocity's right, +Y being up, and a vertical one
    turns toward +X rising and -X falling, so that two fliers meeting
    head on along Y turn apart rather than stall in line;
  - obstacles are spheres, still or moving, that never give way, as
    circles on the ground; they count against the set's obstacle points,
    and are found through the ground obstacles' grid by their bounds on
    the ground (X and Z), its reach counting their fastest speed in
    space, padded so that rounding at a cell's side drops none: the
    same spheres in the same order as a look at every one, which took a
    step of 1,000 fliers from 5.3 ms with no spheres to 27 ms with 4,096,
    where the grid takes 9 ms. Static geometry is the flight volume's.
  - `mnavDebugAvoidance3D` draws fliers as three 16-gons each, one in
    each axis plane, the fewest lines that show a sphere's size from any
    view, with the neighbor lines of the ground call, and spheres as
    rings of `mnav_debugObstacle`. It refuses what `mnavAvoid3D`
    refuses, through the same checks.

## Consequences

- The host owns time: it passes the step and applies the velocities.
- Dense symmetric crowds can still jam with a short time horizon, as
  in RVO2; the horizon is the host's to set.
- Avoidance is local: an agent whose preferred velocity points into a
  block waits at its face, as in RVO2; going round is a path's work.
- Obstacle candidates come from a grid of their bounds whose cells
  double until the entries fit eight per obstacle point; the lines do
  not depend on the cell size.
