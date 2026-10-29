# Avoidance

Local avoidance steers agents round each other and round obstacles while they follow paths. Maul Nav uses ORCA (optimal reciprocal collision avoidance) on the ground and its 3D form for fliers. RVO2, the reference library, computes in binary32, orders equal neighbors by its tree's visiting order, has no priority, and stalls agents that meet in perfect symmetry; each of those is changed here.

## Contents

- [Apart from the navmesh](#apart-from-the-navmesh)
- [The programs](#the-programs)
- [Neighbors and order](#neighbors-and-order)
- [Priority, layers and symmetry](#priority-layers-and-symmetry)
- [Obstacles](#obstacles)
- [Input ranges and edge cases](#input-ranges-and-edge-cases)
- [In space](#in-space)

## Apart from the navmesh

`avoidance.h` and its modules (`src/avoidance.c`, `avoidance3.c`, `orca.c`, `orca3.c`, `obstacle.c`, `crowd.c`) use only the base and the debug buffer; the module graph gives them no edge to a navmesh module, so a host can steer units with no navmesh at all. Walls reach avoidance as segment obstacles the host passes, from `mnavFindWalls`. It is a component of the one library rather than a library of its own, since the family's install files export one.

The host owns time: it passes every agent and the step's length each call and applies the velocities. The set keeps the memory for its limits and one hint for the neighbor grid (below), which changes the work, never a velocity.

## The programs

For each agent, `mnavAvoid` builds an ORCA line per neighbor and per nearby obstacle edge and solves the 2D linear program for the velocity nearest the preferred one within the speed circle; when the lines leave nothing, the 3D program finds the velocity that breaks them least, keeping obstacle lines hard. All of it is binary64, and lines count as parallel within 1e-9.

A velocity the programs leave past the maximum speed, which rounding does when lines meet at a narrow angle, is scaled back onto the speed circle. The scaling rounds too, so a velocity it leaves a unit outside steps each component one unit toward zero until its squared speed is no more than the squared maximum: no agent is ever given more than its maximum speed. A step takes at least a unit of the scaling's rounding off, so a few steps are enough; three were the most over 2.2e7 random velocities and maxima. Below 2^-500 m/s the squares would round absolutely, so there the velocity and the maximum are scaled by 2^600 before they are squared; that scaling is exact. A host that squares such a small velocity unscaled can find it past the squared maximum by that rounding.

## Neighbors and order

Neighbors come from a grid (`crowd.c`): each agent's key is its cell and id, the keys are sorted with a stable merge, and each occupied cell's run is found through a hash table built once a step. An agent searches the rings of cells round its own, nearest first, and stops at a ring that cannot hold a neighbor nearer than the worst it keeps. It keeps the nearest by distance, then id, up to the neighbor limit: the same set a search of every cell in range would find. So the same agents in any order give the same velocities, bit for bit.

On the ground, cells are a half to a sixth of the neighbor range, chosen from the set's last call: a call whose probed cells held under two or over six agents each sizes the next call's cells for three, one in between keeps them, and the first call uses a quarter. Measured on a quiet core, crowds of different density ran fastest at different sizes, and the size changes only the work. In space, cells are cubes half the range wide, since shells of cubes grow with the cube of their count.

## Priority, layers and symmetry

- **Priority.** Of each avoidance between two agents, one takes the other's priority over the sum of both; equal priorities are plain ORCA, half each.
- **Layers.** Agents, obstacles and spheres carry `layers`, agents `ignores`. An agent avoids another agent or an obstacle unless its `ignores` shares a bit with the other's `layers`. Zero for both avoids everything, so zero-initialized input keeps its meaning, where a mask of what to avoid would avoid nothing. Ignored agents are passed over in the neighbor search, not counted against the limit. An agent that avoids one ignoring it takes the whole avoidance instead of the priority's share, since ORCA's split assumes the other side does its part.
- **Symmetry.** Two agents meeting exactly head on can stall, each waiting for the other. An agent held back from its preferred velocity aims 1% of its speed to its right; an agent left free keeps its preferred velocity exactly.

## Obstacles

An obstacle is a circle (one point and a radius), a segment (two points) or a counterclockwise polygon, still or moving. A circle gives the line of an agent that never gives way; an agent already overlapping one must leave it within the step, but no faster than its maximum speed, so that the 3D program, which keeps obstacle lines, always can. Edges give RVO2's obstacle lines, seen from outside only, the nearest first by distance, then id and vertex. A moving obstacle's lines are its static lines built from the agent's velocity relative to it, moved by its velocity.

Obstacle candidates come from a grid of their bounds whose cells double until the entries fit eight per obstacle point; the lines do not depend on the cell size.

## Input ranges and edge cases

Input lies within named ranges: coordinates within `MNAV_MAX_AVOIDANCE_COORDINATE` (1e12 m), velocities and speeds within `MNAV_MAX_AVOIDANCE_SPEED` (1e6 m/s), radii up to `MNAV_MAX_AVOIDANCE_RADIUS`, the step and horizons at least `MNAV_MIN_AVOIDANCE_TIME` (1e-6 s). Merely finite values let an ORCA line lie so far out that its program cancels catastrophically, and an agent limited to 0.3 m/s can then be given a velocity of 1e200 m/s or more; the step and horizons scale the lines by their inverses, so a tiny step does the same. Grid cells saturate at 2^52.

A unit vector of a vector so short that its squared length is subnormal is taken by first scaling it by 2^600, exactly. Without that, an agent a hair from an obstacle corner has a vector to the corner that squares to zero, and dividing by its length gives the agent a NaN velocity. A vector of no length at all gives zero, and the obstacle lines then fall back on the edge's own constraint.

## In space

`mnavAvoid3D` steers fliers sharing a flight volume, on the same set.

- Each neighbor gives a half-space of velocities bounded by a plane, built as RVO2-3D builds it (a cut-off sphere, the cone, or parting within the step), in binary64. Exactly head on, where every side of the cone is as near, the plane's normal is square to the line between them, across world up or else world +X, so that each of the pair takes the other's opposite.
- The new velocity is RVO2-3D's 3D linear program within the speed sphere, with its 4D fallback breaking the planes least. A plane touching the speed sphere leaves a disc of no size; the projection onto its rim falls back on the disc's center, since rounding can leave no offset from the center to scale, and projecting by it would give NaN.
- Neighbors come from the same sorted grid, its cells cubes searched by shells; the table finds a column of cells along Z with one probe, since the sorted keys hold a column's cells in order. That cut the probes per agent from up to 343 to 84.
- The sidestep goes to the preferred velocity's right, +Y being up; a vertical one turns toward +X rising and -X falling, so two fliers meeting head on along Y turn apart.
- Obstacles are spheres that never give way, found through the obstacles' grid by their ground bounds, the reach counting their fastest speed and padded so that rounding at a cell's side drops none. Static geometry is the flight volume's to keep clear of, not avoidance's.
