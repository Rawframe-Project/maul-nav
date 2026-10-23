# Crowds and avoidance

Avoidance steers agents round each other and round obstacles by velocity obstacles (ORCA). It is a component of its own that needs no navmesh: it works on the ground plane, or in space for fliers.

```c
mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
mnavAvoidance* avoidance = NULL;
mnavResult made = mnavCreateAvoidance(&def, &avoidance);

// Each step: positions, velocities and preferred velocities in, new
// velocities out.
mnavResult stepped =
    mnavAvoid(avoidance, agents, agentCount, obstacles, obstacleCount, 0.1, velocities);
```

- **Nothing kept between steps.** The host passes every agent each step, with the step's length, and applies the velocities it gets back; the set keeps nothing from one step that changes the next one's velocities. The same agents in any order get the same velocities, bit for bit; give agents distinct ids, since ids order neighbors at equal distances.
- **Agents** (`mnavAgent`) have a position, the velocity they had, the velocity they want, a radius, a maximum speed, a priority and an id. Between two agents, each takes the other's priority over the sum of both of the avoidance; equal priorities share it. No agent is given a velocity faster than its maximum speed.
- **Obstacles** (`mnavObstacle`) are circles, segments or counterclockwise polygons, still or moving at a velocity; agents take the whole avoidance of an obstacle.
- **Layers.** Groups that should pass through each other (ghosts, or units flying over ground ones) use bits: an agent avoids every agent and obstacle unless its `ignores` shares a bit with the other's `layers`. Zero for both avoids everything. An agent that avoids one that ignores it takes the whole avoidance, so the two still never touch.
- **Ranges.** Input has named ranges (`MNAV_MAX_AVOIDANCE_COORDINATE`, `MNAV_MAX_AVOIDANCE_SPEED`, `MNAV_MAX_AVOIDANCE_RADIUS`, `MNAV_MIN_AVOIDANCE_TIME`), wide enough for any scene, which keep the arithmetic accurate; input past them is refused.
- **Limits.** The def sets the agents and obstacle points per step, the neighbors and obstacles each agent avoids (the nearest first), the neighbor distance and the time horizons. A short horizon lets dense, symmetric crowds jam; a longer one makes agents turn earlier.

Avoidance is local. An agent whose preferred velocity points into a wall waits at its face: going round is a path's work.

## A crowd on a navmesh

The library gives the parts of a crowd and leaves the agents to the host. Each step, each agent's corridor gives its corners, `mnavSteer` the velocity it wants, `mnavAvoid` one clear of the others, and the host moves it along the surface. `samples/navcrowd.c` is a complete program.

```c
mnavSteerDef steer = mnavDefaultSteerDef();   // set maxSpeed to the agents'
for (int32_t i = 0; i < count; ++i)
{
    mnavCorners corners;
    mnavSteering steering;
    if (mnavCorridorCorners(query, navmesh, &corridors[i], &corners) == mnav_success &&
        mnavSteer(&corners, &steer, &steering) == mnav_success)
    {
        agents[i].preferred = (mnavPos2){steering.velocity.x, steering.velocity.z};
    }
}
mnavResult avoided = mnavAvoid(avoidance, agents, count, obstacles, obstacleCount, dt, velocities);
for (int32_t i = 0; i < count && avoided == mnav_success; ++i)
{
    // Limit the change from agents[i].velocity by the agent's acceleration.
    mnavPos3 at = corridors[i].position;
    mnavPos3 wanted = {at.x + velocities[i].x * dt, at.y, at.z + velocities[i].y * dt};
    if (mnavMoveCorridor(query, navmesh, NULL, &corridors[i], wanted, NULL) == mnav_success)
    {
        agents[i].position = (mnavPos2){corridors[i].position.x, corridors[i].position.z};
        agents[i].velocity = velocities[i];
    }
}
```

- **The avoidance agents** take each agent's position on the ground (x and z of its corridor's position), its last velocity, its radius and its speed.
- **Walls.** Avoidance knows nothing of the navmesh: in a packed corridor it pushes agents into the walls, and the surface move holds them back short of where avoidance planned. `mnavFindWalls` gives the walls within a range of an agent, nearest first. Find them again when the agent has moved a quarter of the range. A wall lies on the navmesh's edge, as far as an agent's center may go, so move each out along its normal by the agent's radius before passing it as a two-point obstacle; avoidance keeps an agent its radius from every obstacle, and its center then stops on the edge, where the surface move would stop it too. Agents of different radii need a list each. Skip walls of no length (normal zero), and pass each wall once.
- **Acceleration** is the host's, between avoidance and the move. `navcrowd` limits each change to 8 m/s² times the step.
- **Links.** At an off-mesh link's takeoff `mnavSteer` says `mnav_steerAtLink`: play the traversal, move the agent to the landing point, and move the corridor on.
- **Commits.** After a commit, check each corridor with `mnavCheckCorridor` and replan the ones it breaks with `mnavReplanCorridor` (`samples/obstacles.c`).

## Fliers

Fliers sharing a [flight volume](flight.md) avoid each other in space with `mnavAvoid3D`, on the same avoidance set. `mnavAgent3D` has the fields of `mnavAgent` in three dimensions, and its obstacles are spheres (`mnavSphere`), still or moving, which count against the set's obstacle points.

Walls and terrain are the flight volume's to keep clear of: a flier follows its path and avoids what moves, and the host keeps each step in open space with `mnavFlightRaycast`, sliding along the axes when a step is blocked (`samples/flight.c`). A held-back flier sidesteps to its right, +Y being up, and one rising or falling straight sidesteps toward +X or -X, so that two meeting head on pass instead of stalling.
