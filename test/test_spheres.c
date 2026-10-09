// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spheres near a flier (mnav-0006): found through the obstacle grid,
// they are those a look at every sphere finds, in the same order, for
// shared spots, repeated ids, fast spheres and cells of any size.

#include "avoidance.h"
#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>

enum
{
    SPHERES = 1500,
    AGENTS = 300,
    LIMIT = 12
};

static uint64_t s_state = 0x853C49E6748FEA9Bull;

// A draw in [0, 1), splitmix64's.
static double Random(void)
{
    s_state += 0x9E3779B97F4A7C15ull;
    uint64_t z = s_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return (double)(z >> 11) * 0x1p-53;
}

static mnavSphere s_spheres[SPHERES];
static mnavAgent3D s_agents[AGENTS];

static bool Before(mnavObstacleNear a, mnavObstacleNear b)
{
    if (a.distance != b.distance)
    {
        return a.distance < b.distance;
    }
    uint64_t ia = s_spheres[a.vertex].id;
    uint64_t ib = s_spheres[b.vertex].id;
    return ia != ib ? ia < ib : a.vertex < b.vertex;
}

// The spheres near an agent by looking at every one.
static int32_t Every(const mnavAgent3D* agent, double horizon, mnavObstacleNear* list)
{
    int32_t count = 0;
    for (int32_t s = 0; s < SPHERES; ++s)
    {
        const mnavSphere* sphere = &s_spheres[s];
        mnavPos3 v = sphere->velocity;
        double speed = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        double range = horizon * (agent->maxSpeed + speed) + agent->radius;
        double dx = sphere->center.x - agent->position.x;
        double dy = sphere->center.y - agent->position.y;
        double dz = sphere->center.z - agent->position.z;
        double gap = sqrt(dx * dx + dy * dy + dz * dz) - sphere->radius;
        gap = gap > 0.0 ? gap : 0.0;
        if (gap >= range)
        {
            continue;
        }
        mnavObstacleNear candidate = {gap * gap, s};
        if (count == LIMIT && !Before(candidate, list[count - 1]))
        {
            continue;
        }
        int32_t i = count < LIMIT ? count++ : count - 1;
        while (i > 0 && Before(candidate, list[i - 1]))
        {
            list[i] = list[i - 1];
            i -= 1;
        }
        list[i] = candidate;
    }
    return count;
}

// Spheres and agents in a box, on a coarse lattice so that many share a
// spot or a distance, ids repeating, a few spheres fast: across, or
// with vertical set, only up or down, so that their speed on the ground
// says nothing of it.
static void Scatter(double extent, bool vertical)
{
    for (int32_t s = 0; s < SPHERES; ++s)
    {
        double x = (double)(int32_t)(Random() * 24.0) * extent / 24.0;
        double y = (double)(int32_t)(Random() * 8.0) * extent / 24.0;
        double z = (double)(int32_t)(Random() * 24.0) * extent / 24.0;
        double fast = Random() < 0.05 ? 20.0 : 1.0;
        double vx = (Random() - 0.5) * (vertical ? 1.0 : fast);
        double vy = (Random() - 0.5) * fast;
        s_spheres[s] = (mnavSphere){{x, y, z},
                                    0.25 + (double)(int32_t)(Random() * 4.0) * 0.5,
                                    {vx, vy, 0.0},
                                    (uint64_t)(Random() * 400.0),
                                    0};
    }
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        double x = Random() * extent;
        double y = Random() * extent / 3.0;
        double z = Random() * extent;
        s_agents[i] =
            (mnavAgent3D){{x, y, z}, {0, 0, 0}, {0, 0, 0}, 0.3, 1.5, 1.0, (uint64_t)i, 0, 0};
    }
}

static void Compare(double extent, double cell, bool vertical)
{
    Scatter(extent, vertical);
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.limits.obstacleVertices = SPHERES;
    def.limits.obstacleNeighbors = LIMIT;
    def.neighborDistance = cell;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "a set");
    mnavBuildSphereGrid(avoidance, s_spheres, SPHERES);
    int32_t mismatches = 0;
    int32_t found = 0;
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        mnavObstacleNear expected[LIMIT];
        mnavObstacleNear got[LIMIT];
        int32_t want = Every(&s_agents[i], def.obstacleTimeHorizon, expected);
        int32_t count = mnavNearSpheres(avoidance, &s_agents[i], s_spheres, SPHERES, got);
        bool same = count == want;
        for (int32_t k = 0; same && k < count; ++k)
        {
            same = got[k].vertex == expected[k].vertex && got[k].distance == expected[k].distance;
        }
        mismatches += same ? 0 : 1;
        found += count;
    }
    CHECK(mismatches == 0 && found > 0, "the spheres a look at every one finds");
    mnavDestroyAvoidance(avoidance);
}

int main(void)
{
    Compare(60.0, 10.0, false);
    Compare(60.0, 0.5, false);
    Compare(400.0, 3.0, false);
    Compare(8.0, 1.0, false);
    Compare(60.0, 1.0, true);
    return s_failures == 0 ? 0 : 1;
}
