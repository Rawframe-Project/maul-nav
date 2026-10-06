// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Avoidance on its own, with no navmesh: two rows of eight agents walk
// through each other to swap sides, round a pillar in the middle. Each
// step the host passes every agent's position, velocity and preferred
// velocity, receives new velocities and moves the agents itself. Prints
// how close any two came; returns 0 when every agent arrives without
// overlapping another.

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdio.h>

enum
{
    AGENTS = 16,
    STEPS = 600
};

static const double STEP = 0.1;
static const double RADIUS = 0.4;
static const double SPEED = 1.5;

int main(void)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = NULL;
    if (mnavCreateAvoidance(&def, &avoidance) != mnav_success)
    {
        return 1;
    }
    mnavAgent agents[AGENTS];
    mnavPos2 goals[AGENTS];
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        double x = (double)(i % 8) * 1.5 - 5.25;
        double z = i < 8 ? -10.0 : 10.0;
        agents[i] =
            (mnavAgent){{x, z}, {0.0, 0.0}, {0.0, 0.0}, RADIUS, SPEED, 1.0, (uint64_t)i + 1};
        goals[i] = (mnavPos2){x, -z};
    }
    // A pillar of 1 m radius at the center.
    const mnavPos2 center = {0.0, 0.0};
    const mnavObstacle pillar = {&center, 1, 1.0, {0.0, 0.0}, 1};
    double closest = (double)INFINITY;
    int32_t arrived = 0;
    for (int32_t step = 0; step < STEPS && arrived < AGENTS; ++step)
    {
        for (int32_t i = 0; i < AGENTS; ++i)
        {
            double dx = goals[i].x - agents[i].position.x;
            double dz = goals[i].y - agents[i].position.y;
            double d = sqrt(dx * dx + dz * dz);
            double speed = d < SPEED * STEP ? d / STEP : SPEED;
            agents[i].preferred =
                d > 0.0 ? (mnavPos2){dx / d * speed, dz / d * speed} : (mnavPos2){0.0, 0.0};
        }
        mnavPos2 velocities[AGENTS];
        if (mnavAvoid(avoidance, agents, AGENTS, &pillar, 1, STEP, velocities) != mnav_success)
        {
            return 1;
        }
        arrived = 0;
        for (int32_t i = 0; i < AGENTS; ++i)
        {
            agents[i].velocity = velocities[i];
            agents[i].position.x += velocities[i].x * STEP;
            agents[i].position.y += velocities[i].y * STEP;
            double dx = goals[i].x - agents[i].position.x;
            double dz = goals[i].y - agents[i].position.y;
            arrived += dx * dx + dz * dz < 0.05 * 0.05 ? 1 : 0;
        }
        for (int32_t i = 0; i < AGENTS; ++i)
        {
            for (int32_t j = i + 1; j < AGENTS; ++j)
            {
                double dx = agents[j].position.x - agents[i].position.x;
                double dz = agents[j].position.y - agents[i].position.y;
                double gap = sqrt(dx * dx + dz * dz) - 2.0 * RADIUS;
                closest = gap < closest ? gap : closest;
            }
        }
    }
    printf("%d of %d agents arrived; the closest two came %.3f m apart\n", arrived, AGENTS,
           closest);
    mnavDestroyAvoidance(avoidance);
    return arrived == AGENTS && closest > -0.01 ? 0 : 1;
}
