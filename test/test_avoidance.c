// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Avoidance (mnav-0006): agents steering round each other by ORCA, in any
// order, by priority, within the set's limits.

#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the circle's final positions, the same on every platform.
#define CIRCLE_HASH 0x303f5c6f58dde75aull

enum
{
    CIRCLE = 24
};

static mnavAvoidance* Make(int32_t agents, int32_t neighbors, double horizon)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.limits.agents = agents;
    def.limits.neighbors = neighbors;
    def.timeHorizon = horizon;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "created");
    return avoidance;
}

// Points each agent's preferred velocity at its goal, at speed 1 at most.
static void Prefer(mnavAgent* agents, const mnavPos2* goals, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        double dx = goals[i].x - agents[i].position.x;
        double dy = goals[i].y - agents[i].position.y;
        double length = sqrt(dx * dx + dy * dy);
        double scale = length > 1.0 ? 1.0 / length : 1.0;
        agents[i].preferred = (mnavPos2){dx * scale, dy * scale};
    }
}

// Runs steps of 0.1 s, moving every agent by its new velocity; returns the
// closest any two came, center to center, less their radii.
static double Run(mnavAvoidance* avoidance, mnavAgent* agents, const mnavPos2* goals, int32_t count,
                  int32_t steps)
{
    mnavPos2 velocities[CIRCLE];
    double closest = (double)INFINITY;
    for (int32_t s = 0; s < steps; ++s)
    {
        Prefer(agents, goals, count);
        CHECK(mnavAvoid(avoidance, agents, count, 0.1, velocities) == mnav_success, "stepped");
        for (int32_t i = 0; i < count; ++i)
        {
            agents[i].velocity = velocities[i];
            agents[i].position.x += velocities[i].x * 0.1;
            agents[i].position.y += velocities[i].y * 0.1;
        }
        for (int32_t i = 0; i < count; ++i)
        {
            for (int32_t j = i + 1; j < count; ++j)
            {
                double dx = agents[i].position.x - agents[j].position.x;
                double dy = agents[i].position.y - agents[j].position.y;
                double gap = sqrt(dx * dx + dy * dy) - agents[i].radius - agents[j].radius;
                closest = gap < closest ? gap : closest;
            }
        }
    }
    return closest;
}

static mnavAgent Agent(double x, double y, uint64_t id)
{
    return (mnavAgent){{x, y}, {0.0, 0.0}, {0.0, 0.0}, 0.5, 1.5, 1.0, id};
}

static double Miss(const mnavAgent* agent, mnavPos2 goal)
{
    double dx = agent->position.x - goal.x;
    double dy = agent->position.y - goal.y;
    return sqrt(dx * dx + dy * dy);
}

static void TestHeadOn(void)
{
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent agents[2] = {Agent(-5.0, 0.0, 1), Agent(5.0, 0.0, 2)};
    const mnavPos2 goals[2] = {{5.0, 0.0}, {-5.0, 0.0}};
    double closest = Run(avoidance, agents, goals, 2, 200);
    CHECK(closest > -1e-9, "they never touch");
    CHECK(Miss(&agents[0], goals[0]) < 0.05 && Miss(&agents[1], goals[1]) < 0.05, "both arrive");
    mnavDestroyAvoidance(avoidance);
}

static void TestCircle(void)
{
    // Agents round a circle of 20 m each cross to the opposite point,
    // looking 10 s ahead as RVO2's circle example does: at 2 s, they jam
    // in the middle, in RVO2 as here.
    mnavAvoidance* avoidance = Make(CIRCLE, 10, 10.0);
    mnavAgent agents[CIRCLE];
    mnavPos2 goals[CIRCLE];
    // The circle's points from a fixed table of directions: no libm sin.
    for (int32_t i = 0; i < CIRCLE; ++i)
    {
        double t = (double)i / CIRCLE * 4.0;
        int32_t quarter = (int32_t)t;
        double f = t - quarter;
        double u = 1.0 - 2.0 * f;
        double dx = quarter == 0 ? u : (quarter == 1 ? -1.0 : (quarter == 2 ? -u : 1.0));
        double dy = quarter == 0 ? 1.0 : (quarter == 1 ? u : (quarter == 2 ? -1.0 : -u));
        double length = sqrt(dx * dx + dy * dy);
        agents[i] = Agent(20.0 * dx / length, 20.0 * dy / length, (uint64_t)(100 + i));
        goals[i] = (mnavPos2){-agents[i].position.x, -agents[i].position.y};
    }
    double closest = Run(avoidance, agents, goals, CIRCLE, 600);
    int32_t arrived = 0;
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < CIRCLE; ++i)
    {
        arrived += Miss(&agents[i], goals[i]) < 0.1 ? 1 : 0;
        hash = mnavHash64(hash, &agents[i].position, (int32_t)sizeof(mnavPos2));
    }
    printf("CIRCLE_HASH=%016llx closest=%.6f arrived=%d\n", (unsigned long long)hash, closest,
           arrived);
    // Where the middle crowds, the 3D program breaks some lines by a hair
    // within a step: RVO2 overlaps by 5.7 mm in this same run.
    CHECK(closest > -0.01, "no two overlap by more than 2% of a radius");
    CHECK(arrived == CIRCLE, "every agent arrives");
    CHECK(hash == CIRCLE_HASH, "the pinned hash");
    mnavDestroyAvoidance(avoidance);
}

static void TestAnyOrder(void)
{
    // The same agents in another order get the same velocities, bit for
    // bit, though several stand at equal distances.
    mnavAvoidance* avoidance = Make(16, 3, 2.0);
    mnavAgent agents[6] = {Agent(0.0, 0.0, 7), Agent(1.5, 0.0, 3),  Agent(-1.5, 0.0, 9),
                           Agent(0.0, 1.5, 1), Agent(0.0, -1.5, 4), Agent(3.0, 3.0, 2)};
    for (int32_t i = 0; i < 6; ++i)
    {
        agents[i].preferred = (mnavPos2){1.0 - 0.3 * i, 0.2 * i - 0.5};
        agents[i].velocity = (mnavPos2){0.1 * i, -0.1 * i};
    }
    mnavAgent reversed[6];
    for (int32_t i = 0; i < 6; ++i)
    {
        reversed[i] = agents[5 - i];
    }
    mnavPos2 forward[6];
    mnavPos2 backward[6];
    CHECK(mnavAvoid(avoidance, agents, 6, 0.1, forward) == mnav_success &&
              mnavAvoid(avoidance, reversed, 6, 0.1, backward) == mnav_success,
          "both orders");
    bool same = true;
    for (int32_t i = 0; i < 6; ++i)
    {
        same = same && memcmp(&forward[i], &backward[5 - i], sizeof(mnavPos2)) == 0;
    }
    CHECK(same, "the same velocities");
    mnavDestroyAvoidance(avoidance);
}

static void TestPriority(void)
{
    // Head-on, the agent of far higher priority hardly turns aside.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent agents[2] = {Agent(-5.0, 0.1, 1), Agent(5.0, -0.1, 2)};
    agents[0].priority = 1000.0;
    const mnavPos2 goals[2] = {{5.0, 0.1}, {-5.0, -0.1}};
    double straying[2] = {0.0, 0.0};
    mnavPos2 velocities[2];
    for (int32_t s = 0; s < 120; ++s)
    {
        Prefer(agents, goals, 2);
        CHECK(mnavAvoid(avoidance, agents, 2, 0.1, velocities) == mnav_success, "stepped");
        for (int32_t i = 0; i < 2; ++i)
        {
            agents[i].velocity = velocities[i];
            agents[i].position.x += velocities[i].x * 0.1;
            agents[i].position.y += velocities[i].y * 0.1;
            double off = fabs(agents[i].position.y - goals[i].y);
            straying[i] = off > straying[i] ? off : straying[i];
        }
    }
    CHECK(straying[0] < 0.1 * straying[1] && straying[1] > 0.5, "the low priority gives way");
    mnavDestroyAvoidance(avoidance);
}

static void TestOneSpot(void)
{
    // Two agents on one spot with one velocity part, the lower id one way.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent agents[2] = {Agent(0.0, 0.0, 1), Agent(0.0, 0.0, 2)};
    mnavPos2 velocities[2];
    CHECK(mnavAvoid(avoidance, agents, 2, 0.1, velocities) == mnav_success &&
              isfinite(velocities[0].x) && velocities[0].x < 0.0 && velocities[1].x > 0.0,
          "parted");
    mnavDestroyAvoidance(avoidance);
}

static void TestChecks(void)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = nullptr;
    def.limits.neighbors = 0;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_errorRange && avoidance == nullptr,
          "no neighbours");
    def = mnavDefaultAvoidanceDef();
    def.timeHorizon = 0.0;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_errorRange, "no horizon");
    def = mnavDefaultAvoidanceDef();
    def.neighborDistance = (double)INFINITY;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_errorRange, "an endless range");
    def = mnavDefaultAvoidanceDef();
    def.limits.agents = MNAV_MAX_AVOIDANCE_AGENTS + 1;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_errorRange, "too many agents");
    def = mnavDefaultAvoidanceDef();
    def.cookie = 0;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_errorInvalid &&
              mnavCreateAvoidance(nullptr, &avoidance) == mnav_errorInvalid &&
              mnavCreateAvoidance(&def, nullptr) == mnav_errorInvalid,
          "bad defs");
    avoidance = Make(2, 4, 2.0);
    mnavAgent agents[3] = {Agent(0.0, 0.0, 1), Agent(3.0, 0.0, 2), Agent(6.0, 0.0, 3)};
    mnavPos2 velocities[3];
    CHECK(mnavAvoid(avoidance, agents, 3, 0.1, velocities) == mnav_errorLimit,
          "more agents than the limit");
    CHECK(mnavAvoid(avoidance, agents, 0, 0.1, nullptr) == mnav_success, "none at all");
    CHECK(mnavAvoid(avoidance, agents, 2, 0.0, velocities) == mnav_errorInvalid &&
              mnavAvoid(avoidance, agents, -1, 0.1, velocities) == mnav_errorInvalid &&
              mnavAvoid(avoidance, nullptr, 2, 0.1, velocities) == mnav_errorInvalid &&
              mnavAvoid(nullptr, agents, 2, 0.1, velocities) == mnav_errorInvalid,
          "bad calls");
    const double bad[5] = {(double)NAN, 0.0, -1.0, 0.0, 1e13};
    for (int32_t k = 0; k < 5; ++k)
    {
        mnavAgent broken[2] = {Agent(0.0, 0.0, 1), Agent(3.0, 0.0, 2)};
        broken[1].position.x = k == 0 || k == 4 ? bad[k] : broken[1].position.x;
        broken[1].radius = k == 1 ? bad[k] : broken[1].radius;
        broken[1].maxSpeed = k == 2 ? bad[k] : broken[1].maxSpeed;
        broken[1].priority = k == 3 ? bad[k] : broken[1].priority;
        CHECK(mnavAvoid(avoidance, broken, 2, 0.1, velocities) == mnav_errorInvalid,
              "a broken agent");
    }
    mnavDestroyAvoidance(avoidance);
    mnavDestroyAvoidance(nullptr);
}

int main(void)
{
    TestHeadOn();
    TestCircle();
    TestAnyOrder();
    TestPriority();
    TestOneSpot();
    TestChecks();
    return s_failures == 0 ? 0 : 1;
}
