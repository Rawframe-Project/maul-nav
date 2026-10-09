// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Avoidance in space (mnav-0006): fliers steering round each other and
// round spheres by ORCA, in any order, by priority, within the set's
// limits.

#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the swap's final positions, the same on every platform.
#define SWAP_HASH 0xd66a719793f96444ull

enum
{
    MOST = 64,
    SWAP = 26
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

static mnavAgent3D Agent(double x, double y, double z, uint64_t id)
{
    return (mnavAgent3D){{x, y, z}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.5, 1.5, 1.0, id};
}

static double Length(mnavPos3 v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static mnavPos3 Sub(mnavPos3 a, mnavPos3 b)
{
    return (mnavPos3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static double Miss(const mnavAgent3D* agent, mnavPos3 goal)
{
    return Length(Sub(agent->position, goal));
}

// Points each agent's preferred velocity at its goal, at speed 1 at most.
static void Prefer(mnavAgent3D* agents, const mnavPos3* goals, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        mnavPos3 d = Sub(goals[i], agents[i].position);
        double length = Length(d);
        double scale = length > 1.0 ? 1.0 / length : 1.0;
        agents[i].preferred = (mnavPos3){d.x * scale, d.y * scale, d.z * scale};
    }
}

// The closest any two agents are, center to center, less their radii.
static double Closest(const mnavAgent3D* agents, int32_t count)
{
    double closest = (double)INFINITY;
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t j = i + 1; j < count; ++j)
        {
            double gap = Length(Sub(agents[i].position, agents[j].position)) - agents[i].radius -
                         agents[j].radius;
            closest = gap < closest ? gap : closest;
        }
    }
    return closest;
}

// The closest any agent is to any sphere, less their radii.
static double ClosestSphere(const mnavAgent3D* agents, int32_t count, const mnavSphere* spheres,
                            int32_t sphereCount)
{
    double closest = (double)INFINITY;
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t s = 0; s < sphereCount; ++s)
        {
            double gap = Length(Sub(agents[i].position, spheres[s].center)) - agents[i].radius -
                         spheres[s].radius;
            closest = gap < closest ? gap : closest;
        }
    }
    return closest;
}

// A scene run in steps of 0.1 s.
typedef struct Scene
{
    mnavAvoidance* avoidance;
    mnavAgent3D* agents;
    const mnavPos3* goals;
    int32_t count;
    mnavSphere* spheres;
    int32_t sphereCount;
    // The closest two agents came, and an agent and a sphere.
    double closest;
    double closestSphere;
    // The fastest any agent went past its maximum speed.
    double over;
} Scene;

// Moves every agent by its new velocity and every sphere by its own.
static void Run(Scene* s, int32_t steps)
{
    mnavPos3 velocities[MOST];
    s->closest = (double)INFINITY;
    s->closestSphere = (double)INFINITY;
    s->over = 0.0;
    for (int32_t step = 0; step < steps; ++step)
    {
        Prefer(s->agents, s->goals, s->count);
        CHECK(mnavAvoid3D(s->avoidance, s->agents, s->count, s->spheres, s->sphereCount, 0.1,
                          velocities) == mnav_success,
              "stepped");
        for (int32_t i = 0; i < s->count; ++i)
        {
            mnavAgent3D* a = &s->agents[i];
            double over = Length(velocities[i]) - a->maxSpeed;
            s->over = over > s->over ? over : s->over;
            a->velocity = velocities[i];
            a->position.x += velocities[i].x * 0.1;
            a->position.y += velocities[i].y * 0.1;
            a->position.z += velocities[i].z * 0.1;
        }
        for (int32_t k = 0; k < s->sphereCount; ++k)
        {
            mnavSphere* sphere = &s->spheres[k];
            sphere->center.x += sphere->velocity.x * 0.1;
            sphere->center.y += sphere->velocity.y * 0.1;
            sphere->center.z += sphere->velocity.z * 0.1;
        }
        double closest = Closest(s->agents, s->count);
        s->closest = closest < s->closest ? closest : s->closest;
        closest = ClosestSphere(s->agents, s->count, s->spheres, s->sphereCount);
        s->closestSphere = closest < s->closestSphere ? closest : s->closestSphere;
    }
}

static int32_t Arrived(const Scene* s, double within)
{
    int32_t arrived = 0;
    for (int32_t i = 0; i < s->count; ++i)
    {
        arrived += Miss(&s->agents[i], s->goals[i]) < within ? 1 : 0;
    }
    return arrived;
}

static void TestHeadOn(void)
{
    // Straight at each other, level and then vertical: they pass.
    static const mnavPos3 axes[2] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    for (int32_t a = 0; a < 2; ++a)
    {
        mnavAvoidance* avoidance = Make(16, 10, 2.0);
        mnavPos3 d = axes[a];
        mnavAgent3D agents[2] = {Agent(-5.0 * d.x, -5.0 * d.y, 0.0, 1),
                                 Agent(5.0 * d.x, 5.0 * d.y, 0.0, 2)};
        const mnavPos3 goals[2] = {agents[1].position, agents[0].position};
        Scene s = {avoidance, agents, goals, 2, nullptr, 0, 0.0, 0.0, 0.0};
        Run(&s, 200);
        CHECK(s.closest > -1e-9, "they never touch");
        CHECK(Arrived(&s, 0.05) == 2, "both arrive");
        mnavDestroyAvoidance(avoidance);
    }
}

static void TestSwap(void)
{
    // Fliers on the 26 directions of a cube's corners, edges and faces,
    // 12 m out, each crossing to the opposite point: they crowd in the
    // middle from every side.
    mnavAvoidance* avoidance = Make(SWAP, 10, 10.0);
    mnavAgent3D agents[SWAP];
    mnavPos3 goals[SWAP];
    int32_t n = 0;
    for (int32_t x = -1; x <= 1; ++x)
    {
        for (int32_t y = -1; y <= 1; ++y)
        {
            for (int32_t z = -1; z <= 1; ++z)
            {
                if (x == 0 && y == 0 && z == 0)
                {
                    continue;
                }
                double length = sqrt((double)(x * x + y * y + z * z));
                agents[n] = Agent(12.0 * x / length, 12.0 * y / length, 12.0 * z / length,
                                  (uint64_t)(100 + n));
                goals[n] =
                    (mnavPos3){-agents[n].position.x, -agents[n].position.y, -agents[n].position.z};
                n += 1;
            }
        }
    }
    Scene s = {avoidance, agents, goals, SWAP, nullptr, 0, 0.0, 0.0, 0.0};
    Run(&s, 800);
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < SWAP; ++i)
    {
        hash = mnavHash64(hash, &agents[i].position, (int32_t)sizeof(mnavPos3));
    }
    int32_t arrived = Arrived(&s, 0.1);
    printf("SWAP_HASH=%016llx closest=%.6f arrived=%d over=%g\n", (unsigned long long)hash,
           s.closest, arrived, s.over);
    CHECK(s.closest > -0.01, "no two overlap by more than 2% of a radius");
    CHECK(arrived == SWAP, "every flier arrives");
    CHECK(s.over <= 0.0, "no flier passes its maximum speed");
    CHECK(hash == SWAP_HASH, "the pinned hash");
    mnavDestroyAvoidance(avoidance);
}

static void TestCrossing(void)
{
    // Two streams of eight crossing at right angles, one along X and one
    // along Z, in layers a little apart in height.
    mnavAvoidance* avoidance = Make(16, 10, 3.0);
    mnavAgent3D agents[16];
    mnavPos3 goals[16];
    for (int32_t i = 0; i < 8; ++i)
    {
        double lane = (double)(i % 4) * 1.5 - 2.25;
        double height = (double)(i / 4) * 1.2;
        double back = (double)(i / 4) * 2.0;
        agents[i] = Agent(-12.0 - back, height, lane, (uint64_t)(10 + i));
        goals[i] = (mnavPos3){12.0, height, lane};
        agents[8 + i] = Agent(lane, height + 0.3, -12.0 - back, (uint64_t)(30 + i));
        goals[8 + i] = (mnavPos3){lane, height + 0.3, 12.0};
    }
    Scene s = {avoidance, agents, goals, 16, nullptr, 0, 0.0, 0.0, 0.0};
    Run(&s, 600);
    printf("crossing: closest=%.6f arrived=%d\n", s.closest, Arrived(&s, 0.1));
    CHECK(s.closest > -0.01, "the streams pass through each other");
    CHECK(Arrived(&s, 0.1) == 16, "every flier arrives");
    mnavDestroyAvoidance(avoidance);
}

static uint64_t s_state = 0x5DEECE66Dull;

static double Random(double low, double high)
{
    s_state += 0x9E3779B97F4A7C15ull;
    uint64_t z = s_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return low + (high - low) * ((double)(z >> 11) * 0x1p-53);
}

static void TestAnyOrder(void)
{
    // A dense random swarm, given forward and backward: the same
    // velocities, bit for bit.
    mnavAvoidance* avoidance = Make(MOST, 10, 2.0);
    mnavAgent3D agents[MOST];
    mnavAgent3D reversed[MOST];
    for (int32_t i = 0; i < MOST; ++i)
    {
        double x = Random(-4.0, 4.0);
        double y = Random(-4.0, 4.0);
        double z = Random(-4.0, 4.0);
        agents[i] = Agent(x, y, z, (uint64_t)(1000 - i));
        agents[i].velocity = (mnavPos3){Random(-1.0, 1.0), Random(-1.0, 1.0), Random(-1.0, 1.0)};
        agents[i].preferred = (mnavPos3){-x / 4.0, -y / 4.0, -z / 4.0};
        agents[i].priority = Random(0.5, 2.0);
        reversed[MOST - 1 - i] = agents[i];
    }
    mnavSphere spheres[2] = {{{0.0, 0.0, 0.0}, 1.0, {0.0, 0.0, 0.0}, 7},
                             {{2.0, 1.0, -1.0}, 0.5, {0.5, 0.0, 0.0}, 8}};
    mnavPos3 forward[MOST];
    mnavPos3 backward[MOST];
    CHECK(mnavAvoid3D(avoidance, agents, MOST, spheres, 2, 0.1, forward) == mnav_success,
          "forward");
    mnavSphere swapped[2] = {spheres[1], spheres[0]};
    CHECK(mnavAvoid3D(avoidance, reversed, MOST, swapped, 2, 0.1, backward) == mnav_success,
          "backward");
    bool same = true;
    for (int32_t i = 0; i < MOST; ++i)
    {
        same = same && memcmp(&forward[i], &backward[MOST - 1 - i], sizeof(mnavPos3)) == 0;
    }
    CHECK(same, "the same velocities in any order");
    mnavDestroyAvoidance(avoidance);
}

static void TestPriority(void)
{
    // Head on, one with a priority a thousand times the other's: it hardly
    // turns aside and the other gives way.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent3D agents[2] = {Agent(-5.0, 0.0, 0.0, 1), Agent(5.0, 0.0, 0.0, 2)};
    agents[0].priority = 1000.0;
    const mnavPos3 goals[2] = {{5.0, 0.0, 0.0}, {-5.0, 0.0, 0.0}};
    mnavPos3 velocities[2];
    double strayFirst = 0.0;
    double straySecond = 0.0;
    for (int32_t step = 0; step < 200; ++step)
    {
        Prefer(agents, goals, 2);
        CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 0, 0.1, velocities) == mnav_success,
              "stepped");
        for (int32_t i = 0; i < 2; ++i)
        {
            agents[i].velocity = velocities[i];
            agents[i].position.x += velocities[i].x * 0.1;
            agents[i].position.y += velocities[i].y * 0.1;
            agents[i].position.z += velocities[i].z * 0.1;
        }
        double first = hypot(agents[0].position.y, agents[0].position.z);
        double second = hypot(agents[1].position.y, agents[1].position.z);
        strayFirst = first > strayFirst ? first : strayFirst;
        straySecond = second > straySecond ? second : straySecond;
    }
    CHECK(strayFirst < 0.1 * straySecond && straySecond > 0.5, "the lower priority gives way");
    mnavDestroyAvoidance(avoidance);
}

static void TestOneSpot(void)
{
    // On the same spot with the same velocity: they part.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent3D agents[2] = {Agent(1.0, 2.0, 3.0, 1), Agent(1.0, 2.0, 3.0, 2)};
    mnavPos3 velocities[2];
    CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 0, 0.1, velocities) == mnav_success,
          "stepped");
    CHECK(velocities[0].x < 0.0 && velocities[1].x > 0.0, "opposite ways");
    mnavDestroyAvoidance(avoidance);
}

static void TestSpheres(void)
{
    // A still sphere in the way and a moving one across it: the flier
    // goes round both and arrives.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent3D agents[1] = {Agent(-10.0, 0.0, 0.0, 1)};
    const mnavPos3 goals[1] = {{10.0, 0.0, 0.0}};
    mnavSphere spheres[2] = {{{-4.0, 0.0, 0.0}, 1.5, {0.0, 0.0, 0.0}, 5},
                             {{3.0, 0.0, 8.0}, 1.0, {0.0, 0.0, -1.0}, 6}};
    Scene s = {avoidance, agents, goals, 1, spheres, 2, 0.0, 0.0, 0.0};
    Run(&s, 400);
    printf("spheres: closest=%.6f arrived=%d\n", s.closestSphere, Arrived(&s, 0.1));
    CHECK(s.closestSphere > -1e-6, "it never enters a sphere");
    CHECK(Arrived(&s, 0.1) == 1, "it arrives");
    // Started inside one, it leaves within a few steps and stays out.
    agents[0] = Agent(0.5, 0.0, 0.0, 1);
    spheres[0] = (mnavSphere){{0.0, 0.0, 0.0}, 2.0, {0.0, 0.0, 0.0}, 5};
    const mnavPos3 stay[1] = {{0.5, 0.0, 0.0}};
    s = (Scene){avoidance, agents, stay, 1, spheres, 1, 0.0, 0.0, 0.0};
    Run(&s, 30);
    CHECK(ClosestSphere(agents, 1, spheres, 1) > -1e-9, "it leaves");
    mnavDestroyAvoidance(avoidance);
}

static void TestSphereKept(void)
{
    // Inside a sphere's edge with a neighbour of far higher priority
    // overlapping on the other side: the planes leave nothing, and the
    // sphere's is kept while the neighbour's gives.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent3D agents[2] = {Agent(0.0, 0.0, 0.0, 1), Agent(-0.6, 0.0, 0.0, 2)};
    agents[1].priority = 1e6;
    mnavSphere sphere = {{1.0, 0.0, 0.0}, 1.0, {0.0, 0.0, 0.0}, 9};
    mnavPos3 velocities[2];
    CHECK(mnavAvoid3D(avoidance, agents, 2, &sphere, 1, 0.1, velocities) == mnav_success,
          "stepped");
    CHECK(velocities[0].x <= -1.5 + 1e-9, "out of the sphere at full speed");
    mnavDestroyAvoidance(avoidance);
}

static void TestFastSphere(void)
{
    // A sphere coming at 3 m/s, 5 m away: the flier moves off at once,
    // taking the whole avoidance, and is never touched.
    mnavAvoidance* avoidance = Make(16, 10, 2.0);
    mnavAgent3D agents[1] = {Agent(0.0, 0.0, 0.0, 1)};
    const mnavPos3 goals[1] = {{0.0, 0.0, 0.0}};
    mnavSphere spheres[1] = {{{6.0, 0.3, 0.0}, 1.0, {-3.0, 0.0, 0.0}, 9}};
    mnavPos3 velocities[1];
    CHECK(mnavAvoid3D(avoidance, agents, 1, spheres, 1, 0.1, velocities) == mnav_success,
          "stepped");
    CHECK(Length(velocities[0]) > 0.1, "it moves off at once");
    Scene s = {avoidance, agents, goals, 1, spheres, 1, 0.0, 0.0, 0.0};
    Run(&s, 40);
    CHECK(s.closestSphere > -1e-6, "it is never touched");
    mnavDestroyAvoidance(avoidance);
}

static void TestSphereTie(void)
{
    // Room for one sphere and two at the same distance, the lower id
    // closing in and the other still: the lower id is kept, given in
    // either order, and the flier turns from it.
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.limits.obstacleNeighbors = 1;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "created");
    mnavAgent3D agent = Agent(0.0, 0.0, 0.0, 1);
    agent.preferred = (mnavPos3){1.0, 0.0, 0.0};
    mnavSphere spheres[2] = {{{0.0, 3.0, 0.0}, 1.0, {0.0, -2.0, 0.0}, 5},
                             {{0.0, -3.0, 0.0}, 1.0, {0.0, 0.0, 0.0}, 6}};
    mnavSphere swapped[2] = {spheres[1], spheres[0]};
    mnavPos3 first;
    mnavPos3 second;
    CHECK(mnavAvoid3D(avoidance, &agent, 1, spheres, 2, 0.1, &first) == mnav_success &&
              mnavAvoid3D(avoidance, &agent, 1, swapped, 2, 0.1, &second) == mnav_success,
          "stepped");
    CHECK(memcmp(&first, &agent.preferred, sizeof(mnavPos3)) != 0 &&
              memcmp(&first, &second, sizeof(mnavPos3)) == 0,
          "the lower id kept in either order");
    mnavDestroyAvoidance(avoidance);
}

static void TestChecks(void)
{
    mnavAvoidance* avoidance = Make(2, 4, 2.0);
    mnavAgent3D agents[3] = {Agent(0.0, 0.0, 0.0, 1), Agent(3.0, 0.0, 0.0, 2),
                             Agent(6.0, 0.0, 0.0, 3)};
    mnavSphere sphere = {{0.0, 5.0, 0.0}, 1.0, {0.0, 0.0, 0.0}, 9};
    mnavPos3 out[3];
    CHECK(mnavAvoid3D(nullptr, agents, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid, "no set");
    CHECK(mnavAvoid3D(avoidance, nullptr, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "no agents");
    CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 0, 0.1, nullptr) == mnav_errorInvalid,
          "no output");
    CHECK(mnavAvoid3D(avoidance, agents, -1, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a negative count");
    CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 1, 0.1, out) == mnav_errorInvalid,
          "no spheres");
    CHECK(mnavAvoid3D(avoidance, agents, 2, &sphere, -1, 0.1, out) == mnav_errorInvalid,
          "a negative sphere count");
    CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 0, 0.0, out) == mnav_errorInvalid, "no step");
    CHECK(mnavAvoid3D(avoidance, agents, 2, nullptr, 0, (double)NAN, out) == mnav_errorInvalid,
          "a step not finite");
    CHECK(mnavAvoid3D(avoidance, agents, 3, nullptr, 0, 0.1, out) == mnav_errorLimit,
          "too many agents");
    CHECK(mnavAvoid3D(avoidance, nullptr, 0, nullptr, 0, 0.1, nullptr) == mnav_success,
          "nothing to do");
    mnavAgent3D bad = agents[1];
    bad.position.z = (double)NAN;
    mnavAgent3D pair[2] = {agents[0], bad};
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a position not finite");
    pair[1] = agents[1];
    pair[1].position.z = 2.0 * MNAV_MAX_AVOIDANCE_COORDINATE;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a position out of range");
    pair[1] = agents[1];
    pair[1].preferred.z = 2.0 * MNAV_MAX_AVOIDANCE_SPEED;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a preferred velocity out of range");
    pair[1] = agents[1];
    pair[1].velocity.y = (double)INFINITY;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a velocity not finite");
    pair[1] = agents[1];
    pair[1].radius = 0.0;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid, "no radius");
    pair[1] = agents[1];
    pair[1].maxSpeed = -1.0;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "a negative speed");
    pair[1] = agents[1];
    pair[1].priority = 0.0;
    CHECK(mnavAvoid3D(avoidance, pair, 2, nullptr, 0, 0.1, out) == mnav_errorInvalid,
          "no priority");
    mnavSphere wrong = sphere;
    wrong.radius = 0.0;
    CHECK(mnavAvoid3D(avoidance, agents, 2, &wrong, 1, 0.1, out) == mnav_errorInvalid,
          "a sphere with no radius");
    wrong = sphere;
    wrong.velocity.x = (double)NAN;
    CHECK(mnavAvoid3D(avoidance, agents, 2, &wrong, 1, 0.1, out) == mnav_errorInvalid,
          "a sphere's velocity not finite");
    wrong = sphere;
    wrong.center.y = -2.0 * MNAV_MAX_AVOIDANCE_COORDINATE;
    CHECK(mnavAvoid3D(avoidance, agents, 2, &wrong, 1, 0.1, out) == mnav_errorInvalid,
          "a sphere out of range");
    mnavDestroyAvoidance(avoidance);
    // Spheres count against the set's obstacle points.
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.limits.obstacleVertices = 1;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "created");
    mnavSphere two[2] = {sphere, sphere};
    two[1].id = 10;
    CHECK(mnavAvoid3D(avoidance, agents, 2, two, 1, 0.1, out) == mnav_success, "one sphere");
    CHECK(mnavAvoid3D(avoidance, agents, 2, two, 2, 0.1, out) == mnav_errorLimit,
          "too many spheres");
    mnavDestroyAvoidance(avoidance);
}

int main(void)
{
    TestHeadOn();
    TestSwap();
    TestCrossing();
    TestAnyOrder();
    TestPriority();
    TestOneSpot();
    TestSpheres();
    TestSphereKept();
    TestFastSphere();
    TestSphereTie();
    TestChecks();
    return s_failures == 0 ? 0 : 1;
}
