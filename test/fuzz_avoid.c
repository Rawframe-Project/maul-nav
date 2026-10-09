// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes avoidance input (mnav-0006), which a host passes every step:
// agents and obstacles read from the bytes, with NaNs, infinities, huge
// coordinates, radii and speeds of no size, ids alike, polygons wound
// either way and segments of no length, and a step of any value. A call
// must refuse with a typed status or give finite velocities no faster
// than each agent's limit; the same agents give the same velocities
// again and in the reverse order when their ids differ; drawing them
// ends in a typed status; the set gives back all it took. Bytes left
// over feed the same checks to fliers and spheres in space, and drawing
// them refuses what the call refuses.

#include "counting_allocator.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"
#include "maul-nav/draw.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    MOST_AGENTS = 24,
    MOST_OBSTACLES = 6,
    MOST_POINTS = 6
};

static void ExpectAt(bool condition, int line)
{
    if (!condition)
    {
        fprintf(stderr, "fuzz_avoid.c:%d: check failed\n", line);
        abort();
    }
}

#define Expect(condition) ExpectAt((condition), __LINE__)

typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

// A double: one time in eight its raw bits, otherwise a step of a
// quarter meter from -16 to 16.
static double Value(Reader* r)
{
    if (Byte(r) % 8 != 0)
    {
        return ((double)Byte(r) - 128.0) * 0.125;
    }
    uint64_t bits = 0;
    for (int32_t k = 0; k < 8; ++k)
    {
        bits |= (uint64_t)Byte(r) << (8 * k);
    }
    double d;
    memcpy(&d, &bits, sizeof(d));
    return d;
}

// A size: mostly from 0.1 to 2, sometimes any value.
static double Size(Reader* r)
{
    return Byte(r) % 6 != 0 ? 0.1 + (double)(Byte(r) % 20) * 0.1 : Value(r);
}

static mnavPos2 Pos(Reader* r)
{
    return (mnavPos2){Value(r), Value(r)};
}

static bool Typed(mnavResult result)
{
    return result == mnav_success || result == mnav_errorInvalid || result == mnav_errorLimit;
}

static mnavAgent s_agents[MOST_AGENTS];
static mnavAgent s_reversed[MOST_AGENTS];
static mnavPos2 s_first[MOST_AGENTS];
static mnavPos2 s_second[MOST_AGENTS];
static mnavPos2 s_points[MOST_OBSTACLES][MOST_POINTS];
static mnavObstacle s_obstacles[MOST_OBSTACLES];
static mnavAgent3D s_fliers[MOST_AGENTS];
static mnavAgent3D s_fliersReversed[MOST_AGENTS];
static mnavPos3 s_first3[MOST_AGENTS];
static mnavPos3 s_second3[MOST_AGENTS];
static mnavSphere s_spheres[MOST_OBSTACLES];
static mnavSphere s_spheresReversed[MOST_OBSTACLES];
static mnavDebugVertex s_vertices[1 << 14];
static uint32_t s_lines[1 << 14];

static int32_t ReadAgents(Reader* r, bool* distinct)
{
    int32_t count = Byte(r) % (MOST_AGENTS + 1);
    *distinct = true;
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t id = Byte(r) % 4 == 0 ? (uint64_t)(Byte(r) % 4) : 100u + (uint64_t)i;
        s_agents[i] = (mnavAgent){Pos(r), Pos(r), Pos(r), Size(r), Size(r), Size(r), id};
        for (int32_t j = 0; j < i; ++j)
        {
            *distinct = *distinct && s_agents[j].id != id;
        }
    }
    return count;
}

static int32_t ReadObstacles(Reader* r)
{
    int32_t count = Byte(r) % (MOST_OBSTACLES + 1);
    for (int32_t o = 0; o < count; ++o)
    {
        int32_t points = 1 + Byte(r) % MOST_POINTS;
        for (int32_t k = 0; k < points; ++k)
        {
            s_points[o][k] = Pos(r);
        }
        double radius = points == 1 ? Size(r) : (Byte(r) % 8 == 0 ? Size(r) : 0.0);
        s_obstacles[o] = (mnavObstacle){s_points[o], points, radius, Pos(r), (uint64_t)o + 1};
    }
    return count;
}

// Whether a step is one mnavAvoid3D takes.
static bool GoodStep(double step)
{
    return isfinite(step) && step >= MNAV_MIN_AVOIDANCE_TIME;
}

static mnavPos3 Pos3(Reader* r)
{
    return (mnavPos3){Value(r), Value(r), Value(r)};
}

static int32_t ReadFliers(Reader* r, bool* distinct)
{
    int32_t count = Byte(r) % (MOST_AGENTS + 1);
    *distinct = true;
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t id = Byte(r) % 4 == 0 ? (uint64_t)(Byte(r) % 4) : 100u + (uint64_t)i;
        s_fliers[i] = (mnavAgent3D){Pos3(r), Pos3(r), Pos3(r), Size(r), Size(r), Size(r), id};
        for (int32_t j = 0; j < i; ++j)
        {
            *distinct = *distinct && s_fliers[j].id != id;
        }
    }
    return count;
}

static int32_t ReadSpheres(Reader* r)
{
    int32_t count = Byte(r) % (MOST_OBSTACLES + 1);
    for (int32_t o = 0; o < count; ++o)
    {
        s_spheres[o] = (mnavSphere){Pos3(r), Size(r), Pos3(r), (uint64_t)o + 1};
    }
    return count;
}

// The checks of the ground call, for fliers in space.
static void Space(Reader* r, mnavAvoidance* avoidance)
{
    bool distinct = false;
    int32_t agents = ReadFliers(r, &distinct);
    int32_t spheres = ReadSpheres(r);
    double step = Byte(r) % 8 == 0 ? Value(r) : 0.05 + (double)(Byte(r) % 10) * 0.05;
    mnavResult result =
        mnavAvoid3D(avoidance, s_fliers, agents, s_spheres, spheres, step, s_first3);
    Expect(Typed(result));
    // Drawing refuses what the call refuses, given a good step.
    mnavDebugBuffer buffer = {{0.0, 0.0, 0.0}, s_vertices, 1 << 14, 0, nullptr, 0, 0,
                              s_lines,         1 << 14,    0};
    mnavResult drawn =
        mnavDebugAvoidance3D(avoidance, s_fliers, agents, s_spheres, spheres, &buffer);
    Expect(Typed(drawn) || drawn == mnav_errorCapacity);
    Expect(!GoodStep(step) ||
           (drawn == mnav_errorCapacity ? result == mnav_success : drawn == result));
    if (result != mnav_success)
    {
        return;
    }
    for (int32_t i = 0; i < agents; ++i)
    {
        mnavPos3 v = s_first3[i];
        Expect(isfinite(v.x) && isfinite(v.y) && isfinite(v.z));
        double speed = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        Expect(speed <= s_fliers[i].maxSpeed * (1.0 + 1e-9) + 1e-12);
    }
    Expect(mnavAvoid3D(avoidance, s_fliers, agents, s_spheres, spheres, step, s_second3) ==
               mnav_success &&
           memcmp(s_first3, s_second3, (size_t)agents * sizeof(mnavPos3)) == 0);
    if (!distinct)
    {
        return;
    }
    for (int32_t i = 0; i < agents; ++i)
    {
        s_fliersReversed[i] = s_fliers[agents - 1 - i];
    }
    for (int32_t o = 0; o < spheres; ++o)
    {
        s_spheresReversed[o] = s_spheres[spheres - 1 - o];
    }
    Expect(mnavAvoid3D(avoidance, s_fliersReversed, agents, s_spheresReversed, spheres, step,
                       s_second3) == mnav_success);
    for (int32_t i = 0; i < agents; ++i)
    {
        Expect(memcmp(&s_first3[i], &s_second3[agents - 1 - i], sizeof(mnavPos3)) == 0);
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    Reader r = {data, size, 0};
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.allocator = CountingAllocator();
    def.limits.agents = 1 + Byte(&r) % MOST_AGENTS;
    def.limits.neighbors = 1 + Byte(&r) % 12;
    def.limits.obstacleVertices = Byte(&r) % (MOST_OBSTACLES * MOST_POINTS + 1);
    def.neighborDistance = 1.0 + (double)(Byte(&r) % 16);
    mnavAvoidance* avoidance = nullptr;
    Expect(mnavCreateAvoidance(&def, &avoidance) == mnav_success);
    bool distinct = false;
    int32_t agents = ReadAgents(&r, &distinct);
    int32_t obstacles = ReadObstacles(&r);
    double step = Byte(&r) % 8 == 0 ? Value(&r) : 0.05 + (double)(Byte(&r) % 10) * 0.05;
    mnavResult result =
        mnavAvoid(avoidance, s_agents, agents, s_obstacles, obstacles, step, s_first);
    Expect(Typed(result));
    if (result == mnav_success)
    {
        for (int32_t i = 0; i < agents; ++i)
        {
            double speed = hypot(s_first[i].x, s_first[i].y);
            Expect(isfinite(s_first[i].x) && isfinite(s_first[i].y));
            Expect(speed <= s_agents[i].maxSpeed * (1.0 + 1e-9) + 1e-12);
        }
        // Again, the same; reversed, the same per agent when ids differ.
        Expect(mnavAvoid(avoidance, s_agents, agents, s_obstacles, obstacles, step, s_second) ==
                   mnav_success &&
               memcmp(s_first, s_second, (size_t)agents * sizeof(mnavPos2)) == 0);
        if (distinct)
        {
            for (int32_t i = 0; i < agents; ++i)
            {
                s_reversed[i] = s_agents[agents - 1 - i];
            }
            Expect(mnavAvoid(avoidance, s_reversed, agents, s_obstacles, obstacles, step,
                             s_second) == mnav_success);
            for (int32_t i = 0; i < agents; ++i)
            {
                Expect(memcmp(&s_first[i], &s_second[agents - 1 - i], sizeof(mnavPos2)) == 0);
            }
        }
    }
    mnavDebugBuffer buffer = {{0.0, 0.0, 0.0}, s_vertices, 1 << 14, 0, nullptr, 0, 0,
                              s_lines,         1 << 14,    0};
    mnavResult drawn = mnavDebugAvoidance(avoidance, s_agents, agents, Value(&r), &buffer);
    Expect(Typed(drawn) || drawn == mnav_errorCapacity);
    if (r.at < r.size)
    {
        Space(&r, avoidance);
    }
    mnavDestroyAvoidance(avoidance);
    Expect(s_held == 0);
    return 0;
}
