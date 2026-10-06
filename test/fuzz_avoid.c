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
// ends in a typed status; the set gives back all it took.

#include "counting_allocator.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"
#include "maul-nav/draw.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    MOST_AGENTS = 24,
    MOST_OBSTACLES = 6,
    MOST_POINTS = 6
};

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

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
    mnavDestroyAvoidance(avoidance);
    Expect(s_held == 0);
    return 0;
}
