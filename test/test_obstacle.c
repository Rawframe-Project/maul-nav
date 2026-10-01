// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Avoidance obstacles (mnav-0006): the lines a grid of agents gets from
// concave polygons and segments, touching them included, pinned; which
// obstacles count as near.

#include "obstacle.h"
#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>

// The hash of every line the grid of agents gets.
#define LINES_HASH 0x24500654f967d856ull

static const mnavPos2 s_l[6] = {{-8, -8}, {-2, -8}, {-2, -6}, {-6, -6}, {-6, -2}, {-8, -2}};
static const mnavPos2 s_u[8] = {{2, 2}, {9, 2}, {9, 8}, {7, 8}, {7, 4}, {4, 4}, {4, 8}, {2, 8}};
static const mnavPos2 s_bar[2] = {{-1, 3}, {1, 5}};

static mnavObstacleCell s_cells[16 * 8];
static mnavObstacleCell s_scratch[16 * 8];
static int32_t s_stamps[16];

// A grid over the vertices with cells of the side given.
static mnavObstacleGrid* Grid(const mnavObstacleVertex* vertices, int32_t count, double size)
{
    static mnavObstacleGrid grid;
    grid = (mnavObstacleGrid){s_cells, s_scratch, 0, 16 * 8, s_stamps, 0, 0.0, 0.0};
    mnavBuildObstacleGrid(&grid, vertices, count, size);
    return &grid;
}

// The hash of every line a grid of agents gets with cells of a side.
static uint64_t LinesWith(const mnavObstacleVertex* vertices, int32_t count, double size,
                          int32_t* total, int32_t* touching)
{
    mnavObstacleGrid* grid = Grid(vertices, count, size);
    mnavObstacleNear near[16];
    mnavLine lines[16];
    uint64_t hash = MNAV_HASH_INIT;
    *total = 0;
    *touching = 0;
    const mnavPos2 velocities[4] = {{1.0, 0.0}, {0.0, 1.2}, {-0.8, -0.8}, {0.0, 0.0}};
    for (int32_t gx = 0; gx < 48; ++gx)
    {
        for (int32_t gy = 0; gy < 48; ++gy)
        {
            mnavAgent agent = {{-10.0 + gx * 0.4375, -10.0 + gy * 0.4375},
                               velocities[(gx + gy) % 4],
                               {0.0, 0.0},
                               0.5,
                               1.5,
                               1.0,
                               99};
            int32_t n = mnavNearObstacles(&agent, vertices, grid, 2.0, near, 16);
            int32_t made = mnavObstacleLines(&agent, vertices, near, n, 2.0, 0.1, lines);
            for (int32_t k = 0; k < made; ++k)
            {
                *touching += lines[k].point.x == 0.0 && lines[k].point.y == 0.0 ? 1 : 0;
            }
            *total += made;
            hash = mnavHash64(hash, &made, (int32_t)sizeof(made));
            hash = mnavHash64(hash, lines, (int32_t)((size_t)made * sizeof(mnavLine)));
        }
    }
    return hash;
}

static void TestLinesAroundShapes(void)
{
    const mnavObstacle obstacles[3] = {
        {s_l, 6, 0.0, {0, 0}, 1}, {s_u, 8, 0.0, {0, 0}, 2}, {s_bar, 2, 0.0, {0.3, -0.2}, 3}};
    static mnavObstacleVertex vertices[16];
    int32_t count = 0;
    CHECK(mnavBuildObstacles(obstacles, 3, vertices, 16, &count) == mnav_success && count == 16,
          "built");
    CHECK(vertices[3].convex == false && vertices[0].convex && vertices[14].convex &&
              vertices[15].convex,
          "the L's inner corner is concave, a segment's ends convex");
    int32_t total = 0;
    int32_t touching = 0;
    uint64_t hash = LinesWith(vertices, count, 3.0, &total, &touching);
    int32_t again = 0;
    int32_t touchingAgain = 0;
    // Cells too small for the entries' room double until they fit.
    CHECK(LinesWith(vertices, count, 0.5, &again, &touchingAgain) == hash &&
              LinesWith(vertices, count, 100.0, &again, &touchingAgain) == hash,
          "the same lines with any cell size");
    printf("LINES_HASH=%016llx lines=%d touching=%d\n", (unsigned long long)hash, total, touching);
    CHECK(touching > 20, "agents touching edges and corners");
    CHECK(hash == LINES_HASH, "the pinned hash");
}

static void TestNearestAndTies(void)
{
    // Two circles at equal distances: with room for one, the lower id.
    const mnavPos2 a = {3.0, 0.0};
    const mnavPos2 b = {-3.0, 0.0};
    const mnavObstacle obstacles[2] = {{&a, 1, 1.0, {0, 0}, 9}, {&b, 1, 1.0, {0, 0}, 4}};
    mnavObstacleVertex vertices[2];
    int32_t count = 0;
    mnavObstacleNear near[2];
    mnavAgent agent = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 0.5, 1.5, 1.0, 1};
    CHECK(mnavBuildObstacles(obstacles, 2, vertices, 2, &count) == mnav_success &&
              mnavNearObstacles(&agent, vertices, Grid(vertices, count, 3.0), 2.0, near, 1) == 1 &&
              vertices[near[0].vertex].id == 4,
          "the lower id");
    // Nearer wins over a lower id, and a full list keeps the nearest.
    agent.position.x = 0.5;
    CHECK(mnavNearObstacles(&agent, vertices, Grid(vertices, count, 3.0), 2.0, near, 1) == 1 &&
              vertices[near[0].vertex].id == 9,
          "the nearer");
    // A great circle's edge near the agent, its center far beyond the
    // agent's reach: found through the circle's bounds.
    const mnavPos2 far = {25.0, 0.0};
    const mnavObstacle big = {&far, 1, 20.0, {0, 0}, 5};
    agent.position = (mnavPos2){4.6, 0.0};
    CHECK(mnavBuildObstacles(&big, 1, vertices, 2, &count) == mnav_success &&
              mnavNearObstacles(&agent, vertices, Grid(vertices, count, 3.0), 2.0, near, 1) == 1,
          "a great circle's near edge");
    // A segment of one point twice is no segment.
    const mnavPos2 same[2] = {{1.0, 1.0}, {1.0, 1.0}};
    const mnavObstacle bad = {same, 2, 0.0, {0, 0}, 1};
    CHECK(mnavBuildObstacles(&bad, 1, vertices, 2, &count) == mnav_errorInvalid,
          "a zero-length segment");
}

int main(void)
{
    TestLinesAroundShapes();
    TestNearestAndTies();
    return s_failures == 0 ? 0 : 1;
}
