// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields (mnav-0007): their costs against grid paths, the ways they
// give, several goals, blocked cells and corners, limits and checks.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the random grid's field, the same on every platform.
#define FIELD_HASH 0x85ddae60124acfbaull

enum
{
    WIDTH = 48,
    HEIGHT = 36
};

static mnavAreaType s_areas[WIDTH * HEIGHT];

static mnavGrid Grid(void)
{
    return (mnavGrid){s_areas, WIDTH, HEIGHT, 0.5f};
}

// Walls, three dearer areas and open ground, from a fixed seed.
static void RandomAreas(uint32_t seed)
{
    for (int32_t c = 0; c < WIDTH * HEIGHT; ++c)
    {
        seed = seed * 1664525u + 1013904223u;
        uint32_t r = (seed >> 16) % 20u;
        s_areas[c] = r < 4u ? mnav_areaNone : (mnavAreaType)(r < 14u ? 1u : 2u + r % 3u);
    }
}

static mnavQueryFilter Costs(void)
{
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[2] = 1.5f;
    filter.costs[3] = 3.0f;
    filter.costs[4] = 7.0f;
    return filter;
}

static mnavFlowField* Make(int32_t cells)
{
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.cells = cells;
    mnavFlowField* field = nullptr;
    CHECK(mnavCreateFlowField(&def, &field) == mnav_success, "created");
    return field;
}

static mnavFlow At(const mnavFlowField* field, int32_t x, int32_t y)
{
    mnavFlow flow = {0.0, {-1, -1}};
    CHECK(mnavFlowAt(field, (mnavCell){x, y}, &flow) == mnav_success, "read");
    return flow;
}

static void TestAgainstGridPaths(void)
{
    // Every cell's cost is the cost of the cheapest grid path from it to
    // the goal, and following the field from it reaches the goal.
    RandomAreas(7u);
    mnavCell goal = {30, 20};
    s_areas[goal.y * WIDTH + goal.x] = 1u;
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    mnavFlowField* field = Make(WIDTH * HEIGHT);
    CHECK(mnavBuildFlowField(field, &grid, &filter, &goal, 1) == mnav_success, "built");
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = WIDTH * HEIGHT;
    def.limits.pathLength = 1.0e6f;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    int32_t reached = 0;
    int32_t mismatched = 0;
    int32_t lost = 0;
    for (int32_t y = 0; y < HEIGHT; ++y)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            mnavFlow flow = At(field, x, y);
            mnavGridPath path;
            CHECK(mnavFindGridPath(query, &grid, &filter, (mnavCell){x, y}, goal, &path) ==
                      mnav_success,
                  "path");
            bool found = path.end == mnav_pathFound;
            if (found != isfinite(flow.cost) ||
                (found && fabs(path.cost - flow.cost) > 1e-9 * (1.0 + path.cost)))
            {
                mismatched += 1;
            }
            if (!found)
            {
                lost += flow.next.x == x && flow.next.y == y ? 0 : 1;
                continue;
            }
            reached += 1;
            mnavCell at = {x, y};
            for (int32_t steps = 0; steps < WIDTH * HEIGHT; ++steps)
            {
                mnavFlow here = At(field, at.x, at.y);
                if (here.next.x == at.x && here.next.y == at.y)
                {
                    break;
                }
                CHECK(At(field, here.next.x, here.next.y).cost < here.cost, "downhill");
                at = here.next;
            }
            lost += at.x == goal.x && at.y == goal.y ? 0 : 1;
        }
    }
    CHECK(reached > WIDTH * HEIGHT / 2, "most cells reach the goal");
    CHECK(mismatched == 0, "the grid path's costs");
    CHECK(lost == 0, "every way ends at the goal, every other cell stays");
    mnavDestroyQuery(query);
    mnavDestroyFlowField(field);
}

static void TestSeveralGoals(void)
{
    // With several goals each cell's cost is the least of its costs to
    // each goal alone, exactly; the goals' order does not matter.
    RandomAreas(11u);
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    const mnavCell goals[3] = {{3, 4}, {40, 30}, {25, 2}};
    const mnavCell reversed[3] = {goals[2], goals[1], goals[0]};
    for (int32_t g = 0; g < 3; ++g)
    {
        s_areas[goals[g].y * WIDTH + goals[g].x] = 1u;
    }
    mnavFlowField* all = Make(WIDTH * HEIGHT);
    mnavFlowField* back = Make(WIDTH * HEIGHT);
    mnavFlowField* one[3] = {Make(WIDTH * HEIGHT), Make(WIDTH * HEIGHT), Make(WIDTH * HEIGHT)};
    CHECK(mnavBuildFlowField(all, &grid, &filter, goals, 3) == mnav_success &&
              mnavBuildFlowField(back, &grid, &filter, reversed, 3) == mnav_success,
          "built");
    for (int32_t g = 0; g < 3; ++g)
    {
        CHECK(mnavBuildFlowField(one[g], &grid, &filter, &goals[g], 1) == mnav_success, "one");
    }
    int32_t wrong = 0;
    for (int32_t y = 0; y < HEIGHT; ++y)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            double least = (double)INFINITY;
            for (int32_t g = 0; g < 3; ++g)
            {
                double cost = At(one[g], x, y).cost;
                least = cost < least ? cost : least;
            }
            mnavFlow a = At(all, x, y);
            mnavFlow b = At(back, x, y);
            wrong += a.cost == least && memcmp(&a, &b, sizeof(a)) == 0 ? 0 : 1;
        }
    }
    CHECK(wrong == 0, "the least, in any order");
    mnavDestroyFlowField(all);
    mnavDestroyFlowField(back);
    for (int32_t g = 0; g < 3; ++g)
    {
        mnavDestroyFlowField(one[g]);
    }
}

static void TestCornersAndBlocks(void)
{
    // A 4 by 3 grid:   . # .
    //                  . . #   goal at (2, 2); (2, 1) and (1, 2) blocked.
    //                  . # .   (0, 2) goes round, never across the corner.
    mnavAreaType areas[12];
    memset(areas, 1, sizeof(areas));
    areas[1 * 4 + 2] = mnav_areaNone;
    areas[2 * 4 + 1] = mnav_areaNone;
    areas[0 * 4 + 1] = mnav_areaNone;
    mnavGrid grid = {areas, 4, 3, 1.0f};
    mnavFlowField* field = Make(12);
    const mnavCell goal = {2, 2};
    CHECK(mnavBuildFlowField(field, &grid, nullptr, &goal, 1) == mnav_success, "built");
    mnavFlow corner = At(field, 1, 1);
    CHECK(!isfinite(corner.cost) && corner.next.x == 1 && corner.next.y == 1,
          "no way across blocked corners");
    mnavFlow at = At(field, 2, 2);
    CHECK(at.cost == 0.0 && at.next.x == 2 && at.next.y == 2, "the goal");
    mnavFlow wall = At(field, 1, 2);
    CHECK(!isfinite(wall.cost), "a blocked cell");
    // (3, 1) steps straight down to (3, 2), then left to the goal.
    mnavFlow side = At(field, 3, 1);
    CHECK(side.cost == 2.0 && side.next.x == 3 && side.next.y == 2, "round the corner");
    // A blocked goal is left out; with no goal left nothing is reached.
    const mnavCell blocked = {1, 2};
    CHECK(mnavBuildFlowField(field, &grid, nullptr, &blocked, 1) == mnav_success &&
              !isfinite(At(field, 0, 0).cost),
          "a blocked goal");
    // So are goals in areas the filter leaves out.
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~(uint64_t)2u;
    CHECK(mnavBuildFlowField(field, &grid, &without, &goal, 1) == mnav_success &&
              !isfinite(At(field, 2, 2).cost),
          "a left-out goal");
    mnavDestroyFlowField(field);
}

static void TestPinned(void)
{
    RandomAreas(23u);
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    const mnavCell goals[3] = {{0, 0}, {47, 35}, {20, 18}};
    mnavFlowField* field = Make(WIDTH * HEIGHT);
    CHECK(mnavBuildFlowField(field, &grid, &filter, goals, 3) == mnav_success, "built");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t y = 0; y < HEIGHT; ++y)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            mnavFlow flow = At(field, x, y);
            hash = mnavHash64(hash, &flow, (int32_t)sizeof(flow));
        }
    }
    printf("FIELD_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FIELD_HASH, "the pinned hash");
    mnavDestroyFlowField(field);
}

static void TestChecks(void)
{
    memset(s_areas, 1, sizeof(s_areas));
    mnavGrid grid = Grid();
    mnavFlowField* field = Make(100);
    mnavFlow flow;
    const mnavCell goal = {1, 1};
    CHECK(mnavFlowAt(field, goal, &flow) == mnav_errorInvalid, "nothing built");
    CHECK(mnavBuildFlowField(field, &grid, nullptr, &goal, 1) == mnav_errorLimit, "too many cells");
    grid.width = 10;
    grid.height = 10;
    CHECK(mnavBuildFlowField(field, &grid, nullptr, &goal, 1) == mnav_success &&
              mnavFlowAt(field, (mnavCell){9, 9}, &flow) == mnav_success &&
              mnavFlowAt(field, (mnavCell){10, 0}, &flow) == mnav_errorInvalid &&
              mnavFlowAt(field, (mnavCell){0, -1}, &flow) == mnav_errorInvalid,
          "cells inside the grid");
    const mnavCell outside = {10, 3};
    CHECK(mnavBuildFlowField(field, &grid, nullptr, &outside, 1) == mnav_errorInvalid &&
              mnavFlowAt(field, goal, &flow) == mnav_errorInvalid,
          "a goal outside; the field holds no grid after");
    CHECK(mnavBuildFlowField(field, &grid, nullptr, nullptr, 1) == mnav_errorInvalid &&
              mnavBuildFlowField(field, &grid, nullptr, &goal, -1) == mnav_errorInvalid &&
              mnavBuildFlowField(nullptr, &grid, nullptr, &goal, 1) == mnav_errorInvalid,
          "bad arguments");
    CHECK(mnavBuildFlowField(field, &grid, nullptr, nullptr, 0) == mnav_success &&
              !isfinite(At(field, 5, 5).cost),
          "no goals");
    mnavGrid bad = grid;
    bad.cellSize = 0.0f;
    CHECK(mnavBuildFlowField(field, &bad, nullptr, &goal, 1) == mnav_errorInvalid, "cell size");
    bad = grid;
    bad.areas = nullptr;
    CHECK(mnavBuildFlowField(field, &bad, nullptr, &goal, 1) == mnav_errorInvalid, "no areas");
    mnavQueryFilter broken = mnavDefaultQueryFilter();
    broken.costs[1] = -1.0f;
    CHECK(mnavBuildFlowField(field, &grid, &broken, &goal, 1) == mnav_errorRange, "a bad cost");
    mnavDestroyFlowField(field);
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    mnavFlowField* none = nullptr;
    def.cells = 0;
    CHECK(mnavCreateFlowField(&def, &none) == mnav_errorRange && none == nullptr, "no cells");
    def.cells = MNAV_MAX_FLOW_CELLS + 1;
    CHECK(mnavCreateFlowField(&def, &none) == mnav_errorRange, "too many");
    def = mnavDefaultFlowFieldDef();
    def.cookie = 0;
    CHECK(mnavCreateFlowField(&def, &none) == mnav_errorInvalid, "not a def");
    mnavDestroyFlowField(nullptr);
}

int main(void)
{
    TestAgainstGridPaths();
    TestSeveralGoals();
    TestCornersAndBlocks();
    TestPinned();
    TestChecks();
    return s_failures == 0 ? 0 : 1;
}
