// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields: their costs against grid paths, the ways they
// give, several goals, blocked cells and corners, limits and checks.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/draw.h"
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

static mnavGridFlow* Make(int32_t cells)
{
    mnavGridFlowDef def = mnavDefaultGridFlowDef();
    def.cells = cells;
    mnavGridFlow* field = nullptr;
    CHECK(mnavCreateGridFlow(&def, &field) == mnav_success, "created");
    return field;
}

static mnavCellFlow At(const mnavGridFlow* field, int32_t x, int32_t y)
{
    mnavCellFlow flow = {0.0, {-1, -1}};
    CHECK(mnavGridFlow_GetFlow(field, (mnavCell){x, y}, &flow) == mnav_success, "read");
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
    mnavGridFlow* field = Make(WIDTH * HEIGHT);
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, &filter) == mnav_success, "built");
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
            mnavCellFlow flow = At(field, x, y);
            mnavGridPath path;
            CHECK(mnavQuery_FindGridPath(query, &grid, (mnavCell){x, y}, goal, &filter, &path) ==
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
                mnavCellFlow here = At(field, at.x, at.y);
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
    mnavDestroyGridFlow(field);
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
    mnavGridFlow* all = Make(WIDTH * HEIGHT);
    mnavGridFlow* back = Make(WIDTH * HEIGHT);
    mnavGridFlow* one[3] = {Make(WIDTH * HEIGHT), Make(WIDTH * HEIGHT), Make(WIDTH * HEIGHT)};
    CHECK(mnavGridFlow_Build(all, &grid, goals, 3, &filter) == mnav_success &&
              mnavGridFlow_Build(back, &grid, reversed, 3, &filter) == mnav_success,
          "built");
    for (int32_t g = 0; g < 3; ++g)
    {
        CHECK(mnavGridFlow_Build(one[g], &grid, &goals[g], 1, &filter) == mnav_success, "one");
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
            mnavCellFlow a = At(all, x, y);
            mnavCellFlow b = At(back, x, y);
            wrong += a.cost == least && memcmp(&a, &b, sizeof(a)) == 0 ? 0 : 1;
        }
    }
    CHECK(wrong == 0, "the least, in any order");
    mnavDestroyGridFlow(all);
    mnavDestroyGridFlow(back);
    for (int32_t g = 0; g < 3; ++g)
    {
        mnavDestroyGridFlow(one[g]);
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
    mnavGridFlow* field = Make(12);
    const mnavCell goal = {2, 2};
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, nullptr) == mnav_success, "built");
    mnavCellFlow corner = At(field, 1, 1);
    CHECK(!isfinite(corner.cost) && corner.next.x == 1 && corner.next.y == 1,
          "no way across blocked corners");
    mnavCellFlow at = At(field, 2, 2);
    CHECK(at.cost == 0.0 && at.next.x == 2 && at.next.y == 2, "the goal");
    mnavCellFlow wall = At(field, 1, 2);
    CHECK(!isfinite(wall.cost), "a blocked cell");
    // (3, 1) steps straight down to (3, 2), then left to the goal.
    mnavCellFlow side = At(field, 3, 1);
    CHECK(side.cost == 2.0 && side.next.x == 3 && side.next.y == 2, "round the corner");
    // A blocked goal is left out; with no goal left nothing is reached.
    const mnavCell blocked = {1, 2};
    CHECK(mnavGridFlow_Build(field, &grid, &blocked, 1, nullptr) == mnav_success &&
              !isfinite(At(field, 0, 0).cost),
          "a blocked goal");
    // So are goals in areas the filter leaves out.
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~(uint64_t)2u;
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, &without) == mnav_success &&
              !isfinite(At(field, 2, 2).cost),
          "a left-out goal");
    mnavDestroyGridFlow(field);
}

static void TestPinned(void)
{
    RandomAreas(23u);
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    const mnavCell goals[3] = {{0, 0}, {47, 35}, {20, 18}};
    mnavGridFlow* field = Make(WIDTH * HEIGHT);
    CHECK(mnavGridFlow_Build(field, &grid, goals, 3, &filter) == mnav_success, "built");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t y = 0; y < HEIGHT; ++y)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            mnavCellFlow flow = At(field, x, y);
            hash = mnavHash64(hash, &flow, (int64_t)sizeof(flow));
        }
    }
    printf("FIELD_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FIELD_HASH, "the pinned hash");
    mnavDestroyGridFlow(field);
}

static void TestChecks(void)
{
    memset(s_areas, 1, sizeof(s_areas));
    mnavGrid grid = Grid();
    mnavGridFlow* field = Make(100);
    mnavCellFlow flow;
    const mnavCell goal = {1, 1};
    CHECK(mnavGridFlow_GetFlow(field, goal, &flow) == mnav_errorState, "nothing built");
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, nullptr) == mnav_errorCapacity,
          "too many cells");
    grid.width = 101;
    grid.height = 1;
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, nullptr) == mnav_errorInvalid, "goal off");
    const mnavCell first = {0, 0};
    CHECK(mnavGridFlow_Build(field, &grid, &first, 1, nullptr) == mnav_errorCapacity,
          "one cell past the limit");
    grid.width = 10;
    grid.height = 10;
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, nullptr) == mnav_success &&
              mnavGridFlow_GetFlow(field, (mnavCell){9, 9}, &flow) == mnav_success &&
              mnavGridFlow_GetFlow(field, (mnavCell){10, 0}, &flow) == mnav_errorOutside &&
              mnavGridFlow_GetFlow(field, (mnavCell){0, -1}, &flow) == mnav_errorOutside,
          "cells inside the grid");
    const mnavCell outside = {10, 3};
    CHECK(mnavGridFlow_Build(field, &grid, &outside, 1, nullptr) == mnav_errorInvalid &&
              mnavGridFlow_GetFlow(field, goal, &flow) == mnav_errorState,
          "a goal outside; the field holds no grid after");
    CHECK(mnavGridFlow_Build(field, &grid, nullptr, 1, nullptr) == mnav_errorInvalid &&
              mnavGridFlow_Build(field, &grid, &goal, -1, nullptr) == mnav_errorInvalid &&
              mnavGridFlow_Build(nullptr, &grid, &goal, 1, nullptr) == mnav_errorInvalid,
          "bad arguments");
    CHECK(mnavGridFlow_Build(field, &grid, nullptr, 0, nullptr) == mnav_success &&
              !isfinite(At(field, 5, 5).cost),
          "no goals");
    mnavGrid bad = grid;
    bad.cellSize = 0.0f;
    CHECK(mnavGridFlow_Build(field, &bad, &goal, 1, nullptr) == mnav_errorInvalid, "cell size");
    bad = grid;
    bad.areas = nullptr;
    CHECK(mnavGridFlow_Build(field, &bad, &goal, 1, nullptr) == mnav_errorInvalid, "no areas");
    bad = grid;
    bad.width = MNAV_MAX_GRID_SIDE + 1;
    CHECK(mnavGridFlow_Build(field, &bad, &goal, 1, nullptr) == mnav_errorLimit, "too wide");
    mnavQueryFilter broken = mnavDefaultQueryFilter();
    broken.costs[1] = -1.0f;
    CHECK(mnavGridFlow_Build(field, &grid, &goal, 1, &broken) == mnav_errorLimit, "a bad cost");
    mnavDestroyGridFlow(field);
    mnavGridFlowDef def = mnavDefaultGridFlowDef();
    mnavGridFlow* none = nullptr;
    def.cells = 0;
    CHECK(mnavCreateGridFlow(&def, &none) == mnav_errorInvalid && none == nullptr, "no cells");
    def.cells = MNAV_MAX_GRID_FLOW_CELLS + 1;
    CHECK(mnavCreateGridFlow(&def, &none) == mnav_errorLimit, "too many");
    def = mnavDefaultGridFlowDef();
    def.cookie = 0;
    CHECK(mnavCreateGridFlow(&def, &none) == mnav_errorInvalid, "not a def");
    mnavDestroyGridFlow(nullptr);
}

// A region by its corner and size, as the checks below read it, and the
// cells it covers.
typedef struct Window
{
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
} Window;

static mnavCellRange Cells(Window w)
{
    return (mnavCellRange){w.x, w.y, w.x + w.width - 1, w.y + w.height - 1};
}

// The hash of every cell's way in a region.
static uint64_t RegionHash(const mnavGridFlow* field, Window r)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t y = r.y; y < r.y + r.height; ++y)
    {
        for (int32_t x = r.x; x < r.x + r.width; ++x)
        {
            mnavCellFlow flow = At(field, x, y);
            hash = mnavHash64(hash, &flow, (int64_t)sizeof(flow));
        }
    }
    return hash;
}

static mnavAreaType s_window[WIDTH * HEIGHT];

// A field over a region of the grid is the field of the region's cells
// as a grid of their own.
static void TestRegion(void)
{
    RandomAreas(41u);
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    const Window r = {9, 5, 30, 22};
    for (int32_t y = 0; y < r.height; ++y)
    {
        memcpy(&s_window[y * r.width], &s_areas[(r.y + y) * WIDTH + r.x], (size_t)r.width);
    }
    const mnavGrid window = {s_window, r.width, r.height, grid.cellSize};
    // One goal outside the region, left out.
    const mnavCell goals[3] = {{12, 7}, {30, 20}, {2, 2}};
    const mnavCell local[2] = {{3, 2}, {21, 15}};
    s_areas[7 * WIDTH + 12] = 1;
    s_areas[20 * WIDTH + 30] = 1;
    s_window[2 * r.width + 3] = 1;
    s_window[15 * r.width + 21] = 1;
    mnavGridFlow* field = Make(r.width * r.height);
    mnavGridFlow* alone = Make(r.width * r.height);
    CHECK(mnavGridFlow_Build(field, &grid, goals, 3, &filter) == mnav_errorCapacity,
          "the whole grid is past the limit");
    bool ended = false;
    const mnavCellRange cells = Cells(r);
    CHECK(mnavGridFlow_Begin(field, &grid, &cells, goals, 3, &filter) == mnav_success &&
              mnavGridFlow_Continue(field, &grid, INT32_MAX, &ended) == mnav_success && ended &&
              mnavGridFlow_Build(alone, &window, local, 2, &filter) == mnav_success,
          "a region within the limit");
    int32_t same = 0;
    for (int32_t y = 0; y < r.height; ++y)
    {
        for (int32_t x = 0; x < r.width; ++x)
        {
            mnavCellFlow a = At(field, r.x + x, r.y + y);
            mnavCellFlow b = At(alone, x, y);
            same += a.cost == b.cost && a.next.x == b.next.x + r.x && a.next.y == b.next.y + r.y;
        }
    }
    CHECK(same == r.width * r.height, "the region's own field");
    mnavCellFlow flow = {-1.0, {-1, -1}};
    CHECK(mnavGridFlow_GetFlow(field, (mnavCell){r.x - 1, r.y}, &flow) == mnav_errorOutside &&
              mnavGridFlow_GetFlow(field, (mnavCell){r.x, r.y + r.height}, &flow) ==
                  mnav_errorOutside &&
              flow.cost == 0.0 && flow.next.x == 0 && flow.next.y == 0,
          "cells outside the region, the way zeroed");
    static mnavDebugVertex vertices[WIDTH * HEIGHT * 6];
    static uint32_t lines[WIDTH * HEIGHT * 6];
    mnavDebugBuffer buffer = {{0, 0, 0}, vertices, WIDTH * HEIGHT * 6, 0, nullptr, 0,
                              0,         lines,    WIDTH * HEIGHT * 6, 0};
    float least = 1e9f;
    CHECK(mnavGridFlow_Debug(field, 0.0, &buffer) == mnav_success, "drawn");
    for (int32_t i = 0; i < buffer.vertexCount; ++i)
    {
        least = vertices[i].x < least ? vertices[i].x : least;
    }
    CHECK(least >= (float)r.x * grid.cellSize && least < (float)(r.x + 1) * grid.cellSize,
          "arrows at their grid places");
    mnavCellRange bad = {40, 0, 48, 0};
    CHECK(mnavGridFlow_Begin(field, &grid, &bad, goals, 3, &filter) == mnav_errorInvalid &&
              mnavGridFlow_GetFlow(field, goals[0], &flow) == mnav_errorState,
          "a region past the grid's side; the field holds nothing after");
    bad = (mnavCellRange){0, 30, 4, 36};
    CHECK(mnavGridFlow_Begin(field, &grid, &bad, goals, 3, &filter) == mnav_errorInvalid,
          "a region past the grid's foot");
    bad = (mnavCellRange){1, 0, 0, 4};
    CHECK(mnavGridFlow_Begin(field, &grid, &bad, goals, 3, &filter) == mnav_errorInvalid,
          "a region backward");
    bad = (mnavCellRange){-1, 0, 3, 4};
    CHECK(mnavGridFlow_Begin(field, &grid, &bad, goals, 3, &filter) == mnav_errorInvalid,
          "a region before the grid");
    bad = (mnavCellRange){0, 0, 30, 21};
    CHECK(mnavGridFlow_Begin(field, &grid, &bad, goals, 3, &filter) == mnav_errorCapacity,
          "a region past the limit");
    mnavDestroyGridFlow(field);
    mnavDestroyGridFlow(alone);
}

// Any budget gives the field a build gives; reads wait for the end.
static void TestSteps(void)
{
    RandomAreas(23u);
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    const mnavCell goals[3] = {{0, 0}, {47, 35}, {20, 18}};
    mnavGridFlow* field = Make(WIDTH * HEIGHT);
    const Window whole = {0, 0, WIDTH, HEIGHT};
    const int32_t budgets[3] = {1, 7, 500};
    for (int32_t b = 0; b < 3; ++b)
    {
        CHECK(mnavGridFlow_Begin(field, &grid, nullptr, goals, 3, &filter) == mnav_success,
              "begun");
        mnavCellFlow flow;
        CHECK(mnavGridFlow_GetFlow(field, goals[0], &flow) == mnav_errorState,
              "no reads while working");
        bool ended = false;
        int32_t steps = 0;
        while (!ended)
        {
            CHECK(mnavGridFlow_Continue(field, &grid, budgets[b], &ended) == mnav_success, "step");
            steps += 1;
        }
        CHECK(RegionHash(field, whole) == FIELD_HASH, "the build's field");
        CHECK(steps >= (WIDTH * HEIGHT * 4 / 5) / budgets[b], "steps of the budget");
    }
    bool ended = false;
    CHECK(mnavGridFlow_Continue(field, &grid, 1, &ended) == mnav_success && ended,
          "continuing an ended field");
    // The grid may move in memory between steps.
    CHECK(mnavGridFlow_Begin(field, &grid, nullptr, goals, 3, &filter) == mnav_success &&
              mnavGridFlow_Continue(field, &grid, 100, nullptr) == mnav_success,
          "a first step");
    memcpy(s_window, s_areas, sizeof(s_areas));
    mnavGrid moved = {s_window, WIDTH, HEIGHT, grid.cellSize};
    CHECK(mnavGridFlow_Continue(field, &moved, INT32_MAX, &ended) == mnav_success && ended &&
              RegionHash(field, whole) == FIELD_HASH,
          "the grid moved");
    mnavGrid other = grid;
    other.height = HEIGHT - 1;
    CHECK(mnavGridFlow_Continue(field, &other, 1, &ended) == mnav_errorInvalid, "another size");
    other = grid;
    other.cellSize = 1.0f;
    CHECK(mnavGridFlow_Continue(field, &other, 1, &ended) == mnav_errorInvalid &&
              mnavGridFlow_Continue(field, &grid, 0, &ended) == mnav_errorInvalid &&
              mnavGridFlow_Continue(field, nullptr, 1, &ended) == mnav_errorInvalid,
          "another cell size; no cells; no grid");
    other = grid;
    other.areas = nullptr;
    CHECK(mnavGridFlow_Continue(field, &other, 1, &ended) == mnav_errorInvalid, "no areas");
    // Debug output waits for the end too, and draws nothing when nothing
    // was begun.
    mnavDebugVertex vertex;
    uint32_t index;
    mnavDebugBuffer buffer = {{0, 0, 0}, &vertex, 1, 0, nullptr, 0, 0, &index, 1, 0};
    CHECK(mnavGridFlow_Begin(field, &grid, nullptr, goals, 3, &filter) == mnav_success &&
              mnavGridFlow_Debug(field, 0.0, &buffer) == mnav_errorState,
          "no debug output while working");
    mnavGridFlow* fresh = Make(10);
    CHECK(mnavGridFlow_Continue(fresh, &grid, 1, &ended) == mnav_errorState, "nothing begun");
    CHECK(mnavGridFlow_Debug(fresh, 0.0, &buffer) == mnav_success && buffer.vertexCount == 0,
          "nothing to draw");
    // A begin refused after a field was built leaves nothing to draw.
    CHECK(mnavGridFlow_Build(field, &grid, goals, 3, &filter) == mnav_success &&
              mnavGridFlow_Build(field, &grid, goals, -1, &filter) == mnav_errorInvalid &&
              mnavGridFlow_Debug(field, 0.0, &buffer) == mnav_success && buffer.vertexCount == 0,
          "nothing to draw after a refused begin");
    mnavDestroyGridFlow(fresh);
    mnavDestroyGridFlow(field);
}

static uint32_t s_seed = 7u;

static uint32_t Next(uint32_t bound)
{
    s_seed = s_seed * 1664525u + 1013904223u;
    return (s_seed >> 8) % bound;
}

// How many cells of a region two fields give different ways.
static int32_t Differ(const mnavGridFlow* a, const mnavGridFlow* b, Window r)
{
    int32_t differ = 0;
    for (int32_t y = r.y; y < r.y + r.height; ++y)
    {
        for (int32_t x = r.x; x < r.x + r.width; ++x)
        {
            mnavCellFlow p = At(a, x, y);
            mnavCellFlow q = At(b, x, y);
            differ += memcmp(&p, &q, sizeof(p)) != 0 ? 1 : 0;
        }
    }
    return differ;
}

// Random goal moves and area changes, each repaired in random budgets,
// against a field built afresh after each.
static void RepairHistory(const mnavCellRange* region, uint32_t seed)
{
    RandomAreas(seed);
    s_seed = seed;
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    Window r = region != nullptr ? (Window){region->x0, region->y0, region->x1 - region->x0 + 1,
                                            region->y1 - region->y0 + 1}
                                 : (Window){0, 0, WIDTH, HEIGHT};
    mnavGridFlow* repaired = Make(WIDTH * HEIGHT);
    mnavGridFlow* built = Make(WIDTH * HEIGHT);
    mnavCell goals[6] = {{3, 3}, {40, 30}, {20, 10}, {0, 0}, {47, 0}, {10, 30}};
    int32_t goalCount = 3;
    bool ended = false;
    CHECK(mnavGridFlow_Begin(repaired, &grid, region, goals, goalCount, &filter) == mnav_success &&
              mnavGridFlow_Continue(repaired, &grid, INT32_MAX, &ended) == mnav_success,
          "built first");
    int32_t wrong = 0;
    for (int32_t round = 0; round < 80; ++round)
    {
        mnavCell changed[12];
        int32_t changedCount = 0;
        uint32_t what = Next(4);
        if (what != 1)
        {
            changedCount = 1 + (int32_t)Next(what == 3 ? 12 : 3);
            for (int32_t i = 0; i < changedCount; ++i)
            {
                // Near the last change half the time, so changes meet.
                changed[i] = i > 0 && Next(2) == 0
                                 ? (mnavCell){(changed[i - 1].x + 1) % WIDTH, changed[i - 1].y}
                                 : (mnavCell){(int32_t)Next(WIDTH), (int32_t)Next(HEIGHT)};
                uint32_t a = Next(10);
                s_areas[changed[i].y * WIDTH + changed[i].x] =
                    a < 3u ? mnav_areaNone : (mnavAreaType)(a < 6u ? 1u : a - 4u);
            }
        }
        if (what != 0)
        {
            goalCount = (int32_t)Next(7);
            for (int32_t i = 0; i < goalCount; ++i)
            {
                goals[i] = (mnavCell){(int32_t)Next(WIDTH), (int32_t)Next(HEIGHT)};
            }
        }
        int32_t budget = 1 + (int32_t)Next(400);
        CHECK(mnavGridFlow_Update(repaired, &grid, goals, goalCount, changed, changedCount) ==
                  mnav_success,
              "repair begun");
        ended = false;
        while (!ended)
        {
            CHECK(mnavGridFlow_Continue(repaired, &grid, budget, &ended) == mnav_success, "step");
        }
        CHECK(mnavGridFlow_Begin(built, &grid, region, goals, goalCount, &filter) == mnav_success &&
                  mnavGridFlow_Continue(built, &grid, INT32_MAX, &ended) == mnav_success,
              "built afresh");
        wrong += Differ(repaired, built, r) != 0 ? 1 : 0;
    }
    printf("repairs over %d by %d: %d of 80 differ\n", r.width, r.height, wrong);
    CHECK(wrong == 0, "every repair gives the field a build gives");
    mnavDestroyGridFlow(repaired);
    mnavDestroyGridFlow(built);
}

static void TestRepairChecks(void)
{
    memset(s_areas, 1, sizeof(s_areas));
    mnavGrid grid = Grid();
    mnavGridFlow* field = Make(WIDTH * HEIGHT);
    const mnavCell goal = {4, 4};
    const mnavCell off = {WIDTH, 0};
    CHECK(mnavGridFlow_Update(field, &grid, &goal, 1, nullptr, 0) == mnav_errorState,
          "nothing begun");
    CHECK(mnavGridFlow_Begin(field, &grid, nullptr, &goal, 1, nullptr) == mnav_success &&
              mnavGridFlow_Update(field, &grid, &goal, 1, nullptr, 0) == mnav_errorState,
          "work not ended");
    CHECK(mnavGridFlow_Continue(field, &grid, INT32_MAX, nullptr) == mnav_success, "ended");
    mnavGrid other = grid;
    other.width = WIDTH - 1;
    CHECK(mnavGridFlow_Update(field, &other, &goal, 1, nullptr, 0) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, &off, 1, nullptr, 0) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, &goal, 1, &off, 1) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, nullptr, 1, nullptr, 0) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, &goal, 1, nullptr, 1) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, &goal, -1, nullptr, 0) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, &grid, &goal, 1, &goal, -1) == mnav_errorInvalid &&
              mnavGridFlow_Update(nullptr, &grid, &goal, 1, nullptr, 0) == mnav_errorInvalid &&
              mnavGridFlow_Update(field, nullptr, &goal, 1, nullptr, 0) == mnav_errorInvalid,
          "the checks");
    mnavCellFlow flow;
    CHECK(mnavGridFlow_GetFlow(field, goal, &flow) == mnav_success,
          "a refused repair keeps the field");
    // Nothing changed: the same field, ended at once.
    bool ended = false;
    CHECK(mnavGridFlow_Update(field, &grid, &goal, 1, nullptr, 0) == mnav_success &&
              mnavGridFlow_GetFlow(field, goal, &flow) == mnav_errorState &&
              mnavGridFlow_Continue(field, &grid, 1, &ended) == mnav_success && ended &&
              At(field, 10, 4).next.x == 9,
          "a repair of nothing");
    mnavDestroyGridFlow(field);
}

// A repair cut short by a begin leaves no marks behind: a later repair of
// the same cell raises it again and gives the field a build gives.
static void TestRepairAfterABegin(void)
{
    memset(s_areas, 1, sizeof(s_areas));
    mnavGrid grid = Grid();
    mnavQueryFilter filter = Costs();
    mnavGridFlow* repaired = Make(WIDTH * HEIGHT);
    mnavGridFlow* built = Make(WIDTH * HEIGHT);
    const mnavCell goal = {4, 4};
    // A cell on the way from the east to the goal.
    const mnavCell changed = {10, 4};
    CHECK(mnavGridFlow_Build(repaired, &grid, &goal, 1, &filter) == mnav_success, "built");
    s_areas[changed.y * WIDTH + changed.x] = (mnavAreaType)4;
    CHECK(mnavGridFlow_Update(repaired, &grid, &goal, 1, &changed, 1) == mnav_success,
          "a repair begun");
    CHECK(mnavGridFlow_Build(repaired, &grid, &goal, 1, &filter) == mnav_success,
          "the repair cut short by a build");
    s_areas[changed.y * WIDTH + changed.x] = mnav_areaNone;
    bool ended = false;
    CHECK(mnavGridFlow_Update(repaired, &grid, &goal, 1, &changed, 1) == mnav_success &&
              mnavGridFlow_Continue(repaired, &grid, INT32_MAX, &ended) == mnav_success && ended,
          "the cell repaired again");
    CHECK(mnavGridFlow_Build(built, &grid, &goal, 1, &filter) == mnav_success, "built afresh");
    CHECK(Differ(repaired, built, (Window){0, 0, WIDTH, HEIGHT}) == 0,
          "the repair gives the field a build gives");
    mnavDestroyGridFlow(repaired);
    mnavDestroyGridFlow(built);
}

static void TestRepairs(void)
{
    RepairHistory(nullptr, 3u);
    RepairHistory(nullptr, 99u);
    const mnavCellRange r = {5, 4, 37, 28};
    RepairHistory(&r, 12u);
}

int main(void)
{
    TestAgainstGridPaths();
    TestSeveralGoals();
    TestCornersAndBlocks();
    TestPinned();
    TestChecks();
    TestRegion();
    TestSteps();
    TestRepairs();
    TestRepairAfterABegin();
    TestRepairChecks();
    return s_failures == 0 ? 0 : 1;
}
