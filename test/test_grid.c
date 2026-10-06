// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Grid paths (mnav-0005): jump point search and A*, their limits and ends.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the random grids' paths, the same on every platform.
#define GRIDS_HASH 0xdc50aa439481c4dcull

enum
{
    SIDE = 40
};

static mnavAreaType s_areas[SIDE * SIDE];

static mnavQuery* MakeQuery(int32_t nodes, float pathLength)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    def.limits.pathLength = pathLength;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static mnavGrid Grid(int32_t width, int32_t height)
{
    return (mnavGrid){s_areas, width, height, 0.5f};
}

// A filter under which area 1 costs 1 as before but area 2, which no
// grid here holds, costs 2: the search is A*.
static mnavQueryFilter Weighted(void)
{
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[2] = 2.0f;
    return filter;
}

static void TestOpenGround(void)
{
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    mnavGrid grid = Grid(10, 10);
    mnavGridPath path;
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){9, 9}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.cellCount == 2 && path.cells[1].x == 9 &&
              fabs(path.cost - 9.0 * 1.4142135623730951 * 0.5) < 1e-12,
          "one diagonal");
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){9, 4}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.cellCount == 3 &&
              fabs(path.length - (4.0 * 1.4142135623730951 + 5.0) * 0.5) < 1e-12,
          "a diagonal, then straight");
    mnavQueryFilter weighted = Weighted();
    // A* may take another path of the same length, turning more often.
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){9, 4}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound &&
              fabs(path.length - (4.0 * 1.4142135623730951 + 5.0) * 0.5) < 1e-12,
          "A* finds the same length");
    // One cost for every area, though not 1: still jump point search, at
    // that cost.
    mnavQueryFilter doubled = mnavDefaultQueryFilter();
    for (int32_t a = 0; a < MNAV_AREA_TYPES; ++a)
    {
        doubled.costs[a] = 2.0f;
    }
    CHECK(mnavFindGridPath(query, &grid, &doubled, (mnavCell){0, 0}, (mnavCell){9, 9}, &path) ==
                  mnav_success &&
              path.cellCount == 2 && fabs(path.cost - 9.0 * 1.4142135623730951) < 1e-12,
          "twice the cost");
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){3, 3}, (mnavCell){3, 3}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.cellCount == 1 && path.cost == 0.0,
          "already there");
    mnavDestroyQuery(query);
}

static void TestCornersAndBlocks(void)
{
    // A 2 by 2 grid open on one diagonal: no corner is cut.
    memset(s_areas, mnav_areaNone, sizeof(s_areas));
    s_areas[0] = 1;
    s_areas[3] = 1;
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavGrid grid = Grid(2, 2);
    mnavGridPath path;
    mnavQueryFilter weighted = Weighted();
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){1, 1}, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone && path.cellCount == 1,
          "jump point search cuts no corner");
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){1, 1}, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone && path.cellCount == 1,
          "nor does A*");
    // A filter that wants mnav_areaNone still finds those cells blocked.
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas |= 1u;
    CHECK(mnavFindGridPath(query, &grid, &none, (mnavCell){0, 0}, (mnavCell){1, 0}, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone,
          "mnav_areaNone always blocks");
    // A blocked start: the start alone.
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){1, 0}, (mnavCell){1, 1}, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone && path.cellCount == 1 && path.cells[0].x == 1,
          "a blocked start");
    mnavDestroyQuery(query);
}

static void TestCostsSteerA(void)
{
    // A 9 by 9 grid with a band of area 3 across the middle row but for
    // its last cell: dear enough, the path goes round by the gap.
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    for (int32_t x = 0; x < 8; ++x)
    {
        s_areas[4 * 9 + x] = 3;
    }
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    mnavGrid grid = Grid(9, 9);
    mnavGridPath path;
    mnavQueryFilter dear = mnavDefaultQueryFilter();
    dear.costs[3] = 50.0f;
    CHECK(mnavFindGridPath(query, &grid, &dear, (mnavCell){0, 0}, (mnavCell){0, 8}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.cost < 50.0 && path.length > 8.0,
          "round by the gap");
    dear.costs[3] = 1.5f;
    CHECK(mnavFindGridPath(query, &grid, &dear, (mnavCell){0, 0}, (mnavCell){0, 8}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.cellCount == 2 &&
              fabs(path.cost - (3.0 + 2.0 * 1.25 + 3.0) * 0.5) < 1e-12,
          "straight across a cheap band");
    // A path costs the same both ways, starting or ending on the band.
    mnavGridPath back;
    CHECK(mnavFindGridPath(query, &grid, &dear, (mnavCell){0, 4}, (mnavCell){0, 8}, &path) ==
                  mnav_success &&
              mnavFindGridPath(query, &grid, &dear, (mnavCell){0, 8}, (mnavCell){0, 4}, &back) ==
                  mnav_success &&
              path.cost == back.cost && fabs(path.cost - (1.25 + 3.0) * 0.5) < 1e-12,
          "the same cost both ways");
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~((uint64_t)1 << 3);
    CHECK(mnavFindGridPath(query, &grid, &without, (mnavCell){0, 0}, (mnavCell){0, 8}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.length > 8.0,
          "a left-out area blocks");
    mnavDestroyQuery(query);
}

static void TestLimitsAndEnds(void)
{
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    // A wall down column 20 with no gap.
    for (int32_t y = 0; y < SIDE; ++y)
    {
        s_areas[y * SIDE + 20] = mnav_areaNone;
    }
    mnavGrid grid = Grid(SIDE, SIDE);
    mnavGridPath path;
    mnavQueryFilter weighted = Weighted();
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){39, 0}, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone && path.cells[path.cellCount - 1].x == 19,
          "walled off: as near as it gets");
    mnavDestroyQuery(query);
    query = MakeQuery(8, 1000.0f);
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){39, 0}, &path) ==
                  mnav_success &&
              path.end == mnav_pathOutOfNodes,
          "out of nodes");
    mnavDestroyQuery(query);
    query = MakeQuery(4096, 5.0f);
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){19, 39}, &path) ==
                  mnav_success &&
              path.end == mnav_pathTooLong && path.cellCount == 1,
          "too long: no first step leads within the length");
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){19, 39}, &path) ==
                  mnav_success &&
              path.end == mnav_pathTooLong && path.cellCount == 1,
          "nor under A*");
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){40, 0}, &path) ==
                  mnav_errorInvalid &&
              mnavFindGridPath(query, &grid, nullptr, (mnavCell){-1, 0}, (mnavCell){1, 0}, &path) ==
                  mnav_errorInvalid &&
              mnavFindGridPath(nullptr, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){1, 0},
                               &path) == mnav_errorInvalid &&
              mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){1, 0},
                               nullptr) == mnav_errorInvalid,
          "cells outside, NULL arguments");
    mnavGrid bad = grid;
    bad.cellSize = 0.0f;
    CHECK(mnavFindGridPath(query, &bad, nullptr, (mnavCell){0, 0}, (mnavCell){1, 0}, &path) ==
              mnav_errorInvalid,
          "no cell size");
    bad = grid;
    bad.width = MNAV_MAX_GRID_SIDE + 1;
    CHECK(mnavFindGridPath(query, &bad, nullptr, (mnavCell){0, 0}, (mnavCell){1, 0}, &path) ==
              mnav_errorInvalid,
          "too wide");
    bad = grid;
    bad.areas = nullptr;
    CHECK(mnavFindGridPath(query, &bad, nullptr, (mnavCell){0, 0}, (mnavCell){1, 0}, &path) ==
              mnav_errorInvalid,
          "no areas");
    mnavQueryFilter broken = mnavDefaultQueryFilter();
    broken.cookie = 0;
    CHECK(mnavFindGridPath(query, &grid, &broken, (mnavCell){0, 0}, (mnavCell){1, 0}, &path) ==
              mnav_errorInvalid,
          "a filter not from the default");
    mnavDestroyQuery(query);
}

static void TestPathAsLongAsTheLimit(void)
{
    // Four cells of 0.5 m along a row: 2 m, exactly the length allowed.
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    mnavGrid grid = Grid(SIDE, SIDE);
    mnavGridPath path;
    mnavQueryFilter weighted = Weighted();
    mnavQuery* query = MakeQuery(4096, 2.0f);
    CHECK(mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){4, 0}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.length == 2.0,
          "a path as long as the limit is found");
    CHECK(mnavFindGridPath(query, &grid, &weighted, (mnavCell){0, 0}, (mnavCell){4, 0}, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.length == 2.0,
          "and under A*");
    mnavDestroyQuery(query);
}

static void TestJumpsMatchAStar(void)
{
    // On random grids, a fifth to a third blocked, jump point search and A*
    // agree on whether the end is reached and on the cost; every path is
    // pinned.
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    mnavGrid grid = Grid(SIDE, SIDE);
    mnavQueryFilter weighted = Weighted();
    uint32_t state = 99;
    uint64_t hash = MNAV_HASH_INIT;
    int32_t found = 0;
    int32_t agree = 0;
    for (int32_t trial = 0; trial < 300; ++trial)
    {
        uint32_t blocked = 13 + (uint32_t)trial % 9;
        for (int32_t i = 0; i < SIDE * SIDE; ++i)
        {
            state = state * 1664525u + 1013904223u;
            s_areas[i] = (state >> 8) % 64u < blocked ? mnav_areaNone : mnav_areaWalkable;
        }
        state = state * 1664525u + 1013904223u;
        mnavCell a = {(int32_t)(state >> 8) % SIDE, (int32_t)(state >> 16) % SIDE};
        state = state * 1664525u + 1013904223u;
        mnavCell b = {(int32_t)(state >> 8) % SIDE, (int32_t)(state >> 16) % SIDE};
        s_areas[a.y * SIDE + a.x] = mnav_areaWalkable;
        s_areas[b.y * SIDE + b.x] = mnav_areaWalkable;
        mnavGridPath jumps;
        mnavGridPath plain;
        CHECK(mnavFindGridPath(query, &grid, nullptr, a, b, &jumps) == mnav_success, "jumps");
        hash = mnavHash64(hash, &jumps.end, (int32_t)sizeof(jumps.end));
        hash = mnavHash64(hash, &jumps.cost, (int32_t)sizeof(jumps.cost));
        hash = mnavHash64(hash, jumps.cells, (int32_t)((size_t)jumps.cellCount * sizeof(mnavCell)));
        double cost = jumps.cost;
        mnavPathEnd end = jumps.end;
        CHECK(mnavFindGridPath(query, &grid, &weighted, a, b, &plain) == mnav_success, "A*");
        hash = mnavHash64(hash, plain.cells, (int32_t)((size_t)plain.cellCount * sizeof(mnavCell)));
        found += end == mnav_pathFound ? 1 : 0;
        agree += end == plain.end &&
                         (end != mnav_pathFound || fabs(cost - plain.cost) <= 1e-9 * plain.cost)
                     ? 1
                     : 0;
    }
    printf("GRIDS_HASH=%016llx found=%d agree=%d of 300\n", (unsigned long long)hash, found, agree);
    CHECK(agree == 300 && found > 150, "the same ends and costs");
    CHECK(hash == GRIDS_HASH, "the pinned hash");
    mnavDestroyQuery(query);
}

static void TestEndsASlicedSearch(void)
{
    // A grid search in a context ends the navmesh search sliced in it.
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_tiles[0], s_sizes[0]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a navmesh");
    mnavNearest a;
    mnavNearest b;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){2.0, 0.0, 2.0}, (mnavVec3){1.0f, 2.0f, 1.0f},
                          &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){28.0, 0.0, 28.0},
                              (mnavVec3){1.0f, 2.0f, 1.0f}, &b) == mnav_success,
          "ends");
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    mnavGrid grid = Grid(4, 4);
    mnavGridPath path;
    bool ended = false;
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
                  mnav_success &&
              mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){3, 3}, &path) ==
                  mnav_success &&
              mnavContinuePath(query, navmesh, 16, &ended) == mnav_errorInvalid,
          "the sliced search ended");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestOpenGround();
    TestCornersAndBlocks();
    TestCostsSteerA();
    TestLimitsAndEnds();
    TestPathAsLongAsTheLimit();
    TestJumpsMatchAStar();
    TestEndsASlicedSearch();
    return s_failures == 0 ? 0 : 1;
}
