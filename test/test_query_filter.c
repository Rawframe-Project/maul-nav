// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Query filters: area costs and inclusion in the nearest point, the
// path search and the raycast, and a bad filter refused by every call
// that takes one. Hand cells are 0.25 m.

#include "defs_for.h"
#include "hand_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

// s_handBridge: a bridge of area 2 from (10, 5) to (15, 10) between two
// squares, and a way round it above, three squares of area 1, all clear of
// the tile's sides.
static mnavNavmesh* MakeBridge(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handBridge, sizeof(s_handBridge)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "loaded");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, double x, double z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 1.0f, 0.1f},
                                  filter, &n) == mnav_success,
          "nearest");
    return n;
}

static mnavPath Walk(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter)
{
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavNearest b = On(navmesh, nullptr, 17.5, 7.5);
    mnavPath path = {0};
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, filter,
                             &path) == mnav_success,
          "searched");
    return path;
}

static mnavQuery* MakeQuery(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static void TestCostsChooseTheWay(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQuery* query = MakeQuery();
    mnavPath path = Walk(query, navmesh, nullptr);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 3 && path.cost == 10.0 &&
              path.length == 10.0 && path.pointCount == 2,
          "over the bridge, 10 m");
    // The bridge at 2 per meter: 2.5 + 2 * 5 + 2.5 = 15, still cheaper than
    // the 20 m round it.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[2] = 2.0f;
    path = Walk(query, navmesh, &filter);
    CHECK(path.polygonCount == 3 && path.cost == 15.0 && path.length == 10.0, "dearer, still");
    // At 4 per meter the bridge costs 25: round it, through five squares,
    // 5 + 5 + 2 * sqrt(12.5) m through the edges' midpoints.
    filter.costs[2] = 4.0f;
    path = Walk(query, navmesh, &filter);
    double round = 10.0 + 2.0 * sqrt(12.5);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 5 && path.cost == path.length &&
              fabs(path.length - round) < 1.0e-12 && path.pointCount == 4,
          "round it");
    // Excluded, the same way round; the end square alone, nothing.
    filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    path = Walk(query, navmesh, &filter);
    CHECK(path.polygonCount == 5 && fabs(path.length - round) < 1.0e-12, "the bridge left out");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestCheapAreasKeepTheSearchExact(void)
{
    // Every area at 0.25 per meter: an unscaled heuristic would overstate
    // the cost left; scaled, the search still finds the bridge.
    mnavNavmesh* navmesh = MakeBridge();
    mnavQuery* query = MakeQuery();
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    for (int32_t a = 0; a < MNAV_AREA_TYPES; ++a)
    {
        filter.costs[a] = 0.25f;
    }
    filter.costs[2] = 0.5f;
    mnavPath path = Walk(query, navmesh, &filter);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 3 && path.cost == 3.75 &&
              path.length == 10.0,
          "the cheapest way, 0.625 + 2.5 + 0.625");
    // The bridge at 0.62: 1.25 + 3.1 = 4.35 against 0.25 * 17.07 = 4.27
    // round it. A heuristic scaled by 1 rather than 0.25 would overstate
    // what is left four times over and take the direct way.
    filter.costs[2] = 0.62f;
    path = Walk(query, navmesh, &filter);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 5, "round it, cheaper by a hair");
    // The length limit counts meters, not cost: 12 m allows the bridge
    // even at a cost of 1,000 per meter.
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.pathLength = 12.0f;
    mnavQuery* tight = nullptr;
    CHECK(mnavCreateQuery(&def, &tight) == mnav_success, "tight query");
    filter = mnavDefaultQueryFilter();
    filter.costs[2] = 1000.0f;
    path = Walk(tight, navmesh, &filter);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 3 && path.length == 10.0,
          "within 12 m");
    mnavDestroyQuery(tight);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestExcludedAreasElsewhere(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQuery* query = MakeQuery();
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    // The nearest point skips the bridge: from its middle, the square above.
    mnavNearest n = On(navmesh, &filter, 12.5, 7.5);
    CHECK(n.polygon.slot == 0, "nothing usable within 0.1 m");
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){12.5, 0.0, 9.0}, (mnavVec3){2.0f, 1.0f, 2.0f},
                                  &filter, &n) == mnav_success &&
              n.polygon.polygon == 4 && n.point.z == 10.0,
          "the square above the bridge");
    // A ray along the row meets the bridge as a wall.
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavRay ray = {0};
    CHECK(mnavQuery_Raycast(query, navmesh, a.polygon, a.point, (mnavPos3){17.5, 0.0, 7.5}, &filter,
                            &ray) == mnav_success &&
              ray.end == mnav_rayWall && ray.t == 0.25 && ray.normal.x == -1.0f &&
              ray.polygonCount == 1,
          "the bridge's edge a wall");
    CHECK(mnavQuery_Raycast(query, navmesh, a.polygon, a.point, (mnavPos3){17.5, 0.0, 7.5}, nullptr,
                            &ray) == mnav_success &&
              ray.end == mnav_rayReached,
          "with every area, across");
    // Starting on the bridge, the search may leave it.
    mnavNearest on = On(navmesh, nullptr, 12.5, 7.5);
    mnavNearest b = On(navmesh, nullptr, 17.5, 7.5);
    mnavPath path = {0};
    CHECK(mnavQuery_FindPath(query, navmesh, on.polygon, on.point, b.polygon, b.point, &filter,
                             &path) == mnav_success &&
              path.end == mnav_pathFound && path.polygonCount == 2,
          "off the bridge");
    // The end on the bridge cannot be reached.
    CHECK(mnavQuery_FindPath(query, navmesh, b.polygon, b.point, on.polygon, on.point, &filter,
                             &path) == mnav_success &&
              path.end == mnav_pathNone,
          "onto it, no path");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestFiltersAreChecked(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQuery* query = MakeQuery();
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    mnavNearest n;
    mnavPath path;
    mnavRay ray;
    filter.costs[5] = 0.0f;
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
                  mnav_errorLimit &&
              mnavQuery_FindPath(query, navmesh, a.polygon, a.point, a.polygon, a.point, &filter,
                                 &path) == mnav_errorLimit &&
              mnavQuery_Raycast(query, navmesh, a.polygon, a.point, a.point, &filter, &ray) ==
                  mnav_errorLimit,
          "a cost of 0");
    filter.costs[5] = NAN;
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
              mnav_errorInvalid,
          "a NaN cost");
    filter.costs[5] = MNAV_MAX_AREA_COST * 2.0f;
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
              mnav_errorLimit,
          "too dear");
    filter.costs[5] = INFINITY;
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
              mnav_errorLimit,
          "an infinite cost, past the bound");
    filter.costs[5] = MNAV_MIN_AREA_COST;
    filter.costs[6] = MNAV_MAX_AREA_COST;
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
              mnav_success,
          "the ends of the range");
    filter = (mnavQueryFilter){0};
    CHECK(mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1, 1, 1}, &filter, &n) ==
                  mnav_errorInvalid &&
              mnavQuery_FindPath(query, navmesh, a.polygon, a.point, a.polygon, a.point, &filter,
                                 &path) == mnav_errorInvalid &&
              mnavQuery_Raycast(query, navmesh, a.polygon, a.point, a.point, &filter, &ray) ==
                  mnav_errorInvalid,
          "not from the default");
    filter = mnavDefaultQueryFilter();
    CHECK(filter.areas == ~(uint64_t)1 && filter.costs[0] == 1.0f && filter.costs[63] == 1.0f,
          "the default");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Calls every navmesh call that takes a filter with this one, from a to
// b, and counts those that do not refuse it with the status wanted. The
// corridor calls refuse before they change the corridor.
static int32_t Refusing(mnavQuery* query, const mnavNavmesh* navmesh, mnavCorridor* corridor,
                        mnavNearest a, mnavNearest b, const mnavQueryFilter* f, mnavResult wanted)
{
    const mnavVec3 box = {1.0f, 1.0f, 1.0f};
    mnavPolygonId ids[8];
    mnavFound found;
    mnavNearest n;
    mnavPath path;
    mnavMove move;
    mnavWall walls[4];
    mnavWallsFound wallsFound;
    mnavNearestWall wall;
    mnavRandomPoint point;
    mnavPathEnd end;
    mnavRay ray;
    int32_t valid = 0;
    bool shortened = false;
    mnavLinkGenDef gen = mnavDefaultLinkGenDef();
    mnavLinkDef links[4];
    int32_t count = 0;
    const mnavResult got[18] = {
        mnavNavmesh_FindPolygons(navmesh, a.point, box, f, ids, 8, &found),
        mnavNavmesh_FindNearest(navmesh, a.point, box, f, &n),
        mnavNavmesh_FindRandomPoint(navmesh, 7, f, &point),
        mnavNavmesh_CheckCorridor(navmesh, corridor, f, &valid),
        mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, f, &path),
        mnavQuery_FindShortestPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, f,
                                   &path),
        mnavQuery_BeginPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, f),
        mnavQuery_CheckReachable(query, navmesh, a.polygon, a.point, b.polygon, b.point, f, &end),
        mnavQuery_Raycast(query, navmesh, a.polygon, a.point, b.point, f, &ray),
        mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, a.point, b.point, f, &move),
        mnavQuery_MoveCorridor(query, corridor, navmesh, b.point, f, &move),
        mnavQuery_MoveCorridorTarget(query, corridor, navmesh, a.point, f, &move),
        mnavQuery_ShortcutCorridor(query, corridor, navmesh, b.point, f, &shortened),
        mnavQuery_ReplanCorridor(query, corridor, navmesh, box, f, &path),
        mnavQuery_FindWallDistance(query, navmesh, a.polygon, a.point, 5.0, f, &wall),
        mnavQuery_FindWalls(query, navmesh, a.polygon, a.point, 5.0, f, walls, 4, &wallsFound),
        mnavQuery_FindRandomPointAround(query, navmesh, a.polygon, a.point, 5.0, 7, f, &point),
        mnavQuery_GenerateLinks(query, navmesh, &gen, nullptr, f, links, 4, &count),
    };
    int32_t wrong = 0;
    for (int32_t i = 0; i < 18; ++i)
    {
        if (got[i] != wanted)
        {
            printf("call %d: %d, not %d\n", i, got[i], wanted);
            wrong += 1;
        }
    }
    return wrong;
}

static void TestEveryCallChecksItsFilter(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQuery* query = MakeQuery();
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavNearest b = On(navmesh, nullptr, 17.5, 7.5);
    mnavPath path = Walk(query, navmesh, nullptr);
    static mnavPolygonId buffer[64];
    mnavCorridor corridor;
    CHECK(mnavResetCorridor(&corridor, buffer, 64, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "a corridor");
    const mnavCorridor before = corridor;
    // Area 5 is on no polygon of the bridge: its cost is checked all the
    // same.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[5] = 0.0f;
    CHECK(Refusing(query, navmesh, &corridor, a, b, &filter, mnav_errorLimit) == 0,
          "a cost of 0: past the bound, in every call");
    filter.costs[5] = (float)INFINITY;
    CHECK(Refusing(query, navmesh, &corridor, a, b, &filter, mnav_errorLimit) == 0,
          "an infinite cost: past the bound, in every call");
    filter.costs[5] = (float)NAN;
    CHECK(Refusing(query, navmesh, &corridor, a, b, &filter, mnav_errorInvalid) == 0,
          "a NaN cost: invalid, in every call");
    filter = (mnavQueryFilter){0};
    CHECK(Refusing(query, navmesh, &corridor, a, b, &filter, mnav_errorInvalid) == 0,
          "not from the default: invalid, in every call");
    CHECK(corridor.polygonCount == before.polygonCount &&
              corridor.position.x == before.position.x && corridor.target.x == before.target.x,
          "the corridor kept");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestCostsChooseTheWay();
    TestCheapAreasKeepTheSearchExact();
    TestExcludedAreasElsewhere();
    TestFiltersAreChecked();
    TestEveryCallChecksItsFilter();
    return s_failures == 0 ? 0 : 1;
}
