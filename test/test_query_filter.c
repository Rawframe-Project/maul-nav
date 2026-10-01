// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Query filters (N28): area costs and inclusion in the nearest point, the
// path search and the raycast. Hand cells are 0.25 m.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

static uint8_t s_bytes[8192];

// A bridge of area 2 from (10, 5) to (15, 10) between two squares, and a
// way round it above, three squares of area 1, all clear of the tile's
// sides.
static const HandSquare s_bridge[6] = {{20, 20, 40, 40, {0}, 0}, {40, 20, 60, 40, {0}, 2},
                                       {60, 20, 80, 40, {0}, 0}, {20, 40, 40, 60, {0}, 0},
                                       {40, 40, 60, 60, {0}, 0}, {60, 40, 80, 60, {0}, 0}};

static mnavNavmesh* Make(const HandSquare* squares, int32_t count)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    size_t size = HandTileBytes(s_bytes, 0, squares, count);
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, filter, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 1.0f, 0.1f},
                          &n) == mnav_success,
          "nearest");
    return n;
}

static mnavPath Walk(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter)
{
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavNearest b = On(navmesh, nullptr, 17.5, 7.5);
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, filter, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
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
    mnavNavmesh* navmesh = Make(s_bridge, 6);
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
    mnavNavmesh* navmesh = Make(s_bridge, 6);
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
    mnavNavmesh* navmesh = Make(s_bridge, 6);
    mnavQuery* query = MakeQuery();
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    // The nearest point skips the bridge: from its middle, the square above.
    mnavNearest n = On(navmesh, &filter, 12.5, 7.5);
    CHECK(n.polygon.slot == 0, "nothing usable within 0.1 m");
    CHECK(mnavFindNearest(navmesh, &filter, (mnavPos3){12.5, 0.0, 9.0},
                          (mnavVec3){2.0f, 1.0f, 2.0f}, &n) == mnav_success &&
              n.polygon.polygon == 4 && n.point.z == 10.0,
          "the square above the bridge");
    // A ray along the row meets the bridge as a wall.
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavRay ray = {0};
    CHECK(mnavRaycast(query, navmesh, &filter, a.polygon, a.point, (mnavPos3){17.5, 0.0, 7.5},
                      &ray) == mnav_success &&
              ray.end == mnav_rayWall && ray.t == 0.25 && ray.normalX == -1.0 &&
              ray.polygonCount == 1,
          "the bridge's edge a wall");
    CHECK(mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, (mnavPos3){17.5, 0.0, 7.5},
                      &ray) == mnav_success &&
              ray.end == mnav_rayReached,
          "with every area, across");
    // Starting on the bridge, the search may leave it.
    mnavNearest on = On(navmesh, nullptr, 12.5, 7.5);
    mnavNearest b = On(navmesh, nullptr, 17.5, 7.5);
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, &filter, on.polygon, on.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.polygonCount == 2,
          "off the bridge");
    // The end on the bridge cannot be reached.
    CHECK(mnavFindPath(query, navmesh, &filter, b.polygon, b.point, on.polygon, on.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathNone,
          "onto it, no path");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestFiltersAreChecked(void)
{
    mnavNavmesh* navmesh = Make(s_bridge, 6);
    mnavQuery* query = MakeQuery();
    mnavNearest a = On(navmesh, nullptr, 7.5, 7.5);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    mnavNearest n;
    mnavPath path;
    mnavRay ray;
    filter.costs[5] = 0.0f;
    CHECK(mnavFindNearest(navmesh, &filter, a.point, (mnavVec3){1, 1, 1}, &n) == mnav_errorRange &&
              mnavFindPath(query, navmesh, &filter, a.polygon, a.point, a.polygon, a.point,
                           &path) == mnav_errorRange &&
              mnavRaycast(query, navmesh, &filter, a.polygon, a.point, a.point, &ray) ==
                  mnav_errorRange,
          "a cost of 0");
    filter.costs[5] = NAN;
    CHECK(mnavFindNearest(navmesh, &filter, a.point, (mnavVec3){1, 1, 1}, &n) == mnav_errorRange,
          "a NaN cost");
    filter.costs[5] = MNAV_MAX_AREA_COST * 2.0f;
    CHECK(mnavFindNearest(navmesh, &filter, a.point, (mnavVec3){1, 1, 1}, &n) == mnav_errorRange,
          "too dear");
    filter.costs[5] = MNAV_MIN_AREA_COST;
    filter.costs[6] = MNAV_MAX_AREA_COST;
    CHECK(mnavFindNearest(navmesh, &filter, a.point, (mnavVec3){1, 1, 1}, &n) == mnav_success,
          "the ends of the range");
    filter = (mnavQueryFilter){0};
    CHECK(
        mnavFindNearest(navmesh, &filter, a.point, (mnavVec3){1, 1, 1}, &n) == mnav_errorInvalid &&
            mnavFindPath(query, navmesh, &filter, a.polygon, a.point, a.polygon, a.point, &path) ==
                mnav_errorInvalid &&
            mnavRaycast(query, navmesh, &filter, a.polygon, a.point, a.point, &ray) ==
                mnav_errorInvalid,
        "not from the default");
    filter = mnavDefaultQueryFilter();
    CHECK(filter.areas == ~(uint64_t)1 && filter.costs[0] == 1.0f && filter.costs[63] == 1.0f,
          "the default");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestCostsChooseTheWay();
    TestCheapAreasKeepTheSearchExact();
    TestExcludedAreasElsewhere();
    TestFiltersAreChecked();
    return s_failures == 0 ? 0 : 1;
}
