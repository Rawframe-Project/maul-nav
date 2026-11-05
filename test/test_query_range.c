// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The query range: every navmesh query that takes a point, an end, a
// center or a radius answers one at MNAV_MAX_QUERY_DISTANCE from the
// navmesh's origin and refuses one just past it with mnav_errorLimit; at
// the bound, the promises that past it used to break hold. Hand cells are
// 0.25 m; the origin is (0, 0, 0).

#include "defs_for.h"
#include "hand_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The calls below: one per point, end, center or radius they take.
#define RANGE_CALLS 37

// The bound, and the next double past it.
static const double s_at = MNAV_MAX_QUERY_DISTANCE;

static double Past(void)
{
    return nextafter(MNAV_MAX_QUERY_DISTANCE, (double)INFINITY);
}

// s_handBridge: a bridge of area 2 from (10, 5) to (15, 10) between two
// squares, and a way round it above.
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

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 1.0f, 0.1f},
                                  nullptr, &n) == mnav_success &&
              n.over,
          "on a polygon");
    return n;
}

// What every call needs: a context, the navmesh, a corridor from a to b
// in a buffer of its own, a hierarchy and a navmesh flow field.
typedef struct Callees
{
    mnavQuery* query;
    const mnavNavmesh* navmesh;
    mnavCorridor corridor;
    mnavNearest a;
    mnavNearest b;
    mnavHierarchy* hierarchy;
    mnavNavFlow* navFlow;
} Callees;

// A copy of the corridor, its buffer copied too, with its position or its
// target moved to p.
static mnavCorridor Moved(const mnavCorridor* corridor, mnavPolygonId* buffer, bool target,
                          mnavPos3 p)
{
    mnavCorridor c = *corridor;
    memcpy(buffer, corridor->polygons, (size_t)corridor->polygonCount * sizeof(mnavPolygonId));
    c.polygons = buffer;
    if (target)
    {
        c.target = p;
    }
    else
    {
        c.position = p;
    }
    return c;
}

// The calls that take r rather than p.
static bool TakesRadius(int32_t call)
{
    return call == 32 || call == 34;
}

// Makes every call once with p in each point it takes, and r as each
// radius, the other arguments good; counts the calls not refused with
// mnav_errorLimit where the point or radius they take is within the
// bound, and not refused where it is past.
static int32_t Calls(const Callees* c, mnavPos3 p, double r, bool pointPast, bool radiusPast)
{
    const mnavVec3 box = {1.0f, 1.0f, 1.0f};
    mnavQuery* query = c->query;
    const mnavNavmesh* navmesh = c->navmesh;
    const mnavNearest a = c->a;
    const mnavNearest b = c->b;
    static mnavPolygonId buffers[12][64];
    mnavCorridor moved[12];
    for (int32_t k = 0; k < 12; ++k)
    {
        moved[k] = Moved(&c->corridor, buffers[k], k % 2 == 1, p);
    }
    static mnavPolygonId own[3][64];
    mnavCorridor kept[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        kept[k] = Moved(&c->corridor, own[k], false, c->corridor.position);
    }
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
    mnavCorners corners;
    mnavAreaRun runs[8];
    int32_t count = 0;
    int32_t valid = 0;
    bool shortened = false;
    double height = 0.0;
    const mnavNavFlowGoal goal = {b.polygon, p};
    const mnavTileRange all = {0, 0, 0, 0};
    const mnavResult got[RANGE_CALLS] = {
        mnavNavmesh_FindPolygons(navmesh, p, box, nullptr, ids, 8, &found),
        mnavNavmesh_FindNearest(navmesh, p, box, nullptr, &n),
        mnavQuery_FindPath(query, navmesh, a.polygon, p, b.polygon, b.point, nullptr, &path),
        mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, p, nullptr, &path),
        mnavQuery_FindShortestPath(query, navmesh, a.polygon, p, b.polygon, b.point, nullptr,
                                   &path),
        mnavQuery_FindShortestPath(query, navmesh, a.polygon, a.point, b.polygon, p, nullptr,
                                   &path),
        mnavQuery_BeginPath(query, navmesh, a.polygon, p, b.polygon, b.point, nullptr),
        mnavQuery_BeginPath(query, navmesh, a.polygon, a.point, b.polygon, p, nullptr),
        mnavQuery_CheckReachable(query, navmesh, a.polygon, p, b.polygon, b.point, nullptr, &end),
        mnavQuery_CheckReachable(query, navmesh, a.polygon, a.point, b.polygon, p, nullptr, &end),
        mnavQuery_FindHierarchicalPath(query, c->hierarchy, navmesh, a.polygon, p, b.polygon,
                                       b.point, &path),
        mnavQuery_FindHierarchicalPath(query, c->hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                       p, &path),
        mnavQuery_Raycast(query, navmesh, a.polygon, p, b.point, nullptr, &ray),
        mnavQuery_Raycast(query, navmesh, a.polygon, a.point, p, nullptr, &ray),
        mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, p, b.point, nullptr, &move),
        mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, a.point, p, nullptr, &move),
        mnavNavmesh_CheckCorridor(navmesh, &moved[0], nullptr, &valid),
        mnavNavmesh_CheckCorridor(navmesh, &moved[1], nullptr, &valid),
        mnavQuery_FindCorridorCorners(query, &moved[2], navmesh, &corners),
        mnavQuery_FindCorridorCorners(query, &moved[3], navmesh, &corners),
        mnavQuery_FindCorridorAreas(query, &moved[4], navmesh, nullptr, runs, 8, &count),
        mnavQuery_FindCorridorAreas(query, &moved[5], navmesh, nullptr, runs, 8, &count),
        mnavQuery_MoveCorridor(query, &moved[6], navmesh, b.point, nullptr, &move),
        mnavQuery_MoveCorridor(query, &kept[0], navmesh, p, nullptr, &move),
        mnavQuery_MoveCorridorTarget(query, &moved[7], navmesh, a.point, nullptr, &move),
        mnavQuery_MoveCorridorTarget(query, &kept[1], navmesh, p, nullptr, &move),
        mnavQuery_ShortcutCorridor(query, &moved[8], navmesh, b.point, nullptr, &shortened),
        mnavQuery_ShortcutCorridor(query, &kept[2], navmesh, p, nullptr, &shortened),
        mnavQuery_ReplanCorridor(query, &moved[10], navmesh, box, nullptr, &path),
        mnavQuery_ReplanCorridor(query, &moved[11], navmesh, box, nullptr, &path),
        mnavPolygon_GetHeight(navmesh, a.polygon, p.x, p.z, &height),
        mnavQuery_FindWallDistance(query, navmesh, a.polygon, p, 5.0, nullptr, &wall),
        mnavQuery_FindWallDistance(query, navmesh, a.polygon, a.point, r, nullptr, &wall),
        mnavQuery_FindWalls(query, navmesh, a.polygon, p, 5.0, nullptr, walls, 4, &wallsFound),
        mnavQuery_FindRandomPointAround(query, navmesh, a.polygon, a.point, r, 7, nullptr, &point),
        mnavNavFlow_Build(c->navFlow, navmesh, &goal, 1, nullptr),
        mnavNavFlow_Begin(c->navFlow, navmesh, &all, &goal, 1, nullptr),
    };
    int32_t wrong = 0;
    for (int32_t i = 0; i < RANGE_CALLS; ++i)
    {
        bool past = TakesRadius(i) ? radiusPast : pointPast;
        bool right = past ? got[i] == mnav_errorLimit : got[i] != mnav_errorLimit;
        if (!right)
        {
            printf("call %d: %d\n", i, got[i]);
            wrong += 1;
        }
    }
    return wrong;
}

static void TestEveryCallTakesTheBoundAndRefusesPastIt(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.limits.transitions = 1024;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    Callees c = {query,   navmesh, {0}, On(navmesh, 7.5, 7.5), On(navmesh, 17.5, 7.5),
                 nullptr, nullptr};
    static mnavPolygonId buffer[64];
    mnavPath path;
    CHECK(mnavResetCorridor(&c.corridor, buffer, 64, c.a.polygon, c.a.point) == mnav_success &&
              mnavQuery_FindPath(query, navmesh, c.a.polygon, c.a.point, c.b.polygon, c.b.point,
                                 nullptr, &path) == mnav_success &&
              mnavSetCorridor(&c.corridor, &path) == mnav_success,
          "a corridor");
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.limits = (mnavHierarchyLimits){128, 1024, 8192};
    mnavNavFlowDef navFlowDef = mnavDefaultNavFlowDef();
    navFlowDef.limits.polygons = 1024;
    navFlowDef.limits.tiles = 16;
    navFlowDef.limits.links = 16;
    CHECK(mnavCreateHierarchy(&hierarchyDef, &c.hierarchy) == mnav_success &&
              mnavHierarchy_Build(c.hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavCreateNavFlow(&navFlowDef, &c.navFlow) == mnav_success,
          "the hierarchy and the field");
    double y = c.a.point.y;
    CHECK(Calls(&c, (mnavPos3){s_at, y, 7.5}, s_at, false, false) == 0,
          "every call takes a point and a radius at the bound");
    CHECK(Calls(&c, (mnavPos3){-s_at, y, 7.5}, 5.0, false, false) == 0,
          "every call takes a point at the bound the other way");
    CHECK(Calls(&c, (mnavPos3){Past(), y, 7.5}, 5.0, true, false) == 0,
          "every call refuses a point just past the bound in x");
    CHECK(Calls(&c, (mnavPos3){7.5, y, -Past()}, 5.0, true, false) == 0,
          "every call refuses a point just past the bound in z");
    CHECK(Calls(&c, (mnavPos3){7.5, y, 7.5}, Past(), false, true) == 0,
          "a radius just past the bound refused where a radius is taken");
    mnavDestroyNavFlow(c.navFlow);
    mnavDestroyHierarchy(c.hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The bound checks height too, where a query reads it, and not for the
// ends of a move and a raycast, whose height is never read.
static void TestHeightAndRadius(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest a = On(navmesh, 7.5, 7.5);
    mnavNearest b = On(navmesh, 17.5, 7.5);
    mnavPos3 high = {7.5, Past(), 7.5};
    mnavNearest n;
    mnavPath path;
    mnavNearestWall wall;
    mnavRandomPoint point;
    CHECK(mnavNavmesh_FindNearest(navmesh, high, (mnavVec3){1, 1, 1}, nullptr, &n) ==
                  mnav_errorLimit &&
              mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, high, nullptr,
                                 &path) == mnav_errorLimit &&
              mnavQuery_FindWallDistance(query, navmesh, a.polygon, high, 5.0, nullptr, &wall) ==
                  mnav_errorLimit,
          "a height just past the bound refused");
    mnavMove move;
    mnavRay ray;
    CHECK(mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, a.point,
                                     (mnavPos3){b.point.x, Past(), b.point.z}, nullptr,
                                     &move) == mnav_success &&
              mnavQuery_Raycast(query, navmesh, a.polygon, a.point,
                                (mnavPos3){b.point.x, Past(), b.point.z}, nullptr,
                                &ray) == mnav_success,
          "an end's height, never read, is not checked");
    CHECK(mnavQuery_FindWallDistance(query, navmesh, a.polygon, a.point, Past(), nullptr, &wall) ==
                  mnav_errorLimit &&
              mnavQuery_FindRandomPointAround(query, navmesh, a.polygon, a.point, Past(), 7,
                                              nullptr, &point) == mnav_errorLimit,
          "a radius just past the bound refused");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static bool Finite(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

// At the bound, the promises past it used to break hold: a move stays on
// the navmesh, the walls within the radius are found and a random point
// lies on a polygon in its circle.
static void TestPromisesHoldAtTheBound(void)
{
    mnavNavmesh* navmesh = MakeBridge();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest a = On(navmesh, 7.5, 7.5);
    const double far[3][2] = {{s_at, s_at}, {-s_at, s_at}, {s_at, -s_at}};
    for (int32_t k = 0; k < 3; ++k)
    {
        mnavMove move;
        mnavNearest back;
        CHECK(mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, a.point,
                                         (mnavPos3){far[k][0], a.point.y, far[k][1]}, nullptr,
                                         &move) == mnav_success &&
                  move.end != mnav_moveReached && Finite(move.point) &&
                  mnavNavmesh_FindNearest(navmesh, move.point, (mnavVec3){0.01f, 1.0f, 0.01f},
                                          nullptr, &back) == mnav_success &&
                  back.polygon.slot != 0,
              "a move toward a far point stops on the navmesh");
    }
    mnavNearestWall wall;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, a.polygon, a.point, s_at, nullptr, &wall) ==
                  mnav_success &&
              wall.found && isfinite(wall.wall.distance) && wall.wall.distance <= s_at,
          "a wall found within a radius at the bound");
    mnavPos3 center = {s_at * 0.5, a.point.y, s_at * 0.5};
    CHECK(mnavQuery_FindWallDistance(query, navmesh, a.polygon, center, s_at, nullptr, &wall) ==
                  mnav_success &&
              wall.found && isfinite(wall.wall.distance) && wall.wall.distance <= s_at,
          "a wall found from a far center within the radius");
    mnavRandomPoint point;
    mnavNearest back;
    CHECK(mnavQuery_FindRandomPointAround(query, navmesh, a.polygon, a.point, s_at, 7, nullptr,
                                          &point) == mnav_success &&
              point.polygon.slot != 0 && Finite(point.point) &&
              mnavNavmesh_FindNearest(navmesh, point.point, (mnavVec3){0.01f, 1.0f, 0.01f}, nullptr,
                                      &back) == mnav_success &&
              back.polygon.slot != 0,
          "a random point in a circle of a radius at the bound lies on the navmesh");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestEveryCallTakesTheBoundAndRefusesPastIt();
    TestHeightAndRadius();
    TestPromisesHoldAtTheBound();
    return s_failures == 0 ? 0 : 1;
}
