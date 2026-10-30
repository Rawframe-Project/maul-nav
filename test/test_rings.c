// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The searches within a circle (mnavQuery_FindWallDistance and
// mnavQuery_FindRandomPointAround) over many polygons: the test world in tiles
// of 8 m, its boxes standing across tile sides, so that a circle of 25 m
// spans a hundred polygons and the search's open list grows and is
// reordered. Walls are found at the distance the circle's size allows,
// a center on a wall is at distance 0, random points stay within the
// circle and reachable, and a circle of no area gives its center. The
// tiles are baked here, so nothing is pinned by hash; test_spatial pins the
// random points around a center on the stored world.

#include "defs_for.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

enum
{
    SIDE = 10,
    ROOM = 1 << 16
};

static uint8_t s_small[SIDE * SIDE][ROOM];

static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    mnavBaker* baker = nullptr;
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success,
          "baker and navmesh");
    mnavTriangleMesh world = World();
    for (int32_t t = 0; t < SIDE * SIDE; ++t)
    {
        size_t size = 0;
        CHECK(mnavBaker_BakeTile(baker,
                                 &(mnavBakeInput){&world, 1, nullptr, 0, nullptr, 0, nullptr},
                                 t % SIDE - 1, t / SIDE - 1, nullptr) == mnav_success &&
                  mnavBaker_CopyTile(baker, s_small[t], ROOM, &size) == mnav_success &&
                  mnavNavmesh_StageTile(navmesh, s_small[t], size).result == mnav_success,
              "a tile");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavDestroyBaker(baker);
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n = {0};
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){1.0f, 2.0f, 1.0f},
                                  nullptr, &n) == mnav_success &&
              n.polygon.slot != 0,
          "on the floor");
    return n;
}

static void TestWalls(mnavQuery* query, const mnavNavmesh* navmesh)
{
    // From the world's middle, between boxes: the nearest wall lies a few
    // meters off, and a circle a little short of it finds none.
    mnavNearest c = On(navmesh, 27.0, 27.0);
    mnavNearestWall wide;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, 25.0, nullptr, &wide) ==
                  mnav_success &&
              wide.found && !wide.limited && wide.wall.distance > 0.5 && wide.wall.distance < 25.0,
          "a wall within 25 m");
    mnavNearestWall shorter;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, wide.wall.distance - 0.01,
                                     nullptr, &shorter) == mnav_success &&
              !shorter.found,
          "none just short of it");
    mnavNearestWall just;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, wide.wall.distance + 0.01,
                                     nullptr, &just) == mnav_success &&
              just.found && just.wall.distance == wide.wall.distance,
          "the same wall just past it");
    // From the wall's own point: distance 0, the point itself, and the
    // wall's own normal.
    mnavNearest at = On(navmesh, wide.wall.point.x, wide.wall.point.z);
    mnavNearestWall on;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, at.polygon, wide.wall.point, 5.0, nullptr,
                                     &on) == mnav_success &&
              on.found && on.wall.distance == 0.0 && on.wall.point.x == wide.wall.point.x &&
              on.wall.point.z == wide.wall.point.z &&
              fabs(hypot((double)on.wall.normal.x, (double)on.wall.normal.z) - 1.0) < 1e-6,
          "a center on the wall");
}

static void TestPointsAround(mnavQuery* query, mnavQuery* other, const mnavNavmesh* navmesh)
{
    mnavNearest c = On(navmesh, 27.0, 27.0);
    int32_t outside = 0;
    int32_t unreachable = 0;
    double farthest = 0.0;
    for (uint64_t seed = 0; seed < 100; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavQuery_FindRandomPointAround(query, navmesh, c.polygon, c.point, 25.0, seed,
                                              nullptr, &p) == mnav_success &&
                  p.polygon.slot != 0,
              "picked");
        double d = hypot(p.point.x - c.point.x, p.point.z - c.point.z);
        farthest = d > farthest ? d : farthest;
        outside += d > 25.0 ? 1 : 0;
        mnavPathEnd end;
        CHECK(mnavQuery_CheckReachable(other, navmesh, c.polygon, c.point, p.polygon, p.point,
                                       nullptr, &end) == mnav_success,
              "checked");
        unreachable += end == mnav_pathFound ? 0 : 1;
    }
    printf("rings: farthest %.2f m\n", farthest);
    CHECK(outside == 0 && unreachable == 0 && farthest > 15.0,
          "within the circle, reachable, spread over it");
    // A circle of no area gives its center.
    mnavRandomPoint p;
    CHECK(mnavQuery_FindRandomPointAround(query, navmesh, c.polygon, c.point, 0.0, 7u, nullptr,
                                          &p) == mnav_success &&
              p.polygon.slot == c.polygon.slot && p.polygon.polygon == c.polygon.polygon &&
              p.point.x == c.point.x && p.point.z == c.point.z,
          "a circle of no area");
}

// Every refusal of the circle searches and of heights and random points.
static void TestChecks(mnavQuery* query, const mnavNavmesh* navmesh)
{
    mnavNearest c = On(navmesh, 27.0, 27.0);
    mnavPolygonId never = c.polygon;
    never.slot = 9999;
    mnavPolygonId stale = c.polygon;
    stale.generation += 1;
    mnavQueryFilter bad = mnavDefaultQueryFilter();
    bad.cookie = 0;
    mnavNearestWall wall;
    mnavRandomPoint p;
    double height = 0.0;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, never, c.point, 5.0, nullptr, &wall) ==
                  mnav_errorInvalid &&
              mnavQuery_FindWallDistance(query, navmesh, stale, c.point, 5.0, nullptr, &wall) !=
                  mnav_success &&
              mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, 5.0, &bad, &wall) ==
                  mnav_errorInvalid &&
              mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, -1.0, nullptr,
                                         &wall) == mnav_errorInvalid &&
              mnavQuery_FindWallDistance(query, navmesh, c.polygon, c.point, 5.0, nullptr,
                                         nullptr) == mnav_errorInvalid,
          "wall distance refusals");
    CHECK(mnavQuery_FindRandomPointAround(query, navmesh, never, c.point, 5.0, 1u, nullptr, &p) ==
                  mnav_errorInvalid &&
              mnavQuery_FindRandomPointAround(query, navmesh, c.polygon, c.point, 5.0, 1u, &bad,
                                              &p) == mnav_errorInvalid &&
              mnavQuery_FindRandomPointAround(query, navmesh, c.polygon,
                                              (mnavPos3){(double)NAN, 0.0, 0.0}, 5.0, 1u, nullptr,
                                              &p) == mnav_errorInvalid &&
              mnavQuery_FindRandomPointAround(query, navmesh, c.polygon, c.point, 5.0, 1u, nullptr,
                                              nullptr) == mnav_errorInvalid,
          "random point around refusals");
    CHECK(mnavNavmesh_FindRandomPoint(navmesh, 1u, &bad, &p) == mnav_errorInvalid,
          "a random point with a filter not made by the default");
    CHECK(mnavPolygon_GetHeight(navmesh, never, 27.0, 27.0, &height) == mnav_errorInvalid &&
              mnavPolygon_GetHeight(navmesh, stale, 27.0, 27.0, &height) != mnav_success,
          "heights of polygons not handed out or stale");
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    mnavQuery* other = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success &&
              mnavCreateQuery(&def, &other) == mnav_success,
          "queries");
    TestWalls(query, navmesh);
    TestPointsAround(query, other, navmesh);
    TestChecks(query, navmesh);
    mnavDestroyQuery(other);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
