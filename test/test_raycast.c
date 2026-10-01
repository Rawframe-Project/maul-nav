// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Raycasts (mnav-0005) on hand-built tiles and the baked world. Hand cells are
// 0.25 m.

#include "hand_tile.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

// The hash of the world's rays, the same on every platform.
#define WORLD_RAYS_HASH 0x5a68f1b0a587706full

static uint8_t s_bytes[2][8192];

static mnavNavmesh* Make(const HandSquare* first, int32_t firstCount, const HandSquare* second,
                         int32_t secondCount)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    size_t size = HandTileBytes(s_bytes[0], 0, first, firstCount);
    CHECK(mnavStageTile(navmesh, s_bytes[0], size).result == mnav_success, "first staged");
    if (second != nullptr)
    {
        size = HandTileBytes(s_bytes[1], 1, second, secondCount);
        CHECK(mnavStageTile(navmesh, s_bytes[1], size).result == mnav_success, "second staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavQuery* MakeQuery(int32_t nodes)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static mnavRay Cast(mnavQuery* query, const mnavNavmesh* navmesh, double x0, double z0, double x1,
                    double z1)
{
    mnavNearest a;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x0, 0.0, z0}, (mnavVec3){0.1f, 1.0f, 0.1f},
                          &a) == mnav_success &&
              a.over,
          "the start on a polygon");
    mnavRay ray = {0};
    CHECK(mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, (mnavPos3){x1, 0.0, z1}, &ray) ==
              mnav_success,
          "cast");
    return ray;
}

static const HandSquare s_row[4] = {{0, 0, 20, 20, {0}, 0},
                                    {20, 0, 40, 20, {0}, 0},
                                    {40, 0, 60, 20, {0}, 0},
                                    {60, 0, 80, 20, {0}, 0}};
static const HandSquare s_ell[3] = {
    {0, 0, 20, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}};

static void TestReachAndWalls(void)
{
    mnavNavmesh* navmesh = Make(s_row, 4, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavRay ray = Cast(query, navmesh, 1.0, 2.5, 19.0, 2.5);
    CHECK(ray.end == mnav_rayReached && ray.t == 1.0 && ray.polygonCount == 4 &&
              ray.normalX == 0.0 && ray.normalZ == 0.0,
          "down the row");
    ray = Cast(query, navmesh, 1.0, 2.5, 25.0, 2.5);
    CHECK(ray.end == mnav_rayWall && ray.t == 19.0 / 24.0 && ray.polygonCount == 4 &&
              ray.normalX == -1.0 && ray.normalZ == 0.0,
          "the row's far end, its normal back along the row");
    ray = Cast(query, navmesh, 1.0, 2.5, 1.0, 2.5);
    CHECK(ray.end == mnav_rayReached && ray.t == 1.0 && ray.polygonCount == 1, "no length");
    ray = Cast(query, navmesh, 3.0, 1.0, 3.0, 7.0);
    CHECK(ray.end == mnav_rayWall && ray.t == 4.0 / 6.0 && ray.polygonCount == 1 &&
              ray.normalX == 0.0 && ray.normalZ == -1.0,
          "the row's side");
    // The row's other side lies on the tile's -Z side.
    ray = Cast(query, navmesh, 3.0, 1.0, 3.0, -3.0);
    CHECK(ray.end == mnav_rayNotLoaded && ray.t == 0.25, "the tile's side");
    mnavDestroyQuery(query);
    query = MakeQuery(2);
    ray = Cast(query, navmesh, 1.0, 2.5, 19.0, 2.5);
    // Two polygons crossed, it stops at the edge into the third.
    CHECK(ray.end == mnav_rayOutOfNodes && ray.t == 0.5 && ray.polygonCount == 2,
          "two polygons at most");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestCorners(void)
{
    mnavNavmesh* navmesh = Make(s_ell, 3, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    // Toward (6, 9) the ray meets the first square's top at z = 5.
    mnavRay ray = Cast(query, navmesh, 1.0, 1.0, 6.0, 9.0);
    CHECK(ray.end == mnav_rayWall && ray.t == 0.5 && ray.polygonCount == 1 && ray.normalX == 0.0 &&
              ray.normalZ == -1.0,
          "the wall above the start");
    // Exactly through the L's inner corner, (5, 5): it goes on.
    ray = Cast(query, navmesh, 1.0, 1.0, 9.0, 9.0);
    CHECK(ray.end == mnav_rayReached && ray.polygonCount == 3, "through the corner");
    ray = Cast(query, navmesh, 9.0, 9.0, 1.0, 1.0);
    CHECK(ray.end == mnav_rayReached && ray.polygonCount == 3, "and back through it");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTileSides(void)
{
    // A tall square on tile (0, 0)'s +X side, world z 0 to 20, and one on
    // tile (1, 0)'s -X side, z 0 to 10: the link covers z 0 to 10.
    const HandSquare left[1] = {{100, 0, 128, 80, {0}, 0}};
    const HandSquare right[1] = {{0, 0, 20, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(left, 1, right, 1);
    mnavQuery* query = MakeQuery(64);
    mnavRay ray = Cast(query, navmesh, 26.0, 1.0, 36.0, 7.0);
    CHECK(ray.end == mnav_rayReached && ray.polygonCount == 2, "across the link");
    // Crossing x = 32 at z = 11, past the link: a wall.
    ray = Cast(query, navmesh, 26.0, 5.0, 36.0, 15.0);
    CHECK(ray.end == mnav_rayWall && ray.t == 0.6 && ray.polygonCount == 1 && ray.normalX == -1.0,
          "past the link's end");
    // Crossing exactly at the link's end, z = 10: through, then the right
    // square's top.
    ray = Cast(query, navmesh, 26.0, 4.0, 38.0, 16.0);
    CHECK(ray.end == mnav_rayWall && ray.t == 0.5 && ray.polygonCount == 2 && ray.normalZ == -1.0,
          "the link's end counts");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(left, 1, nullptr, 0);
    query = MakeQuery(64);
    ray = Cast(query, navmesh, 26.0, 1.0, 36.0, 7.0);
    CHECK(ray.end == mnav_rayNotLoaded && ray.t == 0.6 && ray.polygonCount == 1 &&
              ray.normalX == 0.0,
          "nothing loaded beyond");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestArgumentsAreChecked(void)
{
    mnavNavmesh* navmesh = Make(s_row, 4, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavNearest a;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){1.0, 0.0, 1.0}, (mnavVec3){0.1f, 1.0f, 0.1f},
                          &a) == mnav_success,
          "start");
    mnavRay ray;
    mnavPos3 nan = {(double)NAN, 0.0, 0.0};
    CHECK(mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, nan, &ray) ==
                  mnav_errorInvalid &&
              mnavRaycast(query, navmesh, nullptr, a.polygon, nan, a.point, &ray) ==
                  mnav_errorInvalid,
          "points not finite");
    CHECK(mnavRaycast(nullptr, navmesh, nullptr, a.polygon, a.point, a.point, &ray) ==
                  mnav_errorInvalid &&
              mnavRaycast(query, nullptr, nullptr, a.polygon, a.point, a.point, &ray) ==
                  mnav_errorInvalid &&
              mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, a.point, nullptr) ==
                  mnav_errorInvalid,
          "NULL arguments");
    mnavPolygonId bad = a.polygon;
    bad.slot = 9;
    CHECK(mnavRaycast(query, navmesh, nullptr, bad, a.point, a.point, &ray) == mnav_errorInvalid,
          "no such slot");
    size_t size = HandTileBytes(s_bytes[0], 0, s_row, 4);
    CHECK(mnavStageTile(navmesh, s_bytes[0], size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, a.point, &ray) ==
                  mnav_errorStale,
          "stale");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Sixteen ways round, about 30 m long, in exact numbers rather than from a
// C library's sine, which may differ between platforms.
static const double s_fan[16][2] = {{30, 0},  {28, 11.5},   {21, 21},   {11.5, 28},
                                    {0, 30},  {-11.5, 28},  {-21, 21},  {-28, 11.5},
                                    {-30, 0}, {-28, -11.5}, {-21, -21}, {-11.5, -28},
                                    {0, -30}, {11.5, -28},  {21, -21},  {28, -11.5}};

static void TestWorldRays(void)
{
    // Rays fanned from points of the baked world: a reached ray's ground
    // stays on the navmesh, sampled every 0.25 m; all are pinned.
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavQuery* query = MakeQuery(8192);
    uint64_t hash = MNAV_HASH_INIT;
    int32_t ends[4] = {0};
    bool onMesh = true;
    for (int32_t i = 0; i < 64; ++i)
    {
        double x = 4.0 + (i % 8) * 7.5;
        double z = 4.0 + (i / 8) * 7.5;
        mnavNearest a;
        CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){3.0f, 2.0f, 3.0f},
                              &a) == mnav_success,
              "start");
        if (a.polygon.slot == 0)
        {
            continue;
        }
        for (int32_t k = 0; k < 16; ++k)
        {
            mnavPos3 end = {a.point.x + s_fan[k][0], 0.0, a.point.z + s_fan[k][1]};
            mnavRay ray = {0};
            CHECK(mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, end, &ray) ==
                      mnav_success,
                  "cast");
            ends[ray.end] += 1;
            for (int32_t s = 0; s <= 120; ++s)
            {
                double f = ray.t * s / 120.0;
                mnavPos3 p = {a.point.x + f * (end.x - a.point.x), a.point.y,
                              a.point.z + f * (end.z - a.point.z)};
                mnavNearest n;
                onMesh = onMesh &&
                         mnavFindNearest(navmesh, nullptr, p, (mnavVec3){0.5f, 2.0f, 0.5f}, &n) ==
                             mnav_success &&
                         n.polygon.slot != 0 && hypot(p.x - n.point.x, p.z - n.point.z) < 1.0e-9;
            }
            hash = mnavHash64(hash, &ray.end, (int32_t)sizeof(ray.end));
            hash = mnavHash64(hash, &ray.t, (int32_t)sizeof(ray.t));
            hash =
                mnavHash64(hash, ray.polygons, ray.polygonCount * (int32_t)sizeof(mnavPolygonId));
        }
    }
    printf("WORLD_RAYS_HASH=%016llx reached=%d wall=%d notLoaded=%d\n", (unsigned long long)hash,
           ends[0], ends[1], ends[2]);
    CHECK(onMesh, "every ray on the navmesh as far as it went");
    CHECK(ends[0] > 0 && ends[1] > 0 && ends[2] > 0, "every kind of end");
    CHECK(hash == WORLD_RAYS_HASH, "the pinned hash");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestReachAndWalls();
    TestCorners();
    TestTileSides();
    TestArgumentsAreChecked();
    TestWorldRays();
    return s_failures == 0 ? 0 : 1;
}
