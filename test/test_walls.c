// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walls near a point (mnav-0005) on a hand-built square, where they
// lie at exact distances: walls at the same distance keep the order the
// search meets them in, a wall exactly at the radius lies within it, and
// a short buffer holds the nearest without writing past its capacity.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// A navmesh of one hand tile holding one square of 10 m from the origin,
// its corners (0, 0), (0, 10), (10, 10), (10, 0) in that order.
static mnavNavmesh* Square(mnavPolygonId* polygonOut)
{
    const HandSquare square = {0, 0, 40, 40, {0, 0, 0, 0}, 0};
    static uint8_t bytes[2048];
    size_t size = HandTileBytes(bytes, 0, &square, 1);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){5.0, 0.0, 5.0}, (mnavVec3){1.0f, 4.0f, 1.0f},
                          &n) == mnav_success &&
              n.polygon.slot != 0,
          "found");
    *polygonOut = n.polygon;
    return navmesh;
}

static void TestTiesAndTheRadius(void)
{
    // From the middle, the four sides lie 5 m away: with the radius 5 m,
    // all four, in the order of the square's corners.
    mnavPolygonId polygon;
    mnavNavmesh* navmesh = Square(&polygon);
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    const double corners[4][2] = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    mnavWallSegment walls[4];
    mnavWallsFound found;
    CHECK(mnavFindWalls(query, navmesh, nullptr, polygon, (mnavPos3){5.0, 0.0, 5.0}, 5.0, walls, 4,
                        &found) == mnav_success &&
              found.count == 4 && !found.limited,
          "the four sides at the radius");
    bool ordered = true;
    for (int32_t k = 0; k < 4; ++k)
    {
        ordered = ordered && walls[k].distance == 5.0 && walls[k].start.x == corners[k][0] &&
                  walls[k].start.z == corners[k][1] && walls[k].end.x == corners[(k + 1) % 4][0] &&
                  walls[k].end.z == corners[(k + 1) % 4][1];
    }
    CHECK(ordered, "in the order met, at 5 m");
    // Off the middle the nearest comes first: the side x = 0 at 1 m, then
    // z = 0 and z = 10 at 5 m in the order met, then x = 10 at 9 m.
    CHECK(mnavFindWalls(query, navmesh, nullptr, polygon, (mnavPos3){1.0, 0.0, 5.0}, 9.0, walls, 4,
                        &found) == mnav_success &&
              found.count == 4 && walls[0].distance == 1.0 && walls[0].start.z == 0.0 &&
              walls[1].distance == 5.0 && walls[1].start.x == 0.0 && walls[1].start.z == 10.0 &&
              walls[2].distance == 5.0 && walls[2].start.x == 10.0 && walls[2].start.z == 0.0 &&
              walls[3].distance == 9.0,
          "nearest first");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestShortBuffer(void)
{
    // A buffer of two, with a third wall after it as a guard: the two
    // nearest, the guard untouched.
    mnavPolygonId polygon;
    mnavNavmesh* navmesh = Square(&polygon);
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    mnavWallSegment walls[3];
    walls[2] = (mnavWallSegment){.distance = -1.0};
    mnavWallsFound found;
    CHECK(mnavFindWalls(query, navmesh, nullptr, polygon, (mnavPos3){1.0, 0.0, 2.0}, 10.0, walls, 2,
                        &found) == mnav_errorCapacity &&
              found.count == 4,
          "all four counted");
    CHECK(walls[0].distance == 1.0 && walls[1].distance == 2.0 && walls[2].distance == -1.0,
          "the two nearest, nothing past them");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestTiesAndTheRadius();
    TestShortBuffer();
    return s_failures == 0 ? 0 : 1;
}
