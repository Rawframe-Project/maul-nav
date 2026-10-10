// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walls near a point (mnav-0005) on a hand-built square, where they
// lie at exact distances: walls at the same distance keep the order the
// search meets them in, a wall exactly at the radius lies within it, a
// short buffer holds the nearest without writing past its capacity, and
// the part of a tile side no polygon across covers is a wall.

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

static void TestPartlyLinkedSide(void)
{
    // Tile 0 holds a square from x = 24 to 32 m and z = 0 to 10 m, on its
    // east side; tile 1, baked from other geometry (as when only one of
    // two tiles is rebuilt), a square across the seam from z = 0 to 5 m
    // only. The seam's part from z = 5 to 10 m has nothing across it: a
    // wall, to the wall distance and the wall list alike.
    const HandSquare west = {96, 0, 128, 40, {0, 0, 0, 0}, 0};
    const HandSquare east = {0, 0, 32, 20, {0, 0, 0, 0}, 0};
    static uint8_t bytes[2][2048];
    size_t westSize = HandTileBytes(bytes[0], 0, &west, 1);
    size_t eastSize = HandTileBytes(bytes[1], 1, &east, 1);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes[0], westSize).result == mnav_success &&
              mnavStageTile(navmesh, bytes[1], eastSize).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){30.0, 0.0, 8.0},
                          (mnavVec3){1.0f, 4.0f, 1.0f}, &n) == mnav_success &&
              n.polygon.slot != 0,
          "found");
    // From (30, 8): the north side and the seam's open part, 2 m each, in
    // the order of the square's sides; the seam's part runs from z = 10
    // to 5 m, as the side runs.
    mnavWallSegment walls[4];
    mnavWallsFound found;
    CHECK(mnavFindWalls(query, navmesh, nullptr, n.polygon, (mnavPos3){30.0, 0.0, 8.0}, 2.5, walls,
                        4, &found) == mnav_success &&
              found.count == 2,
          "two walls within 2.5 m");
    CHECK(walls[0].distance == 2.0 && walls[0].start.z == 10.0 && walls[0].end.z == 10.0 &&
              walls[1].distance == 2.0 && walls[1].start.x == 32.0 && walls[1].end.x == 32.0 &&
              walls[1].start.z == 10.0 && walls[1].end.z == 5.0 && walls[1].normal.x == -1.0,
          "the north side, then the seam's open part");
    mnavWall wall;
    CHECK(mnavFindWallDistance(query, navmesh, nullptr, n.polygon, (mnavPos3){31.0, 0.0, 8.0}, 2.5,
                               &wall) == mnav_success &&
              wall.found && wall.distance == 1.0 && wall.point.x == 32.0,
          "the seam's open part is the nearest wall");
    // From (30, 2), beside the linked part: only the south side, 2 m away;
    // the open part lies over 3.6 m off.
    CHECK(mnavFindWalls(query, navmesh, nullptr, n.polygon, (mnavPos3){30.0, 0.0, 2.0}, 2.5, walls,
                        4, &found) == mnav_success &&
              found.count == 1 && walls[0].start.z == 0.0 && walls[0].end.z == 0.0,
          "the linked part is no wall");
    mnavDestroyNavmesh(navmesh);
    // The square across from z = 5 to 10 m instead: along the side, from
    // z = 10 to 0 m, the linked part comes first and the open part after
    // it, from z = 5 to 0 m.
    const HandSquare north = {0, 20, 32, 40, {0, 0, 0, 0}, 0};
    eastSize = HandTileBytes(bytes[1], 1, &north, 1);
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes[0], westSize).result == mnav_success &&
              mnavStageTile(navmesh, bytes[1], eastSize).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){30.0, 0.0, 2.0},
                          (mnavVec3){1.0f, 4.0f, 1.0f}, &n) == mnav_success &&
              n.polygon.slot != 0 &&
              mnavFindWalls(query, navmesh, nullptr, n.polygon, (mnavPos3){30.0, 0.0, 2.0}, 2.5,
                            walls, 4, &found) == mnav_success &&
              found.count == 2 && walls[0].start.x == 32.0 && walls[0].start.z == 5.0 &&
              walls[0].end.z == 0.0 && walls[1].start.z == 0.0 && walls[1].end.z == 0.0,
          "the open part after the linked one");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestTiesAndTheRadius();
    TestShortBuffer();
    TestPartlyLinkedSide();
    return s_failures == 0 ? 0 : 1;
}
