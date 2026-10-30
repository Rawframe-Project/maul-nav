// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walls near a point on a hand-built square, where they
// lie at exact distances: walls at the same distance keep the order the
// search meets them in, a wall exactly at the radius lies within it, a
// short buffer holds the nearest without writing past its capacity, and
// the part of a tile side no polygon across covers is a wall.

#include "defs_for.h"
#include "hand_tiles.h"
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
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "created");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handSquare, sizeof(s_handSquare)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "committed");
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){5.0, 0.0, 5.0}, (mnavVec3){1.0f, 4.0f, 1.0f},
                                  nullptr, &n) == mnav_success &&
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
    mnavWall walls[4];
    mnavWallsFound found;
    CHECK(mnavQuery_FindWalls(query, navmesh, polygon, (mnavPos3){5.0, 0.0, 5.0}, 5.0, nullptr,
                              walls, 4, &found) == mnav_success &&
              found.wallCount == 4 && !found.limited,
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
    CHECK(mnavQuery_FindWalls(query, navmesh, polygon, (mnavPos3){1.0, 0.0, 5.0}, 9.0, nullptr,
                              walls, 4, &found) == mnav_success &&
              found.wallCount == 4 && walls[0].distance == 1.0 && walls[0].start.z == 0.0 &&
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
    mnavWall walls[3];
    walls[2] = (mnavWall){.distance = -1.0};
    mnavWallsFound found;
    CHECK(mnavQuery_FindWalls(query, navmesh, polygon, (mnavPos3){1.0, 0.0, 2.0}, 10.0, nullptr,
                              walls, 2, &found) == mnav_errorCapacity &&
              found.wallCount == 4,
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
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handSeamWest, sizeof(s_handSeamWest)).result ==
                  mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handSeamSouth, sizeof(s_handSeamSouth)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "committed");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){30.0, 0.0, 8.0}, (mnavVec3){1.0f, 4.0f, 1.0f},
                                  nullptr, &n) == mnav_success &&
              n.polygon.slot != 0,
          "found");
    // From (30, 8): the north side and the seam's open part, 2 m each, in
    // the order of the square's sides; the seam's part runs from z = 10
    // to 5 m, as the side runs.
    mnavWall walls[4];
    mnavWallsFound found;
    CHECK(mnavQuery_FindWalls(query, navmesh, n.polygon, (mnavPos3){30.0, 0.0, 8.0}, 2.5, nullptr,
                              walls, 4, &found) == mnav_success &&
              found.wallCount == 2,
          "two walls within 2.5 m");
    CHECK(walls[0].distance == 2.0 && walls[0].start.z == 10.0 && walls[0].end.z == 10.0 &&
              walls[1].distance == 2.0 && walls[1].start.x == 32.0 && walls[1].end.x == 32.0 &&
              walls[1].start.z == 10.0 && walls[1].end.z == 5.0 &&
              (double)walls[1].normal.x == -1.0,
          "the north side, then the seam's open part");
    mnavNearestWall wall;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, (mnavPos3){31.0, 0.0, 8.0}, 2.5,
                                     nullptr, &wall) == mnav_success &&
              wall.found && wall.wall.distance == 1.0 && wall.wall.point.x == 32.0,
          "the seam's open part is the nearest wall");
    // From (30, 2), beside the linked part: only the south side, 2 m away;
    // the open part lies over 3.6 m off.
    CHECK(mnavQuery_FindWalls(query, navmesh, n.polygon, (mnavPos3){30.0, 0.0, 2.0}, 2.5, nullptr,
                              walls, 4, &found) == mnav_success &&
              found.wallCount == 1 && walls[0].start.z == 0.0 && walls[0].end.z == 0.0,
          "the linked part is no wall");
    mnavDestroyNavmesh(navmesh);
    mnavNavmeshDef meshDef2 = NavmeshDefFor(&def);
    // The square across from z = 5 to 10 m instead: along the side, from
    // z = 10 to 0 m, the linked part comes first and the open part after
    // it, from z = 5 to 0 m.
    CHECK(mnavCreateNavmesh(&meshDef2, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handSeamWest, sizeof(s_handSeamWest)).result ==
                  mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handSeamNorth, sizeof(s_handSeamNorth)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "committed");
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){30.0, 0.0, 2.0}, (mnavVec3){1.0f, 4.0f, 1.0f},
                                  nullptr, &n) == mnav_success &&
              n.polygon.slot != 0 &&
              mnavQuery_FindWalls(query, navmesh, n.polygon, (mnavPos3){30.0, 0.0, 2.0}, 2.5,
                                  nullptr, walls, 4, &found) == mnav_success &&
              found.wallCount == 2 && walls[0].start.x == 32.0 && walls[0].start.z == 5.0 &&
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
