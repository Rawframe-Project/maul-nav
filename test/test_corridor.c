// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Path corridors: loading paths, their corners, their checks.

#include "defs_for.h"
#include "hand_tiles.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the agents' walks along their corridors.
#define FOLLOW_HASH 0xa805bbb48946a2f2ull

static mnavNavmesh* LoadWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    meshDef.tier = mnav_tierDynamic;
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
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

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z, float box)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){box, 2.0f, box},
                                  nullptr, &n) == mnav_success,
          "nearest");
    return n;
}

static void TestCornersAreThePathsStraightPath(void)
{
    // Loaded into a corridor, every path's corners are its own straight
    // path, bit for bit.
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavQuery* corners = MakeQuery(8192);
    static mnavPolygonId buffer[8192];
    uint32_t state = 5;
    int32_t same = 0;
    int32_t loaded = 0;
    for (int32_t i = 0; i < 200; ++i)
    {
        double p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (double)(state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
        }
        mnavNearest a = On(navmesh, p[0], p[1], 3.0f);
        mnavNearest b = On(navmesh, p[2], p[3], 3.0f);
        if (a.polygon.slot == 0 || b.polygon.slot == 0)
        {
            continue;
        }
        mnavPath path;
        mnavCorridor corridor;
        mnavCorners out;
        CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                                 &path) == mnav_success &&
                  mnavResetCorridor(&corridor, buffer, 8192, a.polygon, a.point) == mnav_success &&
                  mnavSetCorridor(&corridor, &path) == mnav_success,
              "loaded");
        loaded += 1;
        int32_t valid = -1;
        CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success &&
                  valid == path.polygonCount,
              "every polygon valid");
        CHECK(mnavQuery_FindCorridorCorners(corners, &corridor, navmesh, &out) == mnav_success,
              "corners");
        same +=
            out.pointCount == path.pointCount &&
                    memcmp(out.points, path.points, (size_t)path.pointCount * sizeof(mnavPos3)) == 0
                ? 1
                : 0;
    }
    CHECK(loaded == 200 && same == loaded, "the same straight path");
    mnavDestroyQuery(corners);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestChecksAndLimits(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavNearest a = On(navmesh, 2.0, 2.0, 2.0f);
    mnavNearest b = On(navmesh, 62.0, 62.0, 2.0f);
    mnavPath path;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              path.polygonCount > 4,
          "a long path");
    mnavPolygonId buffer[256];
    mnavCorridor corridor;
    CHECK(mnavResetCorridor(&corridor, buffer, 2, a.polygon, a.point) == mnav_success &&
              corridor.count == 1 && mnavSetCorridor(&corridor, &path) == mnav_errorCapacity &&
              corridor.count == 1,
          "too long for two polygons, unchanged");
    mnavCorners out;
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &out) == mnav_success &&
              out.pointCount == 1 && out.points[0].x == a.point.x,
          "a fresh corridor's one point");
    CHECK(mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "loaded");
    mnavQuery* small = MakeQuery(4);
    CHECK(mnavQuery_FindCorridorCorners(small, &corridor, navmesh, &out) == mnav_errorCapacity,
          "more polygons than the context's nodes");
    mnavDestroyQuery(small);
    // Two polygons swapped: not joined from there on.
    mnavPolygonId swap = buffer[1];
    buffer[1] = buffer[3];
    buffer[3] = swap;
    int32_t valid = -1;
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success &&
              valid == 1 &&
              mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &out) == mnav_errorInvalid,
          "not joined after the first");
    buffer[3] = buffer[1];
    buffer[1] = swap;
    // A tile replaced: its polygons' ids go stale.
    mnavTileId tile;
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 0, &tile) == mnav_success, "tile (1, 0)");
    int32_t onOther = -1;
    for (int32_t i = 0; i < corridor.count && onOther < 0; ++i)
    {
        onOther = buffer[i].slot == tile.slot ? i : -1;
    }
    CHECK(onOther > 0, "the path crosses tile (1, 0)");
    CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[1], s_sizes[1]).result == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "tile (1, 0) replaced");
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success &&
              valid == onOther &&
              mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &out) == mnav_errorStale,
          "valid up to its first polygon on the replaced tile");
    CHECK(mnavNavmesh_CheckCorridor(nullptr, &corridor, nullptr, &valid) == mnav_errorInvalid &&
              mnavResetCorridor(&corridor, buffer, 0, a.polygon, a.point) == mnav_errorInvalid &&
              mnavSetCorridor(&corridor, nullptr) == mnav_errorInvalid,
          "bad arguments");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Every corridor call refuses a corridor whose fields are malformed, and
// reads none of its buffer: one polygon more than its capacity, a
// capacity of 0, and no polygon. The buffer holds exactly its capacity,
// so AddressSanitizer would see a read past it. mnavSetCorridor replaces
// the count, so it refuses only the capacity.
static void TestMalformedCorridors(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 2.0, 2.0, 2.0f);
    mnavPolygonId* buffer = malloc(2 * sizeof(mnavPolygonId));
    CHECK(buffer != nullptr, "a buffer of two polygons");
    buffer[0] = a.polygon;
    buffer[1] = a.polygon;
    const mnavCorridor bad[3] = {{a.point, a.point, buffer, 3, 2},
                                 {a.point, a.point, buffer, 1, 0},
                                 {a.point, a.point, buffer, 0, 2}};
    for (int32_t k = 0; k < 3; ++k)
    {
        mnavCorridor c = bad[k];
        int32_t valid = -1;
        mnavCorners corners;
        mnavAreaRun runs[4];
        int32_t count = 0;
        mnavMove move;
        bool shortened = false;
        mnavPath path;
        bool refused =
            mnavNavmesh_CheckCorridor(navmesh, &c, nullptr, &valid) == mnav_errorInvalid &&
            mnavQuery_FindCorridorCorners(query, &c, navmesh, &corners) == mnav_errorInvalid &&
            mnavQuery_FindCorridorAreas(query, &c, navmesh, nullptr, runs, 4, &count) ==
                mnav_errorInvalid &&
            mnavQuery_MoveCorridor(query, &c, navmesh, a.point, nullptr, &move) ==
                mnav_errorInvalid &&
            mnavQuery_MoveCorridorTarget(query, &c, navmesh, a.point, nullptr, &move) ==
                mnav_errorInvalid &&
            mnavQuery_ShortcutCorridor(query, &c, navmesh, a.point, nullptr, &shortened) ==
                mnav_errorInvalid &&
            mnavQuery_ReplanCorridor(query, &c, navmesh, (mnavVec3){1.0f, 1.0f, 1.0f}, nullptr,
                                     &path) == mnav_errorInvalid;
        CHECK(refused && c.count == bad[k].count && c.capacity == bad[k].capacity && valid == 0 &&
                  !shortened,
              "a malformed corridor refused by every call, unchanged, the outputs zeroed");
    }
    mnavPath path;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, a.polygon, a.point, nullptr,
                             &path) == mnav_success &&
              path.polygonCount == 1,
          "a path of one polygon");
    mnavCorridor noRoom = bad[1];
    mnavCorridor stale = {a.point, a.point, buffer, 5, 2};
    CHECK(mnavSetCorridor(&noRoom, &path) == mnav_errorInvalid &&
              mnavSetCorridor(&stale, &path) == mnav_success && stale.count == 1,
          "loading refuses no room, and replaces any count");
    free(buffer);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The areas along the corridor over the squares: a run at the start, one
// into the middle square of area 2 and one out of it, and one where the
// jump lands; then a slanting path, its run starting where it crosses.
static void TestAreaRuns(mnavQuery* query, const mnavNavmesh* navmesh, mnavCorridor* corridor,
                         mnavNearest a)
{
    mnavAreaRun runs[8];
    int32_t count = -1;
    mnavCorners corners;
    mnavAreaType first = 0;
    CHECK(mnavQuery_FindCorridorAreas(query, corridor, navmesh, &corners, runs, 8, &count) ==
                  mnav_success &&
              count == 4 && corners.pointCount == 4,
          "four runs along the corners");
    first = runs[0].area;
    CHECK(runs[0].start.x == a.point.x && runs[0].corner == 0 && runs[1].area == 2 &&
              fabs(runs[1].start.x - 10.0) < 1e-9 && runs[1].corner == 0 && runs[2].area == first &&
              fabs(runs[2].start.x - 15.0) < 1e-9 && runs[3].start.x == 25.5 &&
              runs[3].corner == 2 && runs[3].area == first,
          "into area 2 at 10 m, out at 15 m, and from the landing");
    runs[2].area = 77;
    CHECK(mnavQuery_FindCorridorAreas(query, corridor, navmesh, nullptr, runs, 2, &count) ==
                  mnav_errorCapacity &&
              count == 4 && runs[1].area == 2 && runs[2].area == 77,
          "two places for four runs: the first two, nothing past them, and the count");
    CHECK(mnavQuery_FindCorridorAreas(query, corridor, navmesh, nullptr, nullptr, 0, &count) ==
                  mnav_errorCapacity &&
              count == 4,
          "counted with no places");
    CHECK(mnavQuery_FindCorridorAreas(query, corridor, navmesh, nullptr, runs, 8, nullptr) ==
                  mnav_errorInvalid &&
              mnavQuery_FindCorridorAreas(query, corridor, navmesh, nullptr, nullptr, 1, &count) ==
                  mnav_errorInvalid &&
              mnavQuery_FindCorridorAreas(query, corridor, navmesh, nullptr, runs, -1, &count) ==
                  mnav_errorInvalid,
          "no count, no runs or a negative capacity refused");
    // From (6, 5.5) to (14, 9.5): into area 2 at (10, 7.5).
    mnavNearest from = On(navmesh, 6.0, 5.5, 0.1f);
    mnavNearest to = On(navmesh, 14.0, 9.5, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[8];
    mnavCorridor slant;
    CHECK(mnavQuery_FindPath(query, navmesh, from.polygon, from.point, to.polygon, to.point,
                             nullptr, &path) == mnav_success &&
              mnavResetCorridor(&slant, buffer, 8, from.polygon, from.point) == mnav_success &&
              mnavSetCorridor(&slant, &path) == mnav_success &&
              mnavQuery_FindCorridorAreas(query, &slant, navmesh, nullptr, runs, 8, &count) ==
                  mnav_success &&
              count == 2 && fabs(runs[1].start.x - 10.0) < 1e-9 &&
              fabs(runs[1].start.z - 7.5) < 1e-9 && runs[1].area == 2,
          "a slanting path crosses into area 2 at (10, 7.5)");
}

// A U round a wall (s_handRoundAWall), the bottom middle square of area
// 2: one run into it where the path leaves the wall's corner at (15, 5),
// not where it first meets the line of that portal running down the
// wall's side, and one out of it; the crossings between squares of one
// area add none.
static void TestAreaRunsRoundAWall(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRoundAWall, sizeof(s_handRoundAWall)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "the U loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 14.0, 0.1f);
    mnavNearest b = On(navmesh, 6.0, 1.0, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[16];
    mnavCorridor corridor;
    mnavAreaRun runs[8];
    int32_t count = -1;
    mnavCorners corners;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              path.polygonCount == 7 &&
              mnavResetCorridor(&corridor, buffer, 16, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success &&
              mnavQuery_FindCorridorAreas(query, &corridor, navmesh, &corners, runs, 8, &count) ==
                  mnav_success,
          "round the wall");
    CHECK(corners.pointCount == 4 && count == 3 && runs[1].area == 2 && runs[1].corner == 2 &&
              fabs(runs[1].start.x - 15.0) < 1e-9 && fabs(runs[1].start.z - 5.0) < 1e-9 &&
              runs[2].area == runs[0].area && runs[2].corner == 2 &&
              fabs(runs[2].start.x - 10.0) < 1e-9 && fabs(runs[2].start.z - 25.0 / 9.0) < 1e-9,
          "into area 2 at the wall's corner, out of it at x = 10 m");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestFiltersAndLinks(void)
{
    // A row of squares, the middle one of area 2, and a jump across a gap
    // (s_handRowWithBridge).
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavLinkDef jump = {{19.5, 0.0, 7.5}, {25.5, 0.0, 7.5}, 0.5f, 3.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId id;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRowWithBridge, sizeof(s_handRowWithBridge))
                      .result == mnav_success &&
              mnavStageLink(navmesh, &jump, &id) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 7.5, 0.1f);
    mnavNearest b = On(navmesh, 28.0, 7.5, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[16];
    mnavCorridor corridor;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 &&
              mnavResetCorridor(&corridor, buffer, 16, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "a path with a jump");
    mnavCorners out;
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &out) == mnav_success &&
              out.pointCount == 4 && out.linkCount == 1 && out.links[0].point == 1 &&
              out.links[0].kind == mnav_linkJump && out.points[1].x == 19.5,
          "the jump among the corners");
    TestAreaRuns(query, navmesh, &corridor, a);
    int32_t valid = -1;
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, &filter, &valid) == mnav_success &&
              valid == 1,
          "the middle square left out");
    filter = mnavDefaultQueryFilter();
    filter.kinds &= ~((uint64_t)1 << mnav_linkJump);
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, &filter, &valid) == mnav_success &&
              valid == 3,
          "no jumping: valid to the gap");
    filter.cookie = 0;
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, &filter, &valid) == mnav_errorInvalid,
          "a filter not from the default");
    // Standing on a square left out, the agent may still leave it: the
    // corridor's first polygon counts whatever its area.
    mnavNearest middle = On(navmesh, 12.5, 7.5, 0.1f);
    filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavQuery_FindPath(query, navmesh, middle.polygon, middle.point, b.polygon, b.point,
                             nullptr, &path) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success &&
              mnavNavmesh_CheckCorridor(navmesh, &corridor, &filter, &valid) == mnav_success &&
              valid == corridor.count,
          "starting on a square left out");
    // An id never handed out is invalid, not stale.
    buffer[1].slot = 99;
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &out) == mnav_errorInvalid,
          "an id never handed out");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The polygon indices of a corridor, as one number in base 10.
static int32_t Indices(const mnavCorridor* corridor)
{
    int32_t digits = 0;
    for (int32_t i = 0; i < corridor->count; ++i)
    {
        digits = digits * 10 + (int32_t)corridor->polygons[i].polygon + 1;
    }
    return digits;
}

static void TestMovingTheEnds(void)
{
    // A row of five squares from (5, 5) to (30, 10), 1 to 5 in Indices
    // (s_handRowOfFive).
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRowOfFive, sizeof(s_handRowOfFive)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 7.5, 0.1f);
    mnavNearest b = On(navmesh, 28.0, 7.5, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[5];
    mnavCorridor corridor;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              mnavResetCorridor(&corridor, buffer, 5, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success && Indices(&corridor) == 12345,
          "the whole row");
    mnavMove move;
    CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, (mnavPos3){12.0, 0.0, 7.5}, nullptr,
                                 &move) == mnav_success &&
              move.end == mnav_moveReached && corridor.position.x == 12.0 &&
              Indices(&corridor) == 2345,
          "forward: the first square dropped");
    CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, (mnavPos3){12.0, 0.0, 12.0}, nullptr,
                                 &move) == mnav_success &&
              move.end == mnav_moveWall && corridor.position.z == 10.0 &&
              Indices(&corridor) == 2345,
          "into the side: held at the wall");
    CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, (mnavPos3){6.0, 0.0, 7.5}, nullptr,
                                 nullptr) == mnav_success &&
              Indices(&corridor) == 12345,
          "back: the first square again");
    // Target moves: nearer drops the end, farther again restores it.
    CHECK(mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){18.0, 0.0, 7.5},
                                       nullptr, &move) == mnav_success &&
              corridor.target.x == 18.0 && Indices(&corridor) == 123,
          "the target nearer");
    CHECK(mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){28.0, 0.0, 7.5},
                                       nullptr, nullptr) == mnav_success &&
              Indices(&corridor) == 12345,
          "the target back at the end");
    // A buffer of four: the walk back would not fit.
    mnavPolygonId small[4];
    CHECK(mnavResetCorridor(&corridor, small, 4, a.polygon, a.point) == mnav_success &&
              mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){22.0, 0.0, 7.5},
                                           nullptr, nullptr) == mnav_success &&
              Indices(&corridor) == 1234,
          "four squares");
    CHECK(mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){28.0, 0.0, 7.5},
                                       nullptr, nullptr) == mnav_errorCapacity &&
              Indices(&corridor) == 1234 && corridor.target.x == 22.0,
          "a fifth does not fit; nothing changed");
    // Forward to the second square, then back: the first would not fit.
    CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, (mnavPos3){12.0, 0.0, 7.5}, nullptr,
                                 nullptr) == mnav_success &&
              mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){28.0, 0.0, 7.5},
                                           nullptr, nullptr) == mnav_success &&
              Indices(&corridor) == 2345,
          "the second square on");
    CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, (mnavPos3){8.0, 0.0, 7.5}, nullptr,
                                 nullptr) == mnav_errorCapacity &&
              Indices(&corridor) == 2345 && corridor.position.x == 12.0,
          "back to the first does not fit; nothing changed");
    // A target moved into the row's side stops there.
    CHECK(mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){28.0, 0.0, 12.0},
                                       nullptr, nullptr) == mnav_success &&
              corridor.target.x == 28.0 && corridor.target.z == 10.0,
          "the target held at the wall");
    CHECK(mnavQuery_MoveCorridor(query, nullptr, navmesh, a.point, nullptr, nullptr) ==
                  mnav_errorInvalid &&
              mnavQuery_MoveCorridorTarget(query, nullptr, navmesh, a.point, nullptr, nullptr) ==
                  mnav_errorInvalid,
          "no corridor");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestFollowingCorridors(void)
{
    // Agents on the baked world walk their corridors to their targets in
    // steps of up to 0.5 m toward the first corner: the corridor stays
    // valid all the way, and every walk is pinned.
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    static mnavPolygonId buffer[8192];
    uint32_t state = 77;
    uint64_t hash = MNAV_HASH_INIT;
    int32_t arrived = 0;
    int32_t walks = 0;
    bool valid = true;
    for (int32_t i = 0; i < 40; ++i)
    {
        double p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (double)(state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
        }
        mnavNearest a = On(navmesh, p[0], p[1], 3.0f);
        mnavNearest b = On(navmesh, p[2], p[3], 3.0f);
        mnavPath path;
        mnavCorridor corridor;
        if (a.polygon.slot == 0 || b.polygon.slot == 0 ||
            mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                               &path) != mnav_success ||
            path.end != mnav_pathFound)
        {
            continue;
        }
        walks += 1;
        CHECK(mnavResetCorridor(&corridor, buffer, 8192, a.polygon, a.point) == mnav_success &&
                  mnavSetCorridor(&corridor, &path) == mnav_success,
              "loaded");
        for (int32_t step = 0; step < 400; ++step)
        {
            mnavCorners corners;
            CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners) ==
                      mnav_success,
                  "corners");
            mnavPos3 next = corners.pointCount > 1 ? corners.points[1] : corridor.target;
            double dx = next.x - corridor.position.x;
            double dz = next.z - corridor.position.z;
            double d = sqrt(dx * dx + dz * dz);
            if (corners.pointCount <= 2 && d < 1.0e-9)
            {
                arrived += 1;
                break;
            }
            double f = d > 0.5 ? 0.5 / d : 1.0;
            mnavPos3 wanted = {corridor.position.x + f * dx, 0.0, corridor.position.z + f * dz};
            CHECK(mnavQuery_MoveCorridor(query, &corridor, navmesh, wanted, nullptr, nullptr) ==
                      mnav_success,
                  "stepped");
            int32_t count = -1;
            valid =
                valid &&
                mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &count) == mnav_success &&
                count == corridor.count;
            hash = mnavHash64(hash, &corridor.position, (int64_t)sizeof(corridor.position));
        }
    }
    printf("FOLLOW_HASH=%016llx walks=%d arrived=%d\n", (unsigned long long)hash, walks, arrived);
    CHECK(valid, "the corridor valid at every step");
    CHECK(walks > 30 && arrived == walks, "every agent arrives");
    CHECK(hash == FOLLOW_HASH, "the pinned hash");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestShortcuts(void)
{
    // A 3 by 3 grid of squares from (5, 5) to (20, 20), its centre of
    // area 2 (s_handGridWithCentre). Planned without the centre, the corridor goes round it; a
    // ray across the centre then shortens it.
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    meshDef.tier = mnav_tierDynamic;
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handGridWithCentre, sizeof(s_handGridWithCentre))
                      .result == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 12.5, 0.1f);
    mnavNearest b = On(navmesh, 19.0, 12.5, 0.1f);
    mnavQueryFilter round = mnavDefaultQueryFilter();
    round.areas &= ~((uint64_t)1 << 2);
    mnavPath path;
    mnavPolygonId buffer[16];
    mnavCorridor corridor;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, &round,
                             &path) == mnav_success &&
              mnavResetCorridor(&corridor, buffer, 16, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success && corridor.count == 5,
          "round the centre, five squares");
    bool shortened = true;
    CHECK(mnavQuery_ShortcutCorridor(query, &corridor, navmesh, (mnavPos3){19.0, 0.0, 12.5}, &round,
                                     &shortened) == mnav_success &&
              !shortened && corridor.count == 5,
          "without the centre the ray meets a wall");
    // Through the centre to the far wall: the ray passes the corridor's
    // last square but meets the wall before the point, so nothing changes.
    CHECK(mnavQuery_ShortcutCorridor(query, &corridor, navmesh, (mnavPos3){25.0, 0.0, 12.5},
                                     nullptr, &shortened) == mnav_success &&
              !shortened && corridor.count == 5,
          "a ray that meets a wall shortens nothing");
    CHECK(mnavQuery_ShortcutCorridor(query, &corridor, navmesh, (mnavPos3){19.0, 0.0, 12.5},
                                     nullptr, &shortened) == mnav_success &&
              shortened && Indices(&corridor) == 456,
          "straight across the centre");
    mnavCorners corners;
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners) == mnav_success &&
              corners.pointCount == 2,
          "a straight line now");
    CHECK(mnavQuery_ShortcutCorridor(query, &corridor, navmesh, (mnavPos3){19.0, 0.0, 12.5},
                                     nullptr, &shortened) == mnav_success &&
              !shortened,
          "nothing shorter left");
    CHECK(mnavQuery_ShortcutCorridor(query, &corridor, navmesh, a.point, nullptr, nullptr) ==
              mnav_errorInvalid,
          "nowhere to say");
    // Replanned from the centre square with the centre left out: the
    // corridor's own first polygon is still current and still its start,
    // though no search for the nearest usable polygon would find it.
    mnavNearest centre = On(navmesh, 12.5, 12.5, 0.1f);
    CHECK(mnavResetCorridor(&corridor, buffer, 16, centre.polygon, centre.point) == mnav_success &&
              mnavQuery_MoveCorridorTarget(query, &corridor, navmesh, (mnavPos3){19.0, 0.0, 12.5},
                                           nullptr, nullptr) == mnav_success &&
              mnavQuery_ReplanCorridor(query, &corridor, navmesh, (mnavVec3){0.1f, 1.0f, 0.1f},
                                       &round, &path) == mnav_success &&
              path.end == mnav_pathFound && path.polygons[0].polygon == 4,
          "planned from the corridor's own start");
    // A position off the navmesh with its polygon gone: found again
    // within the box, and the corridor starts on the navmesh.
    CHECK(
        mnavNavmesh_StageTile(navmesh, s_handGridWithCentre, sizeof(s_handGridWithCentre)).result ==
                mnav_success &&
            mnavNavmesh_Commit(navmesh) == mnav_success,
        "the tile replaced");
    corridor.position = (mnavPos3){4.7, 0.0, 12.5};
    CHECK(mnavQuery_ReplanCorridor(query, &corridor, navmesh, (mnavVec3){1.0f, 1.0f, 1.0f}, nullptr,
                                   &path) == mnav_success &&
              path.end == mnav_pathFound && corridor.position.x == 5.0,
          "the position snapped back on");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestReplanning(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavNearest a = On(navmesh, 2.0, 2.0, 2.0f);
    mnavNearest b = On(navmesh, 62.0, 62.0, 2.0f);
    mnavPath path;
    static mnavPolygonId buffer[256];
    mnavCorridor corridor;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "loaded");
    mnavVec3 box = {1.0f, 2.0f, 1.0f};
    // Every tile replaced, the start's and the target's too: the ends
    // are found again and the corridor planned anew.
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "replaced");
    int32_t valid = -1;
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success &&
              valid == 0,
          "nothing valid");
    mnavPos3 target = corridor.target;
    CHECK(mnavQuery_ReplanCorridor(query, &corridor, navmesh, box, nullptr, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound &&
              mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success &&
              valid == corridor.count && corridor.position.x == a.point.x &&
              corridor.target.x == target.x && corridor.target.z == target.z,
          "planned again between the same points");
    // The target's tile gone: no polygon to plan to, the corridor kept.
    int32_t count = corridor.count;
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 1, 1) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "tile (1, 1) removed");
    CHECK(mnavQuery_ReplanCorridor(query, &corridor, navmesh, box, nullptr, &path) ==
                  mnav_success &&
              path.end == mnav_pathNotLoaded && path.polygonCount == 0 && corridor.count == count,
          "nothing loaded at the target");
    CHECK(mnavQuery_ReplanCorridor(query, nullptr, navmesh, box, nullptr, &path) ==
              mnav_errorInvalid,
          "no corridor");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A corridor whose position stands on the seam between two tiles, in the
// first polygon and on its edge to the second: the portal there is
// crossed already, so a target on the seam too is reached straight, not
// round the portal's end.
static void TestStartOnASeam(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavPolygonId above = On(navmesh, 6.0, 32.2, 0.1f).polygon;
    mnavPolygonId below = On(navmesh, 6.0, 31.8, 0.1f).polygon;
    CHECK(above.slot != below.slot && On(navmesh, 7.0, 31.8, 0.1f).polygon.polygon == below.polygon,
          "a polygon each side of the seam, the lower one reaching x = 7 m");
    mnavPolygonId buffer[2] = {above, below};
    mnavCorridor corridor = {{6.0, 0.0, 32.0}, {6.0, 0.0, 32.0}, buffer, 2, 2};
    mnavCorners corners;
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners) == mnav_success &&
              corners.pointCount == 1 && corners.points[0].x == 6.0 && corners.points[0].z == 32.0,
          "at the target: one corner, the target");
    corridor.target = (mnavPos3){7.0, 0.0, 32.0};
    CHECK(mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners) == mnav_success &&
              corners.pointCount == 2 && corners.points[1].x == 7.0 && corners.points[1].z == 32.0,
          "along the seam: straight to the target");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    LoadWorldTiles();
    TestCornersAreThePathsStraightPath();
    TestChecksAndLimits();
    TestMalformedCorridors();
    TestFiltersAndLinks();
    TestMovingTheEnds();
    TestFollowingCorridors();
    TestShortcuts();
    TestReplanning();
    TestStartOnASeam();
    TestAreaRunsRoundAWall();
    return s_failures == 0 ? 0 : 1;
}
