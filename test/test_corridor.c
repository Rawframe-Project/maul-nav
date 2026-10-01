// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Path corridors (mnav-0005): loading paths, their corners, their checks.

#include "hand_tile.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the agents' walks along their corridors.
#define FOLLOW_HASH 0xa805bbb48946a2f2ull

static mnavNavmesh* LoadWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
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

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z, float box)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){box, 2.0f, box},
                          &n) == mnav_success,
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
        CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &path) == mnav_success &&
                  mnavResetCorridor(&corridor, buffer, 8192, a.polygon, a.point) == mnav_success &&
                  mnavSetCorridor(&corridor, &path) == mnav_success,
              "loaded");
        loaded += 1;
        int32_t valid = -1;
        CHECK(mnavCheckCorridor(navmesh, nullptr, &corridor, &valid) == mnav_success &&
                  valid == path.polygonCount,
              "every polygon valid");
        CHECK(mnavCorridorCorners(corners, navmesh, &corridor, &out) == mnav_success, "corners");
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
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.polygonCount > 4,
          "a long path");
    mnavPolygonId buffer[256];
    mnavCorridor corridor;
    CHECK(mnavResetCorridor(&corridor, buffer, 2, a.polygon, a.point) == mnav_success &&
              corridor.count == 1 && mnavSetCorridor(&corridor, &path) == mnav_errorCapacity &&
              corridor.count == 1,
          "too long for two polygons, unchanged");
    mnavCorners out;
    CHECK(mnavCorridorCorners(query, navmesh, &corridor, &out) == mnav_success &&
              out.pointCount == 1 && out.points[0].x == a.point.x,
          "a fresh corridor's one point");
    CHECK(mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "loaded");
    mnavQuery* small = MakeQuery(4);
    CHECK(mnavCorridorCorners(small, navmesh, &corridor, &out) == mnav_errorLimit,
          "more polygons than the context's nodes");
    mnavDestroyQuery(small);
    // Two polygons swapped: not joined from there on.
    mnavPolygonId swap = buffer[1];
    buffer[1] = buffer[3];
    buffer[3] = swap;
    int32_t valid = -1;
    CHECK(mnavCheckCorridor(navmesh, nullptr, &corridor, &valid) == mnav_success && valid == 1 &&
              mnavCorridorCorners(query, navmesh, &corridor, &out) == mnav_errorInvalid,
          "not joined after the first");
    buffer[3] = buffer[1];
    buffer[1] = swap;
    // A tile replaced: its polygons' ids go stale.
    mnavTileId tile;
    CHECK(mnavGetTile(navmesh, 1, 0, &tile) == mnav_success, "tile (1, 0)");
    int32_t onOther = -1;
    for (int32_t i = 0; i < corridor.count && onOther < 0; ++i)
    {
        onOther = buffer[i].slot == tile.slot ? i : -1;
    }
    CHECK(onOther > 0, "the path crosses tile (1, 0)");
    CHECK(mnavStageTile(navmesh, s_tiles[1], s_sizes[1]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "tile (1, 0) replaced");
    CHECK(mnavCheckCorridor(navmesh, nullptr, &corridor, &valid) == mnav_success &&
              valid == onOther &&
              mnavCorridorCorners(query, navmesh, &corridor, &out) == mnav_errorStale,
          "valid up to its first polygon on the replaced tile");
    CHECK(mnavCheckCorridor(nullptr, nullptr, &corridor, &valid) == mnav_errorInvalid &&
              mnavResetCorridor(&corridor, buffer, 0, a.polygon, a.point) == mnav_errorInvalid &&
              mnavSetCorridor(&corridor, nullptr) == mnav_errorInvalid,
          "bad arguments");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestFiltersAndLinks(void)
{
    // A row of squares, the middle one of area 2, and a jump across a gap.
    const HandSquare squares[4] = {{20, 20, 40, 40, {0}, 0},
                                   {40, 20, 60, 40, {0}, 2},
                                   {60, 20, 80, 40, {0}, 0},
                                   {100, 20, 120, 40, {0}, 0}};
    static uint8_t bytes[8192];
    size_t size = HandTileBytes(bytes, 0, squares, 4);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavLinkDef jump = {{19.5, 0.0, 7.5}, {25.5, 0.0, 7.5}, 0.5f, 3.0f, mnav_linkJump, false};
    mnavLinkId id;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavStageLink(navmesh, &jump, &id) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 7.5, 0.1f);
    mnavNearest b = On(navmesh, 28.0, 7.5, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[16];
    mnavCorridor corridor;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 &&
              mnavResetCorridor(&corridor, buffer, 16, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "a path with a jump");
    mnavCorners out;
    CHECK(mnavCorridorCorners(query, navmesh, &corridor, &out) == mnav_success &&
              out.pointCount == 4 && out.linkCount == 1 && out.links[0].point == 1 &&
              out.links[0].kind == mnav_linkJump && out.points[1].x == 19.5,
          "the jump among the corners");
    int32_t valid = -1;
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavCheckCorridor(navmesh, &filter, &corridor, &valid) == mnav_success && valid == 1,
          "the middle square left out");
    filter = mnavDefaultQueryFilter();
    filter.kinds &= ~((uint64_t)1 << mnav_linkJump);
    CHECK(mnavCheckCorridor(navmesh, &filter, &corridor, &valid) == mnav_success && valid == 3,
          "no jumping: valid to the gap");
    filter.cookie = 0;
    CHECK(mnavCheckCorridor(navmesh, &filter, &corridor, &valid) == mnav_errorInvalid,
          "a filter not from the default");
    // Standing on a square left out, the agent may still leave it: the
    // corridor's first polygon counts whatever its area.
    mnavNearest middle = On(navmesh, 12.5, 7.5, 0.1f);
    filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavFindPath(query, navmesh, nullptr, middle.polygon, middle.point, b.polygon, b.point,
                       &path) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success &&
              mnavCheckCorridor(navmesh, &filter, &corridor, &valid) == mnav_success &&
              valid == corridor.count,
          "starting on a square left out");
    // An id never handed out is invalid, not stale.
    buffer[1].slot = 99;
    CHECK(mnavCorridorCorners(query, navmesh, &corridor, &out) == mnav_errorInvalid,
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
    // A row of five squares from (5, 5) to (30, 10), 1 to 5 in Indices.
    const HandSquare row[5] = {{20, 20, 40, 40, {0}, 0},
                               {40, 20, 60, 40, {0}, 0},
                               {60, 20, 80, 40, {0}, 0},
                               {80, 20, 100, 40, {0}, 0},
                               {100, 20, 120, 40, {0}, 0}};
    static uint8_t bytes[8192];
    size_t size = HandTileBytes(bytes, 0, row, 5);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(64);
    mnavNearest a = On(navmesh, 6.0, 7.5, 0.1f);
    mnavNearest b = On(navmesh, 28.0, 7.5, 0.1f);
    mnavPath path;
    mnavPolygonId buffer[5];
    mnavCorridor corridor;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              mnavResetCorridor(&corridor, buffer, 5, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success && Indices(&corridor) == 12345,
          "the whole row");
    mnavMove move;
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, (mnavPos3){12.0, 0.0, 7.5}, &move) ==
                  mnav_success &&
              move.end == mnav_moveReached && corridor.position.x == 12.0 &&
              Indices(&corridor) == 2345,
          "forward: the first square dropped");
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, (mnavPos3){12.0, 0.0, 12.0},
                           &move) == mnav_success &&
              move.end == mnav_moveWall && corridor.position.z == 10.0 &&
              Indices(&corridor) == 2345,
          "into the side: held at the wall");
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, (mnavPos3){6.0, 0.0, 7.5},
                           nullptr) == mnav_success &&
              Indices(&corridor) == 12345,
          "back: the first square again");
    // Target moves: nearer drops the end, farther again restores it.
    CHECK(mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){18.0, 0.0, 7.5},
                                 &move) == mnav_success &&
              corridor.target.x == 18.0 && Indices(&corridor) == 123,
          "the target nearer");
    CHECK(mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){28.0, 0.0, 7.5},
                                 nullptr) == mnav_success &&
              Indices(&corridor) == 12345,
          "the target back at the end");
    // A buffer of four: the walk back would not fit.
    mnavPolygonId small[4];
    CHECK(mnavResetCorridor(&corridor, small, 4, a.polygon, a.point) == mnav_success &&
              mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){22.0, 0.0, 7.5},
                                     nullptr) == mnav_success &&
              Indices(&corridor) == 1234,
          "four squares");
    CHECK(mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){28.0, 0.0, 7.5},
                                 nullptr) == mnav_errorCapacity &&
              Indices(&corridor) == 1234 && corridor.target.x == 22.0,
          "a fifth does not fit; nothing changed");
    // Forward to the second square, then back: the first would not fit.
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, (mnavPos3){12.0, 0.0, 7.5},
                           nullptr) == mnav_success &&
              mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){28.0, 0.0, 7.5},
                                     nullptr) == mnav_success &&
              Indices(&corridor) == 2345,
          "the second square on");
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, (mnavPos3){8.0, 0.0, 7.5},
                           nullptr) == mnav_errorCapacity &&
              Indices(&corridor) == 2345 && corridor.position.x == 12.0,
          "back to the first does not fit; nothing changed");
    // A target moved into the row's side stops there.
    CHECK(mnavMoveCorridorTarget(query, navmesh, nullptr, &corridor, (mnavPos3){28.0, 0.0, 12.0},
                                 nullptr) == mnav_success &&
              corridor.target.x == 28.0 && corridor.target.z == 10.0,
          "the target held at the wall");
    CHECK(mnavMoveCorridor(query, navmesh, nullptr, nullptr, a.point, nullptr) ==
                  mnav_errorInvalid &&
              mnavMoveCorridorTarget(query, navmesh, nullptr, nullptr, a.point, nullptr) ==
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
            mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) !=
                mnav_success ||
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
            CHECK(mnavCorridorCorners(query, navmesh, &corridor, &corners) == mnav_success,
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
            CHECK(mnavMoveCorridor(query, navmesh, nullptr, &corridor, wanted, nullptr) ==
                      mnav_success,
                  "stepped");
            int32_t count = -1;
            valid = valid &&
                    mnavCheckCorridor(navmesh, nullptr, &corridor, &count) == mnav_success &&
                    count == corridor.count;
            hash = mnavHash64(hash, &corridor.position, (int32_t)sizeof(corridor.position));
        }
    }
    printf("FOLLOW_HASH=%016llx walks=%d arrived=%d\n", (unsigned long long)hash, walks, arrived);
    CHECK(valid, "the corridor valid at every step");
    CHECK(walks > 30 && arrived == walks, "every agent arrives");
    CHECK(hash == FOLLOW_HASH, "the pinned hash");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeWorld();
    TestCornersAreThePathsStraightPath();
    TestChecksAndLimits();
    TestFiltersAndLinks();
    TestMovingTheEnds();
    TestFollowingCorridors();
    return s_failures == 0 ? 0 : 1;
}
