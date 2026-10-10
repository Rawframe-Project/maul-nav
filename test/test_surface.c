// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Moving along the surface (mnav-0005) on hand-built tiles and the baked
// world, and the node table a move leaves empty. Hand cells are 0.25 m by
// 0.125 m.

#include "hand_tile.h"
#include "query.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

// The hash of the world's moves, the same on every platform.
#define WORLD_MOVES_HASH 0x0a1f6e7a2f2b8f5aull

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
    CHECK(mnavStageTile(navmesh, s_bytes[0], size).result == mnav_success, "first");
    if (second != nullptr)
    {
        size = HandTileBytes(s_bytes[1], 1, second, secondCount);
        CHECK(mnavStageTile(navmesh, s_bytes[1], size).result == mnav_success, "second");
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

static mnavMove Move(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                     double x0, double z0, double x1, double z1)
{
    mnavNearest a;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x0, 0.0, z0}, (mnavVec3){0.1f, 2.0f, 0.1f},
                          &a) == mnav_success &&
              a.over,
          "the start on a polygon");
    mnavMove move = {0};
    CHECK(mnavMoveAlongSurface(query, navmesh, filter, a.polygon, a.point, (mnavPos3){x1, 0.0, z1},
                               &move) == mnav_success,
          "moved");
    return move;
}

static bool At(const mnavMove* move, double x, double z)
{
    return move->point.x == x && move->point.z == z;
}

// A row of five squares from (5, 5) to (30, 10).
static const HandSquare s_row[5] = {{20, 20, 40, 40, {0}, 0},
                                    {40, 20, 60, 40, {0}, 0},
                                    {60, 20, 80, 40, {0}, 2},
                                    {80, 20, 100, 40, {0}, 0},
                                    {100, 20, 120, 40, {0}, 0}};

static void TestAlongARow(void)
{
    mnavNavmesh* navmesh = Make(s_row, 5, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavMove move = Move(query, navmesh, nullptr, 6.0, 7.5, 18.0, 7.5);
    CHECK(move.end == mnav_moveReached && At(&move, 18.0, 7.5) && move.point.y == 0.0 &&
              move.polygon.polygon == 2 && move.polygonCount == 3 &&
              move.polygons[0].polygon == 0 && move.polygons[2].polygon == 2,
          "three squares along");
    move = Move(query, navmesh, nullptr, 6.0, 7.5, 8.0, 12.0);
    CHECK(move.end == mnav_moveWall && At(&move, 8.0, 10.0) && move.polygonCount == 1,
          "stopped at the row's side, straight below the wanted point");
    move = Move(query, navmesh, nullptr, 28.0, 7.5, 33.0, 8.0);
    CHECK(move.end == mnav_moveWall && At(&move, 30.0, 8.0) && move.polygon.polygon == 4,
          "at the row's end");
    // Straight above the corner the fourth and fifth squares share, both
    // their walls offer (25, 10): the first met, the fourth's, wins.
    move = Move(query, navmesh, nullptr, 6.0, 7.5, 25.0, 12.0);
    CHECK(move.end == mnav_moveWall && At(&move, 25.0, 10.0) && move.polygon.polygon == 3,
          "a tie to the wall met first");
    move = Move(query, navmesh, nullptr, 28.0, 7.5, 28.0, 7.5);
    CHECK(move.end == mnav_moveReached && At(&move, 28.0, 7.5), "no move at all");
    // The middle square left out is a wall.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    move = Move(query, navmesh, &filter, 11.0, 7.5, 18.0, 7.5);
    CHECK(move.end == mnav_moveWall && At(&move, 15.0, 7.5) && move.polygonCount == 1,
          "before a square left out");
    mnavDestroyQuery(query);
    // Two polygons at most.
    query = MakeQuery(2);
    move = Move(query, navmesh, nullptr, 6.0, 7.5, 28.0, 7.5);
    CHECK(move.end == mnav_moveOutOfNodes && move.polygonCount <= 2, "out of nodes");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTileSides(void)
{
    // A square on tile (0, 0)'s +X side, z 0 to 20 m, and one on tile
    // (1, 0)'s -X side, z 0 to 10 m: the side above z = 10 is wall.
    const HandSquare left[1] = {{100, 0, 128, 80, {0}, 0}};
    const HandSquare right[1] = {{0, 0, 20, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(left, 1, right, 1);
    mnavQuery* query = MakeQuery(64);
    mnavMove move = Move(query, navmesh, nullptr, 31.0, 5.0, 33.0, 5.0);
    CHECK(move.end == mnav_moveReached && move.polygonCount == 2, "across the link");
    move = Move(query, navmesh, nullptr, 31.0, 15.0, 33.0, 15.0);
    CHECK(move.end == mnav_moveWall && At(&move, 32.0, 15.0), "the side past the link is wall");
    move = Move(query, navmesh, nullptr, 31.0, 15.0, 33.0, 9.0);
    CHECK(move.end == mnav_moveReached && At(&move, 33.0, 9.0), "down into the link's reach");
    move = Move(query, navmesh, nullptr, 31.0, 15.0, 34.0, 12.0);
    CHECK(move.end == mnav_moveWall && At(&move, 32.0, 12.0), "to the wall beside the link");
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(left, 1, nullptr, 0);
    move = Move(query, navmesh, nullptr, 31.0, 5.0, 34.0, 5.0);
    CHECK(move.end == mnav_moveNotLoaded && At(&move, 32.0, 5.0), "nothing loaded beyond");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestHeights(void)
{
    // A square rising 1 m across 5 m of x.
    const HandSquare slope[1] = {{20, 20, 40, 40, {0, 0, 8, 8}, 0}};
    mnavNavmesh* navmesh = Make(slope, 1, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavMove move = Move(query, navmesh, nullptr, 5.5, 7.5, 7.5, 7.5);
    CHECK(move.end == mnav_moveReached && move.point.y == 0.5, "halfway up");
    move = Move(query, navmesh, nullptr, 5.5, 7.5, 12.0, 7.5);
    CHECK(move.end == mnav_moveWall && move.point.y == 1.0, "at the top edge");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestArguments(void)
{
    mnavNavmesh* navmesh = Make(s_row, 5, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavNearest a;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){6.0, 0.0, 7.5}, (mnavVec3){0.1f, 1, 0.1f},
                          &a) == mnav_success,
          "start");
    mnavMove move;
    mnavPos3 nan = {(double)NAN, 0.0, 0.0};
    CHECK(mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, a.point, nan, &move) ==
                  mnav_errorInvalid &&
              mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, nan, a.point, &move) ==
                  mnav_errorInvalid &&
              mnavMoveAlongSurface(nullptr, navmesh, nullptr, a.polygon, a.point, a.point, &move) ==
                  mnav_errorInvalid &&
              mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, a.point, a.point, nullptr) ==
                  mnav_errorInvalid,
          "bad arguments");
    mnavQueryFilter filter = {0};
    CHECK(mnavMoveAlongSurface(query, navmesh, &filter, a.polygon, a.point, a.point, &move) ==
              mnav_errorInvalid,
          "a filter not from the default");
    size_t size = HandTileBytes(s_bytes[0], 0, s_row, 5);
    CHECK(mnavStageTile(navmesh, s_bytes[0], size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, a.point, a.point, &move) ==
                  mnav_errorStale,
          "stale");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Sixteen ways round, about 6 m long, in exact numbers.
static const double s_fan[16][2] = {{6, 0},  {5.5, 2.3},   {4.2, 4.2},   {2.3, 5.5},
                                    {0, 6},  {-2.3, 5.5},  {-4.2, 4.2},  {-5.5, 2.3},
                                    {-6, 0}, {-5.5, -2.3}, {-4.2, -4.2}, {-2.3, -5.5},
                                    {0, -6}, {2.3, -5.5},  {4.2, -4.2},  {5.5, -2.3}};

static mnavNavmesh* WorldNavmesh(void)
{
    BakeWorld();
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

static void TestWorldMoves(void)
{
    // Moves fanned from points of the baked world: each ends on the
    // navmesh, no further from the wanted point than it began; all pinned.
    mnavNavmesh* navmesh = WorldNavmesh();
    mnavQuery* query = MakeQuery(8192);
    uint64_t hash = MNAV_HASH_INIT;
    int32_t ends[4] = {0};
    bool sound = true;
    for (int32_t i = 0; i < 64; ++i)
    {
        mnavNearest a;
        CHECK(mnavFindNearest(navmesh, nullptr,
                              (mnavPos3){4.0 + (i % 8) * 7.5, 0.0, 4.0 + (i / 8) * 7.5},
                              (mnavVec3){3.0f, 2.0f, 3.0f}, &a) == mnav_success,
              "start");
        for (int32_t k = 0; k < 16 && a.polygon.slot != 0; ++k)
        {
            mnavPos3 wanted = {a.point.x + s_fan[k][0], 0.0, a.point.z + s_fan[k][1]};
            mnavMove move = {0};
            CHECK(mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, a.point, wanted,
                                       &move) == mnav_success,
                  "moved");
            ends[move.end] += 1;
            mnavNearest n;
            double before = hypot(wanted.x - a.point.x, wanted.z - a.point.z);
            double after = hypot(wanted.x - move.point.x, wanted.z - move.point.z);
            sound = sound && after <= before &&
                    mnavFindNearest(navmesh, nullptr, move.point, (mnavVec3){0.5f, 2.0f, 0.5f},
                                    &n) == mnav_success &&
                    n.polygon.slot != 0 &&
                    hypot(n.point.x - move.point.x, n.point.z - move.point.z) < 1.0e-9;
            hash = mnavHash64(hash, &move.end, (int32_t)sizeof(move.end));
            hash = mnavHash64(hash, &move.point, (int32_t)sizeof(move.point));
            hash =
                mnavHash64(hash, move.polygons, move.polygonCount * (int32_t)sizeof(mnavPolygonId));
        }
    }
    printf("WORLD_MOVES_HASH=%016llx reached=%d wall=%d notLoaded=%d\n", (unsigned long long)hash,
           ends[0], ends[1], ends[2]);
    CHECK(sound, "every move ends on the navmesh, no further from where it was headed");
    CHECK(ends[0] > 0 && ends[1] > 0 && ends[2] > 0, "every kind of end");
    CHECK(hash == WORLD_MOVES_HASH, "the pinned hash");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTableLeftEmpty(void)
{
    // A move takes its nodes out of the context's node table again, so
    // that the next search has nothing to clear: in a table of 64 cells,
    // where keys often share a probe chain, every cell is empty after
    // each move that says so.
    mnavNavmesh* navmesh = WorldNavmesh();
    mnavQuery* query = MakeQuery(32);
    int32_t clean = 0;
    int32_t left = 0;
    for (int32_t i = 0; i < 64; ++i)
    {
        mnavNearest a;
        CHECK(mnavFindNearest(navmesh, nullptr,
                              (mnavPos3){4.0 + (i % 8) * 7.5, 0.0, 4.0 + (i / 8) * 7.5},
                              (mnavVec3){3.0f, 2.0f, 3.0f}, &a) == mnav_success,
              "start");
        for (int32_t k = 0; k < 16 && a.polygon.slot != 0; ++k)
        {
            mnavPos3 wanted = {a.point.x + s_fan[k][0], 0.0, a.point.z + s_fan[k][1]};
            mnavMove move = {0};
            CHECK(mnavMoveAlongSurface(query, navmesh, nullptr, a.polygon, a.point, wanted,
                                       &move) == mnav_success,
                  "moved");
            if (!query->tableClean)
            {
                continue;
            }
            clean += 1;
            for (uint32_t c = 0; c <= query->tableMask; ++c)
            {
                left += query->table[c] != MNAV_NO_NODE ? 1 : 0;
            }
        }
    }
    printf("table: %d moves left it clean, %d cells left\n", clean, left);
    CHECK(clean > 500, "most moves leave it clean");
    CHECK(left == 0, "every cell empty");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestSlicedSearchEnded(void)
{
    // A move takes the node table a sliced search was using, so the
    // search ends: going on with it would read the move's nodes.
    mnavNavmesh* navmesh = Make(s_row, 5, nullptr, 0);
    mnavQuery* query = MakeQuery(64);
    mnavNearest a;
    mnavNearest b;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){6.0, 0.0, 7.5}, (mnavVec3){0.1f, 2.0f, 0.1f},
                          &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){29.0, 0.0, 7.5},
                              (mnavVec3){0.1f, 2.0f, 0.1f}, &b) == mnav_success,
          "the ends");
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
              mnav_success,
          "begun");
    bool ended = false;
    CHECK(mnavContinuePath(query, navmesh, 1, &ended) == mnav_success, "a slice");
    Move(query, navmesh, nullptr, 6.0, 7.5, 9.0, 7.5);
    CHECK(mnavContinuePath(query, navmesh, 1, &ended) == mnav_errorInvalid, "ended by the move");
    mnavPath path;
    CHECK(mnavFinishPath(query, navmesh, &path) == mnav_errorInvalid, "nothing to finish");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestAlongARow();
    TestTileSides();
    TestHeights();
    TestArguments();
    TestWorldMoves();
    TestTableLeftEmpty();
    TestSlicedSearchEnded();
    return s_failures == 0 ? 0 : 1;
}
