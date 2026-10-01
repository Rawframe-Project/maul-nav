// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Sliced path searches (mnav-0005): any slicing gives the one search's
// result; a commit makes a search stale; the filter is the search's own.

#include "hand_tile.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

static mnavNavmesh* LoadWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    def.tier = mnav_tierDynamic;
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

static uint64_t HashPath(const mnavPath* path)
{
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, &path->end, (int32_t)sizeof(path->end));
    hash = mnavHash64(hash, &path->cost, (int32_t)sizeof(path->cost));
    hash = mnavHash64(hash, &path->length, (int32_t)sizeof(path->length));
    hash = mnavHash64(hash, path->polygons, path->polygonCount * (int32_t)sizeof(mnavPolygonId));
    return mnavHash64(hash, path->points, path->pointCount * (int32_t)sizeof(mnavPos3));
}

// A search in slices of a number of nodes, with the slices it took.
static uint64_t Sliced(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest a, mnavNearest b,
                       int32_t budget, int32_t* slices)
{
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
              mnav_success,
          "begun");
    bool ended = false;
    *slices = 0;
    while (!ended)
    {
        CHECK(mnavContinuePath(query, navmesh, budget, &ended) == mnav_success, "continued");
        *slices += 1;
    }
    mnavPath path;
    CHECK(mnavFinishPath(query, navmesh, &path) == mnav_success, "finished");
    return HashPath(&path);
}

static void TestSlicesChangeNothing(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavQuery* small = MakeQuery(12);
    uint32_t state = 5;
    int32_t searches = 0;
    int32_t manySlices = 0;
    for (int32_t i = 0; i < 200; ++i)
    {
        double p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (double)(state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
        }
        mnavNearest a;
        mnavNearest b;
        mnavVec3 box = {3.0f, 2.0f, 3.0f};
        CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){p[0], 0.0, p[1]}, box, &a) ==
                      mnav_success &&
                  mnavFindNearest(navmesh, nullptr, (mnavPos3){p[2], 0.0, p[3]}, box, &b) ==
                      mnav_success,
              "nearest");
        if (a.polygon.slot == 0 || b.polygon.slot == 0)
        {
            continue;
        }
        searches += 1;
        mnavPath whole;
        CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &whole) == mnav_success,
              "whole");
        uint64_t expected = HashPath(&whole);
        const int32_t budgets[3] = {1, 3, 50};
        bool same = true;
        for (int32_t k = 0; k < 3; ++k)
        {
            int32_t slices = 0;
            same = same && Sliced(query, navmesh, a, b, budgets[k], &slices) == expected;
            manySlices += k == 0 && slices > 10 ? 1 : 0;
        }
        CHECK(same, "the same result in slices of 1, 3 and 50 nodes");
        // Out of nodes, too, sliced or not.
        CHECK(mnavFindPath(small, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &whole) == mnav_success,
              "small whole");
        expected = HashPath(&whole);
        int32_t slices = 0;
        CHECK(Sliced(small, navmesh, a, b, 2, &slices) == expected, "small, in slices");
    }
    // 48 of the 200 take more than ten slices of one node.
    CHECK(searches == 200 && manySlices >= 40, "searches long enough to slice");
    mnavDestroyQuery(small);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestCommitsMakeSearchesStale(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavNavmesh* other = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavNearest a;
    mnavNearest b;
    mnavVec3 box = {2.0f, 2.0f, 2.0f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){2.0, 0.0, 2.0}, box, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){62.0, 0.0, 62.0}, box, &b) ==
                  mnav_success,
          "ends");
    bool ended = false;
    mnavPath path;
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
                  mnav_success &&
              mnavContinuePath(query, navmesh, 3, &ended) == mnav_success && !ended,
          "under way");
    CHECK(mnavContinuePath(query, other, 3, &ended) == mnav_errorStale &&
              mnavFinishPath(query, other, &path) == mnav_errorStale,
          "not on another navmesh");
    // Staging changes nothing; committing does.
    CHECK(mnavStageTile(navmesh, s_tiles[3], s_sizes[3]).result == mnav_success &&
              mnavContinuePath(query, navmesh, 3, &ended) == mnav_success,
          "staging alone");
    CHECK(mnavCommit(navmesh) == mnav_success &&
              mnavContinuePath(query, navmesh, 3, &ended) == mnav_errorStale &&
              mnavFinishPath(query, navmesh, &path) == mnav_errorStale,
          "stale after a commit");
    // A commit with nothing staged changes nothing.
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){62.0, 0.0, 62.0}, box, &b) == mnav_success &&
              mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
                  mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavContinuePath(query, navmesh, 3, &ended) == mnav_success,
          "an empty commit");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(other);
    mnavDestroyNavmesh(navmesh);
}

static void TestFinishingEarly(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192);
    mnavNearest a;
    mnavNearest b;
    mnavVec3 box = {2.0f, 2.0f, 2.0f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){2.0, 0.0, 2.0}, box, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){62.0, 0.0, 62.0}, box, &b) ==
                  mnav_success,
          "ends");
    bool ended = true;
    mnavPath path;
    CHECK(mnavContinuePath(query, navmesh, 1, &ended) == mnav_errorInvalid &&
              mnavFinishPath(query, navmesh, &path) == mnav_errorInvalid,
          "nothing begun");
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point) ==
                  mnav_success &&
              mnavContinuePath(query, navmesh, 0, &ended) == mnav_errorInvalid &&
              mnavContinuePath(query, navmesh, 1, nullptr) == mnav_errorInvalid &&
              mnavContinuePath(query, navmesh, 4, &ended) == mnav_success && !ended,
          "four nodes closed");
    CHECK(mnavFinishPath(query, navmesh, &path) == mnav_success &&
              path.end == mnav_pathUnfinished && path.polygonCount >= 1 &&
              path.polygons[0].polygon == a.polygon.polygon && path.pointCount >= 2,
          "unfinished, toward the end");
    CHECK(mnavFinishPath(query, navmesh, &path) == mnav_errorInvalid &&
              mnavContinuePath(query, navmesh, 1, &ended) == mnav_errorInvalid,
          "finished once");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTheFilterIsTheSearchsOwn(void)
{
    // A bridge of area 2 between two squares, and a way round it.
    const HandSquare squares[6] = {{20, 20, 40, 40, {0}, 0}, {40, 20, 60, 40, {0}, 2},
                                   {60, 20, 80, 40, {0}, 0}, {20, 40, 40, 60, {0}, 0},
                                   {40, 40, 60, 60, {0}, 0}, {60, 40, 80, 60, {0}, 0}};
    static uint8_t bytes[8192];
    size_t size = HandTileBytes(bytes, 0, squares, 6);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(256);
    mnavNearest a;
    mnavNearest b;
    mnavVec3 box = {0.1f, 1.0f, 0.1f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){7.5, 0.0, 7.5}, box, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){17.5, 0.0, 7.5}, box, &b) ==
                  mnav_success,
          "ends");
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavBeginPath(query, navmesh, &filter, a.polygon, a.point, b.polygon, b.point) ==
              mnav_success,
          "begun without the bridge");
    filter = mnavDefaultQueryFilter();
    bool ended = false;
    mnavPath path;
    CHECK(mnavContinuePath(query, navmesh, 1000, &ended) == mnav_success && ended &&
              mnavFinishPath(query, navmesh, &path) == mnav_success && path.polygonCount == 5,
          "still round the bridge");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeWorld();
    TestSlicesChangeNothing();
    TestCommitsMakeSearchesStale();
    TestFinishingEarly();
    TestTheFilterIsTheSearchsOwn();
    return s_failures == 0 ? 0 : 1;
}
