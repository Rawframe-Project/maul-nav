// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The work the exact shortest path search does: the nodes it makes, from
// a vertex, trying every way to an end it cannot reach, and across the
// world, pinned so that a change to the search's work is a change seen.
// White-box: the API reports no node count, and a node limit cannot stand
// in for one, since a search short of nodes ends on the A* search's
// answer, which tells an unreachable end on far fewer nodes.

#include "hand_tiles.h"
#include "query.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>
#include <stdio.h>

// The nodes the searches below make.
#define SHORTEST_WORK    2584
#define UNREACHABLE_WORK 86
#define VERTEX_WORK      25
#define VERTEX_RING_WORK 7
#define HOLE_WORK        17

// A navmesh of hand tiles (hand_tiles.h) at (0, 0) and, when given, (1, 0).
static mnavNavmesh* Make(const uint8_t* first, size_t firstSize, const uint8_t* second,
                         size_t secondSize)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavStageTile(navmesh, first, firstSize).result == mnav_success, "first staged");
    if (second != nullptr)
    {
        CHECK(mnavStageTile(navmesh, second, secondSize).result == mnav_success, "second staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavQuery* MakeQuery(int32_t nodes, float length)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    def.limits.pathLength = length;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 1.0f, 0.1f},
                          &n) == mnav_success &&
              n.over,
          "on a polygon");
    return n;
}

static mnavPath Shortest(mnavQuery* query, const mnavNavmesh* navmesh,
                         const mnavQueryFilter* filter, double x0, double z0, double x1, double z1)
{
    mnavNearest a = On(navmesh, x0, z0);
    mnavNearest b = On(navmesh, x1, z1);
    mnavPath path = {0};
    CHECK(mnavFindShortestPath(query, navmesh, filter, a.polygon, a.point, b.polygon, b.point,
                               &path) == mnav_success,
          "searched");
    return path;
}

static void TestFromAVertex(void)
{
    // Four squares meeting at (5, 5): from there straight to each.
    mnavNavmesh* navmesh = Make(s_handFourAtVertex, sizeof(s_handFourAtVertex), nullptr, 0);
    mnavQuery* query = MakeQuery(256, 1000.0f);
    int32_t work = 0;
    for (int32_t k = 0; k < 4; ++k)
    {
        double x = k == 0 || k == 3 ? 1.0 : 9.0;
        double z = k < 2 ? 1.0 : 9.0;
        CHECK(Shortest(query, navmesh, nullptr, 5.0, 5.0, x, z).end == mnav_pathFound, "found");
        work += query->nodeCount;
    }
    printf("VERTEX_WORK=%d\n", work);
    CHECK(work == VERTEX_WORK, "the pinned work from the vertex");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestEveryWayToAnUnreachableEnd(void)
{
    // Nine squares and an island: every way tried.
    mnavNavmesh* navmesh = Make(s_handNineAndIsland, sizeof(s_handNineAndIsland), nullptr, 0);
    mnavQuery* query = MakeQuery(1024, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 6.0, 6.0, 27.5, 7.5);
    printf("UNREACHABLE_WORK=%d\n", query->nodeCount);
    CHECK(path.end == mnav_pathNone && query->nodeCount == UNREACHABLE_WORK,
          "every way tried in the pinned number of nodes");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The nodes an exhaustive search makes from a point to an island it
// cannot reach, on squares beside the island (a tile of hand_tiles.h).
static int32_t Exhaust(const uint8_t* tile, size_t size, double x, double z)
{
    mnavNavmesh* navmesh = Make(tile, size, nullptr, 0);
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, x, z, 27.5, 27.5);
    CHECK(path.end == mnav_pathNone, "no way to the island");
    int32_t made = query->nodeCount;
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return made;
}

static void TestPrunesAsItGoes(void)
{
    // From a vertex four squares share, round it: the ring check stops the
    // way coming back round. Round a hole: the best cost at each corner
    // stops the dearer way to it.
    int32_t vertex = Exhaust(s_handFourAndIsland, sizeof(s_handFourAndIsland), 10.0, 10.0);
    int32_t hole = Exhaust(s_handRingAndIsland, sizeof(s_handRingAndIsland), 6.0, 6.0);
    printf("PRUNED_WORK=%d %d\n", vertex, hole);
    CHECK(vertex == VERTEX_RING_WORK && hole == HOLE_WORK, "the pinned work of trying every way");
}

// The next pair of random points of the world on the navmesh, from the
// generator's state; false when either lies off it.
static bool NextPair(const mnavNavmesh* navmesh, uint32_t* state, mnavNearest* a, mnavNearest* b)
{
    double p[4];
    for (int32_t k = 0; k < 4; ++k)
    {
        *state = *state * 1664525u + 1013904223u;
        p[k] = (double)(*state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
    }
    mnavVec3 box = {3.0f, 2.0f, 3.0f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){p[0], 0.0, p[1]}, box, a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){p[2], 0.0, p[3]}, box, b) ==
                  mnav_success,
          "nearest");
    return a->polygon.slot != 0 && b->polygon.slot != 0;
}

// The baked world's four tiles, committed.
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

static void TestTheWorldsWork(void)
{
    // The pairs test_shortest.c searches across the world.
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    int64_t work = 0;
    uint32_t state = 11;
    for (int32_t i = 0; i < 200; ++i)
    {
        mnavNearest a;
        mnavNearest b;
        mnavPath path;
        if (NextPair(navmesh, &state, &a, &b))
        {
            CHECK(mnavFindShortestPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon,
                                       b.point, &path) == mnav_success,
                  "searched");
            work += query->nodeCount;
        }
    }
    printf("SHORTEST_WORK=%lld\n", (long long)work);
    CHECK(work == SHORTEST_WORK, "the pinned work");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    LoadWorldTiles();
    TestFromAVertex();
    TestEveryWayToAnUnreachableEnd();
    TestPrunesAsItGoes();
    TestTheWorldsWork();
    return s_failures == 0 ? 0 : 1;
}
