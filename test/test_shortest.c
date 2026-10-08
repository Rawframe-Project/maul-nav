// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The exact shortest-path search (mnav-0005): its turning points on
// hand-built tiles, where the shortest way is known, then the baked world
// beside the A* search. Hand cells are 0.25 m; heights are 0.

#include "hand_tile.h"
#include "navmesh.h"
#include "query.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

// The hash of the world's searches, the same on every platform.
#define SHORTEST_HASH 0x1dc7c3c72066f1feull
// The nodes those searches make, so that a change to the search's work
// is a change seen.
#define SHORTEST_WORK    2584
#define UNREACHABLE_WORK 86
#define VERTEX_WORK      25
#define VERTEX_RING_WORK 7
#define HOLE_WORK        17

static uint8_t s_bytes[2][8192];

// A navmesh of hand tiles at (0, 0) and, when given, (1, 0).
static mnavNavmesh* Make(const HandSquare* first, int32_t firstCount, const HandSquare* second,
                         int32_t secondCount)
{
    mnavBakeDef def = mnavDefaultBakeDef();
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

static bool At(const mnavPath* path, int32_t i, double x, double z)
{
    return i < path->pointCount && path->points[i].x == x && path->points[i].z == z &&
           path->points[i].y == 0.0;
}

static double Flat(mnavPos3 a, mnavPos3 b)
{
    return hypot(a.x - b.x, a.z - b.z);
}

static double Ground(const mnavPath* path)
{
    double length = 0.0;
    for (int32_t i = 1; i < path->pointCount; ++i)
    {
        length += Flat(path->points[i - 1], path->points[i]);
    }
    return length;
}

// An L of three squares: (0, 0) to (5, 5), east of it to (10, 5), north
// of that to (10, 10).
static const HandSquare s_ell[3] = {
    {0, 0, 20, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}};

static void TestTurnsAtTheCorner(void)
{
    mnavNavmesh* navmesh = Make(s_ell, 3, nullptr, 0);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 4.5, 7.5, 9.5);
    double length = sqrt(6.5) + sqrt(26.5);
    CHECK(path.end == mnav_pathFound && path.pointCount == 3 && At(&path, 0, 2.5, 4.5) &&
              At(&path, 1, 5.0, 5.0) && At(&path, 2, 7.5, 9.5),
          "start, the inner corner, end");
    CHECK(path.polygonCount == 3 && path.polygons[0].polygon == 0 &&
              path.polygons[1].polygon == 1 && path.polygons[2].polygon == 2,
          "the three squares in order");
    CHECK(fabs(path.length - length) < 1e-12 && fabs(path.cost - length) < 1e-12,
          "the length of the two legs, at a cost of 1");
    CHECK(path.linkCount == 0, "no link");
    // Straight within the corridor where nothing is in the way.
    path = Shortest(query, navmesh, nullptr, 1.0, 1.0, 9.0, 4.0);
    CHECK(path.end == mnav_pathFound && path.pointCount == 2 && At(&path, 1, 9.0, 4.0),
          "a straight line");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestEndsOnTheStartOrTheCorner(void)
{
    mnavNavmesh* navmesh = Make(s_ell, 3, nullptr, 0);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 4.5, 2.5, 4.5);
    CHECK(path.end == mnav_pathFound && path.pointCount == 1 && path.length == 0.0,
          "to the start itself");
    path = Shortest(query, navmesh, nullptr, 7.5, 9.5, 5.0, 5.0);
    CHECK(path.end == mnav_pathFound && path.pointCount == 2 && At(&path, 1, 5.0, 5.0),
          "to the corner itself");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTurnsAtATileBorder(void)
{
    // The L again, its corner on the border between two tiles.
    const HandSquare west[1] = {{108, 0, 128, 20, {0}, 0}};
    const HandSquare east[2] = {{0, 0, 20, 20, {0}, 0}, {0, 20, 20, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(west, 1, east, 2);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 29.5, 4.5, 34.5, 9.5);
    CHECK(path.end == mnav_pathFound && path.pointCount == 3 && At(&path, 1, 32.0, 5.0),
          "turns at the corner on the border");
    CHECK(path.polygonCount == 3 && path.polygons[0].slot != path.polygons[1].slot,
          "across the border");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTakesTheShorterSide(void)
{
    // Eight squares about a hole from (5, 5) to (10, 10).
    const HandSquare ring[8] = {{0, 0, 20, 20, {0}, 0},   {20, 0, 40, 20, {0}, 0},
                                {40, 0, 60, 20, {0}, 0},  {40, 20, 60, 40, {0}, 0},
                                {40, 40, 60, 60, {0}, 0}, {20, 40, 40, 60, {0}, 0},
                                {0, 40, 20, 60, {0}, 0},  {0, 20, 20, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(ring, 8, nullptr, 0);
    mnavQuery* query = MakeQuery(256, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 6.0, 12.5, 6.5);
    CHECK(path.end == mnav_pathFound && path.pointCount == 4 && At(&path, 1, 5.0, 5.0) &&
              At(&path, 2, 10.0, 5.0),
          "round the hole's nearer side");
    CHECK(fabs(path.length - (sqrt(7.25) + 5.0 + sqrt(8.5))) < 1e-12, "its length");
    path = Shortest(query, navmesh, nullptr, 2.5, 9.0, 12.5, 8.5);
    CHECK(path.end == mnav_pathFound && At(&path, 1, 5.0, 10.0) && At(&path, 2, 10.0, 10.0),
          "and the other side when it is nearer");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestStartsOnAVertex(void)
{
    // Four squares meeting at (5, 5): a start there sees all of them.
    const HandSquare four[4] = {{0, 0, 20, 20, {0}, 0},
                                {20, 0, 40, 20, {0}, 0},
                                {20, 20, 40, 40, {0}, 0},
                                {0, 20, 20, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(four, 4, nullptr, 0);
    mnavQuery* query = MakeQuery(256, 1000.0f);
    int32_t work = 0;
    for (int32_t k = 0; k < 4; ++k)
    {
        double x = k == 0 || k == 3 ? 1.0 : 9.0;
        double z = k < 2 ? 1.0 : 9.0;
        mnavPath path = Shortest(query, navmesh, nullptr, 5.0, 5.0, x, z);
        CHECK(path.end == mnav_pathFound && path.pointCount == 2 && At(&path, 0, 5.0, 5.0) &&
                  At(&path, 1, x, z),
              "straight to each square");
        work += query->nodeCount;
        path = Shortest(query, navmesh, nullptr, x, z, 5.0, 5.0);
        CHECK(path.end == mnav_pathFound && path.pointCount == 2, "and back to the vertex");
    }
    printf("VERTEX_WORK=%d\n", work);
    CHECK(work == VERTEX_WORK, "the pinned work from the vertex");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestGoesStraightThroughVertices(void)
{
    // Nine squares, (0, 0) to (15, 15): the diagonal passes through the
    // vertices at (5, 5) and (10, 10), where four squares meet and no wall.
    HandSquare grid[9];
    for (int32_t k = 0; k < 9; ++k)
    {
        int32_t x0 = 20 * (k % 3);
        int32_t z0 = 20 * (k / 3);
        grid[k] = (HandSquare){x0, z0, x0 + 20, z0 + 20, {0}, 0};
    }
    mnavNavmesh* navmesh = Make(grid, 9, nullptr, 0);
    mnavQuery* query = MakeQuery(1024, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 2.5, 12.5, 12.5);
    CHECK(path.end == mnav_pathFound && fabs(path.length - sqrt(200.0)) < 1e-12,
          "the diagonal's length");
    path = Shortest(query, navmesh, nullptr, 1.0, 14.0, 14.0, 1.0);
    CHECK(path.end == mnav_pathFound && fabs(path.length - sqrt(338.0)) < 1e-12,
          "the other diagonal, through (5, 10) and (10, 5)");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The links the way through the L crosses with a link from (2.5, 4) to
// (7.5, 6) at a cost, added for the search alone.
static int32_t LinksTaken(mnavQuery* query, mnavNavmesh* navmesh, float cost)
{
    mnavLinkDef def = {{2.5, 0.0, 4.0}, {7.5, 0.0, 6.0}, 0.5f, cost, mnav_linkJump, false, 0.0f};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "link added");
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 4.5, 7.5, 9.5);
    CHECK(path.end == mnav_pathFound, "found");
    int32_t taken = path.linkCount;
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "link removed");
    return taken;
}

static void TestWeighsALinkAgainstTheWalk(void)
{
    // The L, a square far off, and a long cheap link from it into the L
    // that lowers the heuristic's scale to under 0.03 a meter, so that a
    // link in the L is tried before the walk ends. From (2.5, 4) to
    // (7.5, 6), its way costs 4 plus its own; the walk costs 7.98: at 5 the
    // way walks round the corner, at 1 it jumps.
    HandSquare squares[4] = {s_ell[0], s_ell[1], s_ell[2], {100, 100, 120, 120, {0}, 0}};
    mnavNavmesh* navmesh = Make(squares, 4, nullptr, 0);
    mnavLinkDef far = {{27.5, 0.0, 27.5}, {1.0, 0.0, 1.0}, 0.5f, 1.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId farId = {0, 0};
    CHECK(mnavStageLink(navmesh, &far, &farId) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "far link added");
    mnavQuery* query = MakeQuery(256, 1000.0f);
    CHECK(LinksTaken(query, navmesh, 5.0f) == 0, "a dearer link left aside");
    CHECK(LinksTaken(query, navmesh, 1.0f) == 1, "a cheaper link taken");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTellsAnUnreachableEndOnFewNodes(void)
{
    // Nine squares and an island: on 32 nodes the exact search, which needs
    // 86 to try every way, runs out; the A* search, on 25, tells there is
    // none.
    HandSquare squares[10];
    for (int32_t k = 0; k < 9; ++k)
    {
        int32_t x0 = 20 + 20 * (k % 3);
        int32_t z0 = 20 + 20 * (k / 3);
        squares[k] = (HandSquare){x0, z0, x0 + 20, z0 + 20, {0}, 0};
    }
    squares[9] = (HandSquare){100, 20, 120, 40, {0}, 0};
    mnavNavmesh* navmesh = Make(squares, 10, nullptr, 0);
    mnavQuery* query = MakeQuery(32, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 6.0, 6.0, 27.5, 7.5);
    CHECK(path.end == mnav_pathNone, "no way, told on 32 nodes");
    mnavDestroyQuery(query);
    query = MakeQuery(1024, 1000.0f);
    path = Shortest(query, navmesh, nullptr, 6.0, 6.0, 27.5, 7.5);
    printf("UNREACHABLE_WORK=%d\n", query->nodeCount);
    CHECK(path.end == mnav_pathNone && query->nodeCount == UNREACHABLE_WORK,
          "every way tried in the pinned number of nodes");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestRoundsAtAVertexOnTheConesSide(void)
{
    // Found by fuzz_shortest: after the link, the way runs on a line at 45
    // degrees through the vertices (22, 11) and (27, 16), the end on it;
    // rounded, the second vertex fell short of being a corner.
    const HandSquare west[13] = {
        {68, 24, 88, 44, {0}, 0},   {108, 24, 128, 44, {0}, 0}, {48, 44, 68, 64, {0}, 0},
        {68, 44, 88, 64, {0}, 0},   {88, 44, 108, 64, {0}, 0},  {108, 44, 128, 64, {0}, 0},
        {48, 64, 68, 84, {0}, 2},   {68, 64, 88, 84, {0}, 2},   {108, 64, 128, 84, {0}, 0},
        {48, 84, 68, 104, {0}, 2},  {68, 84, 88, 104, {0}, 2},  {88, 84, 108, 104, {0}, 0},
        {108, 84, 128, 104, {0}, 0}};
    const HandSquare east[4] = {{0, 24, 20, 44, {0}, 0},
                                {40, 44, 60, 64, {0}, 0},
                                {60, 44, 80, 64, {0}, 0},
                                {40, 64, 60, 84, {0}, 2}};
    mnavNavmesh* navmesh = Make(west, 13, east, 4);
    mnavLinkDef def = {{43.8, 0.0, 18.35}, {19.4, 0.0, 8.4}, 0.5f, 0.5f, mnav_linkJump, true, 0.0f};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "link added");
    mnavQuery* query = MakeQuery(4096, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 44.4, 17.9, 29.4, 18.4);
    double best = hypot(0.6, 0.45) + 0.5 + hypot(10.0, 10.0);
    CHECK(path.end == mnav_pathFound && fabs(path.cost - best) < 1e-9,
          "on through the vertex the end lies beyond, at the least cost");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The nodes an exhaustive search makes from a point to an island it
// cannot reach, on squares beside the island.
static int32_t Exhaust(const HandSquare* squares, int32_t count, double x, double z)
{
    HandSquare all[HAND_SQUARES];
    for (int32_t k = 0; k < count; ++k)
    {
        all[k] = squares[k];
    }
    all[count] = (HandSquare){100, 100, 120, 120, {0}, 0};
    mnavNavmesh* navmesh = Make(all, count + 1, nullptr, 0);
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
    HandSquare four[4];
    HandSquare ring[8];
    for (int32_t k = 0; k < 9; ++k)
    {
        int32_t x0 = 20 + 20 * (k % 3);
        int32_t z0 = 20 + 20 * (k / 3);
        HandSquare q = {x0, z0, x0 + 20, z0 + 20, {0}, 0};
        if (k < 4)
        {
            four[k] = (HandSquare){
                20 + 20 * (k % 2), 20 + 20 * (k / 2), 40 + 20 * (k % 2), 40 + 20 * (k / 2), {0}, 0};
        }
        if (k != 4)
        {
            ring[k < 4 ? k : k - 1] = q;
        }
    }
    int32_t vertex = Exhaust(four, 4, 10.0, 10.0);
    int32_t hole = Exhaust(ring, 8, 6.0, 6.0);
    printf("PRUNED_WORK=%d %d\n", vertex, hole);
    CHECK(vertex == VERTEX_RING_WORK && hole == HOLE_WORK, "the pinned work of trying every way");
}

static void TestLimitsEndTheSearch(void)
{
    mnavNavmesh* navmesh = Make(s_ell, 3, nullptr, 0);
    mnavQuery* query = MakeQuery(2, 1000.0f);
    mnavPath path = Shortest(query, navmesh, nullptr, 2.5, 4.5, 7.5, 9.5);
    CHECK(path.end == mnav_pathOutOfNodes && path.polygonCount >= 1 &&
              path.polygons[0].polygon == 0 && At(&path, 0, 2.5, 4.5),
          "out of nodes, from the start on");
    mnavDestroyQuery(query);
    query = MakeQuery(64, 4.0f);
    path = Shortest(query, navmesh, nullptr, 2.5, 4.5, 7.5, 9.5);
    CHECK(path.end == mnav_pathTooLong, "too long for the limit");
    mnavDestroyQuery(query);
    // Two squares apart, clear of the tile's sides.
    const HandSquare apart[2] = {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 0}};
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(apart, 2, nullptr, 0);
    query = MakeQuery(64, 1000.0f);
    path = Shortest(query, navmesh, nullptr, 7.5, 7.5, 17.5, 7.5);
    CHECK(path.end == mnav_pathNone && path.polygonCount == 1, "no way across");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestCostsAndRefusals(void)
{
    mnavNavmesh* navmesh = Make(s_ell, 3, nullptr, 0);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavNearest a = On(navmesh, 2.5, 4.5);
    mnavNearest b = On(navmesh, 7.5, 9.5);
    mnavPath path;
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    for (int32_t area = 0; area < MNAV_AREA_TYPES; ++area)
    {
        filter.costs[area] = 2.0f;
    }
    CHECK(mnavFindShortestPath(query, navmesh, &filter, a.polygon, a.point, b.polygon, b.point,
                               &path) == mnav_success &&
              path.end == mnav_pathFound &&
              fabs(path.cost - 2.0 * (sqrt(6.5) + sqrt(26.5))) < 1e-12,
          "every step at the one cost");
    filter.costs[3] = 5.0f;
    filter.areas |= (uint64_t)1 << 3;
    CHECK(mnavFindShortestPath(query, navmesh, &filter, a.polygon, a.point, b.polygon, b.point,
                               &path) == mnav_errorInvalid,
          "included areas of two costs refused");
    filter.areas &= ~((uint64_t)1 << 3);
    CHECK(mnavFindShortestPath(query, navmesh, &filter, a.polygon, a.point, b.polygon, b.point,
                               &path) == mnav_success,
          "a dearer area left out is no matter");
    CHECK(mnavFindShortestPath(nullptr, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                               &path) == mnav_errorInvalid &&
              mnavFindShortestPath(query, nullptr, nullptr, a.polygon, a.point, b.polygon, b.point,
                                   &path) == mnav_errorInvalid &&
              mnavFindShortestPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                                   nullptr) == mnav_errorInvalid,
          "NULL refused");
    mnavPos3 bad = {(double)NAN, 0.0, 1.0};
    CHECK(mnavFindShortestPath(query, navmesh, nullptr, a.polygon, bad, b.polygon, b.point,
                               &path) == mnav_errorInvalid &&
              mnavFindShortestPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, bad,
                                   &path) == mnav_errorInvalid,
          "a point not finite refused");
    mnavPolygonId never = {9, 0, 0};
    CHECK(mnavFindShortestPath(query, navmesh, nullptr, never, a.point, b.polygon, b.point,
                               &path) == mnav_errorInvalid,
          "a polygon that never existed refused");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestCrossesLinks(void)
{
    // Two squares 5 m apart, (5, 5) to (10, 10) and (15, 5) to (20, 10),
    // and a jump from (9, 7.5) to (16, 7.5).
    const HandSquare apart[2] = {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 0}};
    mnavNavmesh* navmesh = Make(apart, 2, nullptr, 0);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavLinkDef def = {{9.0, 0.0, 7.5}, {16.0, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId jump = {0, 0};
    CHECK(mnavStageLink(navmesh, &def, &jump) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "link added");
    mnavPath path = Shortest(query, navmesh, nullptr, 6.0, 6.0, 19.0, 9.0);
    double walk = sqrt(9.0 + 2.25) + sqrt(9.0 + 2.25);
    CHECK(path.end == mnav_pathFound && path.pointCount == 4 && At(&path, 0, 6.0, 6.0) &&
              At(&path, 1, 9.0, 7.5) && At(&path, 2, 16.0, 7.5) && At(&path, 3, 19.0, 9.0),
          "start, takeoff, landing, end");
    CHECK(path.polygonCount == 2 && path.linkCount == 1 && path.links[0].link.slot == jump.slot &&
              path.links[0].kind == mnav_linkJump && path.links[0].point == 1,
          "the jump named at its takeoff point");
    CHECK(fabs(path.cost - (walk + 4.0)) < 1e-12 && fabs(path.length - (walk + 7.0)) < 1e-12,
          "the link's cost, its span in the length");
    path = Shortest(query, navmesh, nullptr, 19.0, 9.0, 6.0, 6.0);
    CHECK(path.end == mnav_pathNone, "not back over a one-way link");
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.kinds &= ~((uint64_t)1 << mnav_linkJump);
    path = Shortest(query, navmesh, &filter, 6.0, 6.0, 19.0, 9.0);
    CHECK(path.end == mnav_pathNone, "an agent that cannot jump");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Whether a path stays on the navmesh, sampled every 0.25 m, to within a
// nanometer on the ground.
static bool OnTheNavmesh(const mnavNavmesh* navmesh, const mnavPath* path)
{
    for (int32_t i = 0; i + 1 < path->pointCount; ++i)
    {
        mnavPos3 a = path->points[i];
        mnavPos3 b = path->points[i + 1];
        int32_t steps = (int32_t)(Flat(a, b) / 0.25) + 1;
        for (int32_t k = 0; k <= steps && path->links == nullptr; ++k)
        {
            double t = (double)k / steps;
            mnavPos3 p = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
            mnavNearest n;
            if (mnavFindNearest(navmesh, nullptr, p, (mnavVec3){0.5f, 1.0f, 0.5f}, &n) !=
                    mnav_success ||
                n.polygon.slot == 0 || hypot(p.x - n.point.x, p.z - n.point.z) > 1.0e-9)
            {
                return false;
            }
        }
    }
    return true;
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

// Searches a pair both ways, checks the exact search against the A* one
// and adds its result to the hash; true when its path is shorter.
static bool Compare(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest a, mnavNearest b,
                    uint64_t* hash, int64_t* work)
{
    mnavPath plain;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &plain) ==
              mnav_success,
          "A* searched");
    double plainGround = Ground(&plain);
    mnavPathEnd plainEnd = plain.end;
    mnavPath exact;
    CHECK(mnavFindShortestPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                               &exact) == mnav_success,
          "searched");
    *work += query->nodeCount;
    CHECK(exact.end == plainEnd, "ends as the A* search does");
    double ground = Ground(&exact);
    bool found = exact.end == mnav_pathFound;
    CHECK(!found || ground <= plainGround + 1e-9, "never longer than the A* search's path");
    CHECK(!found || OnTheNavmesh(navmesh, &exact), "on the navmesh");
    *hash = mnavHash64(*hash, &exact.end, (int32_t)sizeof(exact.end));
    *hash = mnavHash64(*hash, exact.points, exact.pointCount * (int32_t)sizeof(mnavPos3));
    *hash = mnavHash64(*hash, exact.polygons, exact.polygonCount * (int32_t)sizeof(mnavPolygonId));
    *hash = mnavHash64(*hash, &exact.cost, (int32_t)sizeof(exact.cost));
    return found && ground < plainGround - 1e-6;
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

static void TestTheWorldBesideTheAStarSearch(void)
{
    mnavNavmesh* navmesh = LoadWorld();
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    uint64_t hash = MNAV_HASH_INIT;
    int32_t shorter = 0;
    int64_t work = 0;
    uint32_t state = 11;
    for (int32_t i = 0; i < 200; ++i)
    {
        mnavNearest a;
        mnavNearest b;
        if (NextPair(navmesh, &state, &a, &b))
        {
            shorter += Compare(query, navmesh, a, b, &hash, &work) ? 1 : 0;
        }
    }
    printf("SHORTEST_HASH=%016llx shorter=%d work=%lld\n", (unsigned long long)hash, shorter,
           (long long)work);
    CHECK(shorter > 0, "some shorter than the A* search's");
    CHECK(hash == SHORTEST_HASH, "the pinned hash");
    CHECK(work == SHORTEST_WORK, "the pinned work");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeWorld();
    TestTurnsAtTheCorner();
    TestEndsOnTheStartOrTheCorner();
    TestTurnsAtATileBorder();
    TestTakesTheShorterSide();
    TestStartsOnAVertex();
    TestGoesStraightThroughVertices();
    TestWeighsALinkAgainstTheWalk();
    TestTellsAnUnreachableEndOnFewNodes();
    TestRoundsAtAVertexOnTheConesSide();
    TestPrunesAsItGoes();
    TestLimitsEndTheSearch();
    TestCostsAndRefusals();
    TestCrossesLinks();
    TestTheWorldBesideTheAStarSearch();
    return s_failures == 0 ? 0 : 1;
}
