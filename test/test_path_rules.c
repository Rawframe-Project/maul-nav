// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The path's rules on hand-built tiles: the funnel's corners (mnav-0005), and
// the search's ties (mnav-0005). Cells are 0.25 m; heights are 0.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

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

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 1.0f, 0.1f},
                          &n) == mnav_success &&
              n.over,
          "on a polygon");
    return n;
}

static mnavPath Walk(mnavQuery* query, const mnavNavmesh* navmesh, double x0, double z0, double x1,
                     double z1)
{
    mnavNearest a = On(navmesh, x0, z0);
    mnavNearest b = On(navmesh, x1, z1);
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound,
          "found");
    return path;
}

static bool At(const mnavPath* path, int32_t i, double x, double z)
{
    return i < path->pointCount && path->points[i].x == x && path->points[i].z == z &&
           path->points[i].y == 0.0;
}

static mnavQuery* MakeQuery(int32_t nodes)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static void TestStraightWhereTheCorridorAllows(void)
{
    // A row of four squares, then an L whose line clears its corner.
    const HandSquare row[4] = {{0, 0, 20, 20, {0}, 0},
                               {20, 0, 40, 20, {0}, 0},
                               {40, 0, 60, 20, {0}, 0},
                               {60, 0, 80, 20, {0}, 0}};
    mnavNavmesh* navmesh = Make(row, 4, nullptr, 0);
    mnavQuery* query = MakeQuery(256);
    mnavPath path = Walk(query, navmesh, 1.0, 2.5, 19.0, 2.5);
    CHECK(path.polygonCount == 4 && path.pointCount == 2 && At(&path, 0, 1.0, 2.5) &&
              At(&path, 1, 19.0, 2.5),
          "a straight line down the row");
    mnavDestroyNavmesh(navmesh);
    const HandSquare ell[3] = {
        {0, 0, 20, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}};
    navmesh = Make(ell, 3, nullptr, 0);
    path = Walk(query, navmesh, 1.0, 1.0, 9.0, 6.0);
    CHECK(path.polygonCount == 3 && path.pointCount == 2, "the line clears the corner");
    // Toward (6, 9) the line would cross the wall at x = 5 above z = 5.
    path = Walk(query, navmesh, 1.0, 1.0, 6.0, 9.0);
    CHECK(path.pointCount == 3 && At(&path, 1, 5.0, 5.0) && At(&path, 2, 6.0, 9.0),
          "one corner, at the L's inner corner");
    // Back the other way, the same corner.
    path = Walk(query, navmesh, 6.0, 9.0, 1.0, 1.0);
    CHECK(path.pointCount == 3 && At(&path, 1, 5.0, 5.0), "the same corner back");
    mnavDestroyNavmesh(navmesh);
    // The L mirrored, so the corner falls on the funnel's other side.
    const HandSquare mirrored[3] = {
        {40, 0, 60, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}};
    navmesh = Make(mirrored, 3, nullptr, 0);
    path = Walk(query, navmesh, 14.0, 1.0, 9.0, 9.0);
    CHECK(path.pointCount == 3 && At(&path, 1, 10.0, 5.0), "the mirrored corner");
    path = Walk(query, navmesh, 9.0, 9.0, 14.0, 1.0);
    CHECK(path.pointCount == 3 && At(&path, 1, 10.0, 5.0), "the mirrored corner back");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestAroundAnObstacle(void)
{
    // A U round the cells (20, 0) to (40, 20): two corners, both ways.
    const HandSquare u[5] = {{0, 0, 20, 20, {0}, 0},
                             {0, 20, 20, 40, {0}, 0},
                             {20, 20, 40, 40, {0}, 0},
                             {40, 20, 60, 40, {0}, 0},
                             {40, 0, 60, 20, {0}, 0}};
    mnavNavmesh* navmesh = Make(u, 5, nullptr, 0);
    mnavQuery* query = MakeQuery(256);
    mnavPath path = Walk(query, navmesh, 2.5, 1.0, 12.5, 1.0);
    CHECK(path.polygonCount == 5 && path.pointCount == 4 && At(&path, 1, 5.0, 5.0) &&
              At(&path, 2, 10.0, 5.0),
          "over the obstacle's two near corners");
    path = Walk(query, navmesh, 12.5, 1.0, 2.5, 1.0);
    CHECK(path.pointCount == 4 && At(&path, 1, 10.0, 5.0) && At(&path, 2, 5.0, 5.0), "and back");
    mnavDestroyNavmesh(navmesh);
    // The U mirrored, the obstacle above: the corners change sides.
    const HandSquare n[5] = {{0, 20, 20, 40, {0}, 0},
                             {0, 0, 20, 20, {0}, 0},
                             {20, 0, 40, 20, {0}, 0},
                             {40, 0, 60, 20, {0}, 0},
                             {40, 20, 60, 40, {0}, 0}};
    navmesh = Make(n, 5, nullptr, 0);
    path = Walk(query, navmesh, 2.5, 9.0, 12.5, 9.0);
    CHECK(path.pointCount == 4 && At(&path, 1, 5.0, 5.0) && At(&path, 2, 10.0, 5.0),
          "under the obstacle's two near corners");
    path = Walk(query, navmesh, 12.5, 9.0, 2.5, 9.0);
    CHECK(path.pointCount == 4 && At(&path, 1, 10.0, 5.0) && At(&path, 2, 5.0, 5.0),
          "and back under");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestAcrossATileSide(void)
{
    // A square on tile (0, 0)'s +X side meets two on tile (1, 0)'s -X side;
    // the second of them reaches past the first's end, so the way bends at
    // the first's corner on the side, (32, 10).
    const HandSquare left[1] = {{100, 0, 128, 40, {0}, 0}};
    const HandSquare right[2] = {{0, 0, 20, 40, {0}, 0}, {0, 40, 20, 80, {0}, 0}};
    mnavNavmesh* navmesh = Make(left, 1, right, 2);
    mnavQuery* query = MakeQuery(256);
    mnavPath path = Walk(query, navmesh, 26.0, 1.0, 33.0, 18.0);
    CHECK(path.polygonCount == 3 && path.pointCount == 3 && At(&path, 1, 32.0, 10.0),
          "the corner where the tiles' polygons part");
    path = Walk(query, navmesh, 33.0, 18.0, 26.0, 1.0);
    CHECK(path.pointCount == 3 && At(&path, 1, 32.0, 10.0), "the same corner back");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestShortOfTheEnd(void)
{
    // Out of nodes in the U, the path ends at the midpoint of the last edge
    // crossed.
    const HandSquare u[5] = {{0, 0, 20, 20, {0}, 0},
                             {0, 20, 20, 40, {0}, 0},
                             {20, 20, 40, 40, {0}, 0},
                             {40, 20, 60, 40, {0}, 0},
                             {40, 0, 60, 20, {0}, 0}};
    mnavNavmesh* navmesh = Make(u, 5, nullptr, 0);
    mnavQuery* query = MakeQuery(3);
    mnavNearest a = On(navmesh, 2.5, 1.0);
    mnavNearest b = On(navmesh, 12.5, 1.0);
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    // Three nodes reach the edge into the third square, at x = 5 from
    // z = 5 to 10; the line to its midpoint clears the first edge.
    CHECK(path.end == mnav_pathOutOfNodes && path.polygonCount == 3 && path.pointCount == 2 &&
              At(&path, 1, 5.0, 7.5),
          "to the middle of the edge into the third square");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The fewest nodes with which a search reaches its end.
static int32_t FewestNodes(const mnavNavmesh* navmesh, double x0, double z0, double x1, double z1)
{
    mnavNearest a = On(navmesh, x0, z0);
    mnavNearest b = On(navmesh, x1, z1);
    for (int32_t nodes = 1; nodes <= 1024; ++nodes)
    {
        mnavQuery* query = MakeQuery(nodes);
        mnavPath path = {0};
        CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &path) == mnav_success,
              "searched");
        mnavDestroyQuery(query);
        if (path.end == mnav_pathFound)
        {
            return nodes;
        }
    }
    return -1;
}

static void TestTiesGoToTheNodeMadeFirst(void)
{
    // A 4 by 4 grid of squares: from corner to corner, many ways cost the
    // same, and the ties decide the work and the corridor.
    HandSquare grid[16];
    for (int32_t i = 0; i < 16; ++i)
    {
        int32_t x = (i % 4) * 20;
        int32_t z = (i / 4) * 20;
        grid[i] = (HandSquare){x, z, x + 20, z + 20, {0}, 0};
    }
    mnavNavmesh* navmesh = Make(grid, 16, nullptr, 0);
    int32_t diagonal = FewestNodes(navmesh, 2.5, 2.5, 17.5, 17.5);
    int32_t across = FewestNodes(navmesh, 2.5, 2.5, 17.5, 2.5);
    mnavQuery* query = MakeQuery(256);
    mnavPath path = Walk(query, navmesh, 2.5, 2.5, 17.5, 17.5);
    uint64_t corridor = mnavHash64(MNAV_HASH_INIT, path.polygons,
                                   path.polygonCount * (int32_t)sizeof(mnavPolygonId));
    printf("TIES diagonal=%d across=%d corridor=%016llx\n", diagonal, across,
           (unsigned long long)corridor);
    CHECK(diagonal == 26 && across == 8 && corridor == 0x564019609892cfd3ull, "the pinned ties");
    CHECK(path.pointCount == 2, "a straight line through the grid");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The hash of the straight paths through grids with holes.
#define LAYOUTS_HASH 0x3fa695aa6c894494ull

static void TestLayoutsArePinned(void)
{
    // Grids of 4 by 4 squares with some left out, from a fixed generator,
    // walked between square centers: corners fall exactly on the lines
    // between other corners, on both sides of the funnel.
    uint32_t state = 9;
    uint64_t hash = MNAV_HASH_INIT;
    int32_t found = 0;
    mnavQuery* query = MakeQuery(256);
    for (int32_t layout = 0; layout < 60; ++layout)
    {
        HandSquare squares[16];
        int32_t count = 0;
        for (int32_t i = 0; i < 16; ++i)
        {
            state = state * 1664525u + 1013904223u;
            if ((state >> 16) % 4u != 0u)
            {
                int32_t x = (i % 4) * 20;
                int32_t z = (i / 4) * 20;
                squares[count++] = (HandSquare){x, z, x + 20, z + 20, {0}, 0};
            }
        }
        mnavNavmesh* navmesh = Make(squares, count, nullptr, 0);
        for (int32_t a = 0; a < count; ++a)
        {
            for (int32_t b = 0; b < count; ++b)
            {
                mnavNearest from =
                    On(navmesh, squares[a].x0 * 0.25 + 2.5, squares[a].z0 * 0.25 + 2.5);
                mnavNearest to =
                    On(navmesh, squares[b].x0 * 0.25 + 2.5, squares[b].z0 * 0.25 + 2.5);
                mnavPath path = {0};
                CHECK(mnavFindPath(query, navmesh, nullptr, from.polygon, from.point, to.polygon,
                                   to.point, &path) == mnav_success,
                      "searched");
                found += path.end == mnav_pathFound ? 1 : 0;
                hash = mnavHash64(hash, path.points, path.pointCount * (int32_t)sizeof(mnavPos3));
            }
        }
        mnavDestroyNavmesh(navmesh);
    }
    printf("LAYOUTS_HASH=%016llx found=%d\n", (unsigned long long)hash, found);
    CHECK(hash == LAYOUTS_HASH, "the pinned hash");
    mnavDestroyQuery(query);
}

int main(void)
{
    TestStraightWhereTheCorridorAllows();
    TestAroundAnObstacle();
    TestAcrossATileSide();
    TestShortOfTheEnd();
    TestTiesGoToTheNodeMadeFirst();
    TestLayoutsArePinned();
    return s_failures == 0 ? 0 : 1;
}
