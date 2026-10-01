// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Paths across off-mesh links (N30). Hand cells are 0.25 m.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

static uint8_t s_bytes[8192];

static mnavNavmesh* Make(const HandSquare* squares, int32_t count)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    size_t size = HandTileBytes(s_bytes, 0, squares, count);
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    return navmesh;
}

static mnavLinkId AddLink(mnavNavmesh* navmesh, double x0, double x1, float cost, mnavLinkKind kind,
                          bool twoWay)
{
    mnavLinkDef def = {{x0, 0.0, 7.5}, {x1, 0.0, 7.5}, 0.5f, cost, kind, twoWay};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "link added");
    return id;
}

static mnavPath Walk(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                     double x0, double z0, double x1, double z1)
{
    mnavNearest a;
    mnavNearest b;
    mnavVec3 box = {0.1f, 1.0f, 0.1f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x0, 0.0, z0}, box, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){x1, 0.0, z1}, box, &b) == mnav_success &&
              a.over && b.over,
          "ends on the navmesh");
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, filter, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    return path;
}

static bool At(const mnavPath* path, int32_t i, double x, double z)
{
    return i < path->pointCount && path->points[i].x == x && path->points[i].z == z;
}

static mnavQuery* MakeQuery(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

// Two squares 5 m apart: (5, 5) to (10, 10) and (15, 5) to (20, 10).
static const HandSquare s_apart[2] = {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 2}};

static void TestJumpingAGap(void)
{
    mnavNavmesh* navmesh = Make(s_apart, 2);
    mnavQuery* query = MakeQuery();
    mnavPath path = Walk(query, navmesh, nullptr, 6.0, 6.0, 19.0, 9.0);
    CHECK(path.end == mnav_pathNone && path.linkCount == 0, "no way without a link");
    mnavLinkId jump = AddLink(navmesh, 9.0, 16.0, 4.0f, mnav_linkJump, false);
    path = Walk(query, navmesh, nullptr, 6.0, 6.0, 19.0, 9.0);
    double walk = sqrt(9.0 + 2.25) + sqrt(9.0 + 2.25);
    CHECK(path.end == mnav_pathFound && path.polygonCount == 2, "across the gap");
    CHECK(path.pointCount == 4 && At(&path, 0, 6.0, 6.0) && At(&path, 1, 9.0, 7.5) &&
              At(&path, 2, 16.0, 7.5) && At(&path, 3, 19.0, 9.0),
          "start, takeoff, landing, end");
    CHECK(path.linkCount == 1 && path.links[0].link.slot == jump.slot &&
              path.links[0].link.generation == jump.generation &&
              path.links[0].kind == mnav_linkJump && path.links[0].point == 1,
          "the jump named at its takeoff point");
    CHECK(fabs(path.cost - (walk + 4.0)) < 1.0e-12 && fabs(path.length - (walk + 7.0)) < 1.0e-12,
          "the link's cost, its span in the length");
    // One way only.
    path = Walk(query, navmesh, nullptr, 19.0, 9.0, 6.0, 6.0);
    CHECK(path.end == mnav_pathNone, "not back over a one-way link");
    // Jumps not allowed, or the landing's area left out: no way.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.kinds &= ~((uint64_t)1 << mnav_linkJump);
    path = Walk(query, navmesh, &filter, 6.0, 6.0, 19.0, 9.0);
    CHECK(path.end == mnav_pathNone, "an agent that cannot jump");
    filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 2);
    path = Walk(query, navmesh, &filter, 6.0, 6.0, 9.5, 9.0);
    CHECK(path.end == mnav_pathFound && path.linkCount == 0, "the near square alone");
    path = Walk(query, navmesh, &filter, 6.0, 6.0, 19.0, 9.0);
    CHECK(path.end == mnav_pathNone, "no landing on a square left out");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestTwoWaysAndDetached(void)
{
    mnavNavmesh* navmesh = Make(s_apart, 2);
    mnavQuery* query = MakeQuery();
    (void)AddLink(navmesh, 9.0, 16.0, 4.0f, mnav_linkLadder, true);
    mnavPath path = Walk(query, navmesh, nullptr, 19.0, 9.0, 6.0, 6.0);
    CHECK(path.end == mnav_pathFound && path.pointCount == 4 && At(&path, 1, 16.0, 7.5) &&
              At(&path, 2, 9.0, 7.5) && path.links[0].kind == mnav_linkLadder,
          "back over a two-way link, its ends swapped");
    mnavDestroyNavmesh(navmesh);
    // A link whose end does not snap is not crossed.
    navmesh = Make(s_apart, 2);
    (void)AddLink(navmesh, 9.0, 12.5, 1.0f, mnav_linkJump, false);
    path = Walk(query, navmesh, nullptr, 6.0, 6.0, 19.0, 9.0);
    CHECK(path.end == mnav_pathNone, "a detached link");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A row of five squares from (5, 5) to (30, 10).
static const HandSquare s_row[5] = {{20, 20, 40, 40, {0}, 0},
                                    {40, 20, 60, 40, {0}, 0},
                                    {60, 20, 80, 40, {0}, 0},
                                    {80, 20, 100, 40, {0}, 0},
                                    {100, 20, 120, 40, {0}, 0}};

// The fewest nodes with which a walk is found.
static int32_t FewestNodes(const mnavNavmesh* navmesh, const mnavQueryFilter* filter, double x0,
                           double z0, double x1, double z1)
{
    for (int32_t nodes = 1; nodes <= 256; ++nodes)
    {
        mnavQueryDef def = mnavDefaultQueryDef();
        def.limits.nodes = nodes;
        mnavQuery* query = nullptr;
        CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
        mnavPath path = Walk(query, navmesh, filter, x0, z0, x1, z1);
        mnavDestroyQuery(query);
        if (path.end == mnav_pathFound)
        {
            return nodes;
        }
    }
    return -1;
}

static void TestCheapTeleportsKeepTheSearchExact(void)
{
    // Walking from 10.5 to 25 costs 14.5. Walking back to 5.5 and taking a
    // teleport to 29.5 at a cost of 1 costs 5 + 1 + 4.5 = 10.5. Scaled by
    // area costs alone, the heuristic would count the 19.5 m left from the
    // takeoff in full and settle for the walk.
    mnavNavmesh* navmesh = Make(s_row, 5);
    mnavQuery* query = MakeQuery();
    (void)AddLink(navmesh, 5.5, 29.5, 1.0f, mnav_linkTeleport, false);
    mnavPath path = Walk(query, navmesh, nullptr, 10.5, 7.5, 25.0, 7.5);
    CHECK(path.end == mnav_pathFound && path.linkCount == 1 &&
              path.links[0].kind == mnav_linkTeleport && fabs(path.cost - 10.5) < 1.0e-12,
          "back to the teleport");
    CHECK(path.pointCount == 4 && At(&path, 1, 5.5, 7.5) && At(&path, 2, 29.5, 7.5),
          "its takeoff and landing");
    // Dearer than the walk, it is left alone.
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(s_row, 5);
    (void)AddLink(navmesh, 5.5, 29.5, 6.0f, mnav_linkTeleport, false);
    path = Walk(query, navmesh, nullptr, 10.5, 7.5, 25.0, 7.5);
    CHECK(path.end == mnav_pathFound && path.linkCount == 0 && path.cost == 14.5 &&
              path.pointCount == 2,
          "the walk");
    // A teleport of cost 0 still leaves an exact search.
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(s_row, 5);
    (void)AddLink(navmesh, 5.5, 29.5, 0.0f, mnav_linkTeleport, false);
    path = Walk(query, navmesh, nullptr, 10.5, 7.5, 25.0, 7.5);
    CHECK(path.end == mnav_pathFound && path.linkCount == 1 && path.cost == 9.5, "for free");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLeftOutKindsDoNotWeakenTheSearch(void)
{
    // Corner to corner across a 4 by 4 grid of squares from (5, 5) to
    // (25, 25): a heuristic scaled near 0 would open nearly every portal.
    // A teleport of cost 0.01 the agent may not use leaves the search as
    // narrow as without it.
    HandSquare grid[16];
    for (int32_t i = 0; i < 16; ++i)
    {
        int32_t x = 20 + (i % 4) * 20;
        int32_t z = 20 + (i / 4) * 20;
        grid[i] = (HandSquare){x, z, x + 20, z + 20, {0}, 0};
    }
    mnavQueryFilter walker = mnavDefaultQueryFilter();
    walker.kinds &= ~((uint64_t)1 << mnav_linkTeleport);
    mnavNavmesh* navmesh = Make(grid, 16);
    int32_t without = FewestNodes(navmesh, &walker, 6.0, 6.0, 24.0, 24.0);
    mnavDestroyNavmesh(navmesh);
    navmesh = Make(grid, 16);
    mnavLinkDef def = {{24.0, 0.0, 6.0}, {6.0, 0.0, 24.0}, 0.5f, 0.01f, mnav_linkTeleport, false};
    mnavLinkId id;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "teleport added");
    int32_t beside = FewestNodes(navmesh, &walker, 6.0, 6.0, 24.0, 24.0);
    printf("FEWEST without=%d beside=%d\n", without, beside);
    CHECK(without > 0 && beside == without, "as few nodes beside the teleport");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestJumpingAGap();
    TestTwoWaysAndDetached();
    TestCheapTeleportsKeepTheSearchExact();
    TestLeftOutKindsDoNotWeakenTheSearch();
    return s_failures == 0 ? 0 : 1;
}
