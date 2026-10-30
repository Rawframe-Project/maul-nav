// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh links: staging, removal, snapping at each commit, and
// the ways across them the search follows. Hand cells are 0.25 m.

#include "counting_allocator.h"
#include "hand_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// Tile (0, 0) holds two squares apart (s_handApartPair): (5, 5) to
// (10, 10) and (15, 5) to (20, 10); tile (1, 0) one (s_handSquareEast):
// (37, 5) to (42, 10).
static mnavNavmesh* Make(mnavNavmeshDef def)
{
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handApartPair, sizeof(s_handApartPair)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "tile (0, 0)");
    return navmesh;
}

static mnavLinkDef Link(double x0, double z0, double x1, double z1, float radius)
{
    return (mnavLinkDef){{x0, 0.0, z0}, {x1, 0.0, z1}, radius, 4.0f, mnav_linkJump, false, 0.0f};
}

static mnavLinkId Stage(mnavNavmesh* navmesh, mnavLinkDef def)
{
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && id.slot != 0, "staged");
    return id;
}

static mnavLinkState Get(const mnavNavmesh* navmesh, mnavLinkId id)
{
    mnavLinkState state;
    CHECK(mnavLink_GetState(navmesh, id, &state) == mnav_success, "read");
    return state;
}

static bool SameLink(mnavLinkId a, mnavLinkId b)
{
    return a.slot == b.slot && a.generation == b.generation;
}

// The path from (x0, z0) to (x1, z1), both on the squares.
static mnavPath Across(mnavQuery* query, const mnavNavmesh* navmesh, double x0, double z0,
                       double x1, double z1)
{
    mnavNearest a;
    mnavNearest b;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x0, 0.0, z0}, box, nullptr, &a) ==
                  mnav_success &&
              mnavNavmesh_FindNearest(navmesh, (mnavPos3){x1, 0.0, z1}, box, nullptr, &b) ==
                  mnav_success &&
              a.polygon.slot != 0 && b.polygon.slot != 0,
          "both ends on a square");
    mnavPath path;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success,
          "searched");
    return path;
}

static mnavQuery* MakeQuery(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a query");
    return query;
}

static void TestLinksAttachAtTheCommit(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultNavmeshDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    mnavLinkState state;
    CHECK(mnavLink_GetState(navmesh, id, &state) == mnav_errorState && !state.attached,
          "not before the commit");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    state = Get(navmesh, id);
    CHECK(state.attached && state.startPolygon.polygon == 0 && state.endPolygon.polygon == 1 &&
              state.start.x == 7.5 && state.start.z == 7.5 && state.end.x == 17.5 &&
              state.start.y == 0.0,
          "both ends on their squares");
    // One way: a path crosses it from the first square, and none comes
    // back from the second.
    mnavQuery* query = MakeQuery();
    mnavPath there = Across(query, navmesh, 7.5, 7.5, 17.5, 7.5);
    CHECK(there.end == mnav_pathFound && there.linkCount == 1 && SameLink(there.links[0].link, id),
          "across from the first square");
    mnavPath back = Across(query, navmesh, 17.5, 7.5, 7.5, 7.5);
    CHECK(back.end == mnav_pathNone && back.linkCount == 0, "not back from the second");
    // An end 0.4 m off a square's edge snaps onto it; 0.6 m off does not.
    mnavLinkId near = Stage(navmesh, Link(7.5, 7.5, 14.6, 7.5, 0.5f));
    mnavLinkId far = Stage(navmesh, Link(7.5, 7.5, 14.4, 7.5, 0.5f));
    mnavLinkDef twoWay = Link(17.5, 9.0, 7.5, 9.0, 0.0f);
    twoWay.twoWay = true;
    mnavLinkId both = Stage(navmesh, twoWay);
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    state = Get(navmesh, near);
    CHECK(state.attached && state.end.x == 15.0 && state.end.z == 7.5, "snapped to the edge");
    CHECK(!Get(navmesh, far).attached, "too far to snap");
    // Off the square's corner (15, 5) by 0.4 m on both axes: within the
    // search box, but 0.57 m away, past the 0.5 m radius.
    mnavLinkId corner = Stage(navmesh, Link(7.5, 7.5, 14.6, 4.6, 0.5f));
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success && !Get(navmesh, corner).attached,
          "the radius is round");
    CHECK(mnavStageLinkRemoval(navmesh, corner) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "removed again");
    // The two-way link starts on the second square: it leaves the second
    // forward, from its start, and the first in reverse, from its end,
    // which is cheaper there than the first square's two other links.
    CHECK(Get(navmesh, both).attached, "the two-way link attached");
    mnavPath forward = Across(query, navmesh, 17.5, 9.0, 7.5, 9.0);
    CHECK(forward.end == mnav_pathFound && forward.linkCount == 1 &&
              SameLink(forward.links[0].link, both) &&
              forward.points[forward.links[0].point].x == 17.5,
          "forward from the second square");
    mnavPath reverse = Across(query, navmesh, 7.5, 9.0, 17.5, 9.0);
    CHECK(reverse.end == mnav_pathFound && reverse.linkCount == 1 &&
              SameLink(reverse.links[0].link, both) &&
              reverse.points[reverse.links[0].point].x == 7.5,
          "the two-way link crosses back to the first square");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A free link whose ends both lie on one square, from the fuzz target:
// taking it is cheaper than walking, and the path leaves the square and
// comes back. The corridor keeps the two visits apart, so that its
// corners still cross the link.
static void TestLinkBackToItsOwnPolygon(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultNavmeshDef());
    mnavLinkDef loop = Link(5.5, 5.5, 9.5, 9.5, 0.25f);
    loop.cost = 0.0f;
    Stage(navmesh, loop);
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a query");
    mnavNearest a;
    mnavNearest b;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){5.5, 0.0, 6.0}, box, nullptr, &a) ==
                  mnav_success &&
              mnavNavmesh_FindNearest(navmesh, (mnavPos3){9.5, 0.0, 9.0}, box, nullptr, &b) ==
                  mnav_success &&
              a.polygon.polygon == b.polygon.polygon,
          "both ends on the first square");
    mnavPath path;
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 && path.pointCount == 4,
          "through the link");
    CHECK(path.polygonCount == 2 && path.polygons[0].polygon == path.polygons[1].polygon,
          "the square twice, before the link and after it");
    mnavPolygonId buffer[8];
    mnavCorridor corridor;
    mnavCorners corners;
    CHECK(mnavResetCorridor(&corridor, buffer, 8, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success &&
              mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners) == mnav_success &&
              corners.linkCount == 1,
          "the corridor's corners cross it");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLinksFollowTheTiles(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultNavmeshDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 39.5, 7.5, 0.5f));
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success && !Get(navmesh, id).attached,
          "detached while tile (1, 0) is not loaded");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handSquareEast, sizeof(s_handSquareEast)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "tile (1, 0) loaded");
    mnavLinkState state = Get(navmesh, id);
    mnavTileId tile;
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 0, &tile) == mnav_success && state.attached &&
              state.endPolygon.slot == tile.slot,
          "attached to it");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success && !Get(navmesh, id).attached,
          "detached again when it leaves");
    mnavDestroyNavmesh(navmesh);
}

// A link leaves only the polygon it starts on: one from the second square
// to tile (1, 0) gives the first square no way there, though the two
// share a tile.
static void TestLinksLeaveTheirOwnPolygon(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultNavmeshDef());
    mnavLinkId id = Stage(navmesh, Link(17.5, 7.5, 39.5, 7.5, 0.5f));
    CHECK(mnavNavmesh_StageTile(navmesh, s_handSquareEast, sizeof(s_handSquareEast)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success && Get(navmesh, id).attached,
          "attached");
    mnavQuery* query = MakeQuery();
    mnavPath from = Across(query, navmesh, 17.5, 7.5, 39.5, 7.5);
    CHECK(from.end == mnav_pathFound && from.linkCount == 1 && SameLink(from.links[0].link, id),
          "across from the second square");
    CHECK(Across(query, navmesh, 7.5, 7.5, 39.5, 7.5).end == mnav_pathNone,
          "no way from the first");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestRemovalAndIds(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultNavmeshDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success && Get(navmesh, id).attached,
          "still there until the commit");
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success, "staged twice, once");
    mnavLinkState state;
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavLink_GetState(navmesh, id, &state) == mnav_errorStale &&
              mnavStageLinkRemoval(navmesh, id) == mnav_errorStale,
          "gone, the id stale");
    mnavQuery* query = MakeQuery();
    CHECK(Across(query, navmesh, 7.5, 7.5, 17.5, 7.5).end == mnav_pathNone, "no way across left");
    mnavDestroyQuery(query);
    // The slot is reused under a new generation; a link removed before
    // its commit goes at once.
    mnavLinkId next = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(next.slot == id.slot && next.generation == id.generation + 1, "the slot reused");
    CHECK(mnavStageLinkRemoval(navmesh, next) == mnav_success &&
              mnavLink_GetState(navmesh, next, &state) == mnav_errorStale,
          "unstaged at once");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "nothing to commit");
    mnavLinkId bad = {99, 1};
    CHECK(mnavLink_GetState(navmesh, bad, &state) == mnav_errorInvalid &&
              mnavStageLinkRemoval(navmesh, bad) == mnav_errorInvalid,
          "no such slot");
    bad = (mnavLinkId){next.slot, next.generation + 1};
    CHECK(mnavLink_GetState(navmesh, bad, &state) == mnav_errorInvalid,
          "a generation not yet given");
    // A slot at its last generation is never reused: test_link_slots.c.
    mnavDestroyNavmesh(navmesh);
}

static void TestDefsAndLimits(void)
{
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    def.limits.links = 2;
    mnavNavmesh* navmesh = Make(def);
    mnavLinkId id;
    mnavLinkDef link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.radius = -1.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "a negative radius");
    link.radius = MNAV_MAX_LINK_RADIUS * 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "too wide");
    link.radius = INFINITY;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "an infinite radius");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.cost = NAN;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "a NaN cost");
    link.cost = -1.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "a negative cost");
    // A kind past the last, and negative ones, which a signed comparison
    // would let through.
    const mnavLinkKind kinds[3] = {MNAV_LINK_KINDS, -1, INT32_MIN};
    for (int32_t i = 0; i < 3; ++i)
    {
        link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
        link.kind = kinds[i];
        CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "no such kind");
    }
    // Ends farther than bake input may lie: past the extent on the ground
    // or in height, and an edge link whose ends are far enough apart that
    // their difference overflows.
    double ground = (double)def.bake.cellSize * (double)MNAV_MAX_EXTENT_CELLS;
    double height = (double)def.bake.cellHeight * (double)MNAV_MAX_HEIGHT_CELLS;
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.end.x = def.bake.origin.x + ground;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_success &&
              mnavStageLinkRemoval(navmesh, id) == mnav_success,
          "at the extent");
    link.end.x = def.bake.origin.x + ground * 1.001;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "past the extent");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.start.y = def.bake.origin.y - height * 1.001;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "below the extent");
    link = Link(-1.0e308, 7.5, 1.0e308, 7.5, 0.5f);
    link.width = 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "an overflowing edge link");
    link = Link(7.5, 7.5, 7.5 + 1.0e-200, 7.5, 0.5f);
    link.width = 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid,
          "an edge link of ends too near for a direction");
    link = Link(7.5, 7.5, (double)INFINITY, 7.5, 0.5f);
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit,
          "an infinite end, past the extent");
    link = Link(7.5, 7.5, (double)NAN, 7.5, 0.5f);
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "a NaN end");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    CHECK(mnavStageLink(nullptr, &link, &id) == mnav_errorInvalid &&
              mnavStageLink(navmesh, nullptr, &id) == mnav_errorInvalid &&
              mnavStageLink(navmesh, &link, nullptr) == mnav_errorInvalid &&
              mnavStageLinkRemoval(nullptr, id) == mnav_errorInvalid &&
              mnavLink_GetState(nullptr, id, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavLinkId a = Stage(navmesh, link);
    (void)Stage(navmesh, link);
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorCapacity, "past the links limit");
    CHECK(mnavStageLinkRemoval(navmesh, a) == mnav_success &&
              mnavStageLink(navmesh, &link, &id) == mnav_success,
          "room again");
    mnavDestroyNavmesh(navmesh);
}

static void TestFailedCommitKeepsLinksStaged(void)
{
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    def.limits.tiles = 1;
    mnavNavmesh* navmesh = Make(def);
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(mnavNavmesh_StageTile(navmesh, s_handSquareEast, sizeof(s_handSquareEast)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_errorCapacity,
          "past the tiles limit");
    mnavLinkState state;
    CHECK(mnavLink_GetState(navmesh, id, &state) == mnav_errorState, "the link still staged");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success && Get(navmesh, id).attached,
          "committed with the tile unstaged");
    mnavDestroyNavmesh(navmesh);
}

static void TestDestroyGivesLinksBack(void)
{
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    def.allocator = CountingAllocator();
    mnavNavmesh* navmesh = Make(def);
    for (int32_t i = 0; i < 9; ++i)
    {
        (void)Stage(navmesh, Link(7.5, 7.5, 17.5, 5.5 + i * 0.5, 0.5f));
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    (void)Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    mnavDestroyNavmesh(navmesh);
    CHECK(s_held == 0, "all given back");
}

int main(void)
{
    TestLinksAttachAtTheCommit();
    TestLinksFollowTheTiles();
    TestLinkBackToItsOwnPolygon();
    TestLinksLeaveTheirOwnPolygon();
    TestRemovalAndIds();
    TestDefsAndLimits();
    TestFailedCommitKeepsLinksStaged();
    TestDestroyGivesLinksBack();
    return s_failures == 0 ? 0 : 1;
}
