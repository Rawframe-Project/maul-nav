// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh links (mnav-0004): staging, removal, snapping at each commit, and
// the attachments the search will follow. Hand cells are 0.25 m.

#include "counting_allocator.h"
#include "hand_tile.h"
#include "navmesh.h"
#include "offmesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// Two squares apart in tile (0, 0): (5, 5) to (10, 10) and (15, 5) to
// (20, 10); one in tile (1, 0): (37, 5) to (42, 10).
static const HandSquare s_apart[2] = {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 0}};
static const HandSquare s_beyond[1] = {{20, 20, 40, 40, {0}, 0}};
static uint8_t s_bytes[2][4096];
static size_t s_sizes[2];

static mnavNavmesh* Make(mnavBakeDef def)
{
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavStageTile(navmesh, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
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
    CHECK(mnavGetLink(navmesh, id, &state) == mnav_success, "read");
    return state;
}

static void TestLinksAttachAtTheCommit(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultBakeDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    mnavLinkState state;
    CHECK(mnavGetLink(navmesh, id, &state) == mnav_errorNotLoaded && !state.attached,
          "not before the commit");
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    state = Get(navmesh, id);
    CHECK(state.attached && state.startPolygon.polygon == 0 && state.endPolygon.polygon == 1 &&
              state.start.x == 7.5 && state.start.z == 7.5 && state.end.x == 17.5 &&
              state.start.y == 0.0,
          "both ends on their squares");
    int32_t first = -1;
    CHECK(mnavAttachmentsFrom(navmesh, 0, 0, &first) == 1 &&
              mnavAttachmentsFrom(navmesh, 0, 1, &first) == 0,
          "one way: from the first square only");
    // An end 0.4 m off a square's edge snaps onto it; 0.6 m off does not.
    mnavLinkId near = Stage(navmesh, Link(7.5, 7.5, 14.6, 7.5, 0.5f));
    mnavLinkId far = Stage(navmesh, Link(7.5, 7.5, 14.4, 7.5, 0.5f));
    mnavLinkDef twoWay = Link(17.5, 9.0, 7.5, 9.0, 0.0f);
    twoWay.twoWay = true;
    mnavLinkId both = Stage(navmesh, twoWay);
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    state = Get(navmesh, near);
    CHECK(state.attached && state.end.x == 15.0 && state.end.z == 7.5, "snapped to the edge");
    CHECK(!Get(navmesh, far).attached, "too far to snap");
    // Off the square's corner (15, 5) by 0.4 m on both axes: within the
    // search box, but 0.57 m away, past the 0.5 m radius.
    mnavLinkId corner = Stage(navmesh, Link(7.5, 7.5, 14.6, 4.6, 0.5f));
    CHECK(mnavCommit(navmesh) == mnav_success && !Get(navmesh, corner).attached,
          "the radius is round");
    CHECK(mnavStageLinkRemoval(navmesh, corner) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "removed again");
    // The two-way link starts on the second square: it leaves the second
    // forward and the first in reverse, after the first's two links.
    CHECK(Get(navmesh, both).attached && mnavAttachmentsFrom(navmesh, 0, 1, &first) == 1 &&
              !mnavAttachmentOf(navmesh->attachments[first]).reverse,
          "forward from the second square");
    CHECK(mnavAttachmentsFrom(navmesh, 0, 0, &first) == 3, "three from the first square");
    mnavAttachment a = mnavAttachmentOf(navmesh->attachments[first + 2]);
    CHECK(a.slot == 0 && a.polygon == 0 && a.link == (int32_t)both.slot - 1 && a.reverse,
          "the two-way link crosses back to the first square");
    mnavDestroyNavmesh(navmesh);
}

// A free link whose ends both lie on one square, from the fuzz target:
// taking it is cheaper than walking, and the path leaves the square and
// comes back. The corridor keeps the two visits apart, so that its
// corners still cross the link.
static void TestLinkBackToItsOwnPolygon(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultBakeDef());
    mnavLinkDef loop = Link(5.5, 5.5, 9.5, 9.5, 0.25f);
    loop.cost = 0.0f;
    Stage(navmesh, loop);
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a query");
    mnavNearest a;
    mnavNearest b;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){5.5, 0.0, 6.0}, box, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){9.5, 0.0, 9.0}, box, &b) ==
                  mnav_success &&
              a.polygon.polygon == b.polygon.polygon,
          "both ends on the first square");
    mnavPath path;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 && path.pointCount == 4,
          "through the link");
    CHECK(path.polygonCount == 2 && path.polygons[0].polygon == path.polygons[1].polygon,
          "the square twice, before the link and after it");
    mnavPolygonId buffer[8];
    mnavCorridor corridor;
    mnavCorners corners;
    CHECK(mnavResetCorridor(&corridor, buffer, 8, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success &&
              mnavCorridorCorners(query, navmesh, &corridor, &corners) == mnav_success &&
              corners.linkCount == 1,
          "the corridor's corners cross it");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLinksFollowTheTiles(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultBakeDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 39.5, 7.5, 0.5f));
    CHECK(mnavCommit(navmesh) == mnav_success && !Get(navmesh, id).attached,
          "detached while tile (1, 0) is not loaded");
    CHECK(mnavStageTile(navmesh, s_bytes[1], s_sizes[1]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "tile (1, 0) loaded");
    mnavLinkState state = Get(navmesh, id);
    mnavTileId tile;
    CHECK(mnavGetTile(navmesh, 1, 0, &tile) == mnav_success && state.attached &&
              state.endPolygon.slot == tile.slot,
          "attached to it");
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && !Get(navmesh, id).attached,
          "detached again when it leaves");
    mnavDestroyNavmesh(navmesh);
}

static void TestRemovalAndIds(void)
{
    mnavNavmesh* navmesh = Make(mnavDefaultBakeDef());
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success && Get(navmesh, id).attached,
          "still there until the commit");
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success, "staged twice, once");
    mnavLinkState state;
    CHECK(mnavCommit(navmesh) == mnav_success &&
              mnavGetLink(navmesh, id, &state) == mnav_errorStale &&
              mnavStageLinkRemoval(navmesh, id) == mnav_errorStale,
          "gone, the id stale");
    int32_t first = -1;
    CHECK(mnavAttachmentsFrom(navmesh, 0, 0, &first) == 0, "no attachment left");
    // The slot is reused under a new generation; a link removed before
    // its commit goes at once.
    mnavLinkId next = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(next.slot == id.slot && next.generation == id.generation + 1, "the slot reused");
    CHECK(mnavStageLinkRemoval(navmesh, next) == mnav_success &&
              mnavGetLink(navmesh, next, &state) == mnav_errorStale,
          "unstaged at once");
    CHECK(mnavCommit(navmesh) == mnav_success, "nothing to commit");
    mnavLinkId bad = {99, 1};
    CHECK(mnavGetLink(navmesh, bad, &state) == mnav_errorInvalid &&
              mnavStageLinkRemoval(navmesh, bad) == mnav_errorInvalid,
          "no such slot");
    bad = (mnavLinkId){next.slot, next.generation + 1};
    CHECK(mnavGetLink(navmesh, bad, &state) == mnav_errorInvalid, "a generation not yet given");
    // A slot at its last generation is never reused.
    navmesh->links[next.slot - 1].generation = UINT32_MAX;
    mnavLinkId fresh = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(fresh.slot != next.slot && fresh.generation == 1, "a new slot past a worn one");
    mnavDestroyNavmesh(navmesh);
}

static void TestDefsAndLimits(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.links = 2;
    mnavNavmesh* navmesh = Make(def);
    mnavLinkId id;
    mnavLinkDef link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.radius = -1.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "a negative radius");
    link.radius = MNAV_MAX_LINK_RADIUS * 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "too wide");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.cost = NAN;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "a NaN cost");
    link.cost = -1.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "a negative cost");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.kind = MNAV_LINK_KINDS;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "no such kind");
    // Ends farther than bake input may lie: past the extent on the ground
    // or in height, and an edge link whose ends are far enough apart that
    // their difference overflows.
    double ground = (double)def.cellSize * (double)MNAV_MAX_EXTENT_CELLS;
    double height = (double)def.cellHeight * (double)MNAV_MAX_HEIGHT_CELLS;
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.end.x = def.origin.x + ground;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_success &&
              mnavStageLinkRemoval(navmesh, id) == mnav_success,
          "at the extent");
    link.end.x = def.origin.x + ground * 1.001;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "past the extent");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    link.start.y = def.origin.y - height * 1.001;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "below the extent");
    link = Link(-1.0e308, 7.5, 1.0e308, 7.5, 0.5f);
    link.width = 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorRange, "an overflowing edge link");
    link = Link(7.5, 7.5, 7.5 + 1.0e-200, 7.5, 0.5f);
    link.width = 2.0f;
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid,
          "an edge link of ends too near for a direction");
    link = Link(7.5, 7.5, (double)INFINITY, 7.5, 0.5f);
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorInvalid, "an end not finite");
    link = Link(7.5, 7.5, 17.5, 7.5, 0.5f);
    CHECK(mnavStageLink(nullptr, &link, &id) == mnav_errorInvalid &&
              mnavStageLink(navmesh, nullptr, &id) == mnav_errorInvalid &&
              mnavStageLink(navmesh, &link, nullptr) == mnav_errorInvalid &&
              mnavStageLinkRemoval(nullptr, id) == mnav_errorInvalid &&
              mnavGetLink(nullptr, id, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavLinkId a = Stage(navmesh, link);
    (void)Stage(navmesh, link);
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_errorLimit, "past the links limit");
    CHECK(mnavStageLinkRemoval(navmesh, a) == mnav_success &&
              mnavStageLink(navmesh, &link, &id) == mnav_success,
          "room again");
    mnavDestroyNavmesh(navmesh);
}

static void TestFailedCommitKeepsLinksStaged(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tiles = 1;
    mnavNavmesh* navmesh = Make(def);
    mnavLinkId id = Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    CHECK(mnavStageTile(navmesh, s_bytes[1], s_sizes[1]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_errorLimit,
          "past the tiles limit");
    mnavLinkState state;
    CHECK(mnavGetLink(navmesh, id, &state) == mnav_errorNotLoaded, "the link still staged");
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && Get(navmesh, id).attached,
          "committed with the tile unstaged");
    mnavDestroyNavmesh(navmesh);
}

static void TestDestroyGivesLinksBack(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    mnavNavmesh* navmesh = Make(def);
    for (int32_t i = 0; i < 9; ++i)
    {
        (void)Stage(navmesh, Link(7.5, 7.5, 17.5, 5.5 + i * 0.5, 0.5f));
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    (void)Stage(navmesh, Link(7.5, 7.5, 17.5, 7.5, 0.5f));
    mnavDestroyNavmesh(navmesh);
    CHECK(s_held == 0, "all given back");
}

int main(void)
{
    s_sizes[0] = HandTileBytes(s_bytes[0], 0, s_apart, 2);
    s_sizes[1] = HandTileBytes(s_bytes[1], 1, s_beyond, 1);
    TestLinksAttachAtTheCommit();
    TestLinksFollowTheTiles();
    TestLinkBackToItsOwnPolygon();
    TestRemovalAndIds();
    TestDefsAndLimits();
    TestFailedCommitKeepsLinksStaged();
    TestDestroyGivesLinksBack();
    return s_failures == 0 ? 0 : 1;
}
