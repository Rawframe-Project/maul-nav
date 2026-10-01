// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh links (N29): staging, removal, snapping at each commit, and
// the attachments the search will follow. Hand cells are 0.25 m.

#include "hand_tile.h"
#include "navmesh.h"
#include "offmesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

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
    return (mnavLinkDef){{x0, 0.0, z0}, {x1, 0.0, z1}, radius, 4.0f, mnav_linkJump, false};
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

// Counts the bytes held through the def's allocator.
static size_t s_held;

static void* Alloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    void* block = aligned_alloc(alignment, (size + alignment - 1) / alignment * alignment);
    s_held += block != nullptr ? size : 0;
    return block;
}

static void Free(void* block, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held -= size;
    free(block);
}

static void TestDestroyGivesLinksBack(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = (mnavAllocator){Alloc, Free, nullptr};
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
    TestRemovalAndIds();
    TestDefsAndLimits();
    TestFailedCommitKeepsLinksStaged();
    TestDestroyGivesLinksBack();
    return s_failures == 0 ? 0 : 1;
}
