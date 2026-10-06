// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Runtime tiers (mnav-0004): what each tier allows, area changes and link
// toggles through the commit. Hand cells are 0.25 m.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// A row of three squares from (5, 5) to (20, 10), and a fourth beyond a
// gap, from (25, 5) to (30, 10).
static const HandSquare s_row[4] = {{20, 20, 40, 40, {0}, 0},
                                    {40, 20, 60, 40, {0}, 0},
                                    {60, 20, 80, 40, {0}, 0},
                                    {100, 20, 120, 40, {0}, 0}};

static uint8_t s_bytes[8192];
static size_t s_size;

static mnavNavmesh* Load(mnavTier tier)
{
    s_size = HandTileBytes(s_bytes, 0, s_row, 4);
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = tier;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_bytes, s_size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success && mnavGetTier(navmesh) == tier,
          "loaded");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 2.0f, 0.1f},
                          &n) == mnav_success &&
              n.polygon.slot != 0,
          "on a square");
    return n;
}

// The end of a search from the first square to the one given.
static mnavPathEnd Reach(mnavQuery* query, const mnavNavmesh* navmesh, double x)
{
    mnavNearest a = On(navmesh, 6.0, 7.5);
    mnavNearest b = On(navmesh, x, 7.5);
    mnavPath path;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    return path.end;
}

static mnavQuery* MakeQuery(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static void TestTheDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    CHECK(def.tier == mnav_tierStatic, "static by default");
    def.tier = mnav_tierDynamic + 1;
    mnavBakeDefResult checked = mnavValidateBakeDef(&def, nullptr);
    CHECK(checked.result == mnav_errorInvalid && checked.setting == mnav_settingTier,
          "no fourth tier");
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).setting == mnav_settingTier && navmesh == nullptr,
          "not created");
    CHECK(mnavGetTier(nullptr) == mnav_tierStatic, "NULL reads static");
    CHECK(mnavResultName(mnav_errorTier)[5] == 'e', "named");
}

static void TestStatic(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierStatic);
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    mnavLinkDef jump = {{19.5, 0.0, 7.5}, {25.5, 0.0, 7.5}, 0.5f, 3.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId link;
    CHECK(mnavStageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_errorTier,
          "no area changes");
    CHECK(mnavStageLink(navmesh, &jump, &link) == mnav_success &&
              mnavStageLinkEnabled(navmesh, link, false) == mnav_errorTier,
          "links load, but do not toggle");
    CHECK(mnavStageTile(navmesh, s_bytes, s_size).result == mnav_errorTier,
          "no tile replaced in place");
    CHECK(mnavStageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavStageTile(navmesh, s_bytes, s_size).result == mnav_errorTier,
          "nor by a removal in the same commit");
    mnavTileId tile;
    CHECK(mnavCommit(navmesh) == mnav_success &&
              mnavGetTile(navmesh, 0, 0, &tile) == mnav_errorNotLoaded,
          "streamed out");
    CHECK(mnavStageTile(navmesh, s_bytes, s_size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetTile(navmesh, 0, 0, &tile) == mnav_success,
          "and in again");
    mnavDestroyNavmesh(navmesh);
}

static void TestAreas(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierModifiers);
    mnavQuery* query = MakeQuery();
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    mnavAreaType area = 0;
    CHECK(mnavGetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaWalkable && Reach(query, navmesh, 18.0) == mnav_pathFound,
          "walkable, and walked through");
    CHECK(mnavStageArea(navmesh, middle.polygon, 7) == mnav_success &&
              mnavStageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavGetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaWalkable,
          "staged, not yet applied");
    CHECK(mnavCommit(navmesh) == mnav_success &&
              mnavGetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaNone && Reach(query, navmesh, 18.0) == mnav_pathNone,
          "the last staged area applied: blocked");
    CHECK(mnavStageArea(navmesh, middle.polygon, mnav_areaWalkable) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && Reach(query, navmesh, 18.0) == mnav_pathFound,
          "open again");
    // Applied changes are not staged any more: a commit with nothing
    // staged changes nothing, and a search begun before it goes on.
    mnavNearest a = On(navmesh, 6.0, 7.5);
    bool ended = false;
    CHECK(mnavBeginPath(query, navmesh, nullptr, a.polygon, a.point, middle.polygon,
                        middle.point) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavContinuePath(query, navmesh, 1, &ended) == mnav_success,
          "an empty commit after a change");
    CHECK(mnavStageArea(navmesh, middle.polygon, MNAV_AREA_TYPES) == mnav_errorInvalid &&
              mnavStageArea(nullptr, middle.polygon, 1) == mnav_errorInvalid &&
              mnavGetArea(navmesh, middle.polygon, nullptr) == mnav_errorInvalid,
          "bad arguments");
    CHECK(mnavStageTile(navmesh, s_bytes, s_size).result == mnav_errorTier,
          "modifiers replace no tile");
    // A change staged on a tile the commit removes is dropped.
    CHECK(mnavStageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavStageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetArea(navmesh, middle.polygon, &area) == mnav_errorStale &&
              mnavStageArea(navmesh, middle.polygon, 1) == mnav_errorStale,
          "dropped with its tile");
    middle.polygon.slot = 9;
    CHECK(mnavStageArea(navmesh, middle.polygon, 1) == mnav_errorInvalid, "an id never handed out");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestABlockedBox(void)
{
    // A host blocks what a box covers, then finds the blocked polygons
    // again, with a filter that wants mnav_areaNone, to reopen them.
    mnavNavmesh* navmesh = Load(mnav_tierModifiers);
    mnavQuery* query = MakeQuery();
    mnavPolygonId found[8];
    mnavFound count;
    mnavPos3 door = {12.5, 0.0, 7.5};
    mnavVec3 half = {1.0f, 1.0f, 1.0f};
    CHECK(mnavFindPolygons(navmesh, nullptr, door, half, found, 8, &count) == mnav_success &&
              count.count == 1 && !count.incomplete,
          "the middle square");
    CHECK(mnavStageArea(navmesh, found[0], mnav_areaNone) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && Reach(query, navmesh, 18.0) == mnav_pathNone &&
              mnavFindPolygons(navmesh, nullptr, door, half, found, 8, &count) == mnav_success &&
              count.count == 0,
          "blocked, and no longer walkable");
    mnavQueryFilter blocked = mnavDefaultQueryFilter();
    blocked.areas = (uint64_t)1 << mnav_areaNone;
    CHECK(mnavFindPolygons(navmesh, &blocked, door, half, found, 8, &count) == mnav_success &&
              count.count == 1 &&
              mnavStageArea(navmesh, found[0], mnav_areaWalkable) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && Reach(query, navmesh, 18.0) == mnav_pathFound,
          "found blocked, and reopened");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLinkToggles(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierModifiers);
    mnavQuery* query = MakeQuery();
    mnavLinkDef jump = {{19.5, 0.0, 7.5}, {25.5, 0.0, 7.5}, 0.5f, 3.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId link;
    mnavLinkState state;
    CHECK(mnavStageLink(navmesh, &jump, &link) == mnav_success &&
              mnavStageLinkEnabled(navmesh, link, false) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetLink(navmesh, link, &state) == mnav_success && state.attached &&
              !state.enabled && Reach(query, navmesh, 28.0) != mnav_pathFound,
          "added disabled: snapped, not crossed");
    CHECK(mnavStageLinkEnabled(navmesh, link, true) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetLink(navmesh, link, &state) == mnav_success && state.enabled &&
              Reach(query, navmesh, 28.0) == mnav_pathFound,
          "enabled: crossed");
    CHECK(mnavStageLinkEnabled(navmesh, link, false) == mnav_success &&
              mnavGetLink(navmesh, link, &state) == mnav_success && state.enabled,
          "a staged toggle waits for the commit");
    CHECK(mnavCommit(navmesh) == mnav_success && Reach(query, navmesh, 28.0) != mnav_pathFound,
          "disabled again");
    CHECK(mnavStageLinkRemoval(navmesh, link) == mnav_success &&
              mnavStageLinkEnabled(navmesh, link, true) == mnav_errorStale &&
              mnavStageLinkEnabled(nullptr, link, true) == mnav_errorInvalid,
          "not a link staged for removal");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestDynamic(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierDynamic);
    mnavQuery* query = MakeQuery();
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    CHECK(mnavStageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavStageTile(navmesh, s_bytes, s_size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "replaced in one commit");
    mnavNearest again = On(navmesh, 12.5, 7.5);
    mnavAreaType area = 0;
    CHECK(again.polygon.generation != middle.polygon.generation &&
              mnavGetArea(navmesh, again.polygon, &area) == mnav_success &&
              area == mnav_areaWalkable && Reach(query, navmesh, 18.0) == mnav_pathFound,
          "the new tile has its baked areas");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestTheDef();
    TestStatic();
    TestAreas();
    TestABlockedBox();
    TestLinkToggles();
    TestDynamic();
    return s_failures == 0 ? 0 : 1;
}
