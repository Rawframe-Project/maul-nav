// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Runtime tiers: what each tier allows, area changes and link
// toggles through the commit. Hand cells are 0.25 m.

#include "hand_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// s_handRowAndGap: a row of three squares from (5, 5) to (20, 10), and a
// fourth beyond a gap, from (25, 5) to (30, 10).
static mnavNavmesh* Load(mnavTier tier)
{
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    def.tier = tier;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success && mnavNavmesh_GetTier(navmesh) == tier,
          "loaded");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){0.1f, 2.0f, 0.1f},
                                  nullptr, &n) == mnav_success &&
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
    CHECK(mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success,
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
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    CHECK(def.tier == mnav_tierStatic, "static by default");
    def.tier = mnav_tierDynamic + 1;
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDefResult created = mnavCreateNavmesh(&def, &navmesh);
    CHECK(created.result == mnav_errorInvalid && created.setting == mnav_navmeshSettingTier &&
              navmesh == nullptr,
          "no fourth tier");
    CHECK(mnavNavmesh_GetTier(nullptr) == mnav_tierStatic, "NULL reads static");
    CHECK(mnavGetResultName(mnav_errorUnsupported)[5] == 'e', "named");
}

static void TestStatic(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierStatic);
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    mnavLinkDef jump = {{19.5, 0.0, 7.5}, {25.5, 0.0, 7.5}, 0.5f, 3.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId link;
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_errorUnsupported,
          "no area changes");
    CHECK(mnavStageLink(navmesh, &jump, &link) == mnav_success &&
              mnavLink_StageEnable(navmesh, link, false) == mnav_errorUnsupported,
          "links load, but do not toggle");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
              mnav_errorUnsupported,
          "no tile replaced in place");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
                  mnav_errorUnsupported,
          "nor by a removal in the same commit");
    mnavTileId tile;
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavNavmesh_GetTile(navmesh, 0, 0, &tile) == mnav_errorNotLoaded,
          "streamed out");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavNavmesh_GetTile(navmesh, 0, 0, &tile) == mnav_success,
          "and in again");
    mnavDestroyNavmesh(navmesh);
}

static void TestAreas(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierModifiers);
    mnavQuery* query = MakeQuery();
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    mnavAreaType area = 0;
    CHECK(mnavPolygon_GetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaWalkable && Reach(query, navmesh, 18.0) == mnav_pathFound,
          "walkable, and walked through");
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, 7) == mnav_success &&
              mnavPolygon_StageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavPolygon_GetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaWalkable,
          "staged, not yet applied");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavPolygon_GetArea(navmesh, middle.polygon, &area) == mnav_success &&
              area == mnav_areaNone && Reach(query, navmesh, 18.0) == mnav_pathNone,
          "the last staged area applied: blocked");
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, mnav_areaWalkable) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              Reach(query, navmesh, 18.0) == mnav_pathFound,
          "open again");
    // Applied changes are not staged any more: a commit with nothing
    // staged changes nothing, and a search begun before it goes on.
    mnavNearest a = On(navmesh, 6.0, 7.5);
    bool ended = false;
    CHECK(mnavQuery_BeginPath(query, navmesh, a.polygon, a.point, middle.polygon, middle.point,
                              nullptr) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavQuery_ContinuePath(query, navmesh, 1, &ended) == mnav_success,
          "an empty commit after a change");
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, MNAV_AREA_TYPES) == mnav_errorInvalid &&
              mnavPolygon_StageArea(nullptr, middle.polygon, 1) == mnav_errorInvalid &&
              mnavPolygon_GetArea(navmesh, middle.polygon, nullptr) == mnav_errorInvalid,
          "bad arguments");
    CHECK(mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
              mnav_errorUnsupported,
          "modifiers replace no tile");
    // A change staged on a tile the commit removes is dropped.
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavNavmesh_StageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavPolygon_GetArea(navmesh, middle.polygon, &area) == mnav_errorStale &&
              mnavPolygon_StageArea(navmesh, middle.polygon, 1) == mnav_errorStale,
          "dropped with its tile");
    middle.polygon.slot = 9;
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, 1) == mnav_errorInvalid,
          "an id never handed out");
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
    CHECK(mnavNavmesh_FindPolygons(navmesh, door, half, nullptr, found, 8, &count) ==
                  mnav_success &&
              count.polygonCount == 1 && !count.incomplete,
          "the middle square");
    CHECK(mnavPolygon_StageArea(navmesh, found[0], mnav_areaNone) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              Reach(query, navmesh, 18.0) == mnav_pathNone &&
              mnavNavmesh_FindPolygons(navmesh, door, half, nullptr, found, 8, &count) ==
                  mnav_success &&
              count.polygonCount == 0,
          "blocked, and no longer walkable");
    mnavQueryFilter blocked = mnavDefaultQueryFilter();
    blocked.areas = (uint64_t)1 << mnav_areaNone;
    CHECK(mnavNavmesh_FindPolygons(navmesh, door, half, &blocked, found, 8, &count) ==
                  mnav_success &&
              count.polygonCount == 1 &&
              mnavPolygon_StageArea(navmesh, found[0], mnav_areaWalkable) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              Reach(query, navmesh, 18.0) == mnav_pathFound,
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
              mnavLink_StageEnable(navmesh, link, false) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavLink_GetState(navmesh, link, &state) == mnav_success && state.attached &&
              !state.enabled && Reach(query, navmesh, 28.0) != mnav_pathFound,
          "added disabled: snapped, not crossed");
    CHECK(mnavLink_StageEnable(navmesh, link, true) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavLink_GetState(navmesh, link, &state) == mnav_success && state.enabled &&
              Reach(query, navmesh, 28.0) == mnav_pathFound,
          "enabled: crossed");
    bool enabled = false;
    CHECK(mnavLink_StageEnable(navmesh, link, false) == mnav_success &&
              mnavLink_GetState(navmesh, link, &state) == mnav_success && state.enabled &&
              mnavLink_IsEnabled(navmesh, link, &enabled) == mnav_success && enabled,
          "a staged toggle waits for the commit");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavLink_IsEnabled(navmesh, link, &enabled) == mnav_success && !enabled &&
              Reach(query, navmesh, 28.0) != mnav_pathFound,
          "disabled again");
    mnavLinkId staged;
    enabled = true;
    CHECK(mnavStageLink(navmesh, &jump, &staged) == mnav_success &&
              mnavLink_IsEnabled(navmesh, staged, &enabled) == mnav_errorState && !enabled,
          "a link not yet committed has no state");
    CHECK(mnavStageLinkRemoval(navmesh, link) == mnav_success &&
              mnavLink_StageEnable(navmesh, link, true) == mnav_errorStale &&
              mnavLink_StageEnable(nullptr, link, true) == mnav_errorInvalid,
          "not a link staged for removal");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestDynamic(void)
{
    mnavNavmesh* navmesh = Load(mnav_tierDynamic);
    mnavQuery* query = MakeQuery();
    mnavNearest middle = On(navmesh, 12.5, 7.5);
    CHECK(mnavPolygon_StageArea(navmesh, middle.polygon, mnav_areaNone) == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_handRowAndGap, sizeof(s_handRowAndGap)).result ==
                  mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "replaced in one commit");
    mnavNearest again = On(navmesh, 12.5, 7.5);
    mnavAreaType area = 0;
    CHECK(again.polygon.generation != middle.polygon.generation &&
              mnavPolygon_GetArea(navmesh, again.polygon, &area) == mnav_success &&
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
