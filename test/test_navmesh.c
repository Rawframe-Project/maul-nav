// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The navmesh: staged tiles, atomic commits, generation ids and links
// across tile sides, as the API shows them; the link and slot tables
// themselves are test_navmesh_tables.c's.

#include "counting_allocator.h"
#include "hand_tiles.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

static mnavNavmesh* Make(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    return navmesh;
}

static mnavResult StageAt(mnavNavmesh* navmesh, int32_t t)
{
    return mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result;
}

// How many of a run of polygons, from the first, are current and each
// joined to the one before (mnavNavmesh_CheckCorridor).
static int32_t Valid(const mnavNavmesh* navmesh, const mnavPolygonId* polygons, int32_t count)
{
    mnavPolygonId buffer[2];
    mnavCorridor corridor;
    int32_t valid = -1;
    CHECK(mnavResetCorridor(&corridor, buffer, 2, polygons[0], (mnavPos3){0.0, 0.0, 0.0}) ==
              mnav_success,
          "a corridor");
    for (int32_t i = 1; i < count; ++i)
    {
        buffer[i] = polygons[i];
    }
    corridor.count = count;
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &corridor, nullptr, &valid) == mnav_success,
          "checked");
    return valid;
}

static bool Current(const mnavNavmesh* navmesh, mnavPolygonId polygon)
{
    return Valid(navmesh, &polygon, 1) == 1;
}

static void TestStagedTilesAppearAtCommit(void)
{
    mnavNavmesh* navmesh = Make();
    mnavTileId id = {9, 9};
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &id) == mnav_errorNotLoaded && id.slot == 0, "empty");
    CHECK(StageAt(navmesh, 0) == mnav_success, "staged");
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &id) == mnav_errorNotLoaded, "not before the commit");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 1 &&
              id.generation == 1,
          "slot 1, generation 1");
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "nothing staged");
    mnavDestroyNavmesh(navmesh);
}

static void TestReplacingAndRemovingMakeIdsStale(void)
{
    mnavNavmesh* navmesh = Make();
    CHECK(StageAt(navmesh, 0) == mnav_success && StageAt(navmesh, 1) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "two tiles");
    mnavPolygonId old = {1, 1, 0};
    CHECK(Current(navmesh, old), "a current id");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavNavmesh_Commit(navmesh) == mnav_success,
          "replaced");
    CHECK(!Current(navmesh, old), "the old id is stale");
    mnavTileId id;
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 1 &&
              id.generation == 2,
          "the same slot, the next generation");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "removed");
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 0, &id) == mnav_errorNotLoaded, "gone");
    CHECK(StageAt(navmesh, 3) == mnav_success && mnavNavmesh_Commit(navmesh) == mnav_success,
          "another");
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 1, &id) == mnav_success && id.slot == 2 &&
              id.generation == 3,
          "the freed slot, reused with its next generation");
    // The removed tile's links went with it: none leads from (0, 0) into
    // the tile now in its slot, and (1, 0) and (0, 1) are not loaded.
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    mnavNearest a;
    mnavNearest b;
    mnavPath path;
    const mnavVec3 box = {2.0f, 2.0f, 2.0f};
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success &&
              mnavNavmesh_FindNearest(navmesh, (mnavPos3){2.0, 0.0, 2.0}, box, nullptr, &a) ==
                  mnav_success &&
              mnavNavmesh_FindNearest(navmesh, (mnavPos3){62.0, 0.0, 62.0}, box, nullptr, &b) ==
                  mnav_success &&
              mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                                 &path) == mnav_success &&
              path.end == mnav_pathNotLoaded,
          "no way into the slot's new tile");
    mnavDestroyQuery(query);
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 7, 7) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "removing nothing");
    mnavDestroyNavmesh(navmesh);
}

static void TestFailedCommitChangesNothing(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tileLinks = 1;
    def.allocator = CountingAllocator();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavNavmesh_Commit(navmesh) == mnav_success,
          "one tile");
    size_t before = s_held;
    CHECK(StageAt(navmesh, 1) == mnav_success, "staged");
    size_t staged = s_held;
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_errorCapacity, "past the links limit");
    mnavTileId id;
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 0, &id) == mnav_errorNotLoaded, "nothing applied");
    CHECK(s_held == staged, "the change stays staged, nothing taken or given back");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "unstaged by a removal");
    // The commit's new slot and place arrays have room for one more entry;
    // the tile itself, close to a kilobyte, is gone.
    CHECK(s_held < before + 64 && staged > before + 512, "the staged tile freed");
    mnavDestroyNavmesh(navmesh);
    def = mnavDefaultBakeDef();
    def.limits.tiles = 1;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(StageAt(navmesh, 0) == mnav_success && StageAt(navmesh, 1) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_errorCapacity,
          "past the tiles limit");
    mnavDestroyNavmesh(navmesh);
}

static void TestTilesMustMatchTheDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.stepHeight = 0.5f;
    def.allocator = CountingAllocator();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    size_t held = s_held;
    mnavTileResult result = mnavNavmesh_StageTile(navmesh, s_tiles[0], s_sizes[0]);
    CHECK(result.result == mnav_errorFormat && result.section == mnav_tileHeader,
          "baked for another agent");
    result = mnavNavmesh_StageTile(navmesh, s_tiles[0], s_sizes[0] - 1);
    CHECK(result.result == mnav_errorFormat, "damaged bytes");
    CHECK(s_held == held, "nothing staged");
    mnavDestroyNavmesh(navmesh);
    CHECK(mnavNavmesh_StageTile(nullptr, s_tiles[0], s_sizes[0]).result == mnav_errorInvalid &&
              mnavNavmesh_Commit(nullptr) == mnav_errorInvalid &&
              mnavNavmesh_StageTileRemoval(nullptr, 0, 0) == mnav_errorInvalid,
          "no navmesh");
    mnavDestroyNavmesh(nullptr);
}

static void TestDestroyGivesEverythingBack(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(StageAt(navmesh, t) == mnav_success, "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavTileId tile;
    CHECK(StageAt(navmesh, 2) == mnav_success &&
              mnavNavmesh_GetTile(navmesh, 0, 0, &tile) == mnav_success &&
              mnavPolygon_StageArea(navmesh, (mnavPolygonId){tile.slot, tile.generation, 0}, 3) ==
                  mnav_success,
          "staged, not committed");
    CHECK(s_held > 0, "held");
    mnavDestroyNavmesh(navmesh);
    CHECK(s_held == 0, "all given back");
}

// Whether tiles (0, 0) and (1, 0) of one polygon each, facing each other
// across the side between them, are joined there, checked both ways.
static bool Joined(const uint8_t* west, size_t westSize, const uint8_t* east, size_t eastSize)
{
    mnavNavmesh* navmesh = Make();
    mnavTileId a;
    mnavTileId b;
    CHECK(mnavNavmesh_StageTile(navmesh, west, westSize).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, east, eastSize).result == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavNavmesh_GetTile(navmesh, 0, 0, &a) == mnav_success &&
              mnavNavmesh_GetTile(navmesh, 1, 0, &b) == mnav_success,
          "hand tiles committed");
    const mnavPolygonId there[2] = {{a.slot, a.generation, 0}, {b.slot, b.generation, 0}};
    const mnavPolygonId back[2] = {there[1], there[0]};
    bool joined = Valid(navmesh, there, 2) == 2;
    CHECK(joined == (Valid(navmesh, back, 2) == 2), "both ways");
    mnavDestroyNavmesh(navmesh);
    return joined;
}

static void TestLinksNeedHeightsWithinAStepAtBothEnds(void)
{
    // The agent's step is 6 cell heights; the edges facing each other
    // (s_handStep*) are level, a step up, past it, or rising to 7.
    CHECK(Joined(s_handStepWestFlat, sizeof(s_handStepWestFlat), s_handStepEastFlat,
                 sizeof(s_handStepEastFlat)),
          "level edges link");
    CHECK(Joined(s_handStepWestFlat, sizeof(s_handStepWestFlat), s_handStepEastStep,
                 sizeof(s_handStepEastStep)),
          "exactly a step apart links");
    CHECK(!Joined(s_handStepWestFlat, sizeof(s_handStepWestFlat), s_handStepEastPast,
                  sizeof(s_handStepEastPast)),
          "past a step does not");
    CHECK(!Joined(s_handStepWestFlat, sizeof(s_handStepWestFlat), s_handStepEastRising,
                  sizeof(s_handStepEastRising)),
          "close at one end only does not");
    CHECK(Joined(s_handStepWestRising, sizeof(s_handStepWestRising), s_handStepEastRising,
                 sizeof(s_handStepEastRising)),
          "the same slope links");
}

int main(void)
{
    LoadWorldTiles();
    TestLinksNeedHeightsWithinAStepAtBothEnds();
    TestStagedTilesAppearAtCommit();
    TestReplacingAndRemovingMakeIdsStale();
    TestFailedCommitChangesNothing();
    TestTilesMustMatchTheDef();
    TestDestroyGivesEverythingBack();
    return s_failures == 0 ? 0 : 1;
}
