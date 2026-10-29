// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Host code calling back in: from a navmesh's allocator, as it allocates
// and as it frees, every call that would change the navmesh is refused
// with mnav_errorBusy and counted as misuse; from a link generation's
// clearance test, every such call but staging a tile or its removal, and
// every call that uses the generating context. The outer call gives what
// it gives with no call back in.

#include "counting_allocator.h"
#include "stored_tiles.h"
#include "test_harness.h"

#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    LINKS = 512,
    // The navmesh's stage, commit and destroy calls.
    NAVMESH_CALLS = 8,
    // Those a clearance test may make: staging a tile and its removal.
    STAGING_CALLS = 2,
    // The calls that take a query context to search with.
    QUERY_CALLS = 27,
};

static mnavLinkDef s_links[LINKS];
static mnavLinkDef s_again[LINKS];

// What a call back in reaches, and what came of it.
typedef struct Reentry
{
    // The navmesh and the context to call back into; NULL for none.
    mnavNavmesh* navmesh;
    mnavQuery* query;
    // A link and a polygon of the navmesh to name in the calls.
    mnavLinkId link;
    mnavPolygonId polygon;
    // Whether the next allocation or release calls back in, once.
    bool onAlloc;
    bool onFree;
    // Calls made back in, those refused with mnav_errorBusy, and those
    // that succeeded.
    int32_t made;
    int32_t busy;
    int32_t succeeded;
} Reentry;

static Reentry s_reentry;

static void Count(mnavResult result)
{
    s_reentry.made += 1;
    s_reentry.busy += result == mnav_errorBusy ? 1 : 0;
    s_reentry.succeeded += result == mnav_success ? 1 : 0;
}

// Every call that would change the navmesh, the two staging calls first
// and the destroy call last, which returns nothing and is seen only in the
// misuse count. The tile removal is of the place the tile is staged at.
static void CallNavmesh(mnavNavmesh* navmesh)
{
    const Reentry* r = &s_reentry;
    mnavLinkDef def = s_links[0];
    mnavLinkId id = {0};
    Count(mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result);
    Count(mnavStageTileRemoval(navmesh, 0, 0));
    Count(mnavCommit(navmesh));
    Count(mnavStageArea(navmesh, r->polygon, mnav_areaWalkable));
    Count(mnavStageLink(navmesh, &def, &id));
    Count(mnavStageLinkRemoval(navmesh, r->link));
    Count(mnavStageLinkEnabled(navmesh, r->link, false));
    mnavDestroyNavmesh(navmesh);
    s_reentry.made += 1;
}

// Every call that takes the context to search with, given the navmesh
// and NULL elsewhere: a refusal comes before any other check.
static void CallQuery(mnavQuery* q, const mnavNavmesh* n)
{
    const mnavPolygonId p = s_reentry.polygon;
    const mnavPos3 a = {0.0, 0.0, 0.0};
    Count(mnavFindFlightPath(q, nullptr, a, a, nullptr));
    Count(mnavBeginFlightPath(q, nullptr, a, a));
    Count(mnavContinueFlightPath(q, nullptr, 1, nullptr));
    Count(mnavFinishFlightPath(q, nullptr, nullptr));
    Count(mnavBuildHierarchy(nullptr, q, n, nullptr, nullptr));
    Count(mnavUpdateHierarchy(nullptr, q, n, nullptr));
    Count(mnavFindHierarchicalPath(q, nullptr, n, p, a, p, a, nullptr));
    Count(mnavFindPath(q, n, nullptr, p, a, p, a, nullptr));
    Count(mnavBeginPath(q, n, nullptr, p, a, p, a));
    Count(mnavContinuePath(q, n, 1, nullptr));
    Count(mnavFinishPath(q, n, nullptr));
    Count(mnavMoveAlongSurface(q, n, nullptr, p, a, a, nullptr));
    Count(mnavCorridorCorners(q, n, nullptr, nullptr));
    Count(mnavCorridorAreas(q, n, nullptr, nullptr, nullptr, 0, nullptr));
    Count(mnavMoveCorridor(q, n, nullptr, nullptr, a, nullptr));
    Count(mnavMoveCorridorTarget(q, n, nullptr, nullptr, a, nullptr));
    Count(mnavShortcutCorridor(q, n, nullptr, nullptr, a, nullptr));
    Count(mnavReplanCorridor(q, n, nullptr, nullptr, (mnavVec3){1.0f, 1.0f, 1.0f}, nullptr));
    Count(mnavFindGridPath(q, nullptr, nullptr, (mnavCell){0, 0}, (mnavCell){0, 0}, nullptr));
    Count(mnavFindWallDistance(q, n, nullptr, p, a, 1.0, nullptr));
    Count(mnavFindWalls(q, n, nullptr, p, a, 1.0, nullptr, 0, nullptr));
    Count(mnavCheckReachable(q, n, nullptr, p, a, p, a, nullptr));
    Count(mnavRaycast(q, n, nullptr, p, a, a, nullptr));
    Count(mnavFindShortestPath(q, n, nullptr, p, a, p, a, nullptr));
    Count(mnavFindRandomPointAround(q, n, nullptr, p, a, 1.0, 1, nullptr));
    Count(mnavGenerateLinks(q, n, nullptr, nullptr, &(mnavTileRange){0, 0, 0, 0}, nullptr, 0,
                            nullptr));
    mnavDestroyQuery(q);
    s_reentry.made += 1;
}

static void CallBack(void)
{
    if (s_reentry.navmesh != nullptr)
    {
        CallNavmesh(s_reentry.navmesh);
    }
    if (s_reentry.query != nullptr)
    {
        CallQuery(s_reentry.query, s_reentry.navmesh);
    }
}

static void* ReentrantAlloc(size_t size, size_t alignment, void* context)
{
    if (s_reentry.onAlloc)
    {
        s_reentry.onAlloc = false;
        CallBack();
    }
    return CountingAlloc(size, alignment, context);
}

static void ReentrantFree(void* block, size_t size, size_t alignment, void* context)
{
    if (s_reentry.onFree)
    {
        s_reentry.onFree = false;
        CallBack();
    }
    CountingFree(block, size, alignment, context);
}

// The trench (test/trench.h) at the dynamic tier, with a link staged and
// committed; reentrant, through the allocator that calls back in.
static mnavNavmesh* Load(bool reentrant, mnavLinkId* linkOut)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    if (reentrant)
    {
        def.allocator = (mnavAllocator){ReentrantAlloc, ReentrantFree, nullptr};
    }
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavStageTile(navmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavStageLink(navmesh, &s_links[0], linkOut) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the trench, with a link");
    return navmesh;
}

static mnavQuery* Query(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a context");
    return query;
}

static mnavLinkGenDef Def(void)
{
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    def.jumpMax = 3.0f;
    return def;
}

// The links the trench gives, the first of which every navmesh here has.
static int32_t Generate(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavStageTile(navmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the trench");
    mnavQuery* query = Query();
    mnavLinkGenDef gen = Def();
    int32_t count = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &gen, &(mnavTileRange){0, 0, 1, 0}, s_links,
                            LINKS, &count) == mnav_success &&
              count > 0,
          "links to stage");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return count;
}

// The tile ids at the trench's two places, 0 for a place with none: the
// same changes to two navmeshes give the same ids.
static uint64_t Tiles(const mnavNavmesh* navmesh)
{
    uint64_t sum = 0;
    for (int32_t x = 0; x < 2; ++x)
    {
        mnavTileId tile = {0, 0};
        if (mnavGetTile(navmesh, x, 0, &tile) != mnav_success)
        {
            tile = (mnavTileId){0, 0};
        }
        sum = (sum * 31u + tile.slot) * 31u + tile.generation;
    }
    return sum;
}

static mnavPolygonId Polygon(const mnavNavmesh* navmesh)
{
    mnavNearest nearest;
    CHECK(mnavFindNearest(navmesh, nullptr, s_links[0].start, (mnavVec3){1.0f, 2.0f, 1.0f},
                          &nearest) == mnav_success &&
              nearest.polygon.slot != 0,
          "a polygon under the link");
    return nearest.polygon;
}

// A tile staged over a committed one and committed, on the navmesh with
// no call back in and on the reentrant one: the commit allocates the new
// tile's tables and frees the old tile. Stops where the call back in came.
static mnavResult Restage(mnavNavmesh* plain, mnavNavmesh* navmesh, mnavResult* expected)
{
    *expected = mnavStageTile(plain, s_trench0, sizeof(s_trench0)).result;
    mnavResult got = mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result;
    if (got == mnav_success && s_reentry.made == 0)
    {
        *expected = *expected == mnav_success ? mnavCommit(plain) : *expected;
        got = mnavCommit(navmesh);
    }
    return got;
}

// One outer call on two navmeshes, the reentrant one's allocator calling
// back in at the first allocation or release: the same result and tiles,
// every call back in refused, each a misuse.
static void TestAllocator(bool onAlloc)
{
    mnavLinkId plainLink;
    mnavLinkId link;
    mnavNavmesh* plain = Load(false, &plainLink);
    mnavNavmesh* navmesh = Load(true, &link);
    s_reentry = (Reentry){navmesh, nullptr, link, Polygon(navmesh), onAlloc, !onAlloc, 0, 0, 0};
    uint64_t before = mnavGetNavmeshMisuse(navmesh);
    mnavResult expected = mnav_success;
    mnavResult got = Restage(plain, navmesh, &expected);
    CHECK(s_reentry.made == NAVMESH_CALLS, "called back in");
    CHECK(s_reentry.busy == NAVMESH_CALLS - 1, "every call refused as busy");
    CHECK(mnavGetNavmeshMisuse(navmesh) - before == NAVMESH_CALLS, "each a misuse");
    CHECK(got == expected && Tiles(navmesh) == Tiles(plain), "the outer call's result");
    // Out of the host's code, the navmesh takes changes again.
    s_reentry.navmesh = nullptr;
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "changes after");
    CHECK(mnavGetNavmeshMisuse(navmesh) - before == NAVMESH_CALLS, "no more misuse");
    mnavDestroyNavmesh(navmesh);
    mnavDestroyNavmesh(plain);
    CHECK(s_held == 0, "every byte given back");
}

static int32_t s_tests;
static bool s_reenter;

// Passes every other pair, calling back in at the first test.
static bool Clearance(void* context, mnavPos3 from, mnavPos3 to)
{
    (void)context;
    (void)from;
    (void)to;
    if (s_reenter)
    {
        s_reenter = false;
        CallBack();
    }
    return (s_tests++ & 1) == 0;
}

// Link generation, its clearance test calling back into the navmesh and
// the generating context: the same links as with no call back in. The two
// staging calls succeed with no misuse, staging what the next commit
// applies; every other call is refused.
static void TestClearance(void)
{
    mnavLinkId link;
    mnavNavmesh* navmesh = Load(false, &link);
    mnavQuery* query = Query();
    mnavLinkGenDef def = Def();
    def.clear = Clearance;
    int32_t expected = 0;
    s_tests = 0;
    s_reenter = false;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, &(mnavTileRange){0, 0, 1, 0}, s_again,
                            LINKS, &expected) == mnav_success,
          "links with a clearance test");
    s_reentry = (Reentry){navmesh, query, link, Polygon(navmesh), false, false, 0, 0, 0};
    uint64_t tiles = Tiles(navmesh);
    uint64_t navmeshBefore = mnavGetNavmeshMisuse(navmesh);
    uint64_t queryBefore = mnavGetQueryMisuse(query);
    int32_t count = 0;
    s_tests = 0;
    s_reenter = true;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, &(mnavTileRange){0, 0, 1, 0}, s_links,
                            LINKS, &count) == mnav_success &&
              count == expected &&
              memcmp(s_links, s_again, (size_t)count * sizeof(s_links[0])) == 0,
          "the same links");
    CHECK(s_reentry.made == NAVMESH_CALLS + QUERY_CALLS, "called back in");
    CHECK(s_reentry.succeeded == STAGING_CALLS, "staging allowed");
    CHECK(s_reentry.busy == NAVMESH_CALLS - STAGING_CALLS - 1 + QUERY_CALLS - 1,
          "every other call refused as busy");
    CHECK(mnavGetNavmeshMisuse(navmesh) - navmeshBefore == NAVMESH_CALLS - STAGING_CALLS &&
              mnavGetQueryMisuse(query) - queryBefore == QUERY_CALLS,
          "each a misuse, on the object it was made against");
    // What was staged waits for the commit: the removal, staged last.
    s_reentry.navmesh = nullptr;
    s_reentry.query = nullptr;
    mnavTileId tile;
    CHECK(Tiles(navmesh) == tiles && mnavCommit(navmesh) == mnav_success &&
              mnavGetTile(navmesh, 0, 0, &tile) == mnav_errorNotLoaded &&
              mnavGetTile(navmesh, 1, 0, &tile) == mnav_success,
          "staged, then committed");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static mnavQuery* s_own;
static int32_t s_read;

// Reads the navmesh it was given with a context of its own: a ray from
// each end to itself.
static bool Reading(void* context, mnavPos3 from, mnavPos3 to)
{
    const mnavNavmesh* navmesh = context;
    mnavNearest nearest;
    mnavRay ray;
    if (mnavFindNearest(navmesh, nullptr, from, (mnavVec3){1.0f, 2.0f, 1.0f}, &nearest) ==
            mnav_success &&
        nearest.polygon.slot != 0 &&
        mnavRaycast(s_own, navmesh, nullptr, nearest.polygon, nearest.point, nearest.point, &ray) ==
            mnav_success)
    {
        s_read += 1;
    }
    (void)to;
    return true;
}

// What the overview allows: reading the navmesh with a context of one's
// own, counting no misuse.
static void TestClearanceReads(void)
{
    mnavLinkId link;
    mnavNavmesh* navmesh = Load(false, &link);
    mnavQuery* query = Query();
    s_own = Query();
    mnavLinkGenDef def = Def();
    def.clear = Reading;
    def.context = navmesh;
    int32_t count = 0;
    s_read = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, &(mnavTileRange){0, 0, 1, 0}, s_links,
                            LINKS, &count) == mnav_success &&
              count > 0 && s_read > 0,
          "read from the test");
    CHECK(mnavGetNavmeshMisuse(navmesh) == 0 && mnavGetQueryMisuse(query) == 0 &&
              mnavGetQueryMisuse(s_own) == 0,
          "no misuse");
    mnavDestroyQuery(s_own);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestNames(void)
{
    CHECK(mnav_errorBusy == -10 && strcmp(mnavResultName(mnav_errorBusy), "mnav_errorBusy") == 0,
          "the family's value");
    CHECK(mnavGetNavmeshMisuse(nullptr) == 0 && mnavGetQueryMisuse(nullptr) == 0, "NULL");
}

int main(void)
{
    CHECK(Generate() > 0, "links");
    TestNames();
    TestAllocator(true);
    TestAllocator(false);
    TestClearance();
    TestClearanceReads();
    return s_failures == 0 ? 0 : 1;
}
