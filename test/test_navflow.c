// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields over the navmesh (mnav-0013) on the test world: every way
// leads to a goal down falling costs, each cost is its next polygon's
// plus the walk between their places, links are taken one way, and the
// checks; the field pinned.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hashes of every polygon's way, in 32 m tiles and in 8 m tiles, the
// same on every platform.
#define FIELD_HASH       0x0c04e5559539df51ull
#define SMALL_FIELD_HASH 0x73b33f6c52b4cce4ull

enum
{
    SMALL_ROOM = 1 << 16
};

static uint8_t s_small[100][SMALL_ROOM];
static size_t s_smallSizes[100];

static mnavNavmesh* Load(void)
{
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

// The world in tiles of 8 m: tiles -1 to 8 along each side.
static mnavNavmesh* LoadSmall(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    mnavBaker* baker = nullptr;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateNavmesh(&def, &navmesh).result == mnav_success,
          "baker and navmesh");
    mnavTriangleMesh world = World();
    for (int32_t t = 0; t < 100; ++t)
    {
        size_t* size = &s_smallSizes[t];
        CHECK(mnavBakeTile(baker, &world, 1, t % 10 - 1, t / 10 - 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_small[t], SMALL_ROOM, size) == mnav_success &&
                  mnavStageTile(navmesh, s_small[t], *size).result == mnav_success,
              "a small tile");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavDestroyBaker(baker);
    return navmesh;
}

static mnavNavFlowGoal Goal(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n = {0};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0, z}, (mnavVec3){1, 2, 1}, &n) ==
                  mnav_success &&
              n.polygon.slot != 0,
          "a goal");
    return (mnavNavFlowGoal){n.polygon, n.point};
}

// Every polygon id of the navmesh, in slot and index order.
static int32_t Polygons(const mnavNavmesh* navmesh, mnavPolygonId* out, int32_t capacity)
{
    mnavPolygonId ids[4096];
    mnavFound found;
    CHECK(mnavFindPolygons(navmesh, nullptr, (mnavPos3){32, 0, 32}, (mnavVec3){40, 10, 40}, ids,
                           4096, &found) == mnav_success &&
              found.count <= 4096,
          "listed");
    int32_t n = found.count < capacity ? found.count : capacity;
    memcpy(out, ids, (size_t)n * sizeof(mnavPolygonId));
    return n;
}

static bool Same(mnavPolygonId a, mnavPolygonId b)
{
    return a.slot == b.slot && a.generation == b.generation && a.polygon == b.polygon;
}

static mnavPos3 Middle(const mnavPolygonFlow* f)
{
    return (mnavPos3){(f->left.x + f->right.x) * 0.5, (f->left.y + f->right.y) * 0.5,
                      (f->left.z + f->right.z) * 0.5};
}

static double Distance(mnavPos3 a, mnavPos3 b)
{
    return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
}

static void TestWays(mnavNavFlow* field, const mnavNavmesh* navmesh, uint64_t pinned)
{
    const mnavNavFlowGoal goals[2] = {Goal(navmesh, 4, 4), Goal(navmesh, 60, 50)};
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, goals, 2) == mnav_success, "built");
    static mnavPolygonId ids[4096];
    int32_t count = Polygons(navmesh, ids, 4096);
    int32_t reached = 0;
    int32_t chained = 0;
    int32_t exact = 0;
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < count; ++i)
    {
        mnavPolygonFlow f;
        CHECK(mnavNavFlowAt(field, navmesh, ids[i], &f) == mnav_success, "read");
        // Field by field: the struct has padding.
        hash = mnavHash64(hash, &f.cost, (int32_t)sizeof(f.cost));
        hash = mnavHash64(hash, &f.next, (int32_t)sizeof(f.next));
        hash = mnavHash64(hash, &f.left, (int32_t)sizeof(f.left));
        hash = mnavHash64(hash, &f.right, (int32_t)sizeof(f.right));
        hash = mnavHash64(hash, &f.link, (int32_t)sizeof(f.link));
        if (!isfinite(f.cost))
        {
            continue;
        }
        reached += 1;
        // Down the chain to a goal, costs falling all the way.
        mnavPolygonId at = ids[i];
        mnavPolygonFlow step = f;
        int32_t hops = 0;
        bool falling = true;
        while (step.next.slot != 0 && hops < count)
        {
            mnavPolygonFlow after;
            CHECK(mnavNavFlowAt(field, navmesh, step.next, &after) == mnav_success, "read on");
            falling = falling && after.cost < step.cost;
            at = step.next;
            step = after;
            hops += 1;
        }
        chained += falling && step.cost == 0.0 &&
                           (Same(at, goals[0].polygon) || Same(at, goals[1].polygon))
                       ? 1
                       : 0;
        // The cost rebuilt from the next polygon's: the walk between their
        // places, priced by the next polygon's area cost of 1.
        if (f.next.slot != 0)
        {
            mnavPolygonFlow next;
            CHECK(mnavNavFlowAt(field, navmesh, f.next, &next) == mnav_success, "next");
            mnavPos3 there = next.cost == 0.0 ? next.left : Middle(&next);
            exact += next.cost + Distance(Middle(&f), there) == f.cost ? 1 : 0;
        }
        else
        {
            exact += f.cost == 0.0 ? 1 : 0;
        }
    }
    printf("navflow: %d polygons, %d reached; FIELD_HASH=%016llx\n", count, reached,
           (unsigned long long)hash);
    CHECK(reached > count / 2 && chained == reached && exact == reached,
          "every way down to a goal, each cost its next's plus the walk");
    CHECK(hash == pinned, "the pinned hash");
}

static void TestLinks(mnavNavFlow* field, mnavNavmesh* navmesh)
{
    // A one-way teleport from near (60, 60) to the goal's corner, cheap.
    const mnavLinkDef def = {{60, 0, 60}, {6, 0, 6}, 1.0f, 1.0f, mnav_linkTeleport, false, 0.0f};
    mnavLinkId link;
    CHECK(mnavStageLink(navmesh, &def, &link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "linked");
    const mnavNavFlowGoal goal = Goal(navmesh, 4, 4);
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, &goal, 1) == mnav_success, "built");
    mnavNavFlowGoal far = Goal(navmesh, 60, 60);
    mnavPolygonFlow f;
    CHECK(mnavNavFlowAt(field, navmesh, far.polygon, &f) == mnav_success &&
              f.link.slot == link.slot && f.link.generation == link.generation &&
              f.left.x == f.right.x && f.cost < 20.0,
          "the far corner's way takes the link");
    // Its cost: the landing polygon's, the walk from the landing point to
    // that polygon's place, and the link's cost of 1.
    mnavLinkState state;
    mnavPolygonFlow landing;
    CHECK(mnavGetLink(navmesh, link, &state) == mnav_success &&
              mnavNavFlowAt(field, navmesh, f.next, &landing) == mnav_success &&
              f.cost ==
                  landing.cost +
                      Distance(state.end, landing.cost == 0.0 ? landing.left : Middle(&landing)) +
                      1.0,
          "the link's cost added");
    // Toward the far corner, the link is never taken backward: the way is
    // the one a filter barring it gives.
    mnavQueryFilter barred = mnavDefaultQueryFilter();
    barred.kinds &= ~((uint64_t)1 << mnav_linkTeleport);
    mnavPolygonFlow without;
    CHECK(mnavBuildNavFlow(field, navmesh, &barred, &far, 1) == mnav_success &&
              mnavNavFlowAt(field, navmesh, goal.polygon, &without) == mnav_success &&
              mnavBuildNavFlow(field, navmesh, nullptr, &far, 1) == mnav_success &&
              mnavNavFlowAt(field, navmesh, goal.polygon, &f) == mnav_success && f.link.slot == 0 &&
              f.cost == without.cost && Same(f.next, without.next),
          "one way only");
    // A filter barring the kind leaves it out.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.kinds &= ~((uint64_t)1 << mnav_linkTeleport);
    CHECK(mnavBuildNavFlow(field, navmesh, &filter, &goal, 1) == mnav_success &&
              mnavNavFlowAt(field, navmesh, far.polygon, &f) == mnav_success && f.link.slot == 0,
          "a kind barred");
    // A commit makes the field stale.
    CHECK(mnavStageLinkRemoval(navmesh, link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavNavFlowAt(field, navmesh, far.polygon, &f) == mnav_errorStale,
          "stale after a commit");
}

// A strip of area 5 across the world, from x 16 to 24, two tile sides,
// marked by a volume, in 8 m tiles; a link taking off inside it.
static void TestAreas(mnavNavFlow* field)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    mnavBaker* baker = nullptr;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateNavmesh(&def, &navmesh).result == mnav_success,
          "baker and navmesh");
    mnavTriangleMesh world = World();
    const mnavVec2 strip[4] = {{16, -2}, {24, -2}, {24, 70}, {16, 70}};
    const mnavBakeVolume volume = {strip, 4, -1.0f, 1.0f, mnav_volumeArea, 5};
    const mnavBakeInput input = {&world, 1, nullptr, 0, &volume, 1};
    for (int32_t t = 0; t < 100; ++t)
    {
        size_t size = 0;
        CHECK(mnavBakeTileInput(baker, &input, t % 10 - 1, t / 10 - 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_small[t], SMALL_ROOM, &size) == mnav_success &&
                  mnavStageTile(navmesh, s_small[t], size).result == mnav_success,
              "a small tile");
    }
    const mnavLinkDef jump = {{20, 0, 40}, {5, 0, 40}, 1.0f, 1.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId link;
    CHECK(mnavStageLink(navmesh, &jump, &link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    mnavDestroyBaker(baker);
    const mnavNavFlowGoal goal = Goal(navmesh, 4, 40);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 5);
    CHECK(mnavBuildNavFlow(field, navmesh, &filter, &goal, 1) == mnav_success, "built");
    static mnavPolygonId ids[4096];
    int32_t count = Polygons(navmesh, ids, 4096);
    int32_t stripped = 0;
    int32_t beyond = 0;
    int32_t wrong = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        mnavAreaType area = 0;
        mnavPolygonFlow f;
        CHECK(mnavGetArea(navmesh, ids[i], &area) == mnav_success &&
                  mnavNavFlowAt(field, navmesh, ids[i], &f) == mnav_success,
              "read");
        mnavPos3 m = Middle(&f);
        stripped += area == 5 ? 1 : 0;
        // Left out, the strip is reached by nothing and cuts the far side
        // off, the link out of it with it.
        wrong += area == 5 && isfinite(f.cost) ? 1 : 0;
        wrong += area != 5 && isfinite(f.cost) && m.x > 24.0 ? 1 : 0;
        beyond += area != 5 && !isfinite(f.cost) ? 1 : 0;
    }
    printf("navflow areas: %d polygons, %d in the strip, %d cut off\n", count, stripped, beyond);
    CHECK(stripped > 0 && beyond > 0 && wrong == 0, "an area left out is crossed by nothing");
    mnavDestroyNavmesh(navmesh);
}

// Every way over the tiles of a box of places, field by field, polygon by
// polygon: the next polygon by its index alone, so that two navmeshes
// holding the same tiles in other slots compare.
static uint64_t TilesHash(const mnavNavFlow* field, const mnavNavmesh* navmesh, int32_t x0,
                          int32_t z0, int32_t x1, int32_t z1)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t z = z0; z <= z1; ++z)
    {
        for (int32_t x = x0; x <= x1; ++x)
        {
            mnavTileId tile;
            CHECK(mnavGetTile(navmesh, x, z, &tile) == mnav_success, "a tile");
            for (uint32_t i = 0;; ++i)
            {
                mnavPolygonId id = {tile.slot, tile.generation, i};
                mnavAreaType area = 0;
                mnavPolygonFlow f;
                if (mnavGetArea(navmesh, id, &area) != mnav_success)
                {
                    break;
                }
                CHECK(mnavNavFlowAt(field, navmesh, id, &f) == mnav_success, "read");
                hash = mnavHash64(hash, &f.cost, (int32_t)sizeof(f.cost));
                hash = mnavHash64(hash, &f.next.polygon, (int32_t)sizeof(f.next.polygon));
                hash = mnavHash64(hash, &f.left, (int32_t)sizeof(f.left));
                hash = mnavHash64(hash, &f.right, (int32_t)sizeof(f.right));
            }
        }
    }
    return hash;
}

static void TestStepsAndRegions(mnavNavFlow* field)
{
    mnavNavmesh* navmesh = LoadSmall();
    const mnavNavFlowGoal goals[2] = {Goal(navmesh, 18, 30), Goal(navmesh, 60, 6)};
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, goals, 2) == mnav_success, "built");
    uint64_t whole = TilesHash(field, navmesh, -1, -1, 8, 8);
    const int32_t budgets[3] = {1, 7, 64};
    for (int32_t b = 0; b < 3; ++b)
    {
        mnavPolygonFlow f;
        bool ended = false;
        CHECK(mnavBeginNavFlow(field, navmesh, nullptr, nullptr, goals, 2) == mnav_success &&
                  mnavNavFlowAt(field, navmesh, goals[0].polygon, &f) == mnav_errorStale,
              "no reads while working");
        while (!ended)
        {
            CHECK(mnavContinueNavFlow(field, navmesh, budgets[b], &ended) == mnav_success, "step");
        }
        CHECK(TilesHash(field, navmesh, -1, -1, 8, 8) == whole, "the build's field");
    }
    bool ended = false;
    CHECK(mnavContinueNavFlow(field, navmesh, 1, &ended) == mnav_success && ended &&
              mnavContinueNavFlow(field, navmesh, 0, &ended) == mnav_errorInvalid &&
              mnavContinueNavFlow(field, nullptr, 1, &ended) == mnav_errorInvalid,
          "continuing an ended field; the checks");
    // A region's field is the field of a navmesh holding its tiles alone,
    // also with a link from inside it to a tile outside of a lower slot,
    // which a region must not take for one of its own polygons.
    const mnavLinkDef out = {{38, 0, 46}, {20, 0, 4}, 1.0f, 0.0f, mnav_linkDrop, false, 0.0f};
    mnavLinkId outLink;
    CHECK(mnavStageLink(navmesh, &out, &outLink) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a link out of the region");
    const mnavNavFlowRegion region = {1, 2, 4, 5};
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    mnavNavmesh* part = nullptr;
    CHECK(mnavCreateNavmesh(&def, &part).result == mnav_success, "a part");
    for (int32_t z = region.z0; z <= region.z1; ++z)
    {
        for (int32_t x = region.x0; x <= region.x1; ++x)
        {
            int32_t t = (z + 1) * 10 + (x + 1);
            CHECK(mnavStageTile(part, s_small[t], s_smallSizes[t]).result == mnav_success,
                  "staged");
        }
    }
    CHECK(mnavCommit(part) == mnav_success, "committed");
    mnavNavFlowDef small = mnavDefaultNavFlowDef();
    small.polygons = 40;
    mnavNavFlow* alone = nullptr;
    const mnavNavFlowGoal partGoal = Goal(part, 10, 18);
    const mnavNavFlowGoal near[2] = {Goal(navmesh, 10, 18), goals[1]};
    CHECK(mnavCreateNavFlow(&small, &alone) == mnav_success &&
              mnavBuildNavFlow(alone, part, nullptr, &partGoal, 1) == mnav_success,
          "the part's field");
    mnavNavFlow* regional = nullptr;
    CHECK(mnavCreateNavFlow(&small, &regional) == mnav_success &&
              mnavBuildNavFlow(regional, navmesh, nullptr, goals, 2) == mnav_errorLimit &&
              mnavBeginNavFlow(regional, navmesh, nullptr, &region, near, 2) == mnav_success &&
              mnavContinueNavFlow(regional, navmesh, INT32_MAX, &ended) == mnav_success && ended,
          "a region within the limit the whole navmesh passes, the far goal left out");
    CHECK(TilesHash(regional, navmesh, region.x0, region.z0, region.x1, region.z1) ==
              TilesHash(alone, part, region.x0, region.z0, region.x1, region.z1),
          "the region's own field");
    mnavPolygonFlow f;
    CHECK(mnavNavFlowAt(regional, navmesh, goals[1].polygon, &f) == mnav_errorInvalid,
          "a polygon outside the region");
    const mnavNavFlowRegion backward = {4, 2, 1, 5};
    CHECK(mnavBeginNavFlow(regional, navmesh, nullptr, &backward, goals, 2) == mnav_errorInvalid,
          "a region backward");
    // A commit while working: the work is stale.
    const mnavLinkDef jump = {{5, 0, 5}, {9, 0, 5}, 1.0f, 1.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId link;
    CHECK(mnavBeginNavFlow(field, navmesh, nullptr, nullptr, goals, 2) == mnav_success &&
              mnavStageLink(navmesh, &jump, &link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavContinueNavFlow(field, navmesh, 1, &ended) == mnav_errorStale,
          "a commit while working");
    mnavDestroyNavFlow(regional);
    mnavDestroyNavFlow(alone);
    mnavDestroyNavmesh(part);
    mnavDestroyNavmesh(navmesh);
}

static void TestChecks(mnavNavFlow* field, const mnavNavmesh* navmesh)
{
    mnavPolygonFlow f;
    mnavNavFlowGoal goal = Goal(navmesh, 4, 4);
    mnavNavFlowGoal bad = goal;
    bad.point.x = (double)NAN;
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, &bad, 1) == mnav_errorInvalid &&
              mnavNavFlowAt(field, navmesh, goal.polygon, &f) == mnav_errorInvalid,
          "a goal not finite; nothing held after");
    bad = goal;
    bad.polygon.slot = 999;
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, &bad, 1) == mnav_errorInvalid,
          "a polygon never handed out");
    bad = goal;
    bad.polygon.generation += 1;
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, &bad, 1) == mnav_errorInvalid ||
              mnavBuildNavFlow(field, navmesh, nullptr, &bad, 1) == mnav_errorStale,
          "a generation not current");
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, nullptr, 1) == mnav_errorInvalid &&
              mnavBuildNavFlow(field, navmesh, nullptr, &goal, -1) == mnav_errorInvalid &&
              mnavBuildNavFlow(field, nullptr, nullptr, &goal, 1) == mnav_errorInvalid &&
              mnavBuildNavFlow(nullptr, navmesh, nullptr, &goal, 1) == mnav_errorInvalid,
          "bad arguments");
    // No goals: nothing reached.
    CHECK(mnavBuildNavFlow(field, navmesh, nullptr, nullptr, 0) == mnav_success &&
              mnavNavFlowAt(field, navmesh, goal.polygon, &f) == mnav_success &&
              !isfinite(f.cost) && f.next.slot == 0,
          "no goals");
    // A goal on an area left out is left out.
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << mnav_areaWalkable);
    CHECK(mnavBuildNavFlow(field, navmesh, &filter, &goal, 1) == mnav_success &&
              mnavNavFlowAt(field, navmesh, goal.polygon, &f) == mnav_success && !isfinite(f.cost),
          "a goal left out");
    // Too few polygons for the navmesh.
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    def.polygons = 4;
    mnavNavFlow* small = nullptr;
    CHECK(mnavCreateNavFlow(&def, &small) == mnav_success &&
              mnavBuildNavFlow(small, navmesh, nullptr, &goal, 1) == mnav_errorLimit,
          "past the polygon limit");
    mnavDestroyNavFlow(small);
    def.polygons = 0;
    CHECK(mnavCreateNavFlow(&def, &small) == mnav_errorRange && small == nullptr, "no polygons");
    def = mnavDefaultNavFlowDef();
    def.cookie = 0;
    CHECK(mnavCreateNavFlow(&def, &small) == mnav_errorInvalid, "not a def");
    mnavDestroyNavFlow(nullptr);
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    mnavNavFlow* field = nullptr;
    CHECK(mnavCreateNavFlow(&def, &field) == mnav_success, "field");
    TestWays(field, navmesh, FIELD_HASH);
    mnavNavmesh* small = LoadSmall();
    TestWays(field, small, SMALL_FIELD_HASH);
    mnavDestroyNavmesh(small);
    TestChecks(field, navmesh);
    TestLinks(field, navmesh);
    TestAreas(field);
    TestStepsAndRegions(field);
    mnavDestroyNavFlow(field);
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
