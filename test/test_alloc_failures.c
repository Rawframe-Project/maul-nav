// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Every allocation the library makes may fail. An allocator that refuses
// its k-th call, for every k an operation reaches, shows that each failure
// is reported as mnav_errorCapacity, that whatever the operation took is
// given back, and that the object it worked on is as it was: a baker bakes
// the same bytes next time, a commit that failed left the navmesh's last
// state, and every made object is destroyed with nothing held.

#include "counting_allocator.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>
#include <string.h>

// The calls left before one fails, or -1 for none; and the calls made.
static int32_t s_left = -1;
static int32_t s_calls;

static void* Failing(size_t size, size_t alignment, void* context)
{
    s_calls += 1;
    if (s_left == 0)
    {
        return nullptr;
    }
    s_left -= s_left > 0 ? 1 : 0;
    return CountingAlloc(size, alignment, context);
}

static mnavAllocator Allocator(void)
{
    return (mnavAllocator){Failing, CountingFree, nullptr};
}

// Lets k calls through, then fails one; -1 for none.
static void FailAfter(int32_t k)
{
    s_left = k;
    s_calls = 0;
}

static uint8_t s_expected[4][TILE_CAPACITY];
static size_t s_expectedSizes[4];
static uint8_t s_bytes[TILE_CAPACITY];
static uint8_t s_rebuilt[TILE_CAPACITY];

// A terrain under the world's floor and a volume marking an area, so that
// every bake stage and input path allocates.
static const float s_heights[9] = {-0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f};
static const mnavVec2 s_strip[4] = {{10, 10}, {20, 10}, {20, 50}, {10, 50}};

static const mnavVec2 s_all[4] = {{-1, -1}, {65, -1}, {65, 65}, {-1, 65}};

static mnavBakeInput Input(const mnavTriangleMesh* world)
{
    static mnavTerrain terrain;
    static mnavBakeVolume volumes[2];
    terrain = (mnavTerrain){{0.0, 0.0, 0.0}, 32.0f, 32.0f, 3, 3, s_heights, nullptr};
    volumes[0] = (mnavBakeVolume){s_strip, 4, -1.0f, 1.0f, mnav_volumeArea, 5};
    volumes[1] = (mnavBakeVolume){s_all, 4, -1.0f, 1.0f, mnav_volumeInclude, 0};
    return (mnavBakeInput){world, 1, &terrain, 1, volumes, 2, nullptr};
}

static void TestBakes(void)
{
    mnavTriangleMesh world = World();
    mnavBakeInput input = Input(&world);
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    FailAfter(-1);
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "a baker");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavBakeTileInput(baker, &input, t % 2, t / 2, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_expected[t], TILE_CAPACITY, &s_expectedSizes[t]) ==
                      mnav_success,
              "the tiles, with every allocation granted");
    }
    size_t held = s_held;
    int32_t refused = 0;
    bool reported = true;
    bool restored = true;
    for (int32_t t = 0; t < 4; ++t)
    {
        FailAfter(-1);
        CHECK(mnavBakeTileInput(baker, &input, t % 2, t / 2, nullptr) == mnav_success, "counted");
        int32_t calls = s_calls;
        for (int32_t k = 0; k < calls; ++k)
        {
            FailAfter(k);
            mnavBakeReport report;
            mnavResult result = mnavBakeTileInput(baker, &input, t % 2, t / 2, &report);
            refused += result == mnav_errorCapacity ? 1 : 0;
            reported = reported && result == mnav_errorCapacity && report.result == result;
            // The baker holds no more than its own scratch afterwards, and
            // bakes the tile again when the allocator grants all.
            FailAfter(-1);
            size_t size = 0;
            restored = restored &&
                       mnavBakeTileInput(baker, &input, t % 2, t / 2, nullptr) == mnav_success &&
                       mnavCopyBakedTile(baker, s_bytes, TILE_CAPACITY, &size) == mnav_success &&
                       size == s_expectedSizes[t] && memcmp(s_bytes, s_expected[t], size) == 0;
        }
    }
    printf("bakes: %d allocation failures refused\n", refused);
    CHECK(refused > 100 && reported, "every failure a capacity error, in the report too");
    CHECK(restored && s_held >= held, "the baker bakes the same bytes after each");
    // The 2D bake's paths too.
    const mnavVec2 square[4] = {{1, 1}, {31, 1}, {31, 31}, {1, 31}};
    const mnavOutline outline = {square, 4, mnav_areaWalkable};
    FailAfter(-1);
    CHECK(mnavBakeTile2D(baker, &outline, 1, 0, 0, nullptr) == mnav_success, "a 2D tile");
    int32_t calls = s_calls;
    bool flat = true;
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        flat = flat && mnavBakeTile2D(baker, &outline, 1, 0, 0, nullptr) == mnav_errorCapacity;
    }
    CHECK(flat, "2D bakes refused alike");
    FailAfter(-1);
    mnavDestroyBaker(baker);
    CHECK(s_held == 0, "every byte given back");
    // Making a baker.
    bool made = true;
    for (int32_t k = 0; k < 4; ++k)
    {
        FailAfter(k);
        mnavBaker* b = nullptr;
        mnavResult result = mnavCreateBaker(&def, &b).result;
        made = made && (result == mnav_errorCapacity ? b == nullptr && s_held == 0
                                                     : result == mnav_success);
        FailAfter(-1);
        mnavDestroyBaker(b);
    }
    CHECK(made && s_held == 0, "a baker refused whole");
}

// A tile cache's bakes and rebuilds, the baker and the cache each on the
// failing allocator: a failed cached bake keeps nothing for the tile, a
// failed rebuild leaves the cache as it was.
static void TestCache(void)
{
    mnavTriangleMesh world = World();
    mnavBakeInput input = Input(&world);
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    cacheDef.allocator = Allocator();
    FailAfter(-1);
    mnavBaker* baker = nullptr;
    mnavTileCache* cache = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateTileCache(&cacheDef, &cache) == mnav_success,
          "a baker and a cache");
    uint64_t empty = mnavGetTileCacheBytes(cache);
    FailAfter(-1);
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_success, "counted");
    int32_t calls = s_calls;
    bool cached = true;
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        cached = cached &&
                 mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_errorCapacity &&
                 mnavGetTileCacheBytes(cache) == empty;
    }
    CHECK(cached, "cached bakes refused, nothing kept");
    FailAfter(-1);
    const mnavBakeVolume crate = {s_strip, 4, -1.0f, 1.0f, mnav_volumeExclude, 0};
    size_t size = 0;
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_success, "cached");
    FailAfter(-1);
    CHECK(mnavRebuildTile(baker, cache, 0, 0, &crate, 1, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &size) == mnav_success,
          "a rebuild, counted");
    calls = s_calls;
    uint64_t held = mnavGetTileCacheBytes(cache);
    bool rebuilt = true;
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        rebuilt = rebuilt &&
                  mnavRebuildTile(baker, cache, 0, 0, &crate, 1, nullptr) == mnav_errorCapacity;
        FailAfter(-1);
        size_t again = 0;
        rebuilt = rebuilt && mnavGetTileCacheBytes(cache) == held &&
                  mnavRebuildTile(baker, cache, 0, 0, &crate, 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_bytes, TILE_CAPACITY, &again) == mnav_success &&
                  again == size && memcmp(s_bytes, s_rebuilt, size) == 0;
    }
    printf("cache: %d rebuild allocations each refused\n", calls);
    CHECK(calls > 10 && rebuilt, "rebuilds refused, then the same bytes");
    mnavDestroyTileCache(cache);
    mnavDestroyBaker(baker);
    CHECK(s_held == 0, "every byte given back");
}

static mnavResult MakeTileCache(void** out)
{
    mnavTileCacheDef def = mnavDefaultTileCacheDef();
    def.allocator = Allocator();
    return mnavCreateTileCache(&def, (mnavTileCache**)out);
}

static void DestroyTileCache(void* object)
{
    mnavDestroyTileCache(object);
}

// A floor over one tile with a pillar in its middle, which leaves a hole
// in the floor's region that the bake merges into its outline.
static void TestHoles(void)
{
    const mnavVec3 v[12] = {{0, 0, 0},      {0, 0, 32},     {32, 0, 32},    {32, 0, 0},
                            {14, 0, 14},    {18, 0, 14},    {18, 0, 18},    {14, 0, 18},
                            {14, 2.5f, 14}, {18, 2.5f, 14}, {18, 2.5f, 18}, {14, 2.5f, 18}};
    const int32_t i[] = {0, 1, 2,  0, 2,  3, 8, 11, 10, 8, 10, 9, 4, 8,  9, 4, 9, 5,
                         5, 9, 10, 5, 10, 6, 6, 10, 11, 6, 11, 7, 7, 11, 8, 7, 8, 4};
    const mnavTriangleMesh mesh = {v, 12, i, (int32_t)(sizeof(i) / sizeof(i[0]) / 3), nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    FailAfter(-1);
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile(baker, &mesh, 1, 0, 0, &report) == mnav_success && report.polygons > 4,
          "a floor round a pillar");
    // Counted on a baker that has baked before, as every bake below is.
    FailAfter(-1);
    CHECK(mnavBakeTile(baker, &mesh, 1, 0, 0, nullptr) == mnav_success, "again");
    int32_t calls = s_calls;
    bool refused = true;
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        refused = refused && mnavBakeTile(baker, &mesh, 1, 0, 0, nullptr) == mnav_errorCapacity;
    }
    CHECK(refused, "every failure round the hole refused");
    FailAfter(-1);
    mnavDestroyBaker(baker);
    CHECK(s_held == 0, "nothing held");
}

// Stages the four tiles and a two-way link, and commits; committing
// receives whether the commit was reached.
static mnavResult Load(mnavNavmesh* navmesh, bool* committing)
{
    *committing = false;
    for (int32_t t = 0; t < 4; ++t)
    {
        mnavResult result = mnavStageTile(navmesh, s_expected[t], s_expectedSizes[t]).result;
        if (result != mnav_success)
        {
            return result;
        }
    }
    const mnavLinkDef link = {{10, 0, 10}, {50, 0, 50}, 1.0f, 1.0f, mnav_linkJump, true, 0.0f};
    mnavLinkId id;
    mnavResult result = mnavStageLink(navmesh, &link, &id);
    *committing = result == mnav_success;
    return result == mnav_success ? mnavCommit(navmesh) : result;
}

static void TestCommits(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    def.tier = mnav_tierDynamic;
    FailAfter(-1);
    mnavNavmesh* navmesh = nullptr;
    bool committing = false;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              Load(navmesh, &committing) == mnav_success,
          "loaded");
    int32_t calls = s_calls;
    mnavDestroyNavmesh(navmesh);
    CHECK(s_held == 0, "given back");
    int32_t refused = 0;
    int32_t inCommit = 0;
    bool clean = true;
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        navmesh = nullptr;
        mnavResult made = mnavCreateNavmesh(&def, &navmesh).result;
        mnavResult result = made == mnav_success ? Load(navmesh, &committing) : made;
        refused += result == mnav_errorCapacity ? 1 : 0;
        clean = clean && result == mnav_errorCapacity;
        // A refused commit committed nothing: no tile is there, and a retry
        // commits them all.
        FailAfter(-1);
        mnavTileId tile;
        if (navmesh != nullptr && committing)
        {
            inCommit += 1;
            clean = clean && mnavGetTile(navmesh, 0, 0, &tile) != mnav_success &&
                    mnavCommit(navmesh) == mnav_success &&
                    mnavGetTile(navmesh, 1, 1, &tile) == mnav_success;
        }
        mnavDestroyNavmesh(navmesh);
        clean = clean && s_held == 0;
    }
    printf("commits: %d allocation failures refused, %d in the commit\n", refused, inCommit);
    CHECK(refused == calls && inCommit > 0 && clean,
          "each refused, a commit all or nothing, nothing held after");
}

// Makes an object with every allocation granted, then refuses each of its
// allocations in turn.
typedef mnavResult (*Maker)(void** out);
typedef void (*Destroyer)(void* object);

static bool EachRefused(Maker make, Destroyer destroy)
{
    FailAfter(-1);
    void* object = nullptr;
    bool ok = make(&object) == mnav_success;
    int32_t calls = s_calls;
    destroy(object);
    for (int32_t k = 0; k < calls; ++k)
    {
        FailAfter(k);
        object = nullptr;
        ok = ok && make(&object) == mnav_errorCapacity && object == nullptr && s_held == 0;
    }
    FailAfter(-1);
    return ok && calls > 0;
}

static mnavResult MakeQuery(void** out)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.allocator = Allocator();
    return mnavCreateQuery(&def, (mnavQuery**)out);
}

static void DestroyQuery(void* object)
{
    mnavDestroyQuery(object);
}

static mnavResult MakeHierarchy(void** out)
{
    mnavHierarchyDef def = mnavDefaultHierarchyDef();
    def.allocator = Allocator();
    def.limits = (mnavHierarchyLimits){64, 256, 1024};
    return mnavCreateHierarchy(&def, (mnavHierarchy**)out);
}

static void DestroyHierarchy(void* object)
{
    mnavDestroyHierarchy(object);
}

static mnavResult MakeNavFlow(void** out)
{
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    def.allocator = Allocator();
    return mnavCreateNavFlow(&def, (mnavNavFlow**)out);
}

static void DestroyNavFlow(void* object)
{
    mnavDestroyNavFlow(object);
}

static mnavResult MakeFlowField(void** out)
{
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.allocator = Allocator();
    return mnavCreateFlowField(&def, (mnavFlowField**)out);
}

static void DestroyFlowField(void* object)
{
    mnavDestroyFlowField(object);
}

static mnavResult MakeAvoidance(void** out)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.allocator = Allocator();
    return mnavCreateAvoidance(&def, (mnavAvoidance**)out);
}

static void DestroyAvoidance(void* object)
{
    mnavDestroyAvoidance(object);
}

static mnavResult MakeTileIndex(void** out)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    mnavTriangleMesh world = World();
    return mnavCreateTileIndex(&def, &world, 1, (mnavTileIndex**)out, nullptr);
}

static mnavResult MakeTileIndex2D(void** out)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = Allocator();
    static const mnavVec2 ring[4] = {{1, 1}, {40, 1}, {40, 40}, {1, 40}};
    const mnavOutline outlines[2] = {{ring, 4, mnav_areaWalkable}, {s_strip, 4, mnav_areaNone}};
    return mnavCreateTileIndex2D(&def, outlines, 2, (mnavTileIndex**)out, nullptr);
}

static void DestroyTileIndex(void* object)
{
    mnavDestroyTileIndex(object);
}

static void TestObjects(void)
{
    CHECK(EachRefused(MakeQuery, DestroyQuery), "query contexts");
    CHECK(EachRefused(MakeHierarchy, DestroyHierarchy), "hierarchies");
    CHECK(EachRefused(MakeNavFlow, DestroyNavFlow), "navmesh flow fields");
    CHECK(EachRefused(MakeFlowField, DestroyFlowField), "flow fields");
    CHECK(EachRefused(MakeAvoidance, DestroyAvoidance), "avoidance sets");
    CHECK(EachRefused(MakeTileIndex, DestroyTileIndex), "tile indexes");
    CHECK(EachRefused(MakeTileIndex2D, DestroyTileIndex), "tile indexes of outlines");
    CHECK(EachRefused(MakeTileCache, DestroyTileCache), "tile caches");
}

int main(void)
{
    TestBakes();
    TestCache();
    TestHoles();
    TestCommits();
    TestObjects();
    return s_failures == 0 ? 0 : 1;
}
