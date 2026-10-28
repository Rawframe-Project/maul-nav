// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Every owner object's allocator calling back into the object it serves:
// at each release of the object's destruction, the last one, which gives
// back the object's own block, included; and, for the objects whose
// calls allocate, at the first, middle and last allocation of such a call.
// The baker's clock calls back in too. Each time, every call that would
// change the object is refused with mnav_errorBusy and counted as misuse,
// a refused destroy does nothing, and the outer call gives what it gives
// with no call back in.

#include "counting_allocator.h"
#include "stored_tiles.h"
#include "test_harness.h"
#include "trench.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    BAKER,
    CACHE,
    INDEX,
    HIERARCHY,
    FLOW,
    NAVFLOW,
    AVOIDANCE,
    FLIGHT_BAKER,
    VOLUME,
    NAVMESH,
    QUERY,
    KINDS,
    TILE_MOST = 1 << 20
};

static const char* const s_names[KINDS] = {
    "baker",         "tile cache",   "tile index",    "hierarchy", "flow field",   "nav flow",
    "avoidance set", "flight baker", "flight volume", "navmesh",   "query context"};

// The reentrant allocator's allocations and releases so far; s_held
// (counting_allocator.h) counts the bytes it holds.
static int32_t s_allocs;
static int32_t s_frees;
// Which allocation or release, counted down, calls back in; -1 for none.
static int32_t s_allocAt = -1;
static int32_t s_freeAt = -1;

// The object to call back into, of kind s_kind; NULL for none. The calls
// made back in, those refused with mnav_errorBusy, and the object's misuse
// count after them.
static void* s_object;
static int32_t s_kind;
static int32_t s_made;
static int32_t s_refused;
static uint64_t s_misuse;

// The inputs: the trench, a flight tile of it, and a baker and a flight
// baker with no call back in.
static mnavTriangleMesh s_trench;
static uint8_t s_flight[TILE_MOST];
static size_t s_flightSize;
static uint8_t s_bytes[TILE_MOST];
static mnavBaker* s_plainBaker;

static void Count(mnavResult result)
{
    s_made += 1;
    s_refused += result == mnav_errorBusy ? 1 : 0;
}

static mnavBakeInput Input(void)
{
    return (mnavBakeInput){&s_trench, 1, nullptr, 0, nullptr, 0, nullptr};
}

// The calls that would change an object of each kind, with NULL for every
// other argument: a refusal comes before any other check. Those that return
// nothing, the destroy call last, count as made but not refused.
static void CallBaker(void* object)
{
    mnavBaker* baker = object;
    Count(mnavBakeTile(baker, nullptr, 0, 0, nullptr));
    Count(mnavBakeTile2D(baker, nullptr, 0, 0, nullptr));
    Count(mnavBakeTileCached(baker, nullptr, nullptr, 0, 0, nullptr));
    Count(mnavRebuildTile(baker, nullptr, 0, 0, nullptr, 0, nullptr));
    mnavDestroyBaker(baker);
    s_made += 1;
}

static void CallCache(void* object)
{
    mnavTileCache* cache = object;
    Count(mnavBakeTileCached(nullptr, cache, nullptr, 0, 0, nullptr));
    Count(mnavRebuildTile(nullptr, cache, 0, 0, nullptr, 0, nullptr));
    mnavDropCachedTile(cache, 0, 0);
    mnavDestroyTileCache(cache);
    s_made += 2;
}

static void CallIndex(void* object)
{
    mnavDestroyTileIndex(object);
    s_made += 1;
}

static void CallHierarchy(void* object)
{
    mnavHierarchy* hierarchy = object;
    Count(mnavBuildHierarchy(hierarchy, nullptr, nullptr, nullptr, nullptr));
    Count(mnavUpdateHierarchy(hierarchy, nullptr, nullptr, nullptr));
    mnavDestroyHierarchy(hierarchy);
    s_made += 1;
}

static void CallFlow(void* object)
{
    mnavFlowField* field = object;
    Count(mnavBeginFlowField(field, nullptr, nullptr, nullptr, nullptr, 0));
    Count(mnavContinueFlowField(field, nullptr, 1, nullptr));
    Count(mnavUpdateFlowField(field, nullptr, nullptr, 0, nullptr, 0));
    Count(mnavBuildFlowField(field, nullptr, nullptr, nullptr, 0));
    mnavDestroyFlowField(field);
    s_made += 1;
}

static void CallNavFlow(void* object)
{
    mnavNavFlow* field = object;
    Count(mnavBeginNavFlow(field, nullptr, nullptr, nullptr, nullptr, 0));
    Count(mnavContinueNavFlow(field, nullptr, 1, nullptr));
    Count(mnavBuildNavFlow(field, nullptr, nullptr, nullptr, 0));
    mnavDestroyNavFlow(field);
    s_made += 1;
}

static void CallAvoidance(void* object)
{
    mnavAvoidance* avoidance = object;
    Count(mnavAvoid(avoidance, nullptr, 0, nullptr, 0, 1.0, nullptr));
    Count(mnavAvoid3D(avoidance, nullptr, 0, nullptr, 0, 1.0, nullptr));
    Count(mnavDebugAvoidance(avoidance, nullptr, 0, 0.0, nullptr));
    Count(mnavDebugAvoidance3D(avoidance, nullptr, 0, nullptr, 0, nullptr));
    mnavDestroyAvoidance(avoidance);
    s_made += 1;
}

static void CallFlightBaker(void* object)
{
    mnavFlightBaker* baker = object;
    Count(mnavBakeFlightTile(baker, nullptr, 0, 0, nullptr));
    mnavDestroyFlightBaker(baker);
    s_made += 1;
}

static void CallVolume(void* object)
{
    mnavFlightVolume* volume = object;
    Count(mnavStageFlightTile(volume, nullptr, 0).result);
    Count(mnavStageFlightTileRemoval(volume, 0, 0));
    Count(mnavCommitFlight(volume));
    mnavDestroyFlightVolume(volume);
    s_made += 1;
}

static void CallNavmesh(void* object)
{
    mnavNavmesh* navmesh = object;
    const mnavPolygonId none = {0, 0, 0};
    const mnavLinkId link = {0, 0};
    Count(mnavStageTile(navmesh, nullptr, 0).result);
    Count(mnavStageTileRemoval(navmesh, 0, 0));
    Count(mnavCommit(navmesh));
    Count(mnavStageArea(navmesh, none, mnav_areaWalkable));
    Count(mnavStageLink(navmesh, nullptr, nullptr));
    Count(mnavStageLinkRemoval(navmesh, link));
    Count(mnavStageLinkEnabled(navmesh, link, false));
    mnavDestroyNavmesh(navmesh);
    s_made += 1;
}

// One search call stands for every one: test_busy makes them all.
static void CallQuery(void* object)
{
    mnavQuery* query = object;
    const mnavPolygonId none = {0, 0, 0};
    const mnavPos3 a = {0.0, 0.0, 0.0};
    Count(mnavFindPath(query, nullptr, nullptr, none, a, none, a, nullptr));
    mnavDestroyQuery(query);
    s_made += 1;
}

static void (*const s_calls[KINDS])(void*) = {
    CallBaker,     CallCache,       CallIndex,  CallHierarchy, CallFlow, CallNavFlow,
    CallAvoidance, CallFlightBaker, CallVolume, CallNavmesh,   CallQuery};

// How many calls each kind's call back in makes, and how many of them
// return nothing.
static const int32_t s_callCounts[KINDS] = {5, 4, 1, 3, 5, 4, 5, 2, 4, 8, 2};
static const int32_t s_voidCounts[KINDS] = {1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1};

static uint64_t Misuse(int32_t kind, const void* object)
{
    switch (kind)
    {
    case BAKER:
        return mnavGetBakerMisuse(object);
    case CACHE:
        return mnavGetTileCacheMisuse(object);
    case INDEX:
        return mnavGetTileIndexMisuse(object);
    case HIERARCHY:
        return mnavGetHierarchyMisuse(object);
    case FLOW:
        return mnavGetFlowFieldMisuse(object);
    case NAVFLOW:
        return mnavGetNavFlowMisuse(object);
    case AVOIDANCE:
        return mnavGetAvoidanceMisuse(object);
    case FLIGHT_BAKER:
        return mnavGetFlightBakerMisuse(object);
    case VOLUME:
        return mnavGetFlightVolumeMisuse(object);
    case NAVMESH:
        return mnavGetNavmeshMisuse(object);
    default:
        return mnavGetQueryMisuse(object);
    }
}

// Calls back into the object, once, and reads its misuse count while it
// is still there.
static void CallBack(void)
{
    void* object = s_object;
    if (object != nullptr)
    {
        s_object = nullptr;
        s_calls[s_kind](object);
        s_misuse = Misuse(s_kind, object);
    }
}

// Counts a countdown down: true once, as it passes 0, then -1 for good.
static bool Due(int32_t* at)
{
    if (*at < 0)
    {
        return false;
    }
    *at -= 1;
    return *at < 0;
}

static void* ReentrantAlloc(size_t size, size_t alignment, void* context)
{
    s_allocs += 1;
    if (Due(&s_allocAt))
    {
        CallBack();
    }
    return CountingAlloc(size, alignment, context);
}

static void ReentrantFree(void* block, size_t size, size_t alignment, void* context)
{
    s_frees += 1;
    if (Due(&s_freeAt))
    {
        CallBack();
    }
    CountingFree(block, size, alignment, context);
}

static const mnavAllocator s_reentrant = {ReentrantAlloc, ReentrantFree, nullptr};

// Makes an object of each kind through the reentrant allocator, holding
// something to give back: a baked tile, a cached tile, a committed tile.
static void* MakeBaker(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = s_reentrant;
    mnavBaker* baker = nullptr;
    const mnavBakeInput input = Input();
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile(baker, &input, 0, 0, nullptr) == mnav_success,
          "a baker with a tile");
    return baker;
}

static void* MakeCache(void)
{
    mnavTileCacheDef def = mnavDefaultTileCacheDef();
    def.allocator = s_reentrant;
    def.limits.tiles = 4;
    mnavTileCache* cache = nullptr;
    const mnavBakeInput input = Input();
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_success &&
              mnavBakeTileCached(s_plainBaker, cache, &input, 0, 0, nullptr) == mnav_success,
          "a cache with a tile");
    return cache;
}

static void* MakeIndex(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = s_reentrant;
    mnavTileIndex* index = nullptr;
    CHECK(mnavCreateTileIndex(&def, &s_trench, 1, &index, nullptr) == mnav_success, "an index");
    return index;
}

static void* MakeHierarchy(void)
{
    mnavHierarchyDef def = mnavDefaultHierarchyDef();
    def.allocator = s_reentrant;
    def.limits.tiles = 16;
    def.limits.transitions = 64;
    def.limits.edges = 256;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_success, "a hierarchy");
    return hierarchy;
}

static void* MakeFlow(void)
{
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.allocator = s_reentrant;
    def.cells = 1024;
    mnavFlowField* field = nullptr;
    CHECK(mnavCreateFlowField(&def, &field) == mnav_success, "a flow field");
    return field;
}

static void* MakeNavFlow(void)
{
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    def.allocator = s_reentrant;
    def.limits.polygons = 256;
    def.limits.tiles = 16;
    def.limits.links = 16;
    mnavNavFlow* field = nullptr;
    CHECK(mnavCreateNavFlow(&def, &field) == mnav_success, "a nav flow");
    return field;
}

static void* MakeAvoidance(void)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.allocator = s_reentrant;
    def.limits.agents = 16;
    def.limits.obstacleVertices = 16;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "an avoidance set");
    return avoidance;
}

static mnavFlightDef FlightDef(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.allocator = s_reentrant;
    def.floor = -2.0f;
    return def;
}

static void* MakeFlightBaker(void)
{
    mnavFlightDef def = FlightDef();
    mnavFlightBaker* baker = nullptr;
    const mnavBakeInput input = Input();
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success &&
              mnavBakeFlightTile(baker, &input, 0, 0, nullptr) == mnav_success,
          "a flight baker with a tile");
    return baker;
}

static void* MakeVolume(void)
{
    mnavFlightDef def = FlightDef();
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(&def, &volume).result == mnav_success &&
              mnavStageFlightTile(volume, s_flight, s_flightSize).result == mnav_success &&
              mnavCommitFlight(volume) == mnav_success,
          "a flight volume with a tile");
    return volume;
}

static void* MakeNavmesh(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = s_reentrant;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a navmesh with a tile");
    return navmesh;
}

static void* MakeQuery(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.allocator = s_reentrant;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a context");
    return query;
}

static void* (*const s_makes[KINDS])(void) = {
    MakeBaker,     MakeCache,       MakeIndex,  MakeHierarchy, MakeFlow, MakeNavFlow,
    MakeAvoidance, MakeFlightBaker, MakeVolume, MakeNavmesh,   MakeQuery};

static void Destroy(int32_t kind, void* object)
{
    switch (kind)
    {
    case BAKER:
        mnavDestroyBaker(object);
        break;
    case CACHE:
        mnavDestroyTileCache(object);
        break;
    case INDEX:
        mnavDestroyTileIndex(object);
        break;
    case HIERARCHY:
        mnavDestroyHierarchy(object);
        break;
    case FLOW:
        mnavDestroyFlowField(object);
        break;
    case NAVFLOW:
        mnavDestroyNavFlow(object);
        break;
    case AVOIDANCE:
        mnavDestroyAvoidance(object);
        break;
    case FLIGHT_BAKER:
        mnavDestroyFlightBaker(object);
        break;
    case VOLUME:
        mnavDestroyFlightVolume(object);
        break;
    case NAVMESH:
        mnavDestroyNavmesh(object);
        break;
    default:
        mnavDestroyQuery(object);
        break;
    }
}

// Aims the next call back in at an object, counting from nothing.
static void Aim(int32_t kind, void* object)
{
    s_kind = kind;
    s_object = object;
    s_made = 0;
    s_refused = 0;
    s_misuse = 0;
}

// Whether the call back in came and every call made was refused, each
// counting one misuse on top of what the object had counted, but for
// those allowed: calls that leave what the outer call reads alone.
static bool Refused(int32_t kind, uint64_t before, int32_t allowed)
{
    const int32_t calls = s_callCounts[kind];
    return s_object == nullptr && s_made == calls &&
           s_refused == calls - s_voidCounts[kind] - allowed &&
           s_misuse - before == (uint64_t)(calls - allowed);
}

// An object of each kind destroyed, its allocator calling back in at each
// of its releases in turn: every call refused, every byte given back.
static void TestDestroy(int32_t kind)
{
    void* plain = s_makes[kind]();
    s_frees = 0;
    Destroy(kind, plain);
    const int32_t frees = s_frees;
    CHECK(frees > 0 && s_held == 0, s_names[kind]);
    for (int32_t at = 0; at < frees; ++at)
    {
        void* object = s_makes[kind]();
        Aim(kind, object);
        s_freeAt = at;
        Destroy(kind, object);
        s_freeAt = -1;
        CHECK(Refused(kind, 0, 0), s_names[kind]);
        CHECK(s_held == 0, s_names[kind]);
    }
}

// What an outer call gives: its result and a hash of what it made.
typedef struct Outcome
{
    mnavResult result;
    uint64_t hash;
} Outcome;

static Outcome BakeOuter(void* object)
{
    const mnavBakeInput input = Input();
    mnavBaker* baker = object;
    Outcome out = {mnavBakeTile(baker, &input, 1, 0, nullptr), 0};
    size_t size = 0;
    if (mnavCopyBakedTile(baker, s_bytes, sizeof(s_bytes), &size) == mnav_success)
    {
        out.hash = mnavHash64(0, s_bytes, (int32_t)size);
    }
    return out;
}

static Outcome CacheOuter(void* object)
{
    const mnavBakeInput input = Input();
    mnavTileCache* cache = object;
    Outcome out = {mnavBakeTileCached(s_plainBaker, cache, &input, 1, 0, nullptr),
                   mnavGetTileCacheBytes(cache)};
    return out;
}

static Outcome FlightBakeOuter(void* object)
{
    const mnavBakeInput input = Input();
    mnavFlightBaker* baker = object;
    Outcome out = {mnavBakeFlightTile(baker, &input, 1, 0, nullptr), 0};
    size_t size = 0;
    if (mnavCopyFlightTile(baker, s_bytes, sizeof(s_bytes), &size) == mnav_success)
    {
        out.hash = mnavHash64(0, s_bytes, (int32_t)size);
    }
    return out;
}

// The tile staged again over itself and committed.
static Outcome VolumeOuter(void* object)
{
    mnavFlightVolume* volume = object;
    Outcome out = {mnavStageFlightTile(volume, s_flight, s_flightSize).result, 0};
    if (out.result == mnav_success)
    {
        out.result = mnavCommitFlight(volume);
    }
    if (mnavGetFlightTile(volume, 0, 0, &out.hash) != mnav_success)
    {
        out.hash = 0;
    }
    return out;
}

// An outer call that allocates, on an object whose allocator calls back
// in at its first, middle and last allocation: every call refused, and
// the outer call gives what it gives with no call back in.
static void TestOuter(int32_t kind, Outcome (*outer)(void*))
{
    void* plain = s_makes[kind]();
    s_allocs = 0;
    const Outcome expected = outer(plain);
    const int32_t allocs = s_allocs;
    Destroy(kind, plain);
    CHECK(expected.result == mnav_success && allocs > 0, s_names[kind]);
    const int32_t ats[3] = {0, allocs / 2, allocs - 1};
    for (int32_t k = 0; k < 3; ++k)
    {
        void* object = s_makes[kind]();
        uint64_t before = Misuse(kind, object);
        Aim(kind, object);
        s_allocAt = ats[k];
        const Outcome got = outer(object);
        s_allocAt = -1;
        CHECK(Refused(kind, before, 0), s_names[kind]);
        CHECK(got.result == expected.result && got.hash == expected.hash, s_names[kind]);
        Destroy(kind, object);
        CHECK(s_held == 0, s_names[kind]);
    }
}

static bool s_clockCalls;

static uint64_t Clock(void* context)
{
    (void)context;
    if (s_clockCalls)
    {
        s_clockCalls = false;
        CallBack();
    }
    return 0;
}

// A bake whose clock calls back into the baker: the same tile.
static void TestClock(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = s_reentrant;
    def.clock = (mnavClock){Clock, nullptr};
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "a baker with a clock");
    const Outcome expected = BakeOuter(baker);
    Aim(BAKER, baker);
    s_clockCalls = true;
    const Outcome got = BakeOuter(baker);
    CHECK(Refused(BAKER, 0, 0), "the clock's calls refused");
    CHECK(got.result == expected.result && got.hash == expected.hash, "the same tile");
    mnavDestroyBaker(baker);
    CHECK(s_held == 0, "every byte given back");
}

// A cache and a tile index made with the C library's allocator, which a
// call on another object reads or fills.
static mnavTileCache* s_heldCache;
static mnavTileIndex* s_heldIndex;

static Outcome Hashed(mnavResult result, const mnavBaker* baker)
{
    Outcome out = {result, 0};
    size_t size = 0;
    if (mnavCopyBakedTile(baker, s_bytes, sizeof(s_bytes), &size) == mnav_success)
    {
        out.hash = mnavHash64(0, s_bytes, (int32_t)size);
    }
    return out;
}

static Outcome RebuildOuter(void* object)
{
    return Hashed(mnavRebuildTile(object, s_heldCache, 0, 0, nullptr, 0, nullptr), object);
}

static Outcome CachedOuter(void* object)
{
    const mnavBakeInput input = Input();
    return Hashed(mnavBakeTileCached(object, s_heldCache, &input, 1, 0, nullptr), object);
}

static Outcome IndexedOuter(void* object)
{
    mnavBakeInput input = Input();
    input.index = s_heldIndex;
    return Hashed(mnavBakeTile(object, &input, 1, 0, nullptr), object);
}

static Outcome FlightIndexedOuter(void* object)
{
    mnavBakeInput input = Input();
    input.index = s_heldIndex;
    Outcome out = {mnavBakeFlightTile(object, &input, 1, 0, nullptr), 0};
    size_t size = 0;
    if (mnavCopyFlightTile(object, s_bytes, sizeof(s_bytes), &size) == mnav_success)
    {
        out.hash = mnavHash64(0, s_bytes, (int32_t)size);
    }
    return out;
}

// An outer call on an object whose allocator calls back, at its first,
// middle and last allocation, into another object the call reads or
// fills: every call refused, the other object as it was, and the outer
// call gives what it gives with no call back in.
static void TestAcross(int32_t kind, int32_t targetKind, void* target, Outcome (*outer)(void*),
                       int32_t allowed)
{
    void* plain = s_makes[kind]();
    s_allocs = 0;
    const Outcome expected = outer(plain);
    const int32_t allocs = s_allocs;
    Destroy(kind, plain);
    CHECK(expected.result == mnav_success && allocs > 0, s_names[targetKind]);
    const int32_t ats[3] = {0, allocs / 2, allocs - 1};
    for (int32_t k = 0; k < 3; ++k)
    {
        void* object = s_makes[kind]();
        uint64_t before = Misuse(targetKind, target);
        Aim(targetKind, target);
        s_allocAt = ats[k];
        const Outcome got = outer(object);
        s_allocAt = -1;
        CHECK(Refused(targetKind, before, allowed), s_names[targetKind]);
        CHECK(got.result == expected.result && got.hash == expected.hash, s_names[targetKind]);
        Destroy(kind, object);
        CHECK(s_held == 0, s_names[targetKind]);
    }
}

// A context and a navmesh with the trench, made with the C library's
// allocator, which a hierarchy's build uses.
static mnavQuery* s_heldQuery;
static mnavNavmesh* s_heldNavmesh;

static Outcome BuildOuter(void* object)
{
    mnavHierarchyReport report = {0};
    Outcome out = {mnavBuildHierarchy(object, s_heldQuery, s_heldNavmesh, nullptr, &report), 0};
    out.hash = mnavHash64(0, &report, (int32_t)sizeof(report));
    return out;
}

// A build searches with its context, but never runs the context's
// allocator: a context takes its memory when it is made, so the only host
// code a build runs is the hierarchy's allocator.
static void TestContextQuiet(void)
{
    mnavQuery* query = MakeQuery();
    mnavHierarchyDef def = mnavDefaultHierarchyDef();
    def.limits.tiles = 16;
    def.limits.transitions = 64;
    def.limits.edges = 256;
    mnavHierarchy* plain = nullptr;
    CHECK(mnavCreateHierarchy(&def, &plain) == mnav_success, "a hierarchy");
    s_allocs = 0;
    s_frees = 0;
    CHECK(mnavBuildHierarchy(plain, query, s_heldNavmesh, nullptr, nullptr) == mnav_success &&
              mnavUpdateHierarchy(plain, query, s_heldNavmesh, nullptr) == mnav_success,
          "built and updated");
    CHECK(s_allocs == 0 && s_frees == 0, "the context's allocator never ran");
    mnavDestroyHierarchy(plain);
    mnavDestroyQuery(query);
    CHECK(s_held == 0, "every byte given back");
}

static void Setup(void)
{
    s_trench = Trench();
    mnavBakeDef def = mnavDefaultBakeDef();
    CHECK(mnavCreateBaker(&def, &s_plainBaker).result == mnav_success, "a plain baker");
    mnavFlightDef flight = FlightDef();
    flight.allocator = (mnavAllocator){nullptr, nullptr, nullptr};
    mnavFlightBaker* baker = nullptr;
    const mnavBakeInput input = Input();
    CHECK(mnavCreateFlightBaker(&flight, &baker).result == mnav_success &&
              mnavBakeFlightTile(baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavCopyFlightTile(baker, s_flight, sizeof(s_flight), &s_flightSize) == mnav_success,
          "a flight tile");
    mnavDestroyFlightBaker(baker);
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    cacheDef.limits.tiles = 4;
    CHECK(mnavCreateTileCache(&cacheDef, &s_heldCache) == mnav_success &&
              mnavBakeTileCached(s_plainBaker, s_heldCache, &input, 0, 0, nullptr) ==
                  mnav_success &&
              mnavCreateTileIndex(&def, &s_trench, 1, &s_heldIndex, nullptr) == mnav_success,
          "a cache with a tile, and an index");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    CHECK(mnavCreateQuery(&queryDef, &s_heldQuery) == mnav_success &&
              mnavCreateNavmesh(&def, &s_heldNavmesh).result == mnav_success &&
              mnavStageTile(s_heldNavmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavStageTile(s_heldNavmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavCommit(s_heldNavmesh) == mnav_success,
          "a context, and a navmesh with the trench");
}

int main(void)
{
    Setup();
    for (int32_t kind = 0; kind < KINDS; ++kind)
    {
        TestDestroy(kind);
    }
    TestOuter(BAKER, BakeOuter);
    TestOuter(CACHE, CacheOuter);
    TestOuter(FLIGHT_BAKER, FlightBakeOuter);
    TestOuter(VOLUME, VolumeOuter);
    TestClock();
    // Host code calling into the other objects a call uses.
    TestAcross(BAKER, CACHE, s_heldCache, RebuildOuter, 0);
    TestAcross(BAKER, CACHE, s_heldCache, CachedOuter, 0);
    TestAcross(CACHE, BAKER, s_plainBaker, CacheOuter, 0);
    TestAcross(BAKER, INDEX, s_heldIndex, IndexedOuter, 0);
    TestAcross(FLIGHT_BAKER, INDEX, s_heldIndex, FlightIndexedOuter, 0);
    TestOuter(HIERARCHY, BuildOuter);
    TestContextQuiet();
    // The navmesh is read: staging a tile and its removal stay allowed.
    TestAcross(HIERARCHY, QUERY, s_heldQuery, BuildOuter, 0);
    TestAcross(HIERARCHY, NAVMESH, s_heldNavmesh, BuildOuter, 2);
    mnavDestroyNavmesh(s_heldNavmesh);
    mnavDestroyQuery(s_heldQuery);
    mnavDestroyTileIndex(s_heldIndex);
    mnavDestroyTileCache(s_heldCache);
    mnavDestroyBaker(s_plainBaker);
    return s_failures == 0 ? 0 : 1;
}
