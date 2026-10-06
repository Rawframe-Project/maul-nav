// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the inputs of the searches over the whole navmesh (mnav-0004,
// mnav-0008, mnav-0009, mnav-0013): a navmesh of some of the test world's
// tiles with point and edge links read from the bytes, then a hierarchy,
// a navmesh flow field and link generation over it, with defs, filters,
// goals, regions, budgets and ends from the bytes, now and then out of
// range. Every call must end in a typed status. A hierarchy brought up to
// date after an area change must hold the graph a build makes and give
// the same paths; a field built in budgeted steps must be the one built
// at once, its costs never rising along its ways; links generated twice
// must be the same; everything given back when destroyed.

#include "counting_allocator.h"
#include "world.h"

#include "maul-nav/base.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    MOST_LINKS = 6,
    MOST_GOALS = 4,
    PROBES = 12,
    GENERATED = 48
};

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

#include "fuzz_draw.h"

typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

static double Raw(Reader* r)
{
    uint64_t bits = 0;
    for (int32_t k = 0; k < 8; ++k)
    {
        bits |= (uint64_t)Byte(r) << (8 * k);
    }
    double d;
    memcpy(&d, &bits, sizeof(d));
    return d;
}

// A coordinate: one time in sixteen raw bits, otherwise a step of 0.28 m
// from -4 to 67.7 m.
static double Coordinate(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? Raw(r) : (double)Byte(r) * 0.28125 - 4.0;
}

static mnavPos3 Point(Reader* r)
{
    return (mnavPos3){Coordinate(r), Coordinate(r) * 0.0625, Coordinate(r)};
}

// A float setting: one time in sixteen raw bits, otherwise a quarter
// meter step up to the scale.
static float Setting(Reader* r, float scale)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? (float)Raw(r) : (float)(Byte(r) % 64) * 0.015625f * scale;
}

static bool Finite(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

static bool Typed(mnavResult result)
{
    return result <= mnav_success && result >= mnav_errorTier;
}

static bool SameId(mnavPolygonId a, mnavPolygonId b)
{
    return a.slot == b.slot && a.polygon == b.polygon && a.generation == b.generation;
}

static bool SamePos(mnavPos3 a, mnavPos3 b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static mnavQueryFilter Filter(Reader* r)
{
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    uint8_t pick = Byte(r);
    if (pick % 4 == 0)
    {
        filter.areas &= ~((uint64_t)Byte(r) << 1);
        filter.kinds &= ~(uint64_t)Byte(r);
        filter.costs[1 + Byte(r) % 3] = pick % 32 == 0 ? (float)Raw(r) : 0.5f + (float)(pick % 7);
    }
    return filter;
}

static mnavLinkDef Link(Reader* r)
{
    mnavLinkDef link = {Point(r),
                        Point(r),
                        Setting(r, 4.0f),
                        Setting(r, 8.0f),
                        (mnavLinkKind)(Byte(r) % MNAV_LINK_KINDS),
                        (Byte(r) & 1) != 0,
                        0.0f};
    link.width = Byte(r) % 4 == 0 ? Setting(r, 16.0f) : 0.0f;
    return link;
}

// The navmesh: the world's tiles a byte picks, links from the bytes.
static mnavNavmesh* Navmesh(Reader* r)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    def.tier = mnav_tierDynamic;
    def.limits.links = 1 + Byte(r) % 128;
    mnavNavmesh* navmesh = nullptr;
    Expect(mnavCreateNavmesh(&def, &navmesh).result == mnav_success);
    uint8_t tiles = Byte(r);
    for (int32_t t = 0; t < 4; ++t)
    {
        if ((tiles >> t & 1) == 0 || t == 0)
        {
            Expect(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success);
        }
    }
    int32_t links = Byte(r) % (MOST_LINKS + 1);
    for (int32_t k = 0; k < links; ++k)
    {
        mnavLinkDef link = Link(r);
        mnavLinkId id;
        Expect(Typed(mnavStageLink(navmesh, &link, &id)));
    }
    mnavResult committed = mnavCommit(navmesh);
    Expect(Typed(committed));
    if (committed != mnav_success)
    {
        mnavDestroyNavmesh(navmesh);
        return nullptr;
    }
    return navmesh;
}

static mnavPolygonId Near(const mnavNavmesh* navmesh, mnavPos3 at, mnavPos3* pointOut)
{
    mnavNearest nearest = {0};
    mnavResult result =
        mnavFindNearest(navmesh, nullptr, at, (mnavVec3){3.0f, 4.0f, 3.0f}, &nearest);
    Expect(Typed(result));
    *pointOut = nearest.point;
    return nearest.polygon;
}

static void CheckPath(mnavResult result, const mnavPath* path)
{
    Expect(Typed(result));
    if (result != mnav_success)
    {
        return;
    }
    Expect(path->polygonCount >= 0 && path->pointCount >= 0 && path->linkCount >= 0 &&
           path->linkCount <= path->polygonCount);
    for (int32_t i = 0; i < path->pointCount; ++i)
    {
        Expect(Finite(path->points[i]));
    }
}

static mnavPos3 s_points[2][4096];

// Keeps a path's points, which the next search overwrites.
static void KeepPoints(mnavResult result, const mnavPath* path, int32_t index)
{
    if (result == mnav_success)
    {
        Expect(path->pointCount <= 4096);
        memcpy(s_points[index], path->points, (size_t)path->pointCount * sizeof(mnavPos3));
    }
}

static mnavHierarchy* Hierarchy(const mnavHierarchyDef* def, mnavQuery* query,
                                const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                                mnavHierarchyReport* reportOut, mnavResult* resultOut)
{
    mnavHierarchy* h = nullptr;
    Expect(mnavCreateHierarchy(def, &h) == mnav_success);
    *resultOut = mnavBuildHierarchy(h, query, navmesh, filter, reportOut);
    Expect(Typed(*resultOut));
    return h;
}

static void Hierarchies(Reader* r, mnavNavmesh* navmesh, mnavQuery* query,
                        const mnavQueryFilter* filter)
{
    mnavHierarchyDef def = mnavDefaultHierarchyDef();
    def.allocator = CountingAllocator();
    def.clusterTiles = 1 + Byte(r) % 2;
    def.limits.tiles = 1 + Byte(r) % 8;
    def.limits.transitions = 1 + Byte(r) * 2;
    def.limits.edges = 1 + Byte(r) * 16;
    mnavHierarchyReport built;
    mnavResult result;
    mnavHierarchy* h = Hierarchy(&def, query, navmesh, filter, &built, &result);
    mnavPos3 start;
    mnavPos3 end;
    mnavPolygonId from = Near(navmesh, Point(r), &start);
    mnavPolygonId to = Near(navmesh, Point(r), &end);
    mnavPath path = {0};
    mnavResult first = mnavFindHierarchicalPath(query, h, navmesh, from, start, to, end, &path);
    CheckPath(first, &path);
    KeepPoints(first, &path, 0);
    int32_t count = path.pointCount;
    mnavResult again = mnavFindHierarchicalPath(query, h, navmesh, from, start, to, end, &path);
    Expect(again == first);
    Expect(first != mnav_success ||
           (path.pointCount == count &&
            memcmp(s_points[0], path.points, (size_t)count * sizeof(mnavPos3)) == 0));
    // An area change; brought up to date, the graph a build makes.
    mnavPos3 at;
    mnavPolygonId changed = Near(navmesh, Point(r), &at);
    if (result == mnav_success && changed.slot != 0 &&
        mnavStageArea(navmesh, changed, (mnavAreaType)(Byte(r) % 4)) == mnav_success)
    {
        Expect(mnavCommit(navmesh) == mnav_success);
        mnavHierarchyReport updated;
        mnavResult up = mnavUpdateHierarchy(h, query, navmesh, &updated);
        Expect(Typed(up));
        mnavHierarchyReport fresh;
        mnavResult made;
        mnavHierarchy* rebuilt = Hierarchy(&def, query, navmesh, filter, &fresh, &made);
        Expect((up == mnav_success) == (made == mnav_success));
        if (up == mnav_success)
        {
            Expect(updated.clusters == fresh.clusters && updated.transitions == fresh.transitions &&
                   updated.edges == fresh.edges);
            mnavResult ra =
                mnavFindHierarchicalPath(query, h, navmesh, from, start, to, end, &path);
            CheckPath(ra, &path);
            KeepPoints(ra, &path, 1);
            int32_t n = path.pointCount;
            mnavPathEnd ended = path.end;
            mnavResult rb =
                mnavFindHierarchicalPath(query, rebuilt, navmesh, from, start, to, end, &path);
            Expect(ra == rb);
            Expect(ra != mnav_success ||
                   (path.end == ended && path.pointCount == n &&
                    memcmp(s_points[1], path.points, (size_t)n * sizeof(mnavPos3)) == 0));
        }
        mnavDestroyHierarchy(rebuilt);
    }
    mnavDestroyHierarchy(h);
}

static bool SameFlow(const mnavPolygonFlow* a, const mnavPolygonFlow* b)
{
    return memcmp(&a->cost, &b->cost, sizeof(double)) == 0 && SameId(a->next, b->next) &&
           SamePos(a->left, b->left) && SamePos(a->right, b->right) &&
           a->link.slot == b->link.slot && a->link.generation == b->link.generation;
}

typedef struct FieldDrawing
{
    const mnavNavFlow* field;
    const mnavNavmesh* navmesh;
} FieldDrawing;

static mnavResult DrawField(void* context, mnavDebugBuffer* buffer)
{
    const FieldDrawing* d = context;
    return mnavDebugNavFlow(d->field, d->navmesh, buffer);
}

static void Fields(Reader* r, const mnavNavmesh* navmesh, const mnavQueryFilter* filter)
{
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    def.allocator = CountingAllocator();
    def.limits.polygons = 1 + Byte(r) * 4;
    def.limits.tiles = 1 + Byte(r) % 8;
    def.limits.links = 1 + Byte(r) % 128;
    mnavNavFlow* whole = nullptr;
    mnavNavFlow* stepped = nullptr;
    Expect(mnavCreateNavFlow(&def, &whole) == mnav_success &&
           mnavCreateNavFlow(&def, &stepped) == mnav_success);
    mnavNavFlowGoal goals[MOST_GOALS];
    int32_t goalCount = Byte(r) % (MOST_GOALS + 1);
    for (int32_t g = 0; g < goalCount; ++g)
    {
        goals[g].polygon = Near(navmesh, Point(r), &goals[g].point);
        goals[g].point = Byte(r) % 8 == 0 ? Point(r) : goals[g].point;
    }
    mnavResult built = mnavBuildNavFlow(whole, navmesh, filter, goals, goalCount);
    Expect(Typed(built));
    mnavResult begun = mnavBeginNavFlow(stepped, navmesh, filter, nullptr, goals, goalCount);
    Expect(begun == built);
    bool ended = begun != mnav_success;
    for (int32_t step = 0; !ended && step < 100000; ++step)
    {
        Expect(mnavContinueNavFlow(stepped, navmesh, 1 + Byte(r) % 32, &ended) == mnav_success);
    }
    Expect(ended);
    for (int32_t p = 0; built == mnav_success && p < PROBES; ++p)
    {
        mnavPos3 at;
        mnavPolygonId polygon = Near(navmesh, Point(r), &at);
        mnavPolygonFlow a;
        mnavPolygonFlow b;
        mnavResult ra = mnavNavFlowAt(whole, navmesh, polygon, &a);
        Expect(Typed(ra) && mnavNavFlowAt(stepped, navmesh, polygon, &b) == ra);
        if (ra != mnav_success)
        {
            continue;
        }
        Expect(SameFlow(&a, &b) && !isnan(a.cost) && Finite(a.left) && Finite(a.right));
        if (isfinite(a.cost) && a.next.slot != 0)
        {
            mnavPolygonFlow next;
            Expect(mnavNavFlowAt(whole, navmesh, a.next, &next) == mnav_success &&
                   next.cost <= a.cost);
        }
    }
    FieldDrawing drawing = {whole, navmesh};
    Draw(DrawField, &drawing, Byte(r), Byte(r), Byte(r));
    // A region from the bytes, perhaps backward or outside the world.
    mnavNavFlowRegion region = {Byte(r) % 4 - 1, Byte(r) % 4 - 1, Byte(r) % 4 - 1, Byte(r) % 4 - 1};
    begun = mnavBeginNavFlow(stepped, navmesh, filter, &region, goals, goalCount);
    Expect(Typed(begun));
    ended = begun != mnav_success;
    for (int32_t step = 0; !ended && step < 100000; ++step)
    {
        Expect(mnavContinueNavFlow(stepped, navmesh, 1 + Byte(r) % 32, &ended) == mnav_success);
    }
    Expect(ended);
    mnavDestroyNavFlow(stepped);
    mnavDestroyNavFlow(whole);
}

// A clearance test from the bytes' choice: keeps links by a hash of
// their ends.
static bool Clear(void* context, mnavPos3 from, mnavPos3 to)
{
    uint64_t seed = *(const uint8_t*)context;
    uint64_t hash =
        mnavHash64(mnavHash64(seed, &from, (int32_t)sizeof(from)), &to, (int32_t)sizeof(to));
    return (hash & 3u) != 0;
}

static bool SameLink(const mnavLinkDef* a, const mnavLinkDef* b)
{
    return SamePos(a->start, b->start) && SamePos(a->end, b->end) && a->radius == b->radius &&
           a->cost == b->cost && a->kind == b->kind && a->twoWay == b->twoWay &&
           a->width == b->width;
}

static mnavLinkDef s_links[2][GENERATED];

static void Generate(Reader* r, mnavNavmesh* navmesh, mnavQuery* query,
                     const mnavQueryFilter* filter)
{
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    uint8_t pick = Byte(r);
    if (pick % 2 == 0)
    {
        def.spacing = 0.25f + Setting(r, 4.0f);
        def.dropMax = Setting(r, 8.0f);
        def.jumpMax = Setting(r, 8.0f);
        def.climbMax = Setting(r, 2.0f);
        def.detour = Byte(r) % 4 == 0 ? 0.0f : 1.0f + Setting(r, 4.0f);
        def.filterDistance = Setting(r, 2.0f);
        def.radius = Setting(r, 2.0f);
        def.dropCost = Setting(r, 8.0f);
        def.jumpCost = Setting(r, 8.0f);
        def.dropKind = (mnavLinkKind)(Byte(r) % MNAV_LINK_KINDS);
        def.jumpKind = (mnavLinkKind)(Byte(r) % MNAV_LINK_KINDS);
    }
    uint8_t seed = Byte(r);
    def.clear = pick % 4 == 1 ? Clear : nullptr;
    def.context = &seed;
    int32_t x0 = Byte(r) % 4 - 1;
    int32_t z0 = Byte(r) % 4 - 1;
    int32_t x1 = x0 + Byte(r) % 3 - (Byte(r) % 8 == 0 ? 2 : 0);
    int32_t z1 = z0 + Byte(r) % 3;
    int32_t capacity = Byte(r) % (GENERATED + 1);
    int32_t counts[2] = {-1, -1};
    mnavResult results[2];
    for (int32_t k = 0; k < 2; ++k)
    {
        results[k] = mnavGenerateLinks(query, navmesh, filter, &def, x0, z0, x1, z1, s_links[k],
                                       capacity, &counts[k]);
        Expect(Typed(results[k]));
    }
    Expect(results[0] == results[1]);
    if (results[0] != mnav_success && results[0] != mnav_errorCapacity)
    {
        return;
    }
    Expect(counts[0] == counts[1] && counts[0] >= 0);
    int32_t written = counts[0] < capacity ? counts[0] : capacity;
    for (int32_t i = 0; i < written; ++i)
    {
        Expect(SameLink(&s_links[0][i], &s_links[1][i]) && Finite(s_links[0][i].start) &&
               Finite(s_links[0][i].end));
    }
    // The links proposed are links the navmesh takes.
    for (int32_t i = 0; i < written && i < 8; ++i)
    {
        mnavLinkId id;
        Expect(Typed(mnavStageLink(navmesh, &s_links[0][i], &id)));
    }
    Expect(Typed(mnavCommit(navmesh)));
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    static bool baked = false;
    if (!baked)
    {
        BakeWorld();
        baked = true;
    }
    Reader r = {data, size, 0};
    mnavNavmesh* navmesh = Navmesh(&r);
    if (navmesh == nullptr)
    {
        Expect(s_held == 0);
        return 0;
    }
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.allocator = CountingAllocator();
    queryDef.limits.nodes = 16 + Byte(&r) * 8;
    mnavQuery* query = nullptr;
    Expect(mnavCreateQuery(&queryDef, &query) == mnav_success);
    mnavQueryFilter filter = Filter(&r);
    const mnavQueryFilter* f = Byte(&r) % 8 == 0 ? nullptr : &filter;
    uint8_t parts = Byte(&r);
    if ((parts & 1) != 0)
    {
        Hierarchies(&r, navmesh, query, f);
    }
    if ((parts & 2) != 0)
    {
        Fields(&r, navmesh, f);
    }
    if ((parts & 4) != 0)
    {
        Generate(&r, navmesh, query, f);
    }
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    Expect(s_held == 0);
    return 0;
}
