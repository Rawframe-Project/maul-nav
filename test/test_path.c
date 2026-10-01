// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The path search (mnav-0005): corridors, costs, how a search ends, and its
// named limits.

#include "hand_tile.h"
#include "navmesh.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of a set of searches' results, the same on every platform.
#define SEARCH_HASH 0x2dcfb43f488d206full

static mnavNavmesh* Load(const int32_t* tiles, int32_t count)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < count; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[tiles[t]], s_sizes[tiles[t]]).result == mnav_success,
              "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavNavmesh* LoadAll(void)
{
    const int32_t all[4] = {0, 1, 2, 3};
    return Load(all, 4);
}

static mnavQuery* MakeQuery(int32_t nodes, float length)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    def.limits.pathLength = length;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query made");
    return query;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double y, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, y, z}, (mnavVec3){2.0f, 2.0f, 2.0f},
                          &n) == mnav_success &&
              n.polygon.slot != 0,
          "on the navmesh");
    return n;
}

static mnavPath Search(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest a, mnavNearest b)
{
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    return path;
}

static bool SameId(mnavPolygonId a, mnavPolygonId b)
{
    return a.slot == b.slot && a.generation == b.generation && a.polygon == b.polygon;
}

// Whether two polygons share an edge in a tile or a link across tiles.
static bool Adjacent(const mnavNavmesh* navmesh, mnavPolygonId a, mnavPolygonId b)
{
    const mnavTile* tile = nullptr;
    const mnavPolygon* polygon = mnavPolygonOf(navmesh, a, &tile);
    for (int32_t k = 0; polygon != nullptr && k < polygon->count; ++k)
    {
        if (a.slot == b.slot && polygon->neighbors[k] == b.polygon)
        {
            return true;
        }
    }
    for (int32_t l = tile->firstLink[a.polygon]; l < tile->firstLink[a.polygon + 1]; ++l)
    {
        if (SameId(tile->links[l].target, b))
        {
            return true;
        }
    }
    return false;
}

static bool Connected(const mnavNavmesh* navmesh, const mnavPath* path)
{
    for (int32_t i = 0; i + 1 < path->polygonCount; ++i)
    {
        if (!Adjacent(navmesh, path->polygons[i], path->polygons[i + 1]))
        {
            return false;
        }
    }
    return true;
}

static double Length(mnavPos3 a, mnavPos3 b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Whether the straight path stays on the navmesh, sampled every 0.25 m, to
// within a nanometer, and is no longer than the way the search costed.
static bool Walkable(const mnavNavmesh* navmesh, const mnavPath* path)
{
    double length = 0.0;
    for (int32_t i = 0; i + 1 < path->pointCount; ++i)
    {
        mnavPos3 a = path->points[i];
        mnavPos3 b = path->points[i + 1];
        double d = Length(a, b);
        length += d;
        int32_t steps = (int32_t)(d / 0.25) + 1;
        for (int32_t k = 0; k <= steps; ++k)
        {
            double t = (double)k / steps;
            mnavPos3 p = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
            // Segments along walls touch the navmesh's edge, where
            // rounding leaves samples a few ulps outside.
            mnavNearest n;
            if (mnavFindNearest(navmesh, nullptr, p, (mnavVec3){0.5f, 1.0f, 0.5f}, &n) !=
                    mnav_success ||
                n.polygon.slot == 0 || hypot(p.x - n.point.x, p.z - n.point.z) > 1.0e-9)
            {
                return false;
            }
        }
    }
    return length <= path->cost + 1.0e-9;
}

static double Straight(mnavNearest a, mnavNearest b)
{
    double dx = a.point.x - b.point.x;
    double dy = a.point.y - b.point.y;
    double dz = a.point.z - b.point.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static void TestAcrossTheWorld(void)
{
    mnavNavmesh* navmesh = LoadAll();
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    mnavNearest a = On(navmesh, 2.0, 0.0, 2.0);
    mnavNearest b = On(navmesh, 62.0, 0.0, 62.0);
    mnavPath path = Search(query, navmesh, a, b);
    CHECK(path.end == mnav_pathFound, "found");
    CHECK(path.polygonCount > 2 && SameId(path.polygons[0], a.polygon) &&
              SameId(path.polygons[path.polygonCount - 1], b.polygon),
          "from the start polygon to the end polygon");
    CHECK(Connected(navmesh, &path), "each polygon next to the one before");
    CHECK(path.pointCount >= 2 && path.points[0].x == a.point.x &&
              path.points[path.pointCount - 1].z == b.point.z && Walkable(navmesh, &path),
          "a straight path from start to end, on the navmesh");
    double straight = Straight(a, b);
    CHECK(path.cost >= straight && path.cost < straight * 1.15, "close to the straight line");
    // The way back costs the same: the search is optimal over its graph.
    mnavPath back = Search(query, navmesh, b, a);
    CHECK(back.end == mnav_pathFound && fabs(back.cost - path.cost) < 1.0e-9,
          "the same cost both ways");
    // On one polygon, the straight line.
    mnavNearest c = On(navmesh, 2.5, 0.0, 2.0);
    mnavPath same = Search(query, navmesh, a, c);
    CHECK(same.end == mnav_pathFound && same.polygonCount == 1 && same.cost == Straight(a, c),
          "one polygon");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLimitsEndTheSearch(void)
{
    mnavNavmesh* navmesh = LoadAll();
    mnavNearest a = On(navmesh, 2.0, 0.0, 2.0);
    mnavNearest b = On(navmesh, 62.0, 0.0, 62.0);
    mnavQuery* query = MakeQuery(6, 1000.0f);
    mnavPath path = Search(query, navmesh, a, b);
    CHECK(path.end == mnav_pathOutOfNodes, "out of nodes");
    CHECK(path.polygonCount >= 1 && SameId(path.polygons[0], a.polygon) &&
              !SameId(path.polygons[path.polygonCount - 1], b.polygon),
          "a corridor toward the end");
    mnavDestroyQuery(query);
    query = MakeQuery(8192, 40.0f);
    path = Search(query, navmesh, a, b);
    CHECK(path.end == mnav_pathTooLong && path.polygonCount >= 1, "past the length");
    mnavDestroyQuery(query);
    // The limit bounds the whole way through a node, not the way so far:
    // with the end 84 m off no node opens, so a few nodes are enough.
    query = MakeQuery(2, 40.0f);
    path = Search(query, navmesh, a, b);
    CHECK(path.end == mnav_pathTooLong && path.polygonCount == 1, "nothing opened");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The fewest nodes with which a search reaches its end.
static int32_t FewestNodes(const mnavNavmesh* navmesh, mnavNearest a, mnavNearest b)
{
    for (int32_t nodes = 1; nodes <= 8192; ++nodes)
    {
        mnavQuery* query = MakeQuery(nodes, 1000.0f);
        mnavPath path = Search(query, navmesh, a, b);
        mnavDestroyQuery(query);
        if (path.end == mnav_pathFound)
        {
            return nodes;
        }
    }
    return -1;
}

static void TestSearchWorkIsPinned(void)
{
    // The nodes a search needs follow from its rules: the way just come
    // through is never opened again, and ties go to the node made first.
    mnavNavmesh* navmesh = LoadAll();
    int32_t across = FewestNodes(navmesh, On(navmesh, 2.0, 0.0, 2.0), On(navmesh, 62.0, 0.0, 62.0));
    int32_t around =
        FewestNodes(navmesh, On(navmesh, 18.0, 0.0, 22.0), On(navmesh, 26.0, 0.0, 22.0));
    int32_t sides = FewestNodes(navmesh, On(navmesh, 30.0, 0.0, 2.0), On(navmesh, 34.0, 0.0, 62.0));
    printf("FEWEST across=%d around=%d sides=%d\n", across, around, sides);
    CHECK(across == 27 && around == 10 && sides == 29, "the pinned node counts");
    mnavDestroyNavmesh(navmesh);
}

static void TestUnloadedPlacesAreNamed(void)
{
    // Tiles (0, 0) and (1, 1) touch at a corner only.
    const int32_t diagonal[2] = {0, 3};
    mnavNavmesh* navmesh = Load(diagonal, 2);
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    mnavPath path =
        Search(query, navmesh, On(navmesh, 2.0, 0.0, 2.0), On(navmesh, 62.0, 0.0, 62.0));
    CHECK(path.end == mnav_pathNotLoaded, "not loaded, not unreachable");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestUnreachableIsNone(void)
{
    // Two squares inside one tile, apart, touching none of its sides.
    const HandSquare squares[2] = {{10, 10, 30, 30, {0, 0, 0, 0}, 0},
                                   {60, 60, 90, 90, {0, 0, 0, 0}, 0}};
    static uint8_t bytes[2048];
    size_t size = HandTileBytes(bytes, 0, squares, 2);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    mnavNearest a = On(navmesh, 5.0, 0.0, 5.0);
    mnavNearest b = On(navmesh, 18.0, 0.0, 18.0);
    mnavPath path = Search(query, navmesh, a, b);
    CHECK(path.end == mnav_pathNone && path.polygonCount == 1 && path.cost == 0.0,
          "none, the corridor only the start polygon");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestIdsAreChecked(void)
{
    mnavNavmesh* navmesh = LoadAll();
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavNearest a = On(navmesh, 2.0, 0.0, 2.0);
    mnavPath path;
    mnavPolygonId bad = a.polygon;
    bad.slot = 99;
    CHECK(mnavFindPath(query, navmesh, nullptr, bad, a.point, a.polygon, a.point, &path) ==
              mnav_errorInvalid,
          "no such slot");
    bad = a.polygon;
    bad.generation += 1;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, bad, a.point, &path) ==
              mnav_errorInvalid,
          "a generation not yet given");
    bad = a.polygon;
    bad.polygon = 60000;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, bad, a.point, &path) ==
              mnav_errorInvalid,
          "no such polygon");
    CHECK(mnavStageTile(navmesh, s_tiles[0], s_sizes[0]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "tile (0, 0) replaced");
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, a.polygon, a.point, &path) ==
              mnav_errorStale,
          "stale");
    mnavPos3 nan = {(double)NAN, 0.0, 0.0};
    a = On(navmesh, 2.0, 0.0, 2.0);
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, nan, a.polygon, a.point, &path) ==
              mnav_errorInvalid,
          "a point not finite");
    CHECK(mnavFindPath(nullptr, navmesh, nullptr, a.polygon, a.point, a.polygon, a.point, &path) ==
                  mnav_errorInvalid &&
              mnavFindPath(query, nullptr, nullptr, a.polygon, a.point, a.polygon, a.point,
                           &path) == mnav_errorInvalid &&
              mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, a.polygon, a.point,
                           nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavDestroyQuery(query);
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

static void TestQueryDefsAreChecked(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    def.limits.nodes = 0;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorRange && query == nullptr, "no nodes");
    def = mnavDefaultQueryDef();
    def.limits.nodes = MNAV_MAX_QUERY_NODES + 1;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorRange, "too many nodes");
    def = mnavDefaultQueryDef();
    def.limits.pathLength = 0.0f;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorRange, "no length");
    def.limits.pathLength = NAN;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorRange, "a NaN length");
    def.limits.pathLength = MNAV_MAX_PATH_LENGTH * 2.0f;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorRange, "too long");
    def = (mnavQueryDef){0};
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorInvalid, "not from the default");
    CHECK(mnavCreateQuery(nullptr, &query) == mnav_errorInvalid &&
              mnavCreateQuery(&def, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    def = mnavDefaultQueryDef();
    def.allocator = (mnavAllocator){Alloc, Free, nullptr};
    CHECK(mnavCreateQuery(&def, &query) == mnav_success && s_held > 0, "made");
    mnavDestroyQuery(query);
    mnavDestroyQuery(nullptr);
    CHECK(s_held == 0, "all given back");
}

static void TestSearchesArePinned(void)
{
    mnavNavmesh* navmesh = LoadAll();
    mnavQuery* query = MakeQuery(8192, 1000.0f);
    // Searches short of nodes end at the node nearest the end.
    mnavQuery* small = MakeQuery(12, 1000.0f);
    uint64_t hash = MNAV_HASH_INIT;
    int32_t found = 0;
    int32_t partial = 0;
    uint32_t state = 5;
    for (int32_t i = 0; i < 200; ++i)
    {
        double p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (double)(state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
        }
        mnavNearest a;
        mnavNearest b;
        mnavVec3 box = {3.0f, 2.0f, 3.0f};
        CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){p[0], 0.0, p[1]}, box, &a) ==
                      mnav_success &&
                  mnavFindNearest(navmesh, nullptr, (mnavPos3){p[2], 0.0, p[3]}, box, &b) ==
                      mnav_success,
              "nearest");
        if (a.polygon.slot == 0 || b.polygon.slot == 0)
        {
            continue;
        }
        mnavPath cut = Search(small, navmesh, a, b);
        partial += cut.end == mnav_pathOutOfNodes ? 1 : 0;
        hash = mnavHash64(hash, &cut.cost, (int32_t)sizeof(cut.cost));
        hash = mnavHash64(hash, cut.polygons, cut.polygonCount * (int32_t)sizeof(mnavPolygonId));
        mnavPath path = Search(query, navmesh, a, b);
        found += path.end == mnav_pathFound ? 1 : 0;
        CHECK(Connected(navmesh, &path), "connected");
        CHECK(Walkable(navmesh, &path) && Walkable(navmesh, &cut), "on the navmesh");
        hash = mnavHash64(hash, path.points, path.pointCount * (int32_t)sizeof(mnavPos3));
        hash = mnavHash64(hash, cut.points, cut.pointCount * (int32_t)sizeof(mnavPos3));
        hash = mnavHash64(hash, &path.end, (int32_t)sizeof(path.end));
        hash = mnavHash64(hash, &path.cost, (int32_t)sizeof(path.cost));
        hash = mnavHash64(hash, path.polygons, path.polygonCount * (int32_t)sizeof(mnavPolygonId));
    }
    printf("SEARCH_HASH=%016llx found=%d partial=%d\n", (unsigned long long)hash, found, partial);
    CHECK(hash == SEARCH_HASH, "the pinned hash");
    mnavDestroyQuery(small);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeWorld();
    TestAcrossTheWorld();
    TestLimitsEndTheSearch();
    TestSearchWorkIsPinned();
    TestUnloadedPlacesAreNamed();
    TestUnreachableIsNone();
    TestIdsAreChecked();
    TestQueryDefsAreChecked();
    TestSearchesArePinned();
    return s_failures == 0 ? 0 : 1;
}
