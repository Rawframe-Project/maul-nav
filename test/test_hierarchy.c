// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hierarchical paths (mnav-0008): paths through the hierarchy against
// plain ones, with fewer nodes; the graph the same in any load order;
// staleness, fallbacks, limits and checks.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the random pairs' lengths.
#define RANDOM_HASH 0xaeda7ea61a27779eull

// The hash of the graph's counts and the paths' points.
#define PATHS_HASH 0xa356f433fa7507e2ull

enum
{
    TILES = 8,
    CAPACITY = 1 << 18,
    OUTLINES = 1300
};

static mnavVec2 s_points[OUTLINES][4];
static mnavOutline s_outlines[OUTLINES];
static int32_t s_outlineCount;
static uint8_t* s_bytes[TILES * TILES];
static size_t s_sizes[TILES * TILES];

static void Box(float x0, float z0, float x1, float z1, mnavAreaType area)
{
    mnavVec2* p = s_points[s_outlineCount];
    p[0] = (mnavVec2){x0, z0};
    p[1] = (mnavVec2){x1, z0};
    p[2] = (mnavVec2){x1, z1};
    p[3] = (mnavVec2){x0, z1};
    s_outlines[s_outlineCount++] = (mnavOutline){p, 4, area};
}

// A floor of 8 by 8 tiles from (shift, shift), a pillar of 2 m every 8 m
// and, every 64 m, a wall across with one gap of 6 m, the gaps
// zigzagging; rugs of another area, as cheap, across tile sides, so that
// runs of touching links cross them; baked into s_bytes.
static float BakeWorld(float shift)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    float side = def.cellSize * (float)def.tileCells * (float)TILES;
    s_outlineCount = 0;
    Box(shift, shift, shift + side, shift + side, mnav_areaWalkable);
    for (float z = 9.0f; z < side; z += 32.0f)
    {
        for (float x = 26.0f; x < side; x += 32.0f)
        {
            Box(shift + x, shift + z, shift + x + 12.0f, shift + z + 3.0f, 2);
            Box(shift + z, shift + x, shift + z + 3.0f, shift + x + 12.0f, 2);
        }
    }
    for (float z = 4.0f; z < side; z += 8.0f)
    {
        for (float x = 4.0f; x < side; x += 8.0f)
        {
            Box(shift + x, shift + z, shift + x + 2.0f, shift + z + 2.0f, mnav_areaNone);
        }
    }
    int32_t k = 0;
    for (float z = 60.0f; z < side - 10.0f; z += 64.0f)
    {
        float gap = (k++ % 2 == 0) ? side - 30.0f : 20.0f;
        Box(shift + 1.0f, shift + z, shift + gap, shift + z + 1.0f, mnav_areaNone);
        Box(shift + gap + 6.0f, shift + z, shift + side - 1.0f, shift + z + 1.0f, mnav_areaNone);
    }
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    uint8_t* buffer = malloc(CAPACITY);
    int32_t first = (int32_t)floorf(shift / (def.cellSize * (float)def.tileCells));
    for (int32_t z = 0; z < TILES; ++z)
    {
        for (int32_t x = 0; x < TILES; ++x)
        {
            int32_t t = z * TILES + x;
            free(s_bytes[t]);
            CHECK(mnavBakeTile2D(baker, s_outlines, s_outlineCount, first + x, first + z,
                                 nullptr) == mnav_success &&
                      mnavCopyBakedTile(baker, buffer, CAPACITY, &s_sizes[t]) == mnav_success,
                  "baked");
            s_bytes[t] = malloc(s_sizes[t]);
            memcpy(s_bytes[t], buffer, s_sizes[t]);
        }
    }
    free(buffer);
    mnavDestroyBaker(baker);
    return side;
}

// The navmesh with every tile, loaded forward or backward.
static mnavNavmesh* Load(bool backward)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t i = 0; i < TILES * TILES; ++i)
    {
        int32_t t = backward ? TILES * TILES - 1 - i : i;
        CHECK(mnavStageTile(navmesh, s_bytes[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavQuery* Query(int32_t nodes)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    def.limits.pathLength = 1.0e5f;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

// Limits that fit the web build's fixed memory.
static mnavHierarchyDef Small(void)
{
    mnavHierarchyDef def = mnavDefaultHierarchyDef();
    def.limits = (mnavHierarchyLimits){128, 1024, 8192};
    return def;
}

static mnavHierarchy* Hierarchy(int32_t clusterTiles)
{
    mnavHierarchyDef def = Small();
    def.clusterTiles = clusterTiles;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_success, "hierarchy");
    return hierarchy;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){3.0f, 2.0f, 3.0f},
                          &n) == mnav_success &&
              n.polygon.slot != 0,
          "nearest");
    return n;
}

// The ends of the paths tried.
static const double s_pairs[4][4] = {{2.0, 2.0, 254.0, 254.0},
                                     {2.0, 254.0, 254.0, 2.0},
                                     {128.0, 2.0, 128.0, 254.0},
                                     {30.0, 30.0, 60.0, 40.0}};

static void TestAgainstPlainPaths(float side)
{
    (void)side;
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* big = Query(32768);
    mnavQuery* small = Query(1024);
    mnavHierarchy* hierarchy = Hierarchy(2);
    mnavHierarchyReport report;
    CHECK(mnavBuildHierarchy(hierarchy, small, navmesh, nullptr, &report) == mnav_success &&
              report.clusters == 16 && report.transitions > 0 && report.edges > 0,
          "built");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, &report, (int32_t)sizeof(report));
    int32_t starved = 0;
    for (int32_t p = 0; p < 4; ++p)
    {
        mnavNearest a = On(navmesh, s_pairs[p][0], s_pairs[p][1]);
        mnavNearest b = On(navmesh, s_pairs[p][2], s_pairs[p][3]);
        mnavPath plain;
        mnavPath through;
        CHECK(mnavFindPath(big, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &plain) ==
                      mnav_success &&
                  plain.end == mnav_pathFound,
              "plain");
        CHECK(mnavFindHierarchicalPath(small, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                       b.point, &through) == mnav_success &&
                  through.end == mnav_pathFound,
              "through the hierarchy, with 1,024 nodes");
        printf("pair %d: plain %.3f m, hierarchical %.3f m\n", p, plain.length, through.length);
        CHECK(through.length <= plain.length * 1.06 && through.length >= plain.length - 1e-9,
              "within 6% of the plain path");
        hash = mnavHash64(hash, through.points,
                          (int32_t)((size_t)through.pointCount * sizeof(mnavPos3)));
        mnavPath alone;
        CHECK(mnavFindPath(small, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &alone) == mnav_success,
              "plain, with 1,024 nodes");
        starved += alone.end == mnav_pathFound ? 0 : 1;
    }
    CHECK(starved >= 2, "the plain search runs out of nodes where the hierarchy does not");
    printf("PATHS_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == PATHS_HASH, "the pinned hash");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(small);
    mnavDestroyQuery(big);
    mnavDestroyNavmesh(navmesh);
}

static void TestRandomPairs(void)
{
    // Forty pairs of ends from a fixed seed: wherever the plain search
    // finds a way, the hierarchy finds one with 1,024 nodes, within 6%.
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* big = Query(32768);
    mnavQuery* small = Query(1024);
    mnavHierarchy* hierarchy = Hierarchy(2);
    CHECK(mnavBuildHierarchy(hierarchy, small, navmesh, nullptr, nullptr) == mnav_success, "built");
    uint32_t seed = 99u;
    int32_t found = 0;
    int32_t worse = 0;
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t p = 0; p < 40; ++p)
    {
        double v[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            seed = seed * 1664525u + 1013904223u;
            v[k] = 1.0 + 254.0 * (double)(seed >> 8 & 0xFFFFu) / 65536.0;
        }
        mnavNearest a;
        mnavNearest b;
        if (mnavFindNearest(navmesh, nullptr, (mnavPos3){v[0], 0.0, v[1]},
                            (mnavVec3){3.0f, 2.0f, 3.0f}, &a) != mnav_success ||
            mnavFindNearest(navmesh, nullptr, (mnavPos3){v[2], 0.0, v[3]},
                            (mnavVec3){3.0f, 2.0f, 3.0f}, &b) != mnav_success ||
            a.polygon.slot == 0 || b.polygon.slot == 0)
        {
            continue;
        }
        mnavPath plain;
        mnavPath through;
        CHECK(mnavFindPath(big, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &plain) ==
                      mnav_success &&
                  mnavFindHierarchicalPath(small, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                           b.point, &through) == mnav_success,
              "searched");
        if (plain.end != mnav_pathFound)
        {
            continue;
        }
        found += through.end == mnav_pathFound ? 1 : 0;
        worse += through.length <= plain.length * 1.06 ? 0 : 1;
        hash = mnavHash64(hash, &through.length, (int32_t)sizeof(double));
    }
    printf("RANDOM_HASH=%016llx found=%d\n", (unsigned long long)hash, found);
    CHECK(found >= 30 && worse == 0, "every way found, none more than 6% longer");
    CHECK(hash == RANDOM_HASH, "the pinned hash");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(small);
    mnavDestroyQuery(big);
    mnavDestroyNavmesh(navmesh);
}

static void TestLinksBetweenClusters(void)
{
    // A link over the wall at z = 60, from one cluster into the next: the
    // way through it is far shorter than round by the wall's gap.
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(1024);
    mnavHierarchy* hierarchy = Hierarchy(2);
    mnavHierarchyReport before;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, &before) == mnav_success, "built");
    mnavLinkDef def = {{40.0, 0.0, 56.0}, {40.0, 0.0, 66.0}, 1.0f, 2.0f, 0, false, 0.0f};
    mnavLinkId id;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "linked");
    mnavHierarchyReport after;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, &after) == mnav_success &&
              after.transitions == before.transitions + 1,
          "one transition, one way");
    // A hierarchy with room for the sides' transitions alone refuses the
    // link's.
    mnavHierarchyDef tight = Small();
    tight.clusterTiles = 2;
    tight.limits.transitions = before.transitions;
    mnavHierarchy* full = nullptr;
    CHECK(mnavCreateHierarchy(&tight, &full) == mnav_success &&
              mnavBuildHierarchy(full, query, navmesh, nullptr, nullptr) == mnav_errorLimit,
          "no room for the link's transition");
    mnavDestroyHierarchy(full);
    mnavNearest a = On(navmesh, 40.0, 30.0);
    mnavNearest b = On(navmesh, 40.0, 110.0);
    mnavPath path;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 && path.length < 100.0,
          "over the wall");
    // Back the other way the link does not go: round by the gap.
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, b.polygon, b.point, a.polygon,
                                   a.point, &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 0 && path.length > 300.0,
          "round the wall");
    // A filter that does not cross the link's kind leaves it out.
    mnavQueryFilter walkers = mnavDefaultQueryFilter();
    walkers.kinds = 0;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, &walkers, &after) == mnav_success &&
              after.transitions == before.transitions,
          "no transition for a kind not crossed");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestOneWayLinkInTheEndCluster(void)
{
    // A one-way link over the wall at z = 60, both ends in the cluster of
    // tiles (0, 0) to (1, 1): the end, south of the wall, is reached from
    // the north by it, or round by the wall's gap 190 m east.
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(4096);
    mnavHierarchy* hierarchy = Hierarchy(2);
    mnavLinkDef def = {{40.0, 0.0, 63.0}, {40.0, 0.0, 57.0}, 1.0f, 2.0f, 0, false, 0.0f};
    mnavLinkId id;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "linked");
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success, "built");
    // The start two clusters north, too far for the plain search first.
    mnavNearest a = On(navmesh, 40.0, 170.0);
    mnavNearest b = On(navmesh, 40.0, 50.0);
    mnavPath plain;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &plain) ==
                  mnav_success &&
              plain.end == mnav_pathFound && plain.linkCount == 1,
          "the plain path takes the link");
    double plainLength = plain.length;
    mnavPath path;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_success,
          "searched");
    printf("one-way link in the end cluster: plain %.1f m, hierarchical %.1f m, %d links\n",
           plainLength, path.length, path.linkCount);
    CHECK(path.end == mnav_pathFound && path.linkCount == 1 && path.length < plainLength * 1.1,
          "the hierarchical path takes it too");
    // From the end northward the link does not go.
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, b.polygon, b.point, a.polygon,
                                   a.point, &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 0 && path.length > 300.0,
          "never against it");
    // The same link two-way, defined from the south: the way south crosses
    // it from its end to its start.
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success, "removed");
    def = (mnavLinkDef){{40.0, 0.0, 57.0}, {40.0, 0.0, 63.0}, 1.0f, 2.0f, 0, true, 0.0f};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                       b.point, &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 1 && path.length < plainLength * 1.1,
          "a two-way link crossed from its end");
    // A link dearer than the way round: the join prices it, and the path
    // goes round as the plain one does.
    mnavQuery* wide = Query(16384);
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success, "removed");
    def = (mnavLinkDef){{40.0, 0.0, 63.0}, {40.0, 0.0, 57.0}, 1.0f, 900.0f, 0, false, 0.0f};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavFindPath(wide, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &plain) == mnav_success &&
              plain.end == mnav_pathFound && plain.linkCount == 0,
          "the plain path, with the nodes to search round, goes round");
    double roundCost = plain.cost;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 0 && path.cost < roundCost * 1.1,
          "so does the hierarchical one");
    mnavDestroyQuery(wide);
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The hash of the hierarchical paths between the random pairs.
static uint64_t PairsHash(const mnavNavmesh* navmesh, mnavQuery* query, mnavHierarchy* hierarchy)
{
    uint64_t hash = MNAV_HASH_INIT;
    uint32_t seed = 5u;
    for (int32_t p = 0; p < 24; ++p)
    {
        double v[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            seed = seed * 1664525u + 1013904223u;
            v[k] = 1.0 + 254.0 * (double)(seed >> 8 & 0xFFFFu) / 65536.0;
        }
        mnavNearest a;
        mnavNearest b;
        mnavPath path;
        if (mnavFindNearest(navmesh, nullptr, (mnavPos3){v[0], 0.0, v[1]},
                            (mnavVec3){3.0f, 2.0f, 3.0f}, &a) == mnav_success &&
            mnavFindNearest(navmesh, nullptr, (mnavPos3){v[2], 0.0, v[3]},
                            (mnavVec3){3.0f, 2.0f, 3.0f}, &b) == mnav_success &&
            a.polygon.slot != 0 && b.polygon.slot != 0 &&
            mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                     b.point, &path) == mnav_success)
        {
            hash = mnavHash64(hash, path.points,
                              (int32_t)((size_t)path.pointCount * sizeof(mnavPos3)));
        }
    }
    return hash;
}

// Stages the area on polygons along z from x0 to x1, and commits.
static void Paint(mnavNavmesh* navmesh, double x0, double x1, double z, mnavAreaType area)
{
    for (double x = x0; x < x1; x += 2.0)
    {
        mnavNearest n = On(navmesh, x, z);
        CHECK(mnavStageArea(navmesh, n.polygon, area) == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
}

// Updates one hierarchy and builds the other; whether they agree, and
// the update's report.
static bool Agree(mnavHierarchy* updated, mnavHierarchy* fresh, mnavQuery* query,
                  mnavNavmesh* navmesh, const mnavQueryFilter* filter, mnavHierarchyReport* out)
{
    mnavHierarchyReport built;
    bool ok = mnavUpdateHierarchy(updated, query, navmesh, out) == mnav_success &&
              mnavBuildHierarchy(fresh, query, navmesh, filter, &built) == mnav_success;
    return ok && out->transitions == built.transitions && out->edges == built.edges &&
           PairsHash(navmesh, query, updated) == PairsHash(navmesh, query, fresh);
}

static void TestUpdates(void)
{
    // Area 3 costs 5 under this filter and area 4 is left out.
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(1024);
    mnavHierarchy* updated = Hierarchy(2);
    mnavHierarchy* fresh = Hierarchy(2);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[3] = 5.0f;
    filter.areas &= ~((uint64_t)1 << 4);
    mnavHierarchyReport built;
    mnavHierarchyReport report;
    CHECK(mnavBuildHierarchy(updated, query, navmesh, &filter, &built) == mnav_success &&
              built.searches == built.transitions,
          "built, a search per transition");
    CHECK(mnavUpdateHierarchy(updated, query, navmesh, &report) == mnav_success &&
              report.searches == 0,
          "nothing to update");
    // Areas changed in one tile: only its cluster's entries searched.
    Paint(navmesh, 70.0, 90.0, 100.0, 3);
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) && report.searches > 0 &&
              report.searches * 4 < built.searches,
          "a few searches, the same graph and paths");
    int32_t first = report.searches;
    // Again elsewhere: only the new change searched, as by a hierarchy
    // that saw only that change.
    Paint(navmesh, 150.0, 170.0, 200.0, 3);
    mnavHierarchyReport alone;
    CHECK(mnavUpdateHierarchy(fresh, query, navmesh, &alone) == mnav_success &&
              Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == alone.searches && report.searches > 0,
          "only the new change searched");
    printf("updates searched %d and %d of %d\n", first, report.searches, built.searches);
    // A link: built again.
    mnavLinkDef def = {{40.0, 0.0, 56.0}, {40.0, 0.0, 66.0}, 1.0f, 2.0f, 0, true, 0.0f};
    mnavLinkId id;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "linked");
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == report.transitions && report.transitions == built.transitions + 2,
          "built again for a link");
    // The ground the link lands on left out: built again, one transition
    // gone.
    Paint(navmesh, 38.0, 42.0, 66.0, 4);
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == report.transitions && report.transitions == built.transitions + 1,
          "built again for a landing");
    // The link removed, then another with another cost in its place,
    // with no update between: built again all the same.
    CHECK(mnavStageLinkRemoval(navmesh, id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "unlinked");
    def.cost = 40.0f;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "linked again");
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == report.transitions,
          "built again for a link's cost");
    // A tile replaced by another, with a block on it: built again.
    mnavBakeDef bakeDef = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    uint8_t* bytes = malloc(CAPACITY);
    size_t size = 0;
    Box(170.0f, 170.0f, 180.0f, 180.0f, mnav_areaNone);
    CHECK(mnavCreateBaker(&bakeDef, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, s_outlines, s_outlineCount, 5, 5, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, bytes, CAPACITY, &size) == mnav_success &&
              mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a tile replaced");
    s_outlineCount -= 1;
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == report.transitions,
          "built again for a tile");
    // A tile removed: built again.
    CHECK(mnavStageTileRemoval(navmesh, 7, 7) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a tile removed");
    CHECK(Agree(updated, fresh, query, navmesh, &filter, &report) &&
              report.searches == report.transitions,
          "built again without a tile");
    mnavNavmesh* other = Load(false);
    CHECK(mnavUpdateHierarchy(updated, query, other, &report) == mnav_errorInvalid &&
              mnavUpdateHierarchy(nullptr, query, navmesh, &report) == mnav_errorInvalid,
          "another navmesh, or none");
    mnavDestroyNavmesh(other);
    mnavDestroyBaker(baker);
    mnavDestroyHierarchy(fresh);
    mnavDestroyHierarchy(updated);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    free(bytes);
}

static void TestNegativePlaces(void)
{
    // The same world 4 tiles back on both axes, at negative places: the
    // clusters fall on the same tiles, so the graph is the same.
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(2048);
    mnavHierarchy* hierarchy = Hierarchy(2);
    mnavHierarchyReport here;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, &here) == mnav_success, "built");
    mnavDestroyNavmesh(navmesh);
    BakeWorld(-128.0f);
    navmesh = Load(false);
    mnavHierarchyReport back;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, &back) == mnav_success &&
              memcmp(&here, &back, sizeof(here)) == 0,
          "the same graph at negative places");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestLoadOrder(void)
{
    // Tiles loaded backward take other slots; the graph and the paths are
    // the same.
    mnavNavmesh* forward = Load(false);
    mnavNavmesh* backward = Load(true);
    mnavQuery* query = Query(2048);
    mnavHierarchy* a = Hierarchy(2);
    mnavHierarchy* b = Hierarchy(2);
    mnavHierarchyReport ra;
    mnavHierarchyReport rb;
    CHECK(mnavBuildHierarchy(a, query, forward, nullptr, &ra) == mnav_success &&
              mnavBuildHierarchy(b, query, backward, nullptr, &rb) == mnav_success &&
              memcmp(&ra, &rb, sizeof(ra)) == 0,
          "the same graph");
    int32_t same = 0;
    for (int32_t p = 0; p < 4; ++p)
    {
        mnavNearest fa = On(forward, s_pairs[p][0], s_pairs[p][1]);
        mnavNearest fb = On(forward, s_pairs[p][2], s_pairs[p][3]);
        mnavNearest ba = On(backward, s_pairs[p][0], s_pairs[p][1]);
        mnavNearest bb = On(backward, s_pairs[p][2], s_pairs[p][3]);
        mnavPath pf;
        CHECK(mnavFindHierarchicalPath(query, a, forward, fa.polygon, fa.point, fb.polygon,
                                       fb.point, &pf) == mnav_success,
              "forward");
        mnavPos3 points[256];
        int32_t count = pf.pointCount < 256 ? pf.pointCount : 256;
        memcpy(points, pf.points, (size_t)count * sizeof(mnavPos3));
        mnavPath pb;
        CHECK(mnavFindHierarchicalPath(query, b, backward, ba.polygon, ba.point, bb.polygon,
                                       bb.point, &pb) == mnav_success,
              "backward");
        same += pb.pointCount == pf.pointCount &&
                        memcmp(points, pb.points, (size_t)count * sizeof(mnavPos3)) == 0
                    ? 1
                    : 0;
    }
    CHECK(same == 4, "the same paths");
    mnavDestroyHierarchy(a);
    mnavDestroyHierarchy(b);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(forward);
    mnavDestroyNavmesh(backward);
}

// An area change that opens a way adds edges; an update past the edge
// limit is refused, as a build is.
static void TestUpdatePastTheEdgeLimit(void)
{
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(4096);
    // The wall at z = 60 has its gap at x = 226 to 232: closed by an area
    // the filter leaves out.
    Paint(navmesh, 226.0, 232.0, 60.5, 3);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= ~((uint64_t)1 << 3);
    mnavHierarchy* probe = Hierarchy(2);
    mnavHierarchyReport closed;
    CHECK(mnavBuildHierarchy(probe, query, navmesh, &filter, &closed) == mnav_success, "closed");
    mnavDestroyHierarchy(probe);
    mnavHierarchyDef def = Small();
    def.clusterTiles = 2;
    def.limits.edges = closed.edges;
    mnavHierarchy* tight = nullptr;
    CHECK(mnavCreateHierarchy(&def, &tight) == mnav_success &&
              mnavBuildHierarchy(tight, query, navmesh, &filter, nullptr) == mnav_success,
          "built at the limit");
    Paint(navmesh, 226.0, 232.0, 60.5, mnav_areaWalkable);
    mnavHierarchyReport opened;
    mnavResult result = mnavUpdateHierarchy(tight, query, navmesh, &opened);
    printf("edge limit: %d edges closed\n", closed.edges);
    CHECK(result == mnav_errorLimit, "the opened gap's edges past the limit");
    mnavDestroyHierarchy(tight);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A row of tiles removed splits the world: the graph has no way across,
// and the hierarchical search gives the plain search's answer.
static void TestSplitWorld(void)
{
    mnavNavmesh* navmesh = Load(false);
    for (int32_t x = 0; x < TILES; ++x)
    {
        CHECK(mnavStageTileRemoval(navmesh, x, 2) == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "a row removed");
    mnavQuery* query = Query(4096);
    mnavHierarchy* hierarchy = Hierarchy(2);
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success, "built");
    mnavNearest a = On(navmesh, 40.0, 20.0);
    mnavNearest b = On(navmesh, 40.0, 200.0);
    mnavPath plain;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &plain) ==
              mnav_success,
          "plain");
    mnavPathEnd plainEnd = plain.end;
    mnavPath path;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_success &&
              path.end == plainEnd && path.end != mnav_pathFound,
          "no way through the graph: the plain search's answer");
    // Making a hierarchy refuses a NULL out.
    mnavHierarchyDef def = Small();
    CHECK(mnavCreateHierarchy(&def, nullptr) == mnav_errorInvalid, "a NULL out");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void TestStaleAndFallbacks(void)
{
    mnavNavmesh* navmesh = Load(false);
    mnavNavmesh* other = Load(false);
    mnavQuery* query = Query(32768);
    mnavHierarchy* hierarchy = Hierarchy(2);
    mnavNearest a = On(navmesh, 2.0, 2.0);
    mnavNearest b = On(navmesh, 254.0, 254.0);
    mnavPath path;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_errorInvalid,
          "no graph yet");
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success, "built");
    CHECK(mnavFindHierarchicalPath(query, hierarchy, other, a.polygon, a.point, b.polygon, b.point,
                                   &path) == mnav_errorInvalid,
          "another navmesh");
    // Within one cluster: the plain path, exactly.
    mnavNearest c = On(navmesh, 10.0, 12.0);
    mnavNearest d = On(navmesh, 50.0, 40.0);
    mnavPath plain;
    CHECK(mnavFindPath(query, navmesh, nullptr, c.polygon, c.point, d.polygon, d.point, &plain) ==
              mnav_success,
          "plain");
    double plainLength = plain.length;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, c.polygon, c.point, d.polygon,
                                   d.point, &path) == mnav_success &&
              path.length == plainLength,
          "one cluster");
    // An area change commits: stale until built again.
    CHECK(mnavStageArea(navmesh, a.polygon, 3) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                   b.point, &path) == mnav_errorStale,
          "stale");
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, b.polygon,
                                       b.point, &path) == mnav_success &&
              path.end == mnav_pathFound,
          "built again");
    // A filter leaving the end's area out: no way through the graph, and
    // the plain search's answer.
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~((uint64_t)1 << 3);
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, &without, nullptr) == mnav_success,
          "built with a filter");
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, b.polygon, b.point, a.polygon,
                                   a.point, &path) == mnav_success &&
              path.end != mnav_pathFound,
          "an end the filter leaves out");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(other);
    mnavDestroyNavmesh(navmesh);
}

static void TestLimitsAndChecks(void)
{
    mnavNavmesh* navmesh = Load(false);
    mnavQuery* query = Query(4096);
    mnavHierarchyDef def = Small();
    mnavHierarchy* hierarchy = nullptr;
    def.limits.transitions = 4;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_errorLimit,
          "too many transitions");
    mnavNearest a = On(navmesh, 2.0, 2.0);
    mnavPath path;
    CHECK(mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, a.polygon,
                                   a.point, &path) == mnav_errorInvalid,
          "no graph after a failed build");
    mnavDestroyHierarchy(hierarchy);
    def = Small();
    def.limits.edges = 4;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_errorLimit,
          "too many edges");
    mnavDestroyHierarchy(hierarchy);
    def = Small();
    def.limits.tiles = 63;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_errorLimit,
          "too many tiles");
    mnavDestroyHierarchy(hierarchy);
    mnavQuery* tiny = Query(16);
    hierarchy = Hierarchy(2);
    CHECK(mnavBuildHierarchy(hierarchy, tiny, navmesh, nullptr, nullptr) == mnav_errorLimit,
          "a cluster needing more nodes");
    mnavQueryFilter broken = mnavDefaultQueryFilter();
    broken.cookie = 0;
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, &broken, nullptr) == mnav_errorInvalid &&
              mnavBuildHierarchy(nullptr, query, navmesh, nullptr, nullptr) == mnav_errorInvalid &&
              mnavBuildHierarchy(hierarchy, nullptr, navmesh, nullptr, nullptr) ==
                  mnav_errorInvalid,
          "bad arguments");
    CHECK(mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon,
                                       (mnavPos3){(double)NAN, 0.0, 0.0}, a.polygon, a.point,
                                       &path) == mnav_errorInvalid &&
              mnavFindHierarchicalPath(query, hierarchy, navmesh, a.polygon, a.point, a.polygon,
                                       a.point, nullptr) == mnav_errorInvalid,
          "bad query arguments");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(tiny);
    def = mnavDefaultHierarchyDef();
    def.clusterTiles = 0;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_errorRange && hierarchy == nullptr,
          "no cluster side");
    def.clusterTiles = MNAV_MAX_CLUSTER_TILES + 1;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_errorRange, "too wide");
    def = mnavDefaultHierarchyDef();
    def.cookie = 0;
    CHECK(mnavCreateHierarchy(&def, &hierarchy) == mnav_errorInvalid, "not a def");
    mnavDestroyHierarchy(nullptr);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    float side = BakeWorld(0.0f);
    TestAgainstPlainPaths(side);
    TestRandomPairs();
    TestLoadOrder();
    TestStaleAndFallbacks();
    TestLimitsAndChecks();
    TestLinksBetweenClusters();
    TestOneWayLinkInTheEndCluster();
    TestSplitWorld();
    TestUpdatePastTheEdgeLimit();
    TestUpdates();
    TestNegativePlaces();
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        free(s_bytes[t]);
    }
    return s_failures == 0 ? 0 : 1;
}
