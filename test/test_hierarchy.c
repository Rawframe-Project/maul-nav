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

// The hash of the graph's counts and the paths' points.
#define PATHS_HASH 0x3f78af37cca771bdull

enum
{
    TILES = 8,
    CAPACITY = 1 << 18,
    OUTLINES = 1100
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

// A floor of 8 by 8 tiles, a pillar of 2 m every 8 m and, every 64 m, a
// wall across with one gap of 6 m, the gaps zigzagging; baked once.
static float BakeWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    float side = def.cellSize * (float)def.tileCells * (float)TILES;
    Box(0.0f, 0.0f, side, side, mnav_areaWalkable);
    for (float z = 4.0f; z < side; z += 8.0f)
    {
        for (float x = 4.0f; x < side; x += 8.0f)
        {
            Box(x, z, x + 2.0f, z + 2.0f, mnav_areaNone);
        }
    }
    int32_t k = 0;
    for (float z = 60.0f; z < side - 10.0f; z += 64.0f)
    {
        float gap = (k++ % 2 == 0) ? side - 30.0f : 20.0f;
        Box(1.0f, z, gap, z + 1.0f, mnav_areaNone);
        Box(gap + 6.0f, z, side - 1.0f, z + 1.0f, mnav_areaNone);
    }
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    uint8_t* buffer = malloc(CAPACITY);
    for (int32_t z = 0; z < TILES; ++z)
    {
        for (int32_t x = 0; x < TILES; ++x)
        {
            int32_t t = z * TILES + x;
            CHECK(mnavBakeTile2D(baker, s_outlines, s_outlineCount, x, z, nullptr) ==
                          mnav_success &&
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
    def.tier = mnav_tierModifiers;
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
        CHECK(through.length <= plain.length * 1.1 && through.length >= plain.length - 1e-9,
              "within 10% of the plain path");
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
    float side = BakeWorld();
    TestAgainstPlainPaths(side);
    TestLoadOrder();
    TestStaleAndFallbacks();
    TestLimitsAndChecks();
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        free(s_bytes[t]);
    }
    return s_failures == 0 ? 0 : 1;
}
