// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spatial queries (mnav-0005): heights, the nearest wall, random points
// and reachability, on a gently sloped floor with a block and a walled
// ring whose inside no one can reach.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of fifty random points, and of the points around a center,
// the same on every platform.
#define POINTS_HASH 0x023d605e8f7ed13eull
#define AROUND_HASH 0xedab71c0c9bd9019ull

enum
{
    BOXES = 5,
    VERTICES = 4 + 8 * BOXES,
    TRIANGLES = 2 + 12 * BOXES,
    CAPACITY = 1 << 17
};

// The floor rises 0.05 m a meter along X.
#define SLOPE 0.05f

static mnavVec3 s_vertices[VERTICES];
static int32_t s_indices[TRIANGLES * 3];
static uint8_t s_tiles[4][CAPACITY];
static size_t s_sizes[4];

static mnavTriangleMesh World(void)
{
    const mnavVec3 floor[4] = {{-1.0f, -SLOPE, -1.0f},
                               {-1.0f, -SLOPE, 65.0f},
                               {65.0f, 65.0f * SLOPE, 65.0f},
                               {65.0f, 65.0f * SLOPE, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    // A block, and four walls enclosing x and z from 44 to 56.
    const float boxes[BOXES][4] = {
        {30, 10, 34, 12}, {43, 43, 57, 44}, {43, 56, 57, 57}, {43, 44, 44, 56}, {56, 44, 57, 56}};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        mnavVec3* v = &s_vertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            float x = boxes[b][(k & 1) ? 2 : 0];
            v[k] = (mnavVec3){x, x * SLOPE + ((k & 4) ? 3.0f : -0.5f), boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, VERTICES, s_indices, TRIANGLES, nullptr};
}

static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierModifiers;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavBakeTile(baker, &world, 1, t & 1, t >> 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_tiles[t], CAPACITY, &s_sizes[t]) == mnav_success &&
                  mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "baked and staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavDestroyBaker(baker);
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, x * (double)SLOPE, z},
                          (mnavVec3){0.5f, 1.0f, 0.5f}, &n) == mnav_success &&
              n.polygon.slot != 0,
          "nearest");
    return n;
}

static mnavQuery* Query(int32_t nodes)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static void TestHeights(const mnavNavmesh* navmesh)
{
    // The height a polygon gives at the nearest point's ground place is
    // that point's height, and follows the slope.
    int32_t off = 0;
    for (int32_t i = 0; i < 100; ++i)
    {
        double x = 2.0 + (double)(i % 10) * 6.1;
        double z = 2.0 + (double)(i / 10) * 3.3;
        mnavNearest n;
        CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, x * (double)SLOPE, z},
                              (mnavVec3){0.5f, 1.0f, 0.5f}, &n) == mnav_success,
              "nearest");
        if (n.polygon.slot == 0)
        {
            continue;
        }
        double height = 0.0;
        CHECK(mnavGetHeight(navmesh, n.polygon, n.point.x, n.point.z, &height) == mnav_success &&
                  height == n.point.y,
              "the nearest point's height");
        off += fabs(height - n.point.x * (double)SLOPE) < 0.3 ? 0 : 1;
    }
    CHECK(off == 0, "on the slope");
    mnavNearest n = On(navmesh, 5.0, 5.0);
    double height = 0.0;
    CHECK(mnavGetHeight(navmesh, n.polygon, 500.0, 5.0, &height) == mnav_errorRange,
          "outside the polygon");
    CHECK(mnavGetHeight(navmesh, n.polygon, (double)NAN, 5.0, &height) == mnav_errorInvalid &&
              mnavGetHeight(nullptr, n.polygon, 5.0, 5.0, &height) == mnav_errorInvalid &&
              mnavGetHeight(navmesh, n.polygon, 5.0, 5.0, nullptr) == mnav_errorInvalid,
          "bad arguments");
}

static void TestWalls(mnavNavmesh* navmesh)
{
    // West of the block from x = 30, which the navmesh keeps a radius from.
    mnavQuery* query = Query(4096);
    mnavNearest n = On(navmesh, 27.0, 11.0);
    mnavWall wall;
    CHECK(mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, 6.0, &wall) ==
                  mnav_success &&
              wall.found && !wall.limited,
          "found");
    printf("wall %.3f m at (%.3f %.3f), normal (%.3f %.3f)\n", wall.distance, wall.point.x,
           wall.point.z, wall.normal.x, wall.normal.z);
    CHECK(wall.distance > 2.0 && wall.distance < 2.8 && wall.normal.x < -0.99 &&
              fabs(wall.point.z - n.point.z) < 1e-6,
          "the block's face, straight east");
    CHECK(fabs(wall.distance - hypot(wall.point.x - n.point.x, wall.point.z - n.point.z)) < 1e-9,
          "the distance to the point");
    CHECK(mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, 1.0, &wall) ==
                  mnav_success &&
              !wall.found && wall.distance == 1.0,
          "none within 1 m");
    // Walls behind the filter's refusals: none of the floor's area allowed.
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas = 0;
    CHECK(mnavFindWallDistance(query, navmesh, &none, n.polygon, n.point, 6.0, &wall) ==
                  mnav_success &&
              wall.found && wall.distance < 2.0,
          "the polygon's own edges as walls");
    // A small radius needs few nodes.
    mnavQuery* few = Query(16);
    CHECK(mnavFindWallDistance(few, navmesh, nullptr, n.polygon, n.point, 3.0, &wall) ==
                  mnav_success &&
              !wall.limited,
          "a small radius, a small search");
    mnavDestroyQuery(few);
    // Across the tile side at x = 32, an area the filter leaves out: the
    // side is a wall.
    mnavNearest near = On(navmesh, 30.0, 25.0);
    mnavPolygonId across[16];
    mnavFound found;
    CHECK(mnavFindPolygons(navmesh, nullptr, (mnavPos3){32.5, 1.6, 25.0},
                           (mnavVec3){0.4f, 1.0f, 6.0f}, across, 16, &found) == mnav_success &&
              found.count > 0 && found.count <= 16,
          "the polygons beyond the side");
    for (int32_t i = 0; i < found.count; ++i)
    {
        CHECK(mnavStageArea(navmesh, across[i], 3) == mnav_success, "painted");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~((uint64_t)1 << 3);
    CHECK(mnavFindWallDistance(query, navmesh, &without, near.polygon, near.point, 6.0, &wall) ==
                  mnav_success &&
              wall.found && fabs(wall.point.x - 32.0) < 1e-6 &&
              fabs(wall.distance - (32.0 - near.point.x)) < 1e-6,
          "the tile side as a wall");
    for (int32_t i = 0; i < found.count; ++i)
    {
        CHECK(mnavStageArea(navmesh, across[i], mnav_areaWalkable) == mnav_success, "painted back");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavQuery* tiny = Query(2);
    CHECK(mnavFindWallDistance(tiny, navmesh, nullptr, n.polygon, n.point, 60.0, &wall) ==
                  mnav_success &&
              wall.limited,
          "cut short by the node limit");
    CHECK(mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, -1.0, &wall) ==
                  mnav_errorInvalid &&
              mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, (double)INFINITY,
                                   &wall) == mnav_errorInvalid,
          "bad radii");
    mnavDestroyQuery(tiny);
    mnavDestroyQuery(query);
}

static void TestWallAtTheRadius(mnavNavmesh* navmesh)
{
    // A wall exactly as far as the radius lies within it. From centers
    // over the world (those on the navmesh), the nearest wall within
    // 20 m, then a search with the radius set to its distance: the same
    // wall, at the same distance and point.
    mnavQuery* query = Query(4096);
    int32_t tried = 0;
    bool same = true;
    for (int32_t i = 0; i < 13; ++i)
    {
        for (int32_t k = 0; k < 13; ++k)
        {
            double x = 2.0 + 5.0 * i;
            double z = 2.0 + 5.0 * k;
            mnavNearest n = {0};
            mnavWall far;
            mnavWall exact;
            if (mnavFindNearest(navmesh, nullptr, (mnavPos3){x, x * (double)SLOPE, z},
                                (mnavVec3){0.5f, 2.0f, 0.5f}, &n) != mnav_success ||
                n.polygon.slot == 0 ||
                mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, 20.0, &far) !=
                    mnav_success ||
                !far.found)
            {
                continue;
            }
            tried += 1;
            same = same &&
                   mnavFindWallDistance(query, navmesh, nullptr, n.polygon, n.point, far.distance,
                                        &exact) == mnav_success &&
                   exact.found && exact.distance == far.distance && exact.point.x == far.point.x &&
                   exact.point.z == far.point.z;
        }
    }
    printf("walls at the radius: %d centers\n", tried);
    CHECK(tried > 100 && same, "a wall at exactly the radius is found");
    mnavDestroyQuery(query);
}

static void TestRandomPoints(const mnavNavmesh* navmesh)
{
    // Every point lies on its polygon; the same seed gives the same point;
    // the halves of the floor, of near equal area, get near equal shares.
    uint64_t hash = MNAV_HASH_INIT;
    int32_t left = 0;
    int32_t off = 0;
    for (uint64_t seed = 0; seed < 2000; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavFindRandomPoint(navmesh, nullptr, seed, &p) == mnav_success &&
                  p.polygon.slot != 0,
              "picked");
        double height = 0.0;
        off += mnavGetHeight(navmesh, p.polygon, p.point.x, p.point.z, &height) == mnav_success &&
                       height == p.point.y
                   ? 0
                   : 1;
        left += p.point.x < 32.0 ? 1 : 0;
        if (seed < 50)
        {
            hash = mnavHash64(hash, &p.point, (int32_t)sizeof(p.point));
        }
    }
    printf("POINTS_HASH=%016llx left=%d\n", (unsigned long long)hash, left);
    CHECK(off == 0, "on their polygons");
    CHECK(left > 900 && left < 1150, "near half on each side");
    CHECK(hash == POINTS_HASH, "the pinned hash");
    mnavRandomPoint a;
    mnavRandomPoint b;
    CHECK(mnavFindRandomPoint(navmesh, nullptr, 77u, &a) == mnav_success &&
              mnavFindRandomPoint(navmesh, nullptr, 77u, &b) == mnav_success &&
              memcmp(&a, &b, sizeof(a)) == 0,
          "the same seed, the same point");
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas = 0;
    CHECK(mnavFindRandomPoint(navmesh, &none, 1u, &a) == mnav_success && a.polygon.slot == 0,
          "nothing to pick");
    CHECK(mnavFindRandomPoint(nullptr, nullptr, 1u, &a) == mnav_errorInvalid &&
              mnavFindRandomPoint(navmesh, nullptr, 1u, nullptr) == mnav_errorInvalid,
          "bad arguments");
}

static void TestAroundAndReachable(const mnavNavmesh* navmesh)
{
    mnavQuery* query = Query(4096);
    mnavQuery* other = Query(4096);
    mnavNearest center = On(navmesh, 10.0, 10.0);
    mnavNearest inside = On(navmesh, 50.0, 50.0);
    mnavNearest far = On(navmesh, 60.0, 5.0);
    mnavPathEnd end = mnav_pathFound;
    CHECK(mnavCheckReachable(query, navmesh, nullptr, center.polygon, center.point, far.polygon,
                             far.point, &end) == mnav_success &&
              end == mnav_pathFound,
          "across the floor");
    CHECK(mnavCheckReachable(query, navmesh, nullptr, center.polygon, center.point, inside.polygon,
                             inside.point, &end) == mnav_success &&
              end == mnav_pathNotLoaded,
          "not into the ring, though unloaded sides keep it from saying so");
    mnavQuery* tiny = Query(4);
    CHECK(mnavCheckReachable(tiny, navmesh, nullptr, center.polygon, center.point, far.polygon,
                             far.point, &end) == mnav_success &&
              end == mnav_pathOutOfNodes,
          "cannot tell with four nodes");
    // Points around the center are reachable and within the circle.
    int32_t unreachable = 0;
    int32_t inner = 0;
    double farthest = 0.0;
    uint64_t hash = MNAV_HASH_INIT;
    for (uint64_t seed = 0; seed < 200; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavFindRandomPointAround(query, navmesh, nullptr, center.polygon, center.point, 8.0,
                                        seed, &p) == mnav_success &&
                  p.polygon.slot != 0,
              "picked");
        CHECK(mnavCheckReachable(other, navmesh, nullptr, center.polygon, center.point, p.polygon,
                                 p.point, &end) == mnav_success,
              "checked");
        unreachable += end == mnav_pathFound ? 0 : 1;
        double d = hypot(p.point.x - center.point.x, p.point.z - center.point.z);
        farthest = d > farthest ? d : farthest;
        inner += d < 8.0 / sqrt(2.0) ? 1 : 0;
        hash = mnavHash64(hash, &p.point, (int32_t)sizeof(p.point));
    }
    printf("around: farthest %.3f m, %d of 200 in the inner half, AROUND_HASH=%016llx\n", farthest,
           inner, (unsigned long long)hash);
    CHECK(unreachable == 0 && farthest <= 8.0, "reachable, within the circle");
    CHECK(inner > 80 && inner < 120, "half in the inner half of the area");
    CHECK(hash == AROUND_HASH, "the pinned hash");
    // From inside the ring, the points stay inside.
    int32_t escaped = 0;
    for (uint64_t seed = 0; seed < 50; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavFindRandomPointAround(query, navmesh, nullptr, inside.polygon, inside.point, 30.0,
                                        seed, &p) == mnav_success,
              "picked");
        escaped +=
            p.point.x > 44.0 && p.point.x < 56.0 && p.point.z > 44.0 && p.point.z < 56.0 ? 0 : 1;
    }
    CHECK(escaped == 0, "inside the ring");
    mnavDestroyQuery(tiny);
    mnavDestroyQuery(other);
    mnavDestroyQuery(query);
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    TestHeights(navmesh);
    TestWalls(navmesh);
    TestWallAtTheRadius(navmesh);
    TestRandomPoints(navmesh);
    TestAroundAndReachable(navmesh);
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
