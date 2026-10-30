// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spatial queries: heights, the nearest wall, the walls near a point and
// reachability, on a gently sloped floor with a block and a walled ring
// whose inside no one can reach; random points, over the whole navmesh and
// around a center, on the stored test world, where they are pinned.

#include "defs_for.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of fifty random points, and of the points around a center, on
// the stored test world, the same on every platform.
#define POINTS_HASH 0x1945e3706d561b5eull
#define AROUND_HASH 0x5710d3c4ee6332f9ull

enum
{
    SLOPE_BOXES = 5,
    SLOPE_VERTICES = 4 + 8 * SLOPE_BOXES,
    SLOPE_TRIANGLES = 2 + 12 * SLOPE_BOXES,
    CAPACITY = 1 << 17
};

// The floor rises 0.05 m a meter along X.
#define SLOPE 0.05f

static mnavVec3 s_slopeVertices[SLOPE_VERTICES];
static int32_t s_slopeIndices[SLOPE_TRIANGLES * 3];
static uint8_t s_slopeTiles[4][CAPACITY];
static size_t s_slopeSizes[4];

static mnavTriangleMesh Slope(void)
{
    const mnavVec3 floor[4] = {{-1.0f, -SLOPE, -1.0f},
                               {-1.0f, -SLOPE, 65.0f},
                               {65.0f, 65.0f * SLOPE, 65.0f},
                               {65.0f, 65.0f * SLOPE, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_slopeVertices, floor, sizeof(floor));
    memcpy(s_slopeIndices, quad, sizeof(quad));
    // A block, and four walls enclosing x and z from 44 to 56.
    const float boxes[SLOPE_BOXES][4] = {
        {30, 10, 34, 12}, {43, 43, 57, 44}, {43, 56, 57, 57}, {43, 44, 44, 56}, {56, 44, 57, 56}};
    for (int32_t b = 0; b < SLOPE_BOXES; ++b)
    {
        mnavVec3* v = &s_slopeVertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            float x = boxes[b][(k & 1) ? 2 : 0];
            v[k] = (mnavVec3){x, x * SLOPE + ((k & 4) ? 3.0f : -0.5f), boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_slopeIndices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_slopeVertices, SLOPE_VERTICES, s_slopeIndices, SLOPE_TRIANGLES,
                              nullptr};
}

static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = Slope();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    meshDef.tier = mnav_tierModifiers;
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavBaker_BakeTile(baker,
                                 &(mnavBakeInput){&world, 1, nullptr, 0, nullptr, 0, nullptr},
                                 t & 1, t >> 1, nullptr) == mnav_success &&
                  mnavBaker_CopyTile(baker, s_slopeTiles[t], CAPACITY, &s_slopeSizes[t]) ==
                      mnav_success &&
                  mnavNavmesh_StageTile(navmesh, s_slopeTiles[t], s_slopeSizes[t]).result ==
                      mnav_success,
              "baked and staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavDestroyBaker(baker);
    return navmesh;
}

// The test world from its stored tiles.
static mnavNavmesh* LoadWorld(void)
{
    LoadWorldTiles();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, x * (double)SLOPE, z},
                                  (mnavVec3){0.5f, 1.0f, 0.5f}, nullptr, &n) == mnav_success &&
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
        CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, x * (double)SLOPE, z},
                                      (mnavVec3){0.5f, 1.0f, 0.5f}, nullptr, &n) == mnav_success,
              "nearest");
        if (n.polygon.slot == 0)
        {
            continue;
        }
        double height = 0.0;
        CHECK(mnavPolygon_GetHeight(navmesh, n.polygon, n.point.x, n.point.z, &height) ==
                      mnav_success &&
                  height == n.point.y,
              "the nearest point's height");
        off += fabs(height - n.point.x * (double)SLOPE) < 0.3 ? 0 : 1;
    }
    CHECK(off == 0, "on the slope");
    mnavNearest n = On(navmesh, 5.0, 5.0);
    double height = -1.0;
    CHECK(mnavPolygon_GetHeight(navmesh, n.polygon, 500.0, 5.0, &height) == mnav_errorOutside &&
              height == 0.0,
          "outside the polygon, the height zeroed");
    CHECK(mnavPolygon_GetHeight(navmesh, n.polygon, (double)NAN, 5.0, &height) ==
                  mnav_errorInvalid &&
              mnavPolygon_GetHeight(nullptr, n.polygon, 5.0, 5.0, &height) == mnav_errorInvalid &&
              mnavPolygon_GetHeight(navmesh, n.polygon, 5.0, 5.0, nullptr) == mnav_errorInvalid,
          "bad arguments");
}

// Whether two walls are the same, field by field: a wall's binary32
// normal leaves padding before its distance, which memcmp would compare.
static bool SameWall(const mnavWall* a, const mnavWall* b)
{
    return memcmp(&a->start, &b->start, sizeof(a->start)) == 0 &&
           memcmp(&a->end, &b->end, sizeof(a->end)) == 0 &&
           memcmp(&a->point, &b->point, sizeof(a->point)) == 0 &&
           memcmp(&a->normal, &b->normal, sizeof(a->normal)) == 0 &&
           memcmp(&a->distance, &b->distance, sizeof(a->distance)) == 0;
}

static void TestWalls(mnavNavmesh* navmesh)
{
    // West of the block from x = 30, which the navmesh keeps a radius from.
    mnavQuery* query = Query(4096);
    mnavNearest n = On(navmesh, 27.0, 11.0);
    mnavNearestWall wall;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, 6.0, nullptr, &wall) ==
                  mnav_success &&
              wall.found && !wall.limited,
          "found");
    printf("wall %.3f m at (%.3f %.3f), normal (%.3f %.3f)\n", wall.wall.distance,
           wall.wall.point.x, wall.wall.point.z, (double)wall.wall.normal.x,
           (double)wall.wall.normal.z);
    CHECK(wall.wall.distance > 2.0 && wall.wall.distance < 2.8 &&
              (double)wall.wall.normal.x < -0.99 && fabs(wall.wall.point.z - n.point.z) < 1e-6,
          "the block's face, straight east");
    CHECK(fabs(wall.wall.distance -
               hypot(wall.wall.point.x - n.point.x, wall.wall.point.z - n.point.z)) < 1e-9,
          "the distance to the point");
    CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, 1.0, nullptr, &wall) ==
                  mnav_success &&
              !wall.found && SameWall(&wall.wall, &(mnavWall){0}),
          "none within 1 m");
    // Walls behind the filter's refusals: none of the floor's area allowed.
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas = 0;
    CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, 6.0, &none, &wall) ==
                  mnav_success &&
              wall.found && wall.wall.distance < 2.0,
          "the polygon's own edges as walls");
    // A small radius needs few nodes.
    mnavQuery* few = Query(16);
    CHECK(mnavQuery_FindWallDistance(few, navmesh, n.polygon, n.point, 3.0, nullptr, &wall) ==
                  mnav_success &&
              !wall.limited,
          "a small radius, a small search");
    mnavDestroyQuery(few);
    // Across the tile side at x = 32, an area the filter leaves out: the
    // side is a wall.
    mnavNearest near = On(navmesh, 30.0, 25.0);
    mnavPolygonId across[16];
    mnavFound found;
    CHECK(mnavNavmesh_FindPolygons(navmesh, (mnavPos3){32.5, 1.6, 25.0},
                                   (mnavVec3){0.4f, 1.0f, 6.0f}, nullptr, across, 16,
                                   &found) == mnav_success &&
              found.count > 0 && found.count <= 16,
          "the polygons beyond the side");
    for (int32_t i = 0; i < found.count; ++i)
    {
        CHECK(mnavPolygon_StageArea(navmesh, across[i], 3) == mnav_success, "painted");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~((uint64_t)1 << 3);
    CHECK(mnavQuery_FindWallDistance(query, navmesh, near.polygon, near.point, 6.0, &without,
                                     &wall) == mnav_success &&
              wall.found && fabs(wall.wall.point.x - 32.0) < 1e-6 &&
              fabs(wall.wall.distance - (32.0 - near.point.x)) < 1e-6,
          "the tile side as a wall");
    for (int32_t i = 0; i < found.count; ++i)
    {
        CHECK(mnavPolygon_StageArea(navmesh, across[i], mnav_areaWalkable) == mnav_success,
              "painted back");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    mnavQuery* tiny = Query(2);
    CHECK(mnavQuery_FindWallDistance(tiny, navmesh, n.polygon, n.point, 60.0, nullptr, &wall) ==
                  mnav_success &&
              wall.limited,
          "cut short by the node limit");
    CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, -1.0, nullptr, &wall) ==
                  mnav_errorInvalid &&
              mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, (double)INFINITY,
                                         nullptr, &wall) == mnav_errorInvalid,
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
            mnavNearestWall far;
            mnavNearestWall exact;
            if (mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, x * (double)SLOPE, z},
                                        (mnavVec3){0.5f, 2.0f, 0.5f}, nullptr,
                                        &n) != mnav_success ||
                n.polygon.slot == 0 ||
                mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, 20.0, nullptr,
                                           &far) != mnav_success ||
                !far.found)
            {
                continue;
            }
            tried += 1;
            same = same &&
                   mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, far.wall.distance,
                                              nullptr, &exact) == mnav_success &&
                   exact.found && exact.wall.distance == far.wall.distance &&
                   exact.wall.point.x == far.wall.point.x && exact.wall.point.z == far.wall.point.z;
        }
    }
    printf("walls at the radius: %d centers\n", tried);
    CHECK(tried > 100 && same, "a wall at exactly the radius is found");
    mnavDestroyQuery(query);
}

// Whether a wall is sound: within the radius at the distance it gives
// from the center, not nearer than the one before it, its normal a unit
// across it with the navmesh on that side.
static bool SoundWall(const mnavNavmesh* navmesh, mnavPos3 center, double radius,
                      const mnavWall* wall, const mnavWall* before)
{
    double ex = wall->end.x - wall->start.x;
    double ez = wall->end.z - wall->start.z;
    double length = hypot(ex, ez);
    double t =
        ((center.x - wall->start.x) * ex + (center.z - wall->start.z) * ez) / (length * length);
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    double distance =
        hypot(center.x - (wall->start.x + t * ex), center.z - (wall->start.z + t * ez));
    mnavPos3 inside = {(wall->start.x + wall->end.x) * 0.5 + (double)wall->normal.x * 0.01,
                       (wall->start.y + wall->end.y) * 0.5,
                       (wall->start.z + wall->end.z) * 0.5 + (double)wall->normal.z * 0.01};
    mnavNearest n;
    return wall->distance <= radius && fabs(wall->distance - distance) < 1e-9 &&
           fabs(wall->point.x - (wall->start.x + t * ex)) < 1e-9 &&
           fabs(wall->point.z - (wall->start.z + t * ez)) < 1e-9 &&
           (before == nullptr || before->distance <= wall->distance) &&
           // The normal is binary32: unit and square to the edge to its rounding.
           fabs(hypot((double)wall->normal.x, (double)wall->normal.z) - 1.0) < 1e-6 &&
           wall->normal.y == 0.0f &&
           fabs((double)wall->normal.x * ex + (double)wall->normal.z * ez) < 1e-6 * length &&
           mnavNavmesh_FindNearest(navmesh, inside, (mnavVec3){0.05f, 1.0f, 0.05f}, nullptr, &n) ==
               mnav_success &&
           n.polygon.slot != 0 && hypot(n.point.x - inside.x, n.point.z - inside.z) < 1e-6;
}

static void TestWallList(const mnavNavmesh* navmesh)
{
    // From centers over the world, the walls within 8 m: sound, nearest
    // first, the first the wall distance's; a buffer of three holds the
    // first three and counts all, and no buffer counts.
    mnavQuery* query = Query(4096);
    static mnavWall walls[256];
    mnavWall few[3];
    int32_t tried = 0;
    int32_t listed = 0;
    int32_t cut = 0;
    bool sound = true;
    for (int32_t i = 0; i < 13; ++i)
    {
        for (int32_t k = 0; k < 13; ++k)
        {
            double x = 2.0 + 5.0 * i;
            double z = 2.0 + 5.0 * k;
            mnavNearest n = {0};
            if (mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, x * (double)SLOPE, z},
                                        (mnavVec3){0.5f, 2.0f, 0.5f}, nullptr,
                                        &n) != mnav_success ||
                n.polygon.slot == 0)
            {
                continue;
            }
            mnavNearestWall wall;
            mnavWallsFound found;
            CHECK(mnavQuery_FindWallDistance(query, navmesh, n.polygon, n.point, 8.0, nullptr,
                                             &wall) == mnav_success &&
                      mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 8.0, nullptr, walls,
                                          256, &found) == mnav_success &&
                      !found.limited,
                  "listed");
            tried += 1;
            listed += found.count;
            sound = sound && (found.count > 0) == wall.found &&
                    (found.count == 0 || SameWall(&walls[0], &wall.wall));
            for (int32_t w = 0; w < found.count; ++w)
            {
                sound = sound && SoundWall(navmesh, n.point, 8.0, &walls[w],
                                           w > 0 ? &walls[w - 1] : nullptr);
            }
            mnavWallsFound some;
            mnavResult result = mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 8.0,
                                                    nullptr, few, 3, &some);
            int32_t held = found.count < 3 ? found.count : 3;
            sound = sound && result == (found.count > 3 ? mnav_errorCapacity : mnav_success) &&
                    some.count == found.count;
            for (int32_t w = 0; w < held; ++w)
            {
                sound = sound && SameWall(&few[w], &walls[w]);
            }
            cut += found.count > 3 ? 1 : 0;
            result = mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 8.0, nullptr, nullptr,
                                         0, &some);
            sound = sound && result == (found.count > 0 ? mnav_errorCapacity : mnav_success) &&
                    some.count == found.count;
        }
    }
    printf("wall lists: %d centers, %d walls, %d cut to three\n", tried, listed, cut);
    CHECK(tried > 100 && listed > tried && cut > 0, "walls found");
    CHECK(sound, "sound, nearest first, the wall distance's first, the nearest kept");
    // None of the floor's area allowed: the polygon's own edges, six at most.
    mnavNearest n = On(navmesh, 27.0, 11.0);
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas = 0;
    mnavWallsFound found;
    CHECK(mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 60.0, &none, walls, 256,
                              &found) == mnav_success &&
              found.count >= 3 && found.count <= 6,
          "the polygon's own edges");
    mnavQuery* tiny = Query(2);
    CHECK(mnavQuery_FindWalls(tiny, navmesh, n.polygon, n.point, 60.0, nullptr, walls, 256,
                              &found) == mnav_success &&
              found.limited,
          "cut short by the node limit");
    mnavDestroyQuery(tiny);
    CHECK(mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 6.0, nullptr, walls, -1,
                              &found) == mnav_errorInvalid &&
              mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 6.0, nullptr, nullptr, 2,
                                  &found) == mnav_errorInvalid &&
              mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, 6.0, nullptr, walls, 256,
                                  nullptr) == mnav_errorInvalid &&
              mnavQuery_FindWalls(query, navmesh, n.polygon, n.point, (double)NAN, nullptr, walls,
                                  256, &found) == mnav_errorInvalid &&
              mnavQuery_FindWalls(nullptr, navmesh, n.polygon, n.point, 6.0, nullptr, walls, 256,
                                  &found) == mnav_errorInvalid,
          "bad arguments");
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
        CHECK(mnavNavmesh_FindRandomPoint(navmesh, seed, nullptr, &p) == mnav_success &&
                  p.polygon.slot != 0,
              "picked");
        double height = 0.0;
        off += mnavPolygon_GetHeight(navmesh, p.polygon, p.point.x, p.point.z, &height) ==
                           mnav_success &&
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
    CHECK(mnavNavmesh_FindRandomPoint(navmesh, 77u, nullptr, &a) == mnav_success &&
              mnavNavmesh_FindRandomPoint(navmesh, 77u, nullptr, &b) == mnav_success &&
              memcmp(&a, &b, sizeof(a)) == 0,
          "the same seed, the same point");
    mnavQueryFilter none = mnavDefaultQueryFilter();
    none.areas = 0;
    CHECK(mnavNavmesh_FindRandomPoint(navmesh, 1u, &none, &a) == mnav_success &&
              a.polygon.slot == 0,
          "nothing to pick");
    CHECK(mnavNavmesh_FindRandomPoint(nullptr, 1u, nullptr, &a) == mnav_errorInvalid &&
              mnavNavmesh_FindRandomPoint(navmesh, 1u, nullptr, nullptr) == mnav_errorInvalid,
          "bad arguments");
}

static void TestAroundAndReachable(const mnavNavmesh* navmesh)
{
    mnavQuery* query = Query(4096);
    mnavNearest center = On(navmesh, 10.0, 10.0);
    mnavNearest inside = On(navmesh, 50.0, 50.0);
    mnavNearest far = On(navmesh, 60.0, 5.0);
    mnavPathEnd end = mnav_pathFound;
    CHECK(mnavQuery_CheckReachable(query, navmesh, center.polygon, center.point, far.polygon,
                                   far.point, nullptr, &end) == mnav_success &&
              end == mnav_pathFound,
          "across the floor");
    CHECK(mnavQuery_CheckReachable(query, navmesh, center.polygon, center.point, inside.polygon,
                                   inside.point, nullptr, &end) == mnav_success &&
              end == mnav_pathNotLoaded,
          "not into the ring, though unloaded sides keep it from saying so");
    mnavQuery* tiny = Query(4);
    CHECK(mnavQuery_CheckReachable(tiny, navmesh, center.polygon, center.point, far.polygon,
                                   far.point, nullptr, &end) == mnav_success &&
              end == mnav_pathOutOfNodes,
          "cannot tell with four nodes");
    // From inside the ring, the points stay inside.
    int32_t escaped = 0;
    for (uint64_t seed = 0; seed < 50; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavQuery_FindRandomPointAround(query, navmesh, inside.polygon, inside.point, 30.0,
                                              seed, nullptr, &p) == mnav_success,
              "picked");
        escaped +=
            p.point.x > 44.0 && p.point.x < 56.0 && p.point.z > 44.0 && p.point.z < 56.0 ? 0 : 1;
    }
    CHECK(escaped == 0, "inside the ring");
    mnavDestroyQuery(tiny);
    mnavDestroyQuery(query);
}

// Points around a center on the stored world are reachable and within the
// circle, half of them in its inner half.
static void TestAround(const mnavNavmesh* navmesh)
{
    mnavQuery* query = Query(4096);
    mnavQuery* other = Query(4096);
    mnavNearest center;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){10.0, 0.0, 10.0},
                                  (mnavVec3){0.5f, 1.0f, 0.5f}, nullptr, &center) == mnav_success &&
              center.polygon.slot != 0,
          "nearest");
    mnavPathEnd end = mnav_pathFound;
    int32_t unreachable = 0;
    int32_t inner = 0;
    double farthest = 0.0;
    uint64_t hash = MNAV_HASH_INIT;
    for (uint64_t seed = 0; seed < 200; ++seed)
    {
        mnavRandomPoint p;
        CHECK(mnavQuery_FindRandomPointAround(query, navmesh, center.polygon, center.point, 8.0,
                                              seed, nullptr, &p) == mnav_success &&
                  p.polygon.slot != 0,
              "picked");
        CHECK(mnavQuery_CheckReachable(other, navmesh, center.polygon, center.point, p.polygon,
                                       p.point, nullptr, &end) == mnav_success,
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
    mnavDestroyQuery(other);
    mnavDestroyQuery(query);
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    TestHeights(navmesh);
    TestWalls(navmesh);
    TestWallAtTheRadius(navmesh);
    TestWallList(navmesh);
    TestAroundAndReachable(navmesh);
    mnavDestroyNavmesh(navmesh);
    mnavNavmesh* world = LoadWorld();
    TestRandomPoints(world);
    TestAround(world);
    mnavDestroyNavmesh(world);
    return s_failures == 0 ? 0 : 1;
}
