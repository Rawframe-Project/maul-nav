// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The guide's C snippets (docs/guide.md), each as written there, run
// over the test world: tools/check_docs.py checks that every snippet in
// the guide appears here unchanged, so the guide cannot drift from the
// API. Each function gives a snippet what it uses and checks what it
// keeps.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stddef.h>
#include <stdint.h>

static uint8_t s_bytes[TILE_CAPACITY];

static void GuideObstacles(mnavBaker* baker, const mnavTriangleMesh* world, int32_t tileX,
                           int32_t tileZ)
{
    mnavTriangleMesh mesh = *world;
    mnavBakeReport report;
    // clang-format off
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();   // 1024 tiles in 256 MiB
    mnavTileCache* cache = NULL;
    mnavResult cached = mnavCreateTileCache(&cacheDef, &cache);
    mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};
    if (cached == mnav_success)
    {
        cached = mnavBakeTileCached(baker, cache, &input, tileX, tileZ, &report);
    }

    // A crate on the tile: a ring on the ground and a height range.
    const mnavVec2 ring[4] = {{36, 4}, {38, 4}, {38, 6}, {36, 6}};
    const mnavBakeVolume crate = {ring, 4, -1.0f, 2.0f, mnav_volumeExclude, 0};
    mnavResult rebuilt = mnavRebuildTile(baker, cache, tileX, tileZ, &crate, 1, &report);
    // clang-format on
    CHECK(cached == mnav_success && rebuilt == mnav_success && report.polygons > 0,
          "the obstacle snippet rebuilds a tile");
    mnavDestroyTileCache(cache);
}

static void GuideBake(void)
{
    mnavTriangleMesh world = World();
    const mnavVec3* vertices = world.vertices;
    int32_t vertexCount = world.vertexCount;
    const int32_t* indices = world.indices;
    int32_t triangleCount = world.triangleCount;
    int32_t tileX = 1;
    int32_t tileZ = 0;
    uint8_t* bytes = s_bytes;
    size_t capacity = sizeof(s_bytes);
    // clang-format off
    mnavBakeDef def = mnavDefaultBakeDef();   // agent, cells, limits, tier
    mnavBaker* baker = NULL;
    mnavBakeDefResult made = mnavCreateBaker(&def, &baker);
    // made.result; on mnav_errorInvalid, made.setting names the bad setting.

    mnavTriangleMesh mesh = {vertices, vertexCount, indices, triangleCount, NULL};
    mnavBakeReport report;
    size_t size = 0;
    mnavResult baked = mnavBakeTile(baker, &mesh, 1, tileX, tileZ, &report);
    if (baked == mnav_success)
    {
        baked = mnavCopyBakedTile(baker, bytes, capacity, &size);   // the tile's bytes
    }
    // clang-format on
    CHECK(made.result == mnav_success && baked == mnav_success && size > 0 && report.polygons > 0,
          "the bake snippet bakes a tile");
    GuideObstacles(baker, &mesh, tileX, tileZ);
    mnavDestroyBaker(baker);
}

static void GuideNavmesh(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    const uint8_t* bytes = s_tiles[0];
    size_t size = s_sizes[0];
    // clang-format off
    mnavNavmesh* navmesh = NULL;
    mnavBakeDefResult created = mnavCreateNavmesh(&def, &navmesh);   // the bake def
    mnavTileResult staged = mnavStageTile(navmesh, bytes, size);     // copied
    // staged.result; refused bytes are named by section and element
    mnavResult committed = mnavCommit(navmesh);
    // clang-format on
    CHECK(created.result == mnav_success && staged.result == mnav_success &&
              committed == mnav_success,
          "the navmesh snippet loads a tile");
    mnavDestroyNavmesh(navmesh);
}

// The world's four tiles in one navmesh, for the snippets that query.
static mnavNavmesh* Loaded(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static void GuideQueryAndCorridor(void)
{
    mnavNavmesh* navmesh = Loaded();
    // clang-format off
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    mnavResult opened = mnavCreateQuery(&queryDef, &query);

    mnavNearest a, b;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};      // half extents of the search
    mnavResult nearA = mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 8.0}, box, &a);
    mnavResult nearB = mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 56.0}, box, &b);
    // a.polygon.slot is 0 when no polygon lies in the box

    mnavPath path;
    mnavResult searched =
        mnavFindPath(query, navmesh, NULL, a.polygon, a.point, b.polygon, b.point, &path);
    // path.end: found, partial (out of nodes or too long), no path, not loaded
    // path.points: the straight path; path.polygons: the corridor; path.links
    // clang-format on
    CHECK(opened == mnav_success && nearA == mnav_success && nearB == mnav_success &&
              a.polygon.slot != 0 && b.polygon.slot != 0 && searched == mnav_success &&
              path.end == mnav_pathFound && path.pointCount >= 2,
          "the query snippet finds a path");
    mnavPos3 wanted = {a.point.x + 0.5, a.point.y, a.point.z + 0.5};
    // clang-format off
    mnavPolygonId buffer[256];
    mnavCorridor corridor;
    mnavResult reset = mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point);
    mnavResult set = mnavSetCorridor(&corridor, &path);   // mnav_errorCapacity past 256

    // Each tick:
    mnavCorners corners;
    mnavResult cornered = mnavCorridorCorners(query, navmesh, &corridor, &corners);
    // head for corners.points[1]
    mnavResult moved = mnavMoveCorridor(query, navmesh, NULL, &corridor, wanted, NULL);
    // clang-format on
    CHECK(reset == mnav_success && set == mnav_success && cornered == mnav_success &&
              corners.pointCount >= 2 && moved == mnav_success,
          "the corridor snippet follows the path");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void GuideAvoidance(void)
{
    const mnavAgent agents[2] = {{{-5.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, 0.5, 1.5, 1.0, 1},
                                 {{5.0, 0.0}, {-1.0, 0.0}, {-1.0, 0.0}, 0.5, 1.5, 1.0, 2}};
    int32_t agentCount = 2;
    const mnavObstacle* obstacles = nullptr;
    int32_t obstacleCount = 0;
    mnavPos2 velocities[2];
    // clang-format off
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = NULL;
    mnavResult made = mnavCreateAvoidance(&def, &avoidance);

    // Each step: positions, velocities and preferred velocities in, new
    // velocities out.
    mnavResult stepped =
        mnavAvoid(avoidance, agents, agentCount, obstacles, obstacleCount, 0.1, velocities);
    // clang-format on
    CHECK(made == mnav_success && stepped == mnav_success, "the avoidance snippet steps");
    mnavDestroyAvoidance(avoidance);
}

int main(void)
{
    BakeWorld();
    GuideBake();
    GuideNavmesh();
    GuideQueryAndCorridor();
    GuideAvoidance();
    return s_failures == 0 ? 0 : 1;
}
