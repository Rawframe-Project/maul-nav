// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The C snippets of docs/user/, each as written there, run over the test
// world: tools/check_guide.py checks that every snippet appears here
// unchanged, so the user documentation cannot drift from the API. Each
// function gives a snippet what it uses and checks what it keeps.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

static uint8_t s_bytes[TILE_CAPACITY];

static void GuideObstacles(mnavBaker* baker, const mnavTriangleMesh* world, int32_t tileX,
                           int32_t tileZ)
{
    mnavTriangleMesh mesh = *world;
    mnavBakeReport report;
    // clang-format off
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();   // 4096 tiles in 64 MiB
    mnavTileCache* cache = NULL;
    mnavResult cached = mnavCreateTileCache(&cacheDef, &cache);
    mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};
    if (cached == mnav_success)
    {
        cached = mnavBaker_BakeTileCached(baker, cache, &input, tileX, tileZ, &report);
    }

    // A crate on the tile: a ring on the ground and a height range.
    const mnavVec2 ring[4] = {{36, 4}, {38, 4}, {38, 6}, {36, 6}};
    const mnavBakeVolume crate = {ring, 4, -1.0f, 2.0f, mnav_volumeExclude, 0};
    mnavResult rebuilt = mnavBaker_RebuildTile(baker, cache, tileX, tileZ, &crate, 1, &report);
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
    // made.result; on a refused def, made.setting names the bad setting.

    mnavTriangleMesh mesh = {vertices, vertexCount, indices, triangleCount, NULL};
    mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};   // meshes, terrains, volumes, index
    mnavBakeReport report;
    size_t size = 0;
    mnavResult baked = mnavBaker_BakeTile(baker, &input, tileX, tileZ, &report);
    if (baked == mnav_success)
    {
        baked = mnavBaker_CopyTile(baker, bytes, capacity, &size);   // the tile's bytes
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
    mnavTileResult staged = mnavNavmesh_StageTile(navmesh, bytes, size);     // copied
    // staged.result; refused bytes are named by section and element
    mnavResult committed = mnavNavmesh_Commit(navmesh);
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
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
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
    mnavResult nearA = mnavNavmesh_FindNearest(navmesh, (mnavPos3){8.0, 0.0, 8.0}, box, NULL, &a);
    mnavResult nearB = mnavNavmesh_FindNearest(navmesh, (mnavPos3){8.0, 0.0, 56.0}, box, NULL, &b);
    // a.polygon.slot is 0 when no polygon lies in the box

    mnavPath path;
    mnavResult searched =
        mnavQuery_FindPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, NULL, &path);
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
    mnavResult cornered = mnavQuery_FindCorridorCorners(query, &corridor, navmesh, &corners);
    // head for corners.points[1]
    mnavResult moved = mnavQuery_MoveCorridor(query, &corridor, navmesh, wanted, NULL, NULL);
    // clang-format on
    CHECK(reset == mnav_success && set == mnav_success && cornered == mnav_success &&
              corners.pointCount >= 2 && moved == mnav_success,
          "the corridor snippet follows the path");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

static void GuideAvoidance(void)
{
    const mnavAgent agents[2] = {{{-5.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, 0.5, 1.5, 1.0, 1, 0, 0},
                                 {{5.0, 0.0}, {-1.0, 0.0}, {-1.0, 0.0}, 0.5, 1.5, 1.0, 2, 0, 0}};
    int32_t agentCount = 2;
    const mnavObstacle* obstacles = nullptr;
    int32_t obstacleCount = 0;
    mnavVec2 velocities[2];
    // clang-format off
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = NULL;
    mnavResult made = mnavCreateAvoidance(&def, &avoidance);

    // Each step: positions, velocities and preferred velocities in, new
    // velocities out.
    mnavResult stepped =
        mnavAvoidance_Avoid(avoidance, agents, agentCount, obstacles, obstacleCount, 0.1, velocities);
    // clang-format on
    CHECK(made == mnav_success && stepped == mnav_success, "the avoidance snippet steps");
    mnavDestroyAvoidance(avoidance);
}

// Two agents meeting head on along the world's floor, stepped by the
// crowd snippet until both pass each other.
static void GuideCrowd(void)
{
    mnavNavmesh* navmesh = Loaded();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    mnavAvoidanceDef avoidanceDef = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success &&
              mnavCreateAvoidance(&avoidanceDef, &avoidance) == mnav_success,
          "a context and an avoidance set");
    enum
    {
        COUNT = 2
    };
    int32_t count = COUNT;
    static mnavPolygonId buffers[COUNT][64];
    mnavCorridor corridors[COUNT];
    mnavAgent agents[COUNT];
    mnavVec2 velocities[COUNT];
    const mnavObstacle* obstacles = nullptr;
    int32_t obstacleCount = 0;
    double dt = 0.1;
    const double ends[COUNT][2] = {{4.0, 16.0}, {16.0, 16.0}};
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    for (int32_t i = 0; i < COUNT; ++i)
    {
        mnavNearest from;
        mnavNearest to;
        mnavPath path;
        CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){ends[i][0], 0.0, ends[i][1]}, box,
                                      nullptr, &from) == mnav_success &&
                  mnavNavmesh_FindNearest(navmesh, (mnavPos3){ends[1 - i][0], 0.0, ends[1 - i][1]},
                                          box, nullptr, &to) == mnav_success &&
                  mnavQuery_FindPath(query, navmesh, from.polygon, from.point, to.polygon, to.point,
                                     nullptr, &path) == mnav_success &&
                  mnavResetCorridor(&corridors[i], buffers[i], 64, from.polygon, from.point) ==
                      mnav_success &&
                  mnavSetCorridor(&corridors[i], &path) == mnav_success,
              "an agent's corridor");
        agents[i] = (mnavAgent){.position = {from.point.x, from.point.z},
                                .radius = 0.4,
                                .maxSpeed = 3.5,
                                .priority = 1.0,
                                .id = (uint64_t)i + 1};
    }
    double closest = 1e9;
    for (int32_t step = 0; step < 100; ++step)
    {
        // clang-format off
        mnavSteerDef steer = mnavDefaultSteerDef();   // set maxSpeed to the agents'
        for (int32_t i = 0; i < count; ++i)
        {
            mnavCorners corners;
            mnavSteering steering;
            if (mnavQuery_FindCorridorCorners(query, &corridors[i], navmesh, &corners) == mnav_success &&
                mnavSteer(&corners, &steer, &steering) == mnav_success)
            {
                agents[i].preferred = (mnavVec2){steering.velocity.x, steering.velocity.z};
            }
        }
        mnavResult avoided = mnavAvoidance_Avoid(avoidance, agents, count, obstacles, obstacleCount, dt, velocities);
        for (int32_t i = 0; i < count && avoided == mnav_success; ++i)
        {
            // Limit the change from agents[i].velocity by the agent's acceleration.
            mnavPos3 at = corridors[i].position;
            mnavPos3 wanted = {at.x + (double)velocities[i].x * dt, at.y,
                               at.z + (double)velocities[i].y * dt};
            if (mnavQuery_MoveCorridor(query, &corridors[i], navmesh, wanted, NULL, NULL) == mnav_success)
            {
                agents[i].position = (mnavPos2){corridors[i].position.x, corridors[i].position.z};
                agents[i].velocity = velocities[i];
            }
        }
        // clang-format on
        double gap = hypot(agents[1].position.x - agents[0].position.x,
                           agents[1].position.y - agents[0].position.y);
        closest = gap < closest ? gap : closest;
    }
    CHECK(agents[0].position.x > 12.0 && agents[1].position.x < 8.0 && closest > 0.79,
          "the crowd snippet walks two agents past each other");
    mnavDestroyAvoidance(avoidance);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    LoadWorldTiles();
    GuideBake();
    GuideNavmesh();
    GuideQueryAndCorridor();
    GuideAvoidance();
    GuideCrowd();
    return s_failures == 0 ? 0 : 1;
}
