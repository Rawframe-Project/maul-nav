// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Scenes of flat one-cell squares on a patch of tile (0, 0), baked by a
// baker of the test's def and read back cell by cell through a navmesh,
// for the public suites of the bake's per-span stages.

#ifndef MAUL_NAV_TEST_SQUARES_H
#define MAUL_NAV_TEST_SQUARES_H

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// The default agent in cells of 0.125 m: 2 m tall, a step of 0.75 m.
enum
{
    HEIGHT = 16,
    STEP = 6,
    SIDE = 9,
    AT = 40,
    ROOF = 40,
    MORE = 96,
    MOST = SIDE * SIDE + MORE + 1,
    NO_FLOOR = -100,
    NO_POLYGON = 0xFF,
    TILE_ROOM = 1 << 20
};

// A square of SIDE by SIDE cells at cell AT of tile (0, 0), a flat square
// of one cell at each cell's level, more squares, and a roof over them
// all, or none: by default at level ROOF and solid. A square at level k fills its cells
// from height k to k + 1, so a span's top is its level plus 1. Squares of
// mnav_areaNone are solid but not walkable.
typedef struct Square
{
    int32_t x;
    int32_t z;
    int32_t level;
    mnavAreaType area;
} Square;

typedef struct Scene
{
    int32_t level[SIDE][SIDE];
    mnavAreaType area[SIDE][SIDE];
    Square more[MORE];
    int32_t moreCount;
    bool roof;
    int32_t roofLevel;
    mnavAreaType roofArea;
} Scene;

// Every cell's floor at a level, walkable, under a roof or not.
static inline Scene Flat(int32_t level, bool roof)
{
    Scene scene = {0};
    scene.roof = roof;
    scene.roofLevel = ROOF;
    scene.roofArea = mnav_areaNone;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            scene.level[z][x] = level;
            scene.area[z][x] = mnav_areaWalkable;
        }
    }
    return scene;
}

static inline void Add(Scene* scene, int32_t x, int32_t z, int32_t level, mnavAreaType area)
{
    scene->more[scene->moreCount++] = (Square){x, z, level, area};
}

// A square on every cell from (x0, z0) to (x1, z1).
static inline void AddCells(Scene* scene, int32_t x0, int32_t z0, int32_t x1, int32_t z1,
                            int32_t level, mnavAreaType area)
{
    for (int32_t z = z0; z <= z1; ++z)
    {
        for (int32_t x = x0; x <= x1; ++x)
        {
            Add(scene, x, z, level, area);
        }
    }
}

static mnavBakeDef s_def;
static mnavBaker* s_baker;
static mnavNavmesh* s_navmesh;
// The report of the last scene baked.
static mnavBakeReport s_report;
static uint8_t s_tile[TILE_ROOM];
// The bytes of the last scene's tile in s_tile.
static size_t s_tileSize;
static mnavVec3 s_vertices[4 * MOST];
static int32_t s_indices[6 * MOST];
static mnavAreaType s_areas[2 * MOST];

// Puts square n of cells sides by sides from a cell.
static inline void Put(int32_t n, Square s, int32_t sides)
{
    float x0 = (float)(AT + s.x) * 0.25f;
    float z0 = (float)(AT + s.z) * 0.25f;
    float x1 = (float)(AT + s.x + sides) * 0.25f;
    float z1 = (float)(AT + s.z + sides) * 0.25f;
    float y = (float)s.level * 0.125f;
    s_vertices[4 * n + 0] = (mnavVec3){x0, y, z0};
    s_vertices[4 * n + 1] = (mnavVec3){x1, y, z0};
    s_vertices[4 * n + 2] = (mnavVec3){x1, y, z1};
    s_vertices[4 * n + 3] = (mnavVec3){x0, y, z1};
    const int32_t corners[6] = {0, 3, 2, 0, 2, 1};
    for (int32_t k = 0; k < 6; ++k)
    {
        s_indices[6 * n + k] = 4 * n + corners[k];
    }
    s_areas[2 * n] = s.area;
    s_areas[2 * n + 1] = s.area;
}

// Bakes the input and loads its tile in a fresh navmesh.
static inline void LoadInput(const mnavBakeInput* input)
{
    CHECK(mnavBakeTile(s_baker, input, 0, 0, &s_report) == mnav_success &&
              mnavCopyBakedTile(s_baker, s_tile, TILE_ROOM, &s_tileSize) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, s_tileSize).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
}

// Bakes the scene and loads its tile in a fresh navmesh.
static inline void Load(const Scene* scene)
{
    int32_t n = 0;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            if (scene->level[z][x] != NO_FLOOR)
            {
                Put(n++, (Square){x, z, scene->level[z][x], scene->area[z][x]}, 1);
            }
        }
    }
    for (int32_t k = 0; k < scene->moreCount; ++k)
    {
        Put(n++, scene->more[k], 1);
    }
    if (scene->roof)
    {
        Put(n++, (Square){0, 0, scene->roofLevel, scene->roofArea}, SIDE);
    }
    const mnavTriangleMesh mesh = {s_vertices, 4 * n, s_indices, 2 * n, s_areas};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    LoadInput(&input);
}

// The area of the polygon over cell (x, z) of the scene within a meter
// of a level, or NO_POLYGON. A column of these scenes holds one walkable
// span at most, and a polygon's surface over a single raised or lowered
// cell may lie nearer its neighbors' height.
static inline mnavAreaType At(int32_t x, int32_t z, int32_t level)
{
    mnavPos3 center = {(AT + x + 0.5) * 0.25, level * 0.125, (AT + z + 0.5) * 0.25};
    mnavNearest n;
    mnavAreaType area = NO_POLYGON;
    CHECK(mnavFindNearest(s_navmesh, nullptr, center, (mnavVec3){0.01f, 1.0f, 0.01f}, &n) ==
              mnav_success,
          "nearest");
    if (n.polygon.slot != 0 && n.over)
    {
        CHECK(mnavGetArea(s_navmesh, n.polygon, &area) == mnav_success, "area");
    }
    return area;
}

// The default def, but an agent of no radius, regions of any size and no
// edge error, so that the polygons cover exactly the cells the stages
// leave walkable.
static inline mnavBakeDef Exact(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    return def;
}

// Bakes the scenes from now on with a baker of the def.
static inline void UseDef(const mnavBakeDef* def)
{
    mnavDestroyBaker(s_baker);
    s_baker = nullptr;
    s_def = *def;
    CHECK(mnavCreateBaker(&s_def, &s_baker).result == mnav_success, "baker");
}

// Destroys the baker and the navmesh.
static inline void Done(void)
{
    mnavDestroyNavmesh(s_navmesh);
    mnavDestroyBaker(s_baker);
    s_navmesh = nullptr;
    s_baker = nullptr;
}

#endif // MAUL_NAV_TEST_SQUARES_H
