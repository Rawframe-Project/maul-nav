// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Terrains (mnav-0003): a terrain bakes to the same bytes as the triangle
// mesh of its cells; holes; the checks; terrains with meshes.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the terrain's tile, the same on every platform.
#define TERRAIN_HASH 0x96d19eef7162a8f8ull

enum
{
    SIDE = 40,
    CELLS = (SIDE - 1) * (SIDE - 1),
    TILE_ROOM = 1 << 20
};

static float s_heights[SIDE * SIDE];
static mnavAreaType s_areas[CELLS];
static mnavVec3 s_vertices[SIDE * SIDE];
static int32_t s_indices[CELLS * 6];
static uint8_t s_first[TILE_ROOM];
static uint8_t s_second[TILE_ROOM];

// A slope rising along X with a plateau, 1 m between samples, starting 2 m
// before tile (0, 0); the plateau's sides are too steep to climb.
static mnavTerrain Terrain(void)
{
    for (int32_t r = 0; r < SIDE; ++r)
    {
        for (int32_t c = 0; c < SIDE; ++c)
        {
            int32_t dx = c - 18;
            int32_t dz = r - 14;
            s_heights[r * SIDE + c] = 0.125f * (float)c + (dx * dx + dz * dz < 36 ? 1.5f : 0.0f);
        }
    }
    return (mnavTerrain){{-2.0f, 1.0f, -2.0f}, 1.0f, 1.0f, SIDE, SIDE, s_heights, nullptr};
}

// The same cells as a triangle mesh, split and wound as the terrain is.
static mnavTriangleMesh Mesh(const mnavTerrain* t)
{
    int32_t w = t->columns;
    for (int32_t r = 0; r < t->rows; ++r)
    {
        for (int32_t c = 0; c < w; ++c)
        {
            s_vertices[r * w + c] =
                (mnavVec3){(float)((double)t->origin.x + (double)c * (double)t->spacingX),
                           (float)((double)t->origin.y + (double)t->heights[r * w + c]),
                           (float)((double)t->origin.z + (double)r * (double)t->spacingZ)};
        }
    }
    int32_t n = 0;
    for (int32_t r = 0; r + 1 < t->rows; ++r)
    {
        for (int32_t c = 0; c + 1 < w; ++c)
        {
            int32_t a = r * w + c;
            const int32_t corners[6] = {a, a + w, a + w + 1, a, a + w + 1, a + 1};
            memcpy(&s_indices[n], corners, sizeof(corners));
            n += 6;
        }
    }
    return (mnavTriangleMesh){s_vertices, w * t->rows, s_indices, n / 3, nullptr};
}

// Whether a terrain bakes tile (0, 0) to the same bytes as the mesh of
// its cells.
static bool Same(mnavBaker* baker, const mnavTerrain* terrain, mnavBakeReport* report)
{
    mnavTriangleMesh mesh = Mesh(terrain);
    const mnavBakeInput input = {nullptr, 0, terrain, 1, nullptr, 0, nullptr};
    mnavBakeReport other;
    size_t first = 0;
    size_t second = 0;
    bool baked = mnavBakeTileInput(baker, &input, 0, 0, report) == mnav_success &&
                 mnavCopyBakedTile(baker, s_first, TILE_ROOM, &first) == mnav_success &&
                 mnavBakeTile(baker, &mesh, 1, 0, 0, &other) == mnav_success &&
                 mnavCopyBakedTile(baker, s_second, TILE_ROOM, &second) == mnav_success;
    return baked && report->triangles == other.triangles && first == second &&
           memcmp(s_first, s_second, first) == 0;
}

static size_t Bake(mnavBaker* baker, const mnavBakeInput* input, uint8_t* out,
                   mnavBakeReport* report)
{
    size_t size = 0;
    CHECK(mnavBakeTileInput(baker, input, 0, 0, report) == mnav_success &&
              mnavCopyBakedTile(baker, out, TILE_ROOM, &size) == mnav_success,
          "baked");
    return size;
}

static void TestLikeItsMesh(mnavBaker* baker)
{
    mnavTerrain terrain = Terrain();
    mnavBakeReport a;
    CHECK(Same(baker, &terrain, &a), "the same bytes as the mesh of its cells");
    size_t first = 0;
    CHECK(mnavCopyBakedTile(baker, s_first, TILE_ROOM, &first) == mnav_success, "copied");
    printf("terrain: %d triangles, %d polygons, %zu bytes\n", a.triangles, a.polygons, first);
    CHECK(a.polygons > 0 && a.triangles < CELLS * 2, "only the cells near the tile");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, s_first, (int32_t)first);
    printf("TERRAIN_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == TERRAIN_HASH, "the pinned hash");
    // Cells whose edges fall on the tile's bounds, border included (1.25
    // m outside the tile), on both sides.
    mnavTerrain edged = terrain;
    edged.origin = (mnavVec3){-3.25f, 1.0f, -3.25f};
    CHECK(Same(baker, &edged, &a), "cells on the tile's bounds");
    // Ending inside the tile; then a spacing that rounds.
    mnavTerrain inside = terrain;
    inside.columns = 20;
    inside.rows = 20;
    CHECK(Same(baker, &inside, &a) && a.triangles == 19 * 19 * 2, "ending inside the tile");
    mnavTerrain odd = terrain;
    odd.origin = (mnavVec3){-1.3f, 1.0f, 3.1f};
    odd.spacingX = 0.7f;
    odd.spacingZ = 0.9f;
    CHECK(Same(baker, &odd, &a), "spacings that round");
    const mnavBakeInput fromTerrain = {nullptr, 0, &terrain, 1, nullptr, 0, nullptr};
    // A hole of 6 by 6 cells.
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    for (int32_t r = 10; r < 16; ++r)
    {
        for (int32_t c = 10; c < 16; ++c)
        {
            s_areas[r * (SIDE - 1) + c] = mnav_areaNone;
        }
    }
    terrain.areas = s_areas;
    mnavBakeReport holed;
    mnavBakeReport whole;
    terrain.areas = nullptr;
    Bake(baker, &fromTerrain, s_second, &whole);
    terrain.areas = s_areas;
    Bake(baker, &fromTerrain, s_second, &holed);
    CHECK(holed.triangles == whole.triangles - 72 && holed.spans < whole.spans - 30,
          "a hole has no ground");
    // Far from the terrain, a tile has nothing of it.
    mnavBakeReport far;
    CHECK(mnavBakeTileInput(baker, &fromTerrain, 4, 4, &far) == mnav_success &&
              far.triangles == 0 && far.polygons == 0,
          "a tile it misses");
}

static int32_t Refused(mnavBaker* baker, const mnavTerrain* terrain, mnavResult result,
                       mnavInputElement element)
{
    const mnavTriangleMesh none = {nullptr, 0, nullptr, 0, nullptr};
    const mnavBakeInput input = {&none, 1, terrain, 1, nullptr, 0, nullptr};
    mnavBakeReport report;
    bool refused = mnavBakeTileInput(baker, &input, 0, 0, &report) == result &&
                   report.stage == mnav_stageInput && report.mesh == 1 &&
                   report.input.element == element;
    return refused ? report.input.index : -2;
}

static void TestChecks(mnavBaker* baker)
{
    mnavTerrain good = Terrain();
    mnavTerrain t = good;
    t.columns = 1;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "one column");
    t = good;
    t.rows = MNAV_MAX_TERRAIN_SIDE + 1;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "too many rows");
    t = good;
    t.spacingZ = 0.0f;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "no spacing");
    t = good;
    t.spacingX = (float)INFINITY;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "an endless spacing");
    t = good;
    t.heights = nullptr;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "no heights");
    t = good;
    t.origin.y = (float)NAN;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementNone) == -1, "a NaN origin");
    s_heights[77] = (float)NAN;
    CHECK(Refused(baker, &good, mnav_errorInvalid, mnav_elementSample) == 77, "a NaN height");
    s_heights[77] = 0.0f;
    s_heights[78] = 1e9f;
    CHECK(Refused(baker, &good, mnav_errorRange, mnav_elementSample) == 78, "a height too far");
    s_heights[78] = 0.0f;
    t = good;
    t.spacingX = 1e7f;
    CHECK(Refused(baker, &t, mnav_errorRange, mnav_elementSample) == 1, "a sample too far");
    memset(s_areas, mnav_areaWalkable, sizeof(s_areas));
    s_areas[5] = MNAV_AREA_TYPES;
    t = good;
    t.areas = s_areas;
    CHECK(Refused(baker, &t, mnav_errorInvalid, mnav_elementCell) == 5, "an area out of range");
    const mnavBakeInput missing = {nullptr, 0, nullptr, 1, nullptr, 0, nullptr};
    CHECK(mnavBakeTileInput(baker, &missing, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavBakeTileInput(baker, nullptr, 0, 0, nullptr) == mnav_errorInvalid,
          "terrains missing; no input");
}

static void TestLimits(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.inputTriangles = CELLS * 2 - 1;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTerrain terrain = Terrain();
    const mnavBakeInput input = {nullptr, 0, &terrain, 1, nullptr, 0, nullptr};
    mnavBakeReport report;
    CHECK(mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_errorLimit && report.mesh == 0,
          "one triangle over the input limit");
    // A mesh's triangles and a terrain's count together.
    mnavTriangleMesh mesh = Mesh(&terrain);
    mesh.triangleCount = 1;
    const mnavBakeInput both = {&mesh, 1, &terrain, 1, nullptr, 0, nullptr};
    def.limits.inputTriangles = CELLS * 2;
    mnavDestroyBaker(baker);
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_success &&
              mnavBakeTileInput(baker, &both, 0, 0, &report) == mnav_errorLimit,
          "a mesh's and a terrain's triangles together over the input limit");
    mnavDestroyBaker(baker);
    def = mnavDefaultBakeDef();
    def.limits.tileTriangles = 100;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    CHECK(mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_errorLimit &&
              report.stage == mnav_stageRasterize,
          "over the tile's triangle limit");
    mnavDestroyBaker(baker);
}

int main(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    TestLikeItsMesh(baker);
    TestChecks(baker);
    mnavDestroyBaker(baker);
    TestLimits();
    return s_failures == 0 ? 0 : 1;
}
