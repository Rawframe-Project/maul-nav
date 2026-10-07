// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tile indexes (mnav-0014): a bake given one makes the same tile, to the
// byte, as a bake reading every triangle; the refusals; the checks of the
// triangles read; the memory limit.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

enum
{
    MESHES = 3,
    TRIANGLES = 160,
    TILE_ROOM = 1 << 20
};

static mnavVec3 s_vertices[MESHES][TRIANGLES * 3];
static int32_t s_indices[MESHES][TRIANGLES * 3];
static mnavAreaType s_areas[MESHES][TRIANGLES];
static uint8_t s_plain[TILE_ROOM];
static uint8_t s_indexed[TILE_ROOM];
static uint32_t s_seed = 7u;

static float Random(float low, float high)
{
    s_seed = s_seed * 1664525u + 1013904223u;
    return low + (high - low) * (float)(s_seed >> 8) / 16777216.0f;
}

// A def of small tiles: 32 cells of 0.25 m, 8 m, and a border of 5 cells.
static mnavBakeDef Def(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    return def;
}

// Random meshes about tiles 0 to 3 of a def of 8 m tiles: floors and
// slivers, triangles across tiles, and walls exactly on tile sides and on
// the borders' edges, 1.25 m either side of them.
static void MakeMeshes(mnavTriangleMesh meshes[MESHES], float offset)
{
    for (int32_t m = 0; m < MESHES; ++m)
    {
        for (int32_t t = 0; t < TRIANGLES; ++t)
        {
            mnavVec3* v = &s_vertices[m][t * 3];
            float x = Random(-4.0f, 36.0f);
            float z = Random(-4.0f, 36.0f);
            float y = Random(0.0f, 2.0f);
            int32_t kind = t % 4;
            if (kind == 0)
            {
                // A floor triangle up to 3 m across, wound up.
                float s = Random(0.2f, 3.0f);
                v[0] = (mnavVec3){x, y, z};
                v[1] = (mnavVec3){x, y, z + s};
                v[2] = (mnavVec3){x + s, y, z};
            }
            else if (kind == 1)
            {
                // A wall standing on a tile's side or a border's edge.
                const float edges[3] = {0.0f, -1.25f, 1.25f};
                float at = 8.0f * (float)(int32_t)Random(0.0f, 5.0f) + edges[t % 3];
                v[0] = (mnavVec3){at, 0.0f, z};
                v[1] = (mnavVec3){at, 2.0f, z + 0.5f};
                v[2] = (mnavVec3){at, 0.0f, z + 1.0f};
            }
            else if (kind == 2)
            {
                // A large triangle across several tiles.
                v[0] = (mnavVec3){x, y, z};
                v[1] = (mnavVec3){Random(-4.0f, 36.0f), y, Random(-4.0f, 36.0f)};
                v[2] = (mnavVec3){Random(-4.0f, 36.0f), y + 0.5f, Random(-4.0f, 36.0f)};
            }
            else
            {
                // A sliver.
                v[0] = (mnavVec3){x, y, z};
                v[1] = (mnavVec3){x + 0.001f, y, z + 2.0f};
                v[2] = (mnavVec3){x + 2.0f, y, z + 0.001f};
            }
            for (int32_t c = 0; c < 3; ++c)
            {
                v[c].x += offset;
                v[c].z += offset;
                s_indices[m][t * 3 + c] = t * 3 + c;
            }
            s_areas[m][t] = (mnavAreaType)(1 + t % 3);
        }
        meshes[m] = (mnavTriangleMesh){s_vertices[m], TRIANGLES * 3, s_indices[m], TRIANGLES,
                                       m == 1 ? nullptr : s_areas[m]};
    }
}

// Bakes a tile without and with the index; whether both give the same
// result, report and bytes.
static bool BakesAlike(mnavBaker* baker, const mnavBakeInput* input, int32_t x, int32_t z)
{
    mnavBakeInput indexed = *input;
    mnavBakeInput plain = *input;
    plain.index = nullptr;
    mnavBakeReport a;
    mnavBakeReport b;
    size_t first = 0;
    size_t second = 0;
    mnavResult ra = mnavBakeTileInput(baker, &plain, x, z, &a);
    if (ra == mnav_success && mnavCopyBakedTile(baker, s_plain, TILE_ROOM, &first) != mnav_success)
    {
        return false;
    }
    mnavResult rb = mnavBakeTileInput(baker, &indexed, x, z, &b);
    if (rb == mnav_success &&
        mnavCopyBakedTile(baker, s_indexed, TILE_ROOM, &second) != mnav_success)
    {
        return false;
    }
    return ra == rb && a.triangles == b.triangles && a.fingerprint == b.fingerprint &&
           first == second && memcmp(s_plain, s_indexed, first) == 0;
}

static void TestSameBytes(float offset, int32_t from, int32_t to, const char* where)
{
    mnavBakeDef def = Def();
    mnavTriangleMesh meshes[MESHES];
    MakeMeshes(meshes, offset);
    mnavBaker* baker = nullptr;
    mnavTileIndex* index = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateTileIndex(&def, meshes, MESHES, &index, &report) == mnav_success &&
              report.memoryPeak > 0,
          "made");
    const mnavBakeInput input = {meshes, MESHES, nullptr, 0, nullptr, 0, index};
    int32_t alike = 0;
    int32_t tiles = 0;
    for (int32_t z = from; z <= to; ++z)
    {
        for (int32_t x = from; x <= to; ++x)
        {
            alike += BakesAlike(baker, &input, x, z) ? 1 : 0;
            tiles += 1;
        }
    }
    printf("tile index, %s: %d of %d tiles alike\n", where, alike, tiles);
    CHECK(alike == tiles, where);
    mnavDestroyTileIndex(index);
    mnavDestroyBaker(baker);
}

static void TestRefusals(void)
{
    mnavBakeDef def = Def();
    mnavTriangleMesh meshes[MESHES];
    MakeMeshes(meshes, 0.0f);
    mnavTileIndex* index = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateTileIndex(&def, meshes, MESHES, nullptr, &report) == mnav_errorInvalid &&
              mnavCreateTileIndex(nullptr, meshes, MESHES, &index, &report) == mnav_errorInvalid &&
              mnavCreateTileIndex(&def, nullptr, 1, &index, &report) == mnav_errorInvalid &&
              mnavCreateTileIndex(&def, meshes, -1, &index, nullptr) == mnav_errorInvalid &&
              index == nullptr,
          "NULL and negative arguments refused");
    mnavTriangleMesh bad[MESHES];
    memcpy(bad, meshes, sizeof(bad));
    int32_t wrong[3] = {0, 1, TRIANGLES * 3};
    bad[2] = (mnavTriangleMesh){s_vertices[2], TRIANGLES * 3, wrong, 1, nullptr};
    CHECK(mnavCreateTileIndex(&def, bad, MESHES, &index, &report) == mnav_errorInvalid &&
              report.mesh == 2 && report.input.element == mnav_elementTriangle && index == nullptr,
          "a mesh its check refuses, named");
    mnavBakeDef tight = def;
    tight.limits.memoryBytes = 256;
    CHECK(mnavCreateTileIndex(&tight, meshes, MESHES, &index, &report) == mnav_errorLimit &&
              index == nullptr,
          "past the memory limit");
    CHECK(mnavCreateTileIndex(&def, meshes, 0, &index, &report) == mnav_success, "no meshes");
    mnavDestroyTileIndex(index);
    mnavDestroyTileIndex(nullptr);

    // An index made for another grid or other meshes is refused.
    CHECK(mnavCreateTileIndex(&def, meshes, MESHES, &index, &report) == mnav_success, "made");
    mnavBaker* baker = nullptr;
    mnavBakeDef wider = def;
    wider.agent.radius = 1.0f;
    mnavBakeDef larger = def;
    larger.tileCells = 64;
    mnavBakeDef finer = def;
    finer.cellSize = 0.2f;
    const mnavBakeDef* others[3] = {&wider, &larger, &finer};
    mnavBakeInput input = {meshes, MESHES, nullptr, 0, nullptr, 0, index};
    for (int32_t k = 0; k < 3; ++k)
    {
        CHECK(mnavCreateBaker(others[k], &baker).result == mnav_success &&
                  mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_errorInvalid,
              "another grid refused");
        mnavDestroyBaker(baker);
    }
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    input.meshCount = MESHES - 1;
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_errorInvalid,
          "another number of meshes refused");
    input.meshCount = MESHES;
    meshes[1].triangleCount -= 1;
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_errorInvalid,
          "another number of triangles refused");
    meshes[1].triangleCount += 1;

    // A triangle changed since is checked as it is read.
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_success && report.triangles > 0,
          "baked");
    for (int32_t t = 0; t < TRIANGLES; ++t)
    {
        s_indices[0][t * 3] = -1;
    }
    mnavResult result = mnavBakeTileInput(baker, &input, 1, 1, &report);
    CHECK(result == mnav_errorInvalid && report.mesh == 0 &&
              report.input.element == mnav_elementTriangle,
          "a corner out of range, refused as read");
    for (int32_t t = 0; t < TRIANGLES; ++t)
    {
        s_indices[0][t * 3] = t * 3;
    }
    static mnavVec3 kept[TRIANGLES * 3];
    memcpy(kept, s_vertices[2], sizeof(kept));
    for (int32_t v = 0; v < TRIANGLES * 3; ++v)
    {
        s_vertices[2][v].y = NAN;
    }
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_errorInvalid &&
              report.mesh == 2 && report.input.element == mnav_elementVertex,
          "a vertex not finite, refused as read");
    memcpy(s_vertices[2], kept, sizeof(kept));
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_success, "restored");
    mnavDestroyBaker(baker);
    mnavDestroyTileIndex(index);
}

int main(void)
{
    TestSameBytes(0.0f, -1, 4, "tiles -1 to 4");
    // Near the largest extent, 2^22 cells of 0.25 m, where the tiles'
    // corners round most: the meshes reach tiles 131062 to 131067.
    TestSameBytes(1048500.0f, 131061, 131068, "near the extent");
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
