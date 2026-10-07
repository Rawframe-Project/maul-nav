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

// Where the meshes lie: the first tile along X and Z and how many tiles
// they span each way.
typedef struct Place
{
    int32_t tileX;
    int32_t tileZ;
    int32_t columns;
    int32_t rows;
} Place;

// A place along one axis where the bake's frames have an edge, computed
// as the bake computes them, a cell count times the cell size, at times a
// float to either side: a tile's side or its border's edge.
static float Edge(const mnavBakeDef* def, int32_t border, int32_t first, int32_t count)
{
    const int32_t offsets[4] = {-border, 0, def->tileCells, def->tileCells + border};
    int32_t tile = first - 1 + (int32_t)Random(0.0f, (float)count + 2.0f);
    int64_t cell = (int64_t)tile * def->tileCells + offsets[(int32_t)Random(0.0f, 4.0f)];
    float at = (float)cell * def->cellSize;
    float side = Random(0.0f, 3.0f);
    return side < 1.0f ? nextafterf(at, -INFINITY) : (side < 2.0f ? at : nextafterf(at, INFINITY));
}

// Random meshes about a place: floors and slivers, triangles across
// tiles, and walls standing on the frames' edges along X and along Z.
static void MakeMeshes(mnavTriangleMesh meshes[MESHES], const mnavBakeDef* def, Place place)
{
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(def, &cells).result == mnav_success, "a valid def");
    double side = (double)def->tileCells * (double)def->cellSize;
    float x0 = (float)((double)place.tileX * side - 4.0);
    float x1 = (float)((double)(place.tileX + place.columns) * side + 4.0);
    float z0 = (float)((double)place.tileZ * side - 4.0);
    float z1 = (float)((double)(place.tileZ + place.rows) * side + 4.0);
    for (int32_t m = 0; m < MESHES; ++m)
    {
        for (int32_t t = 0; t < TRIANGLES; ++t)
        {
            mnavVec3* v = &s_vertices[m][t * 3];
            float x = Random(x0, x1);
            float z = Random(z0, z1);
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
            else if (kind == 1 && t % 8 == 1)
            {
                // A wall on a frame's edge across X.
                float at = Edge(def, cells.border, place.tileX, place.columns);
                v[0] = (mnavVec3){at, 0.0f, z};
                v[1] = (mnavVec3){at, 2.0f, z + 0.5f};
                v[2] = (mnavVec3){at, 0.0f, z + 1.0f};
            }
            else if (kind == 1)
            {
                // A wall on a frame's edge across Z.
                float at = Edge(def, cells.border, place.tileZ, place.rows);
                v[0] = (mnavVec3){x, 0.0f, at};
                v[1] = (mnavVec3){x + 0.5f, 2.0f, at};
                v[2] = (mnavVec3){x + 1.0f, 0.0f, at};
            }
            else if (kind == 2)
            {
                // A large triangle across several tiles.
                v[0] = (mnavVec3){x, y, z};
                v[1] = (mnavVec3){Random(x0, x1), y, Random(z0, z1)};
                v[2] = (mnavVec3){Random(x0, x1), y + 0.5f, Random(z0, z1)};
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

// Bakes every tile the meshes reach, and three more on every side, past
// the tiles the index lists (walls stand on frames' edges a tile out),
// with and without the index.
static void TestSameBytes(const mnavBakeDef* def, Place place, const char* where)
{
    mnavTriangleMesh meshes[MESHES];
    MakeMeshes(meshes, def, place);
    mnavBaker* baker = nullptr;
    mnavTileIndex* index = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success &&
              mnavCreateTileIndex(def, meshes, MESHES, &index, &report) == mnav_success &&
              report.memoryPeak > 0,
          "made");
    const mnavBakeInput input = {meshes, MESHES, nullptr, 0, nullptr, 0, index};
    int32_t alike = 0;
    int32_t tiles = 0;
    for (int32_t z = place.tileZ - 3; z < place.tileZ + place.rows + 3; ++z)
    {
        for (int32_t x = place.tileX - 3; x < place.tileX + place.columns + 3; ++x)
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
    MakeMeshes(meshes, &def, (Place){0, 0, 4, 4});
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
    // Only each triangle's third vertex past the extent: every corner is
    // checked.
    for (int32_t v = 2; v < TRIANGLES * 3; v += 3)
    {
        s_vertices[2][v].x = 1e30f;
    }
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_errorRange && report.mesh == 2 &&
              report.input.element == mnav_elementVertex,
          "a third corner past the extent, refused as read");
    memcpy(s_vertices[2], kept, sizeof(kept));
    CHECK(mnavBakeTileInput(baker, &input, 1, 1, &report) == mnav_success, "restored");
    mnavDestroyBaker(baker);
    mnavDestroyTileIndex(index);
}

int main(void)
{
    mnavBakeDef def = Def();
    // About the origin, below it on both axes, more tiles along X than Z.
    TestSameBytes(&def, (Place){-3, -2, 6, 3}, "about the origin");
    // Near the largest extent, 2^22 cells: of 0.25 m, where a frame's
    // corner is exact, and of 0.3 m, where it rounds by up to half a cell.
    TestSameBytes(&def, (Place){131060, 131062, 3, 5}, "near the extent");
    def.cellSize = 0.3f;
    TestSameBytes(&def, (Place){131060, 131062, 5, 3}, "near the extent, rounding");
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
