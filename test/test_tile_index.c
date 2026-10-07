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

enum
{
    OUTLINES = 120,
    RING = 12
};

static mnavVec2 s_points[OUTLINES][RING];
static mnavOutline s_outlines[OUTLINES];

// Random outlines about a place: a floor under all, boxes standing on the
// frames' edges, obstructions among them, and rings of many points.
static void MakeOutlines(const mnavBakeDef* def, Place place)
{
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(def, &cells).result == mnav_success, "a valid def");
    double side = (double)def->tileCells * (double)def->cellSize;
    float x0 = (float)((double)place.tileX * side - 4.0);
    float x1 = (float)((double)(place.tileX + place.columns) * side + 4.0);
    float z0 = (float)((double)place.tileZ * side - 4.0);
    float z1 = (float)((double)(place.tileZ + place.rows) * side + 4.0);
    for (int32_t o = 0; o < OUTLINES; ++o)
    {
        mnavVec2* p = s_points[o];
        int32_t count = 4;
        if (o == 0)
        {
            const mnavVec2 floor[4] = {{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}};
            memcpy(p, floor, sizeof(floor));
        }
        else if (o % 3 == 0)
        {
            // A ring of many points, up to 4 m across.
            float cx = Random(x0, x1);
            float cz = Random(z0, z1);
            float r = Random(0.5f, 4.0f);
            count = RING;
            for (int32_t i = 0; i < RING; ++i)
            {
                // Every other ring a star whose last point reaches
                // furthest, so no bound may skip a point.
                double a = 6.283185307179586 * (double)i / (double)RING;
                float reach = o % 6 == 3 && i == RING - 1 ? 2.0f * r : r;
                p[i] = (mnavVec2){cx + reach * (float)cos(a), cz + reach * (float)sin(a)};
            }
        }
        else
        {
            // A box with a side on a frame's edge.
            float ax =
                o % 2 == 0 ? Edge(def, cells.border, place.tileX, place.columns) : Random(x0, x1);
            float az =
                o % 2 == 1 ? Edge(def, cells.border, place.tileZ, place.rows) : Random(z0, z1);
            float w = Random(0.3f, 3.0f);
            const mnavVec2 box[4] = {{ax, az}, {ax + w, az}, {ax + w, az + w}, {ax, az + w}};
            memcpy(p, box, sizeof(box));
        }
        mnavAreaType area =
            o == 0 ? mnav_areaWalkable : (o % 4 == 1 ? mnav_areaNone : (mnavAreaType)(2 + o % 3));
        s_outlines[o] = (mnavOutline){p, count, area};
    }
}

// Bakes a 2D tile by mnavBakeTile2D, and by mnavBakeTile2DInput without
// and with the index; whether all three agree.
static bool Bakes2DAlike(mnavBaker* baker, const mnavTileIndex* index, int32_t x, int32_t z)
{
    static uint8_t third[TILE_ROOM];
    uint8_t* out[3] = {s_plain, s_indexed, third};
    size_t sizes[3] = {0, 0, 0};
    mnavResult results[3];
    mnavBakeReport reports[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        const mnavBake2DInput input = {s_outlines, OUTLINES, k == 2 ? index : nullptr};
        results[k] = k == 0 ? mnavBakeTile2D(baker, s_outlines, OUTLINES, x, z, &reports[k])
                            : mnavBakeTile2DInput(baker, &input, x, z, &reports[k]);
        if (results[k] == mnav_success &&
            mnavCopyBakedTile(baker, out[k], TILE_ROOM, &sizes[k]) != mnav_success)
        {
            return false;
        }
    }
    bool same = true;
    for (int32_t k = 1; k < 3; ++k)
    {
        same = same && results[k] == results[0] && reports[k].triangles == reports[0].triangles &&
               reports[k].fingerprint == reports[0].fingerprint && sizes[k] == sizes[0] &&
               memcmp(out[k], out[0], sizes[0]) == 0;
    }
    return same;
}

static void TestSameBytes2D(const mnavBakeDef* def, Place place, const char* where)
{
    MakeOutlines(def, place);
    mnavBaker* baker = nullptr;
    mnavTileIndex* index = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success &&
              mnavCreateTileIndex2D(def, s_outlines, OUTLINES, &index, &report) == mnav_success,
          "made");
    int32_t alike = 0;
    int32_t tiles = 0;
    for (int32_t z = place.tileZ - 3; z < place.tileZ + place.rows + 3; ++z)
    {
        for (int32_t x = place.tileX - 3; x < place.tileX + place.columns + 3; ++x)
        {
            alike += Bakes2DAlike(baker, index, x, z) ? 1 : 0;
            tiles += 1;
        }
    }
    printf("tile index of outlines, %s: %d of %d tiles alike\n", where, alike, tiles);
    CHECK(alike == tiles, where);
    mnavDestroyTileIndex(index);
    mnavDestroyBaker(baker);
}

static void TestRefusals2D(void)
{
    mnavBakeDef def = Def();
    MakeOutlines(&def, (Place){0, 0, 3, 3});
    mnavTriangleMesh meshes[MESHES];
    MakeMeshes(meshes, &def, (Place){0, 0, 3, 3});
    mnavTileIndex* outlines = nullptr;
    mnavTileIndex* triangles = nullptr;
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateTileIndex2D(&def, s_outlines, OUTLINES, &outlines, &report) == mnav_success &&
              mnavCreateTileIndex(&def, meshes, MESHES, &triangles, &report) == mnav_success &&
              mnavCreateBaker(&def, &baker).result == mnav_success,
          "made");
    mnavBake2DInput flat = {s_outlines, OUTLINES, triangles};
    CHECK(mnavBakeTile2DInput(baker, &flat, 1, 1, &report) == mnav_errorInvalid,
          "an index of meshes refused by a 2D bake");
    const mnavBakeInput solid = {meshes, MESHES, nullptr, 0, nullptr, 0, outlines};
    CHECK(mnavBakeTileInput(baker, &solid, 1, 1, &report) == mnav_errorInvalid,
          "an index of outlines refused by a 3D bake");
    // The kind is checked even where the counts agree: one outline of 4
    // points, one mesh of 4 triangles.
    mnavTileIndex* square = nullptr;
    CHECK(mnavCreateTileIndex2D(&def, s_outlines + 1, 1, &square, &report) == mnav_success, "made");
    mnavTriangleMesh four = meshes[0];
    four.triangleCount = 4;
    const mnavBakeInput alike = {&four, 1, nullptr, 0, nullptr, 0, square};
    CHECK(s_outlines[1].pointCount == 4 &&
              mnavBakeTileInput(baker, &alike, 0, 0, &report) == mnav_errorInvalid,
          "an index of outlines refused where the counts agree");
    mnavDestroyTileIndex(square);
    flat.index = outlines;
    flat.outlineCount = OUTLINES - 1;
    CHECK(mnavBakeTile2DInput(baker, &flat, 1, 1, &report) == mnav_errorInvalid,
          "another number of outlines refused");
    flat.outlineCount = OUTLINES;
    s_outlines[3].pointCount -= 1;
    CHECK(mnavBakeTile2DInput(baker, &flat, 1, 1, &report) == mnav_errorInvalid,
          "another number of points refused");
    s_outlines[3].pointCount += 1;
    CHECK(mnavBakeTile2DInput(baker, &flat, 1, 1, &report) == mnav_success &&
              mnavBakeTile2DInput(baker, nullptr, 1, 1, &report) == mnav_errorInvalid,
          "fits again; a NULL input refused");
    // An outline changed since is checked as it is read.
    s_points[0][1].x = NAN;
    CHECK(mnavBakeTile2DInput(baker, &flat, 1, 1, &report) == mnav_errorInvalid && report.mesh == 0,
          "a point not finite, refused as read");
    mnavTileIndex* none = nullptr;
    mnavBakeDef tight = def;
    tight.limits.memoryBytes = 256;
    CHECK(mnavCreateTileIndex2D(&tight, s_outlines, OUTLINES, &none, &report) ==
                  mnav_errorInvalid &&
              report.mesh == 0 && none == nullptr,
          "an outline its check refuses, named");
    MakeOutlines(&def, (Place){0, 0, 3, 3});
    s_points[2][1].x = -1e30f;
    CHECK(mnavCreateTileIndex2D(&def, s_outlines, OUTLINES, &none, &report) == mnav_errorRange &&
              report.mesh == 2 && none == nullptr,
          "a point past the extent, named");
    MakeOutlines(&def, (Place){0, 0, 3, 3});
    CHECK(mnavCreateTileIndex2D(&tight, s_outlines, OUTLINES, &none, &report) == mnav_errorLimit &&
              none == nullptr,
          "past the memory limit");
    mnavDestroyBaker(baker);
    mnavDestroyTileIndex(triangles);
    mnavDestroyTileIndex(outlines);
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
    mnavVec3 corner = s_vertices[2][5];
    s_vertices[2][5].z = 1e30f;
    CHECK(mnavCreateTileIndex(&def, meshes, MESHES, &index, &report) == mnav_errorRange &&
              report.mesh == 2 && report.input.element == mnav_elementVertex && index == nullptr,
          "a vertex past the extent, named");
    s_vertices[2][5] = corner;
    mnavBakeDef tight = def;
    tight.limits.memoryBytes = 256;
    CHECK(mnavCreateTileIndex(&tight, meshes, MESHES, &index, &report) == mnav_errorLimit &&
              index == nullptr,
          "past the memory limit");
    // Two triangles near opposite corners of the extent, in tiles of 16
    // cells of 0.25 m: about 500,000 tiles a side, more than 2^31 - 1 in
    // all, though each triangle reaches only a few.
    mnavBakeDef small = def;
    small.tileCells = 16;
    const mnavVec3 far[6] = {{-1.0e6f, 0.0f, -1.0e6f},    {-1.0e6f, 0.0f, -999999.0f},
                             {-999999.0f, 0.0f, -1.0e6f}, {1.0e6f, 0.0f, 1.0e6f},
                             {999999.0f, 0.0f, 1.0e6f},   {1.0e6f, 0.0f, 999999.0f}};
    const int32_t corners[6] = {0, 1, 2, 3, 4, 5};
    const mnavTriangleMesh apart = {far, 6, corners, 2, nullptr};
    CHECK(mnavCreateTileIndex(&small, &apart, 1, &index, &report) == mnav_errorLimit &&
              index == nullptr,
          "a grid of more tiles than 2^31 - 1 refused");
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
    mnavBakeDef flat = Def();
    TestSameBytes2D(&flat, (Place){-3, -2, 6, 3}, "about the origin");
    flat.cellSize = 0.3f;
    TestSameBytes2D(&flat, (Place){131060, 131062, 5, 3}, "near the extent, rounding");
    TestRefusals2D();
    return s_failures == 0 ? 0 : 1;
}
