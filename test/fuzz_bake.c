// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the bake's input (mnav-0002, mnav-0003): triangle meshes,
// terrains and volumes, or 2D outlines, read from the bytes as a host
// might hand them over, with NaNs, infinities, huge values, indices out of
// range and areas past the last. Most values are drawn from a small set of
// sensible ones so that bakes reach every stage. A bake must refuse with a
// typed status or succeed; a tile it makes must bake again to the same
// bytes, load into a navmesh, and every byte taken must come back.

#include "counting_allocator.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    MOST_VERTICES = 16,
    MOST_TRIANGLES = 16,
    MOST_SIDE = 8,
    MOST_POINTS = 8,
    TILE_ROOM = 1 << 20
};

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

// The bytes, read in order, 0 past the end.
typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

// A float: one time in eight its raw bits, otherwise a quarter meter step
// from -16 to 16 m.
static float Float(Reader* r)
{
    uint8_t pick = Byte(r);
    if (pick % 8 == 0)
    {
        uint32_t bits = (uint32_t)Byte(r) | (uint32_t)Byte(r) << 8 | (uint32_t)Byte(r) << 16 |
                        (uint32_t)Byte(r) << 24;
        float f;
        memcpy(&f, &bits, sizeof(f));
        return f;
    }
    return ((float)Byte(r) - 64.0f) * 0.25f;
}

// An area: one time in sixteen past the last type, often walkable.
static mnavAreaType Area(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? (mnavAreaType)(MNAV_AREA_TYPES + pick % 8)
                          : (mnavAreaType)(pick % 4 == 0 ? 0 : 1 + pick % 3);
}

// Everything one input holds.
typedef struct Input
{
    mnavVec3 vertices[2][MOST_VERTICES];
    int32_t indices[2][MOST_TRIANGLES * 3];
    mnavAreaType areas[2][MOST_TRIANGLES];
    mnavTriangleMesh meshes[2];
    float heights[MOST_SIDE * MOST_SIDE];
    mnavAreaType cells[MOST_SIDE * MOST_SIDE];
    mnavTerrain terrain;
    mnavVec2 points[3][MOST_POINTS];
    mnavBakeVolume volumes[2];
    mnavOutline outlines[3];
} Input;

static void ReadMesh(Reader* r, Input* in, int32_t m)
{
    int32_t vertices = Byte(r) % (MOST_VERTICES + 1);
    int32_t triangles = Byte(r) % (MOST_TRIANGLES + 1);
    for (int32_t v = 0; v < vertices; ++v)
    {
        in->vertices[m][v] = (mnavVec3){Float(r), Float(r), Float(r)};
    }
    for (int32_t i = 0; i < triangles * 3; ++i)
    {
        // Now and then one before the first or past the last.
        in->indices[m][i] = (int32_t)(Byte(r) % (uint32_t)(vertices + 2)) - 1;
    }
    for (int32_t t = 0; t < triangles; ++t)
    {
        in->areas[m][t] = Area(r);
    }
    bool areas = Byte(r) % 2 == 0;
    in->meshes[m] = (mnavTriangleMesh){in->vertices[m], vertices, in->indices[m], triangles,
                                       areas ? in->areas[m] : nullptr};
}

static void ReadTerrain(Reader* r, Input* in)
{
    // Sides from 0 to MOST_SIDE, those under 2 refused.
    int32_t columns = Byte(r) % (MOST_SIDE + 1);
    int32_t rows = Byte(r) % (MOST_SIDE + 1);
    for (int32_t i = 0; i < columns * rows; ++i)
    {
        in->heights[i] = Float(r) * 0.25f;
    }
    for (int32_t i = 0; i < MOST_SIDE * MOST_SIDE; ++i)
    {
        in->cells[i] = i < (columns - 1) * (rows - 1) ? Area(r) : 0;
    }
    float spacing = Byte(r) % 4 == 0 ? Float(r) : 0.5f + (float)(Byte(r) % 8);
    bool areas = Byte(r) % 2 == 0;
    in->terrain = (mnavTerrain){{Float(r), Float(r) * 0.25f, Float(r)},
                                spacing,
                                spacing,
                                columns,
                                rows,
                                in->heights,
                                areas ? in->cells : nullptr};
}

// A ring of up to MOST_POINTS points, its count from 0 so that short
// ones come up.
static int32_t ReadRing(Reader* r, mnavVec2* points)
{
    int32_t count = Byte(r) % (MOST_POINTS + 1);
    for (int32_t i = 0; i < count; ++i)
    {
        points[i] = (mnavVec2){Float(r), Float(r)};
    }
    return count;
}

// Bakes one tile twice, checking the outcome; returns the tile's size or
// 0.
static size_t BakeTwice(mnavBaker* baker, const Input* in, bool flat, int32_t outlines,
                        const mnavBakeInput* input, int32_t tileX, int32_t tileZ, uint8_t* tile)
{
    mnavBakeReport report;
    mnavResult result = flat ? mnavBakeTile2D(baker, in->outlines, outlines, tileX, tileZ, &report)
                             : mnavBakeTileInput(baker, input, tileX, tileZ, &report);
    Expect(result == report.result);
    if (result != mnav_success)
    {
        Expect(result == mnav_errorInvalid || result == mnav_errorRange ||
               result == mnav_errorLimit);
        return 0;
    }
    size_t size = 0;
    Expect(mnavCopyBakedTile(baker, tile, TILE_ROOM, &size) == mnav_success && size > 0);
    mnavBakeReport again;
    mnavResult second = flat ? mnavBakeTile2D(baker, in->outlines, outlines, tileX, tileZ, &again)
                             : mnavBakeTileInput(baker, input, tileX, tileZ, &again);
    static uint8_t copy[TILE_ROOM];
    size_t copySize = 0;
    Expect(second == mnav_success && again.fingerprint == report.fingerprint &&
           mnavCopyBakedTile(baker, copy, TILE_ROOM, &copySize) == mnav_success &&
           copySize == size && memcmp(copy, tile, size) == 0);
    return size;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    Reader r = {data, size, 0};
    static Input in;
    memset(&in, 0, sizeof(in));
    uint8_t mode = Byte(&r);
    bool flat = mode % 2 == 1;
    int32_t tileX = (int32_t)(Byte(&r) % 8) - 4;
    int32_t tileZ = (int32_t)(Byte(&r) % 8) - 4;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    def.tileCells = 32;
    def.limits.memoryBytes = (uint64_t)64 << 20;
    mnavBaker* baker = nullptr;
    Expect(mnavCreateBaker(&def, &baker).result == mnav_success);
    int32_t meshes = Byte(&r) % 3;
    for (int32_t m = 0; m < meshes; ++m)
    {
        ReadMesh(&r, &in, m);
    }
    bool terrain = Byte(&r) % 2 == 0;
    if (terrain)
    {
        ReadTerrain(&r, &in);
    }
    int32_t volumes = Byte(&r) % 3;
    for (int32_t v = 0; v < volumes; ++v)
    {
        int32_t count = ReadRing(&r, in.points[v]);
        float low = Float(&r);
        in.volumes[v] = (mnavBakeVolume){in.points[v], count,   low, low + (float)(Byte(&r) % 16),
                                         Byte(&r) % 4, Area(&r)};
    }
    int32_t outlines = Byte(&r) % 4;
    for (int32_t o = 0; o < outlines && flat; ++o)
    {
        int32_t count = ReadRing(&r, in.points[o]);
        in.outlines[o] = (mnavOutline){in.points[o], count, Area(&r)};
    }
    const mnavBakeInput input = {in.meshes,  meshes,  &in.terrain, terrain ? 1 : 0,
                                 in.volumes, volumes, nullptr};
    static uint8_t tile[TILE_ROOM];
    size_t tileSize = BakeTwice(baker, &in, flat, outlines, &input, tileX, tileZ, tile);
    // A tile index of the meshes (mnav-0014), made only when they all
    // pass their checks, gives the same result and the same bytes.
    mnavTileIndex* index = nullptr;
    if (flat ? mnavCreateTileIndex2D(&def, in.outlines, outlines, &index, nullptr) == mnav_success
             : mnavCreateTileIndex(&def, in.meshes, meshes, &index, nullptr) == mnav_success)
    {
        mnavBakeInput indexed = input;
        indexed.index = index;
        const mnavBake2DInput flatIndexed = {in.outlines, outlines, index};
        mnavBakeReport report;
        mnavResult result = flat ? mnavBakeTile2DInput(baker, &flatIndexed, tileX, tileZ, &report)
                                 : mnavBakeTileInput(baker, &indexed, tileX, tileZ, &report);
        static uint8_t copy[TILE_ROOM];
        size_t copySize = 0;
        Expect((result == mnav_success) == (tileSize > 0));
        Expect(result != mnav_success ||
               (mnavCopyBakedTile(baker, copy, TILE_ROOM, &copySize) == mnav_success &&
                copySize == tileSize && memcmp(copy, tile, tileSize) == 0));
        mnavDestroyTileIndex(index);
    }
    mnavDestroyBaker(baker);
    if (tileSize > 0)
    {
        mnavNavmesh* navmesh = nullptr;
        Expect(mnavCreateNavmesh(&def, &navmesh).result == mnav_success);
        Expect(mnavStageTile(navmesh, tile, tileSize).result == mnav_success &&
               mnavCommit(navmesh) == mnav_success);
        mnavDestroyNavmesh(navmesh);
    }
    Expect(s_held == 0);
    return 0;
}
