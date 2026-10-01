// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tiles built by hand for white-box tests: up to four unlinked square
// polygons, each flat or sloped, encoded with the default def's settings.

#ifndef MAUL_NAV_TEST_HAND_TILE_H
#define MAUL_NAV_TEST_HAND_TILE_H

#include "allocator.h"
#include "detail.h"
#include "polymesh.h"
#include "test_harness.h"
#include "tile.h"

#include "maul-nav/bake.h"

#include <stdint.h>
#include <string.h>

// A square from cell (x0, z0) to (x1, z1); heights in cell heights above
// the offset at its corners (x0, z0), (x0, z1), (x1, z1), (x1, z0).
typedef struct HandSquare
{
    int32_t x0;
    int32_t z0;
    int32_t x1;
    int32_t z1;
    int32_t y[4];
} HandSquare;

static inline uint8_t HandSide(int32_t ax, int32_t az, int32_t bx, int32_t bz, int32_t size)
{
    if (ax == 0 && bx == 0)
    {
        return 1;
    }
    if (az == size && bz == size)
    {
        return 2;
    }
    if (ax == size && bx == size)
    {
        return 3;
    }
    return az == 0 && bz == 0 ? 4 : 0;
}

// Encodes the squares as the tile at (place, 0) into out; returns its size.
static inline size_t HandTileBytes(uint8_t* out, int32_t place, const HandSquare* squares,
                                   int32_t count)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    mnavMeshVertex vertices[16];
    uint8_t removable[16] = {0};
    mnavPolygon polygons[4];
    mnavDetailVertex detailVertices[16];
    mnavDetailPart parts[4];
    mnavDetailTriangle triangles[8];
    for (int32_t s = 0; s < count; ++s)
    {
        const HandSquare* q = &squares[s];
        const int32_t xs[4] = {q->x0, q->x0, q->x1, q->x1};
        const int32_t zs[4] = {q->z0, q->z1, q->z1, q->z0};
        mnavPolygon* polygon = &polygons[s];
        *polygon = (mnavPolygon){{0}, {0}, {0}, 4, 1, 0};
        memset(polygon->vertices, 0xFF, sizeof(polygon->vertices));
        memset(polygon->neighbors, 0xFF, sizeof(polygon->neighbors));
        for (int32_t k = 0; k < 4; ++k)
        {
            int32_t v = s * 4 + k;
            vertices[v] =
                (mnavMeshVertex){(uint16_t)xs[k], (uint16_t)(32768 + q->y[k]), (uint16_t)zs[k]};
            detailVertices[v] = (mnavDetailVertex){xs[k] * 16, 32768 + q->y[k], zs[k] * 16};
            polygon->vertices[k] = (uint16_t)v;
            polygon->sides[k] =
                HandSide(xs[k], zs[k], xs[(k + 1) % 4], zs[(k + 1) % 4], def.tileCells);
        }
        parts[s] = (mnavDetailPart){s * 4, s * 2, 4, 2};
        triangles[s * 2] = (mnavDetailTriangle){{0, 1, 2}, 3};
        triangles[s * 2 + 1] = (mnavDetailTriangle){{0, 2, 3}, 6};
    }
    mnavPolyMesh mesh = {vertices, removable, count * 4, 16, polygons, count, 4, def.tileCells, 0};
    mnavDetailMesh detail = {
        parts, count, detailVertices, count * 4, 16, triangles, count * 2, 8, 0, 0, 0};
    mnavTileInfo info = {mnavGetVersion(),
                         0,
                         place,
                         0,
                         def.tileCells,
                         def.cellSize,
                         def.cellHeight,
                         cells.agentHeight,
                         cells.agentRadius,
                         cells.agentStep,
                         def.origin};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    uint8_t* bytes = nullptr;
    size_t size = 0;
    CHECK(mnavEncodeTile(&memory, &info, &mesh, &detail, &bytes, &size) == mnav_success, "encoded");
    memcpy(out, bytes, size);
    mnavReleaseTileBytes(&memory, bytes, size);
    return size;
}

#endif // MAUL_NAV_TEST_HAND_TILE_H
