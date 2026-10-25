// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tiles built by hand: up to HAND_SQUARES rectangular polygons, each flat
// or sloped, their shared corners welded and the polygons sharing a whole
// edge linked, encoded with the default def's settings. test_hand_tiles.c
// writes the tiles of the suites that test through the public headers
// into hand_tiles.h; the white-box suites build theirs here.

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

enum
{
    HAND_SQUARES = 16
};

// A square from cell (x0, z0) to (x1, z1); heights in cell heights above
// the offset at its corners (x0, z0), (x0, z1), (x1, z1), (x1, z0).
typedef struct HandSquare
{
    int32_t x0;
    int32_t z0;
    int32_t x1;
    int32_t z1;
    int32_t y[4];
    // The area type; 0 for mnav_areaWalkable.
    mnavAreaType area;
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

// The index of a vertex among the first count, added when new.
static inline uint16_t HandWeld(mnavMeshVertex* vertices, int32_t* count, mnavMeshVertex v)
{
    for (int32_t i = 0; i < *count; ++i)
    {
        if (vertices[i].x == v.x && vertices[i].y == v.y && vertices[i].z == v.z)
        {
            return (uint16_t)i;
        }
    }
    vertices[*count] = v;
    return (uint16_t)(*count)++;
}

// Links polygons whose edges run between the same two vertices.
static inline void HandLink(mnavPolygon* polygons, int32_t count)
{
    for (int32_t p = 0; p < count; ++p)
    {
        for (int32_t k = 0; k < 4; ++k)
        {
            for (int32_t q = 0; q < count; ++q)
            {
                for (int32_t j = 0; j < 4; ++j)
                {
                    bool shared = polygons[q].vertices[j] == polygons[p].vertices[(k + 1) % 4] &&
                                  polygons[q].vertices[(j + 1) % 4] == polygons[p].vertices[k];
                    polygons[p].neighbors[k] = shared ? (uint16_t)q : polygons[p].neighbors[k];
                }
            }
        }
    }
}

// Encodes the squares as the tile at (place, 0) into out, the first detail
// triangle of square degenerate (none when -1) the degenerate (0, 2, 2), as
// a tile from elsewhere may hold; returns its size.
static inline size_t HandTileBytesWith(uint8_t* out, int32_t place, const HandSquare* squares,
                                       int32_t count, int32_t degenerate)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    mnavMeshVertex vertices[HAND_SQUARES * 4];
    uint8_t removable[HAND_SQUARES * 4] = {0};
    mnavPolygon polygons[HAND_SQUARES];
    mnavDetailVertex detailVertices[HAND_SQUARES * 4];
    mnavDetailPart parts[HAND_SQUARES];
    mnavDetailTriangle triangles[HAND_SQUARES * 2];
    int32_t vertexCount = 0;
    for (int32_t s = 0; s < count; ++s)
    {
        const HandSquare* q = &squares[s];
        const int32_t xs[4] = {q->x0, q->x0, q->x1, q->x1};
        const int32_t zs[4] = {q->z0, q->z1, q->z1, q->z0};
        mnavPolygon* polygon = &polygons[s];
        *polygon = (mnavPolygon){{0}, {0}, {0}, 4, q->area != 0 ? q->area : 1, 0};
        memset(polygon->vertices, 0xFF, sizeof(polygon->vertices));
        memset(polygon->neighbors, 0xFF, sizeof(polygon->neighbors));
        for (int32_t k = 0; k < 4; ++k)
        {
            int32_t v = s * 4 + k;
            mnavMeshVertex corner = {(uint16_t)xs[k], (uint16_t)(32768 + q->y[k]), (uint16_t)zs[k]};
            detailVertices[v] = (mnavDetailVertex){xs[k] * 16, 32768 + q->y[k], zs[k] * 16};
            polygon->vertices[k] = HandWeld(vertices, &vertexCount, corner);
            polygon->sides[k] =
                HandSide(xs[k], zs[k], xs[(k + 1) % 4], zs[(k + 1) % 4], def.tileCells);
        }
        parts[s] = (mnavDetailPart){s * 4, s * 2, 4, 2};
        triangles[s * 2] = s == degenerate ? (mnavDetailTriangle){{0, 2, 2}, 0}
                                           : (mnavDetailTriangle){{0, 1, 2}, 3};
        triangles[s * 2 + 1] = (mnavDetailTriangle){{0, 2, 3}, 6};
    }
    HandLink(polygons, count);
    mnavPolyMesh mesh = {vertices,         removable,     vertexCount,
                         HAND_SQUARES * 4, polygons,      count,
                         HAND_SQUARES,     def.tileCells, 0};
    mnavDetailMesh detail = {parts,     count,     detailVertices,   count * 4, HAND_SQUARES * 4,
                             triangles, count * 2, HAND_SQUARES * 2, 0,         0,
                             0};
    mnavTileInfo info = {MNAV_BAKE_GENERATOR,
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

// path_rules' layouts: grids of 4 by 4 squares of 5 m with some left out,
// square i at (i % 4, i / 4) kept unless a draw of the generator falls on
// 0 modulo 4. The generator runs from HAND_LAYOUT_SEED through every
// layout in turn, 16 draws a layout. hand_tiles.h stores a few of the
// first HAND_LAYOUT_COUNT (test_hand_tiles.c); path_rules_dense builds
// them all.
enum
{
    HAND_LAYOUT_SEED = 9,
    HAND_LAYOUT_COUNT = 60
};

// The squares of layout k, the mask of those kept; returns their count.
static inline int32_t HandLayout(int32_t k, HandSquare* squares, uint32_t* maskOut)
{
    uint32_t state = HAND_LAYOUT_SEED;
    int32_t count = 0;
    uint32_t mask = 0;
    for (int32_t layout = 0; layout <= k; ++layout)
    {
        count = 0;
        mask = 0;
        for (int32_t i = 0; i < 16; ++i)
        {
            state = state * 1664525u + 1013904223u;
            if ((state >> 16) % 4u != 0u)
            {
                int32_t x = (i % 4) * 20;
                int32_t z = (i / 4) * 20;
                squares[count++] = (HandSquare){x, z, x + 20, z + 20, {0}, 0};
                mask |= 1u << i;
            }
        }
    }
    *maskOut = mask;
    return count;
}

// Encodes the squares as the tile at (place, 0) into out; returns its size.
static inline size_t HandTileBytes(uint8_t* out, int32_t place, const HandSquare* squares,
                                   int32_t count)
{
    return HandTileBytesWith(out, place, squares, count, -1);
}

#endif // MAUL_NAV_TEST_HAND_TILE_H
