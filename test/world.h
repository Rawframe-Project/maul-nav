// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A world over four 32 m tiles, a floor with boxes 1.5 m tall standing
// across their sides (lower than the agent, so nothing is walkable
// inside them), its tiles read from test/stored_tiles.h into s_tiles.

#ifndef MAUL_NAV_TEST_WORLD_H
#define MAUL_NAV_TEST_WORLD_H

#include "stored_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>
#include <string.h>

enum
{
    BOXES = 6,
    WORLD_VERTICES = 4 + 8 * BOXES,
    WORLD_TRIANGLES = 2 + 12 * BOXES,
    TILE_CAPACITY = 1 << 16
};

static mnavVec3 s_vertices[WORLD_VERTICES];
static int32_t s_indices[WORLD_TRIANGLES * 3];

// A floor over four 32 m tiles, with boxes standing across their sides.
static inline mnavTriangleMesh World(void)
{
    const mnavVec3 floor[4] = {
        {-1.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 65.0f}, {65.0f, 0.0f, 65.0f}, {65.0f, 0.0f, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    const float boxes[BOXES][4] = {{30, 10, 34, 12}, {30, 40, 33, 44}, {8, 30, 12, 34},
                                   {50, 31, 52, 35}, {20, 20, 24, 24}, {31, 31, 33, 33}};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        mnavVec3* v = &s_vertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] = (mnavVec3){boxes[b][(k & 1) ? 2 : 0], (k & 4) ? 1.5f : 0.0f,
                              boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, WORLD_VERTICES, s_indices, WORLD_TRIANGLES, nullptr};
}

// The world's four tiles, as stored (test/stored_tiles.h): the bake of
// World() with the default def, kept fixed so that the suites that query
// them pin query results, not the bake's.
static uint8_t s_tiles[4][TILE_CAPACITY];
static size_t s_sizes[4];

static inline void LoadWorldTiles(void)
{
    const uint8_t* stored[4] = {s_ground0, s_ground1, s_ground2, s_ground3};
    const size_t sizes[4] = {sizeof(s_ground0), sizeof(s_ground1), sizeof(s_ground2),
                             sizeof(s_ground3)};
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(sizes[t] <= TILE_CAPACITY, "the stored tile fits");
        memcpy(s_tiles[t], stored[t], sizes[t]);
        s_sizes[t] = sizes[t];
    }
}

#endif // MAUL_NAV_TEST_WORLD_H
