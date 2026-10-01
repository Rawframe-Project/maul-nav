// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A world over four 32 m tiles, a floor with boxes 1.5 m tall standing
// across their sides (lower than the agent, so nothing is walkable
// inside them), baked once with the default def into s_tiles.

#ifndef MAUL_NAV_TEST_WORLD_H
#define MAUL_NAV_TEST_WORLD_H

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

// The world's four tiles, baked once.
static uint8_t s_tiles[4][TILE_CAPACITY];
static size_t s_sizes[4];

static inline void BakeWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    for (int32_t t = 0; t < 4; ++t)
    {
        mnavBakeReport report;
        CHECK(mnavBakeTile(baker, &world, 1, t & 1, t >> 1, &report) == mnav_success, "baked");
        CHECK(mnavCopyBakedTile(baker, s_tiles[t], TILE_CAPACITY, &s_sizes[t]) == mnav_success,
              "copied");
    }
    mnavDestroyBaker(baker);
}

#endif // MAUL_NAV_TEST_WORLD_H
