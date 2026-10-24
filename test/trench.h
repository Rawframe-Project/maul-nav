// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The scene of the link generation tests, over two 32 m tiles along X.

#ifndef MAUL_NAV_TEST_TRENCH_H
#define MAUL_NAV_TEST_TRENCH_H

#include "maul-nav/bake.h"

#include <stdint.h>
#include <string.h>

enum
{
    TRENCH_BOXES = 4,
    TRENCH_VERTICES = 4 + 8 * TRENCH_BOXES,
    TRENCH_TRIANGLES = 2 + 12 * TRENCH_BOXES
};

static mnavVec3 s_trenchVertices[TRENCH_VERTICES];
static int32_t s_trenchIndices[TRENCH_TRIANGLES * 3];

// A trench floor 6 m down; slab A from x = -1 to 30 and slab B from 31.2
// on, a gap of 1.2 m between; a platform 1.5 m high on A, a pillar on B.
// The boxes' tops face up, so that they are walkable.
static inline mnavTriangleMesh Trench(void)
{
    const mnavVec3 floor[4] = {
        {-1.0f, -6.0f, -1.0f}, {-1.0f, -6.0f, 33.0f}, {65.0f, -6.0f, 33.0f}, {65.0f, -6.0f, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_trenchVertices, floor, sizeof(floor));
    memcpy(s_trenchIndices, quad, sizeof(quad));
    // x0, z0, x1, z1, y0, y1.
    const float boxes[TRENCH_BOXES][6] = {{-1, -1, 30, 33, -1, 0},
                                          {31.2f, -1, 65, 33, -1, 0},
                                          {10, 10, 20, 22, 0, 1.5f},
                                          {45, 15, 46, 16, 0, 3}};
    for (int32_t b = 0; b < TRENCH_BOXES; ++b)
    {
        mnavVec3* v = &s_trenchVertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] = (mnavVec3){boxes[b][(k & 1) ? 2 : 0], boxes[b][(k & 4) ? 5 : 4],
                              boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 7, 5, 4, 6, 7, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_trenchIndices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_trenchVertices, TRENCH_VERTICES, s_trenchIndices, TRENCH_TRIANGLES,
                              nullptr};
}

#endif // MAUL_NAV_TEST_TRENCH_H
