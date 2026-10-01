// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A fixed soup of triangles over tile (0, 0) for the bake's determinism
// tests.

#ifndef MAUL_NAV_TEST_SOUP_H
#define MAUL_NAV_TEST_SOUP_H

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    SOUP_TRIANGLES = 600
};

static inline uint32_t Next(uint32_t* state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state >> 8;
}

// Triangles of every orientation and area, each within 4 m, over tile
// (0, 0) and its border, from a fixed generator.
static inline void MakeSoup(mnavVec3* vertices, mnavAreaType* areas)
{
    uint32_t state = 7;
    for (int32_t t = 0; t < SOUP_TRIANGLES; ++t)
    {
        float cx = (float)(Next(&state) % 34000u) / 1000.0f - 1.0f;
        float cy = (float)(Next(&state) % 8000u) / 1000.0f - 2.0f;
        float cz = (float)(Next(&state) % 34000u) / 1000.0f - 1.0f;
        for (int32_t k = 0; k < 3; ++k)
        {
            float x = cx + (float)(Next(&state) % 4000u) / 1000.0f - 2.0f;
            float y = cy + (float)(Next(&state) % 4000u) / 1000.0f - 2.0f;
            float z = cz + (float)(Next(&state) % 4000u) / 1000.0f - 2.0f;
            vertices[t * 3 + k] = (mnavVec3){x, y, z};
        }
    }
    for (int32_t t = 0; t < SOUP_TRIANGLES; ++t)
    {
        areas[t] = (mnavAreaType)(Next(&state) % 8u);
    }
}

enum
{
    LEVEL_BOXES = 24,
    LEVEL_VERTICES = 4 + 8 * (LEVEL_BOXES + 1) + 4,
    LEVEL_TRIANGLES = 2 + 12 * (LEVEL_BOXES + 1) + 2
};

// The corners of an axis-aligned box: bit 0 picks x, bit 1 y, bit 2 z.
static inline int32_t AddBox(mnavVec3* vertices, int32_t* indices, int32_t vertex, int32_t index,
                             mnavVec3 low, mnavVec3 high)
{
    for (int32_t k = 0; k < 8; ++k)
    {
        vertices[vertex + k] = (mnavVec3){(k & 1) ? high.x : low.x, (k & 2) ? high.y : low.y,
                                          (k & 4) ? high.z : low.z};
    }
    // Two triangles per face, each counter-clockwise seen from outside.
    static const int32_t faces[36] = {2, 6, 7, 2, 7, 3, 0, 1, 5, 0, 5, 4, 0, 4, 6, 0, 6, 2,
                                      1, 3, 7, 1, 7, 5, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6};
    for (int32_t k = 0; k < 36; ++k)
    {
        indices[index + k] = vertex + faces[k];
    }
    return index + 36;
}

// A small level over tile (0, 0): a floor, boxes from curbs to walls, a
// platform 1 m up and a ramp to it, from a fixed generator. Fills
// LEVEL_VERTICES vertices and LEVEL_TRIANGLES triangles.
static inline void MakeLevel(mnavVec3* vertices, int32_t* indices)
{
    vertices[0] = (mnavVec3){-1.0f, 0.0f, -1.0f};
    vertices[1] = (mnavVec3){-1.0f, 0.0f, 33.0f};
    vertices[2] = (mnavVec3){33.0f, 0.0f, 33.0f};
    vertices[3] = (mnavVec3){33.0f, 0.0f, -1.0f};
    const int32_t floorIndices[6] = {0, 1, 2, 0, 2, 3};
    for (int32_t k = 0; k < 6; ++k)
    {
        indices[k] = floorIndices[k];
    }
    int32_t vertex = 4;
    int32_t index = 6;
    uint32_t state = 11;
    for (int32_t b = 0; b < LEVEL_BOXES; ++b)
    {
        float x = (float)(Next(&state) % 2600u) / 100.0f;
        float z = (float)(Next(&state) % 2600u) / 100.0f;
        float sx = 0.3f + (float)(Next(&state) % 300u) / 100.0f;
        float sz = 0.3f + (float)(Next(&state) % 300u) / 100.0f;
        float h = 0.1f + (float)(Next(&state) % 290u) / 100.0f;
        index = AddBox(vertices, indices, vertex, index, (mnavVec3){x, 0.0f, z},
                       (mnavVec3){x + sx, h, z + sz});
        vertex += 8;
    }
    index = AddBox(vertices, indices, vertex, index, (mnavVec3){20.0f, 0.0f, 20.0f},
                   (mnavVec3){30.0f, 1.0f, 30.0f});
    vertex += 8;
    // A ramp from the floor at z = 16 m up to the platform's edge at 20 m.
    vertices[vertex + 0] = (mnavVec3){22.0f, 0.0f, 16.0f};
    vertices[vertex + 1] = (mnavVec3){22.0f, 1.0f, 20.0f};
    vertices[vertex + 2] = (mnavVec3){26.0f, 1.0f, 20.0f};
    vertices[vertex + 3] = (mnavVec3){26.0f, 0.0f, 16.0f};
    const int32_t ramp[6] = {0, 1, 2, 0, 2, 3};
    for (int32_t k = 0; k < 6; ++k)
    {
        indices[index + k] = vertex + ramp[k];
    }
}

#endif // MAUL_NAV_TEST_SOUP_H
