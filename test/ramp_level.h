// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A floor, and a ramp from it up to a deck 3 m high on a box, both
// crossing the tile's +X side at x = 32 m: the floor and the ramp overlap,
// so they are two regions of one area, and their boundary meets the
// tile's side. The scene may be turned a quarter turn at a time round the
// tile's middle, to cross each of its sides.

#ifndef MAUL_NAV_TEST_RAMP_LEVEL_H
#define MAUL_NAV_TEST_RAMP_LEVEL_H

#include "soup.h"

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    RAMP_LEVEL_VERTICES = 16,
    RAMP_LEVEL_INDICES = 48
};

// A place on the ground turned a quarter turn round the tile's middle,
// (16, 16) m, turns times.
static inline void TurnOnTile(float* x, float* z, int32_t turns)
{
    for (int32_t t = 0; t < turns; ++t)
    {
        float nx = 32.0f - *z;
        *z = *x;
        *x = nx;
    }
}

// The level turned a quarter turn turns times. Returns the number of
// triangles.
static inline int32_t MakeRampLevel(mnavVec3* vertices, int32_t* indices, int32_t turns)
{
    const mnavVec3 corners[8] = {{-1.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 41.0f}, {41.0f, 0.0f, 41.0f},
                                 {41.0f, 0.0f, -1.0f}, {28.0f, 0.0f, 10.0f}, {28.0f, 3.0f, 22.0f},
                                 {36.0f, 3.0f, 22.0f}, {36.0f, 0.0f, 10.0f}};
    const int32_t quads[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    for (int32_t k = 0; k < 8; ++k)
    {
        vertices[k] = corners[k];
    }
    for (int32_t k = 0; k < 12; ++k)
    {
        indices[k] = quads[k];
    }
    int32_t index = AddBox(vertices, indices, 8, 12, (mnavVec3){28.0f, 2.8f, 22.0f},
                           (mnavVec3){36.0f, 3.0f, 30.0f});
    for (int32_t k = 0; k < RAMP_LEVEL_VERTICES; ++k)
    {
        TurnOnTile(&vertices[k].x, &vertices[k].z, turns);
    }
    return index / 3;
}

#endif // MAUL_NAV_TEST_RAMP_LEVEL_H
