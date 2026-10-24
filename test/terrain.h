// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A slope rising along X with a plateau, 1 m between samples, starting 2 m
// before tile (0, 0); the plateau's sides are too steep to climb.

#ifndef MAUL_NAV_TEST_TERRAIN_H
#define MAUL_NAV_TEST_TERRAIN_H

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    TERRAIN_SIDE = 40
};

static float s_terrainHeights[TERRAIN_SIDE * TERRAIN_SIDE];

static inline mnavTerrain Terrain(void)
{
    for (int32_t r = 0; r < TERRAIN_SIDE; ++r)
    {
        for (int32_t c = 0; c < TERRAIN_SIDE; ++c)
        {
            int32_t dx = c - 18;
            int32_t dz = r - 14;
            s_terrainHeights[r * TERRAIN_SIDE + c] =
                0.125f * (float)c + (dx * dx + dz * dz < 36 ? 1.5f : 0.0f);
        }
    }
    return (mnavTerrain){{-2.0f, 1.0f, -2.0f}, 1.0f,   1.0f, TERRAIN_SIDE, TERRAIN_SIDE,
                         s_terrainHeights,     nullptr};
}

#endif // MAUL_NAV_TEST_TERRAIN_H
