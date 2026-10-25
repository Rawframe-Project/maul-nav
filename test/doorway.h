// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The scene of the doorway crowd, over two 32 m tiles along X: two rooms
// of 30 m by 32 m, 4 m apart, joined by a doorway 6 m wide that crosses
// the seam of the two tiles. Its tiles are stored in stored_tiles.h.

#ifndef MAUL_NAV_TEST_DOORWAY_H
#define MAUL_NAV_TEST_DOORWAY_H

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    DOORWAY_QUADS = 3
};

static mnavVec3 s_doorwayVertices[DOORWAY_QUADS * 4];
static int32_t s_doorwayIndices[DOORWAY_QUADS * 6];

// The crowd's agents are 0.25 m in radius; the other settings are written
// out, so that a later default does not change the stored tiles' def.
static inline mnavBakeDef DoorwayDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = 0.25f;
    def.cellHeight = 0.125f;
    def.tileCells = 128;
    def.agent.radius = 0.25f;
    def.agent.height = 2.0f;
    def.agent.stepHeight = 0.75f;
    def.agent.maxSlopeDegrees = 45.0f;
    return def;
}

// The floor as flat quads at y = 0, each two triangles facing up: the west
// room, the east room and the doorway between them.
static inline mnavTriangleMesh Doorway(void)
{
    // x0, z0, x1, z1.
    const float quads[DOORWAY_QUADS][4] = {{0, 0, 30, 32}, {34, 0, 64, 32}, {30, 13, 34, 19}};
    for (int32_t k = 0; k < DOORWAY_QUADS; ++k)
    {
        mnavVec3* v = &s_doorwayVertices[k * 4];
        v[0] = (mnavVec3){quads[k][0], 0.0f, quads[k][1]};
        v[1] = (mnavVec3){quads[k][0], 0.0f, quads[k][3]};
        v[2] = (mnavVec3){quads[k][2], 0.0f, quads[k][3]};
        v[3] = (mnavVec3){quads[k][2], 0.0f, quads[k][1]};
        const int32_t corners[6] = {0, 1, 2, 0, 2, 3};
        for (int32_t c = 0; c < 6; ++c)
        {
            s_doorwayIndices[k * 6 + c] = k * 4 + corners[c];
        }
    }
    return (mnavTriangleMesh){s_doorwayVertices, DOORWAY_QUADS * 4, s_doorwayIndices,
                              DOORWAY_QUADS * 2, nullptr};
}

#endif // MAUL_NAV_TEST_DOORWAY_H
