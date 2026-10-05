// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential harness (mnav-0011): the scenes both libraries bake,
// and Maul Nav's side of each comparison, in C for the C++ harness.

#ifndef MAUL_NAV_TEST_DIFF_H
#define MAUL_NAV_TEST_DIFF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A scene: triangles, three floats a vertex, facing up where walkable.
    typedef struct DiffScene
    {
        const char* name;
        const float* vertices;
        int32_t vertexCount;
        const int32_t* indices;
        int32_t triangleCount;
    } DiffScene;

    int32_t DiffSceneCount(void);

    // Builds scene i into memory the next call reuses.
    DiffScene DiffGetScene(int32_t i);

    typedef struct DiffMaul DiffMaul;

    // Bakes every 32 m tile a scene reaches with the default def and loads
    // them; NULL on failure.
    DiffMaul* DiffMaulBuild(const DiffScene* scene);

    void DiffMaulDestroy(DiffMaul* maul);

    // A random point on the navmesh from a seed; false when there is none.
    bool DiffMaulPoint(DiffMaul* maul, uint64_t seed, float out[3]);

    // A path between the polygons nearest two points, within 0.5 m across
    // and 2 m up or down: 1 reached, with the straight path's length; 0
    // not reached; -1 when a point has no polygon near.
    int32_t DiffMaulPath(DiffMaul* maul, const float a[3], const float b[3], double* length);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_TEST_DIFF_H
