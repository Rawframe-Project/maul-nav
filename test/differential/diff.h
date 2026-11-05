// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential harness: the scenes both libraries bake,
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

    // Bakes every tile as DiffMaulBuild does, without loading them; the
    // tiles' bytes in all, or -1 on failure.
    int64_t DiffMaulBake(const DiffScene* scene);

    void DiffMaulDestroy(DiffMaul* maul);

    // A random point on the navmesh from a seed; false when there is none.
    bool DiffMaulPoint(DiffMaul* maul, uint64_t seed, float out[3]);

    // A reached path's shape: its corners, three floats each, up to
    // capacity; how many it has, which may be more; and how many polygons
    // its corridor crosses.
    typedef struct DiffRoute
    {
        float* corners;
        int32_t capacity;
        int32_t cornerCount;
        int32_t polygonCount;
    } DiffRoute;

    // A path between the polygons nearest two points, within 0.5 m across
    // and 2 m up or down: 1 reached, with the straight path's length; 0
    // not reached; -1 when a point has no polygon near. nodes, which may
    // be NULL, receives the nodes the search made, and route, which may be
    // NULL, a reached path's shape.
    int32_t DiffMaulPath(DiffMaul* maul, const float a[3], const float b[3], double* length,
                         int32_t* nodes, DiffRoute* route);

    // A ray on the ground from the point on the polygon nearest a, as
    // DiffMaulPath finds it, toward b: 1 stopped by a wall, with how far
    // it went as a fraction of the way on the ground; 0 reached b; 2 any
    // other end (a tile side unloaded, the node limit); -1 when a has no
    // polygon near. start receives the snapped start point and, at a
    // wall, normal the wall's normal on the ground.
    int32_t DiffMaulRaycast(DiffMaul* maul, const float a[3], const float b[3], float start[3],
                            double* t, float normal[3]);

    // The nearest wall within radius of the point on the polygon nearest
    // at, as DiffMaulPath snaps it: 1 found, with its distance on the
    // ground; 0 none within the radius; 2 the search cut short by the node
    // limit; -1 when at has no polygon near. center receives the snapped
    // point and wall the wall's point nearest it.
    int32_t DiffMaulWallDistance(DiffMaul* maul, const float at[3], double radius, float center[3],
                                 double* distance, float wall[3]);

    // The walls within radius on the ground of the point at, on the first
    // polygon under it, or under the point 0.01 m back toward the start,
    // that the ray from a toward b, as DiffMaulRaycast casts it, crossed,
    // as mnavQuery_FindWalls lists them, and some 0.01 m farther: their count,
    // up to capacity, with each wall's two ends as six floats in segments;
    // -1 when a has no polygon near, the ray failed or crossed no polygon
    // under at; -2 when the search failed, ran into the node limit or found
    // more than capacity.
    int32_t DiffMaulRayWalls(DiffMaul* maul, const float a[3], const float b[3], const double at[2],
                             double radius, float* segments, int32_t capacity);

    // A move along the surface from the point on the polygon nearest a,
    // as DiffMaulPath snaps it, toward the point to: 0 reached to; 1
    // stopped by a wall; 2 any other end (a tile side unloaded, the node
    // limit, a failure); -1 when a has no polygon near. start receives the
    // snapped start point and end the point the move ended at.
    int32_t DiffMaulMove(DiffMaul* maul, const float a[3], const float to[3], float start[3],
                         float end[3]);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_TEST_DIFF_H
