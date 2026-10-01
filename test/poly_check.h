// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Polygon mesh tests' helpers: contour sets from ground points, the
// mesh's validity and its area.

#ifndef MAUL_NAV_TEST_POLY_CHECK_H
#define MAUL_NAV_TEST_POLY_CHECK_H

#include "allocator.h"
#include "contour.h"
#include "polymesh.h"
#include "test_harness.h"

#include <stdint.h>

// A contour set of rings given as ground points, one region each.
static inline mnavContourSet Rings(mnavMemory* memory, const int32_t* points, const int32_t* counts,
                                   int32_t ringCount)
{
    mnavContourSet set = {0};
    int32_t total = 0;
    for (int32_t r = 0; r < ringCount; ++r)
    {
        total += counts[r];
    }
    CHECK(mnavReserve(memory, (void**)&set.vertices, &set.vertexCapacity, 0, total,
                      sizeof(mnavContourVertex), alignof(mnavContourVertex)) == mnav_success,
          "vertices");
    CHECK(mnavReserve(memory, (void**)&set.contours, &set.capacity, 0, ringCount,
                      sizeof(mnavContour), alignof(mnavContour)) == mnav_success,
          "contours");
    for (int32_t r = 0; r < ringCount; ++r)
    {
        set.contours[set.count++] =
            (mnavContour){set.vertexCount, counts[r], (uint32_t)r + 1, 1, false};
        for (int32_t k = 0; k < counts[r]; ++k)
        {
            const int32_t* p = &points[2 * (set.vertexCount)];
            set.vertices[set.vertexCount++] = (mnavContourVertex){p[0], 100, p[1], 0, 0};
        }
    }
    return set;
}

static inline int64_t Turn(const mnavPolyMesh* mesh, const mnavPolygon* polygon, int32_t k)
{
    const mnavMeshVertex* a = &mesh->vertices[polygon->vertices[k]];
    const mnavMeshVertex* b = &mesh->vertices[polygon->vertices[(k + 1) % polygon->count]];
    const mnavMeshVertex* c = &mesh->vertices[polygon->vertices[(k + 2) % polygon->count]];
    return (int64_t)(b->x - a->x) * (c->z - a->z) - (int64_t)(c->x - a->x) * (b->z - a->z);
}

// Every polygon convex, every link answered by a link back.
static inline bool Valid(const mnavPolyMesh* mesh)
{
    for (int32_t p = 0; p < mesh->polygonCount; ++p)
    {
        const mnavPolygon* polygon = &mesh->polygons[p];
        if (polygon->count < 3 || polygon->count > MNAV_POLYGON_VERTICES)
        {
            return false;
        }
        for (int32_t k = 0; k < polygon->count; ++k)
        {
            if (Turn(mesh, polygon, k) > 0)
            {
                return false;
            }
            uint16_t n = polygon->neighbors[k];
            if (n == MNAV_NO_INDEX)
            {
                continue;
            }
            bool back = false;
            for (int32_t j = 0; j < mesh->polygons[n].count; ++j)
            {
                back = back || mesh->polygons[n].neighbors[j] == p;
            }
            if (!back)
            {
                return false;
            }
        }
    }
    return true;
}

static inline int64_t TwiceArea(const mnavPolyMesh* mesh)
{
    int64_t area = 0;
    for (int32_t p = 0; p < mesh->polygonCount; ++p)
    {
        const mnavPolygon* polygon = &mesh->polygons[p];
        for (int32_t i = 0, j = polygon->count - 1; i < polygon->count; j = i++)
        {
            const mnavMeshVertex* a = &mesh->vertices[polygon->vertices[i]];
            const mnavMeshVertex* b = &mesh->vertices[polygon->vertices[j]];
            area += (int64_t)a->x * b->z - (int64_t)b->x * a->z;
        }
    }
    return area;
}

// Builds the polygons and links them.
static inline mnavResult BuildLinked(mnavMemory* memory, const mnavContourSet* set,
                                     int32_t tileCells, int32_t maxVertices, int32_t maxPolygons,
                                     mnavPolyMesh* mesh)
{
    mnavResult result = mnavBuildPolyMesh(memory, set, tileCells, maxVertices, maxPolygons, mesh);
    return result == mnav_success ? mnavLinkPolyMesh(memory, mesh) : result;
}

#endif // MAUL_NAV_TEST_POLY_CHECK_H
