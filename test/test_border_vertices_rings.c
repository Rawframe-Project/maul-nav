// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Removing tile-border vertices from hand-built rings, for what the
// public bakes do not reach: a vertex inside the mesh, a lone polygon's
// tip, polygons of two areas or touching only at the vertex, a slit, a
// corner of the outline, the polygon limit, and a second vertex among the
// polygons the first removal left. test_border_vertices.c bakes the case
// bakes reach, and test_alloc_failures.c refuses its allocations.

#include "allocator.h"
#include "border_vertices.h"
#include "contour.h"
#include "poly_check.h"
#include "polymesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// Marks every ring vertex at (x, z) as a tile-border vertex.
static void Flag(mnavContourSet* set, int32_t x, int32_t z)
{
    for (int32_t v = 0; v < set->vertexCount; ++v)
    {
        if (set->vertices[v].x == x && set->vertices[v].z == z)
        {
            set->vertices[v].flags |= MNAV_VERTEX_TILE_BORDER;
        }
    }
}

static bool HasVertex(const mnavPolyMesh* mesh, int32_t x, int32_t z)
{
    for (int32_t v = 0; v < mesh->vertexCount; ++v)
    {
        if (mesh->vertices[v].x == x && mesh->vertices[v].z == z)
        {
            return true;
        }
    }
    return false;
}

// Whether polygon p has the given ground points, in order.
static bool PolygonIs(const mnavPolyMesh* mesh, int32_t p, const int32_t* points, int32_t count)
{
    if (p >= mesh->polygonCount || mesh->polygons[p].count != count)
    {
        return false;
    }
    const mnavPolygon* polygon = &mesh->polygons[p];
    for (int32_t k = 0; k < count; ++k)
    {
        const mnavMeshVertex* v = &mesh->vertices[polygon->vertices[k]];
        if (v->x != points[2 * k] || v->z != points[2 * k + 1])
        {
            return false;
        }
    }
    return true;
}

// Builds, removes and links; returns the removal's result.
static mnavResult Run(mnavMemory* memory, mnavContourSet* set, int32_t maxPolygons,
                      mnavPolyMesh* mesh)
{
    CHECK(mnavBuildPolyMesh(memory, set, 10, 100, 100, mesh) == mnav_success, "built");
    mnavResult result = mnavRemoveBorderVertices(memory, mesh, maxPolygons);
    if (result == mnav_success)
    {
        CHECK(mnavLinkPolyMesh(memory, mesh) == mnav_success, "linked");
    }
    return result;
}

static void Finish(mnavMemory* memory, mnavContourSet* set, mnavPolyMesh* mesh)
{
    mnavReleasePolyMesh(memory, mesh);
    mnavReleaseContours(memory, set);
    CHECK(memory->used == 0, "everything released");
}

static void TestNewPolygonsCountAgainstTheLimit(void)
{
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5, 10, 0};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    Flag(&set, 5, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 1, &mesh) == mnav_errorCapacity, "two polygons past a limit of one");
    mnavReleasePolyMesh(&memory, &mesh);
    CHECK(Run(&memory, &set, 2, &mesh) == mnav_success, "at the limit");
    Finish(&memory, &set, &mesh);
}

static void TestSecondVertexAmongMixedPolygons(void)
{
    // Three squares in a row, (5, 0) and (10, 0) on the tile's side: the
    // first removal leaves polygons of the mixed region round the second
    // vertex, and the second removal keeps that region.
    const int32_t points[24] = {0,  0, 0,  5, 5,  5, 5,  0, 5,  0, 5,  5,
                                10, 5, 10, 0, 10, 0, 10, 5, 15, 5, 15, 0};
    const int32_t counts[3] = {4, 4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 3);
    Flag(&set, 5, 0);
    Flag(&set, 10, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "removed");
    CHECK(!HasVertex(&mesh, 5, 0) && !HasVertex(&mesh, 10, 0), "both vertices gone");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == 150, "valid, the same area");
    for (int32_t p = 0; p < mesh.polygonCount; ++p)
    {
        CHECK(mesh.polygons[p].region == MNAV_MIXED_REGION, "the mixed region kept");
    }
    Finish(&memory, &set, &mesh);
}

static void TestVertexInsideTheMeshIsRemoved(void)
{
    // Four squares round (5, 5), each its own region: a closed chain.
    const int32_t points[32] = {0, 0, 0, 5,  5, 5,  5, 0, 5, 5, 5, 10, 10, 10, 10, 5,
                                0, 5, 0, 10, 5, 10, 5, 5, 5, 0, 5, 5,  10, 5,  10, 0};
    const int32_t counts[4] = {4, 4, 4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 4);
    Flag(&set, 5, 5);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "removed");
    CHECK(mesh.vertexCount == 8 && !HasVertex(&mesh, 5, 5), "the vertex is gone");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == 200, "valid, the same area");
    // The closed chain's ring has each of its 8 vertices once.
    const int32_t hexagon[12] = {10, 5, 5, 0, 0, 0, 0, 5, 5, 10, 10, 10};
    const int32_t left[6] = {0, 5, 0, 10, 5, 10};
    const int32_t right[6] = {10, 5, 10, 0, 5, 0};
    CHECK(mesh.polygonCount == 3 && PolygonIs(&mesh, 0, hexagon, 6) &&
              PolygonIs(&mesh, 1, left, 3) && PolygonIs(&mesh, 2, right, 3),
          "the hole's polygons");
    Finish(&memory, &set, &mesh);
}

static void TestTipOfALonePolygonStays(void)
{
    const int32_t points[8] = {0, 0, 0, 10, 10, 10, 10, 0};
    const int32_t counts[1] = {4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    Flag(&set, 10, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "ran");
    CHECK(mesh.vertexCount == 4 && HasVertex(&mesh, 10, 0), "the tip stays");
    int32_t flagged = 0;
    for (int32_t v = 0; v < mesh.vertexCount; ++v)
    {
        flagged += mesh.removable[v];
    }
    CHECK(flagged == 1, "still flagged");
    Finish(&memory, &set, &mesh);
}

static void TestDifferentAreasStay(void)
{
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5, 10, 0};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    set.contours[1].area = 2;
    Flag(&set, 5, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "ran");
    CHECK(mesh.vertexCount == 6 && HasVertex(&mesh, 5, 0), "the vertex stays");
    Finish(&memory, &set, &mesh);
}

static void TestPolygonsTouchingOnlyAtTheVertexStay(void)
{
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 5, 5, 10, 10, 10, 10, 5};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    Flag(&set, 5, 5);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "ran");
    CHECK(mesh.vertexCount == 7 && HasVertex(&mesh, 5, 5), "the vertex stays");
    Finish(&memory, &set, &mesh);
}

static void TestVertexAtTheEndOfASlitStays(void)
{
    // Four triangles fanned round (5, 5) from (10, 5) to (15, 5): the
    // outline runs in along a slit to (5, 5) and out again, so the chain's
    // ends are in line with the vertex but both on one side of it.
    const int32_t points[24] = {5, 5, 10, 5, 5, 0,  5, 5, 5, 0,  0,  5,
                                5, 5, 0,  5, 5, 10, 5, 5, 5, 10, 15, 5};
    const int32_t counts[4] = {3, 3, 3, 3};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 4);
    Flag(&set, 5, 5);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    int64_t area = TwiceArea(&mesh);
    CHECK(mnavRemoveBorderVertices(&memory, &mesh, 100) == mnav_success, "ran");
    CHECK(HasVertex(&mesh, 5, 5) && TwiceArea(&mesh) == area, "the vertex and the area stay");
    Finish(&memory, &set, &mesh);
}

static void TestCornerOfTheOutlineStays(void)
{
    // A square and a triangle meeting at x = 5; the outline turns at
    // (5, 0), so removing it would change the walkable area.
    const int32_t points[14] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5};
    const int32_t counts[2] = {4, 3};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    Flag(&set, 5, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "ran");
    CHECK(Valid(&mesh), "valid");
    CHECK(HasVertex(&mesh, 5, 0), "the corner stays");
    Finish(&memory, &set, &mesh);
}

int main(void)
{
    TestNewPolygonsCountAgainstTheLimit();
    TestSecondVertexAmongMixedPolygons();
    TestVertexInsideTheMeshIsRemoved();
    TestTipOfALonePolygonStays();
    TestDifferentAreasStay();
    TestPolygonsTouchingOnlyAtTheVertexStay();
    TestCornerOfTheOutlineStays();
    TestVertexAtTheEndOfASlitStays();
    return s_failures == 0 ? 0 : 1;
}
