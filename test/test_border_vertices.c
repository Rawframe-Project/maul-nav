// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Removing tile-border vertices from a tile's polygons (N19).

#include "allocator.h"
#include "border_vertices.h"
#include "compact.h"
#include "contour.h"
#include "erode.h"
#include "filter.h"
#include "heightfield.h"
#include "holes.h"
#include "poly_check.h"
#include "polymesh.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hash of the ramp level's mesh after removal, the same on every
// platform.
#define RAMP_MESH_HASH 0xd0628e38b973be85ull

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

static void TestVertexBetweenTwoRegionsIsRemoved(void)
{
    // Two squares, regions 1 and 2, meeting at x = 5; (5, 0) lies on the
    // tile's -Z side.
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5, 10, 0};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    Flag(&set, 5, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 100, &mesh) == mnav_success, "removed");
    CHECK(mesh.vertexCount == 5 && !HasVertex(&mesh, 5, 0), "the vertex is gone");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == 100, "valid, the same area");
    for (int32_t p = 0; p < mesh.polygonCount; ++p)
    {
        CHECK(mesh.polygons[p].region == MNAV_MIXED_REGION && mesh.polygons[p].area == 1,
              "mixed region, the area kept");
    }
    Finish(&memory, &set, &mesh);
}

static void TestNewPolygonsCountAgainstTheLimit(void)
{
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5, 10, 0};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    Flag(&set, 5, 0);
    mnavPolyMesh mesh;
    CHECK(Run(&memory, &set, 1, &mesh) == mnav_errorLimit, "two polygons past a limit of one");
    mnavReleasePolyMesh(&memory, &mesh);
    CHECK(Run(&memory, &set, 2, &mesh) == mnav_success, "at the limit");
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

// A floor, and a ramp from it up to a deck 3 m high, both crossing the
// tile's +X side at x = 32 m: the floor and the ramp overlap, so they are
// two regions, and their boundary meets the tile side. Returns the number
// of triangles.
static int32_t MakeRampLevel(mnavVec3* vertices, int32_t* indices)
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
    return index / 3;
}

static uint64_t HashMesh(const mnavPolyMesh* mesh)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t v = 0; v < mesh->vertexCount; ++v)
    {
        uint16_t fields[4] = {mesh->vertices[v].x, mesh->vertices[v].y, mesh->vertices[v].z,
                              mesh->removable[v]};
        hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
    }
    for (int32_t p = 0; p < mesh->polygonCount; ++p)
    {
        const mnavPolygon* polygon = &mesh->polygons[p];
        hash = mnavHash64(hash, polygon->vertices, (int32_t)sizeof(polygon->vertices));
        hash = mnavHash64(hash, polygon->neighbors, (int32_t)sizeof(polygon->neighbors));
        hash = mnavHash64(hash, polygon->sides, (int32_t)sizeof(polygon->sides));
        uint32_t tail[3] = {polygon->count, polygon->area, polygon->region};
        hash = mnavHash64(hash, tail, (int32_t)sizeof(tail));
    }
    return hash;
}

static void TestRampLevelLosesItsBorderVertex(void)
{
    mnavVec3 vertices[16];
    int32_t indices[48];
    int32_t triangles = MakeRampLevel(vertices, indices);
    mnavTriangleMesh input = {vertices, 16, indices, triangles, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    mnavMemory memory = mnavMakeMemory(def.allocator, def.limits.memoryBytes);
    mnavHeightfield heightfield;
    CHECK(mnavBuildHeightfield(&memory, &def, &cells, &input, 1, 0, 0, &heightfield) ==
              mnav_success,
          "rasterized");
    mnavFilterWalkable(&heightfield, cells.agentHeight, cells.agentStep);
    mnavCompactField compact;
    CHECK(mnavBuildCompactField(&memory, &heightfield, cells.agentHeight, cells.agentStep,
                                &compact) == mnav_success,
          "compacted");
    CHECK(mnavErode(&memory, &compact, cells.agentRadius) == mnav_success, "eroded");
    mnavRegionMap regions;
    CHECK(mnavBuildRegions(&memory, &compact, cells.border, cells.minRegion, &regions) ==
              mnav_success,
          "regions");
    CHECK(regions.count == 2, "the floor and the ramp");
    mnavContourSet set;
    CHECK(mnavBuildContours(&memory, &compact, &regions, cells.border, cells.edgeError,
                            cells.edgeLength, &set) == mnav_success,
          "contours");
    CHECK(mnavMergeHoles(&memory, &set, regions.count) == mnav_success, "merged");
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, def.tileCells, def.limits.tileVertices,
                            def.limits.tilePolygons, &mesh) == mnav_success,
          "polygons");
    int32_t flagged = 0;
    for (int32_t v = 0; v < mesh.vertexCount; ++v)
    {
        flagged += mesh.removable[v];
    }
    int32_t before = mesh.vertexCount;
    int64_t area = TwiceArea(&mesh);
    CHECK(flagged == 1, "one vertex where the regions meet the tile side");
    CHECK(mnavRemoveBorderVertices(&memory, &mesh, def.limits.tilePolygons) == mnav_success,
          "removed");
    CHECK(mnavLinkPolyMesh(&memory, &mesh) == mnav_success, "linked");
    CHECK(mesh.vertexCount == before - 1, "the vertex is gone");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == area, "valid, the same area");
    uint64_t hash = HashMesh(&mesh);
    printf("RAMP_MESH_HASH=%016llx vertices=%d polygons=%d\n", (unsigned long long)hash,
           mesh.vertexCount, mesh.polygonCount);
    CHECK(hash == RAMP_MESH_HASH, "the pinned hash");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestVertexBetweenTwoRegionsIsRemoved();
    TestNewPolygonsCountAgainstTheLimit();
    TestVertexInsideTheMeshIsRemoved();
    TestTipOfALonePolygonStays();
    TestDifferentAreasStay();
    TestPolygonsTouchingOnlyAtTheVertexStay();
    TestCornerOfTheOutlineStays();
    TestVertexAtTheEndOfASlitStays();
    TestRampLevelLosesItsBorderVertex();
    return s_failures == 0 ? 0 : 1;
}
