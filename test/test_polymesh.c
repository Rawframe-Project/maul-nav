// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Triangulating rings and merging them into a linked polygon mesh (N18).

#include "allocator.h"
#include "compact.h"
#include "contour.h"
#include "erode.h"
#include "filter.h"
#include "heightfield.h"
#include "holes.h"
#include "polymesh.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hash of the level's polygon mesh, the same on every platform.
#define LEVEL_MESH_HASH 0xd0616f2cf70a7954ull

// A contour set of rings given as ground points, one region each.
static mnavContourSet Rings(mnavMemory* memory, const int32_t* points, const int32_t* counts,
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

static int64_t Turn(const mnavPolyMesh* mesh, const mnavPolygon* polygon, int32_t k)
{
    const mnavMeshVertex* a = &mesh->vertices[polygon->vertices[k]];
    const mnavMeshVertex* b = &mesh->vertices[polygon->vertices[(k + 1) % polygon->count]];
    const mnavMeshVertex* c = &mesh->vertices[polygon->vertices[(k + 2) % polygon->count]];
    return (int64_t)(b->x - a->x) * (c->z - a->z) - (int64_t)(c->x - a->x) * (b->z - a->z);
}

// Every polygon convex, every link answered by a link back.
static bool Valid(const mnavPolyMesh* mesh)
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

static int64_t TwiceArea(const mnavPolyMesh* mesh)
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

static void TestSquareIsOneQuadOnFourSides(void)
{
    const int32_t points[8] = {0, 0, 0, 10, 10, 10, 10, 0};
    const int32_t counts[1] = {4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 10, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.polygonCount == 1 && mesh.polygons[0].count == 4, "one quad");
    CHECK(mesh.vertexCount == 4, "four vertices");
    int32_t sides = 0;
    for (int32_t k = 0; k < 4; ++k)
    {
        sides |= 1 << mesh.polygons[0].sides[k];
    }
    CHECK(sides == 0x1E, "an edge on each of the four sides");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == 200, "valid, the square's area");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    CHECK(memory.used == 0, "everything released");
}

static void TestConcaveRingBecomesLinkedConvexPolygons(void)
{
    // An L: no single convex polygon covers it.
    const int32_t points[12] = {0, 0, 0, 10, 4, 10, 4, 4, 10, 4, 10, 0};
    const int32_t counts[1] = {6};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.polygonCount == 2, "two convex pieces");
    CHECK(Valid(&mesh), "convex and linked both ways");
    CHECK(mesh.polygons[0].neighbors[0] != MNAV_NO_INDEX ||
              mesh.polygons[0].neighbors[1] != MNAV_NO_INDEX ||
              mesh.polygons[0].neighbors[2] != MNAV_NO_INDEX ||
              mesh.polygons[0].neighbors[3] != MNAV_NO_INDEX,
          "the pieces are linked");
    CHECK(TwiceArea(&mesh) == 2 * 64, "the L's area");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
}

static void TestRingsWeldAndLinkAcrossRegions(void)
{
    // Two squares side by side, each its own region, sharing x = 5.
    const int32_t points[16] = {0, 0, 0, 5, 5, 5, 5, 0, 5, 0, 5, 5, 10, 5, 10, 0};
    const int32_t counts[2] = {4, 4};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 2);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.vertexCount == 6, "the shared edge's ends welded");
    CHECK(mesh.polygonCount == 2, "one quad per region");
    CHECK(mesh.polygons[0].region != mesh.polygons[1].region, "the regions are kept");
    CHECK(Valid(&mesh), "linked both ways across the regions");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
}

static void TestLimitsAreTyped(void)
{
    const int32_t points[12] = {0, 0, 0, 10, 4, 10, 4, 4, 10, 4, 10, 0};
    const int32_t counts[1] = {6};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 5, 100, &mesh) == mnav_errorLimit, "vertices");
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 1, &mesh) == mnav_errorLimit, "polygons");
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 6, 2, &mesh) == mnav_success, "at the limits");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    CHECK(memory.used == 0, "nothing held after the refusals");
}

static void TestTangledRingIsCounted(void)
{
    // A ring whose edges cross: no ear can be cut to the end.
    const int32_t points[12] = {0, 0, 0, 10, 10, 0, 10, 10, 4, 12, 2, 8};
    const int32_t counts[1] = {6};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.failedRings == 1, "counted");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
}

static void TestBridgedRingWithAHiddenVertex(void)
{
    // A square of 20 with three holes bridged in, found by searching
    // random holes. Vertex (9, 12) lies inside the ear (0, 20), (20, 20),
    // (9, 10), but both its edges run to the positions of that ear's
    // diagonal (one to the other copy of (0, 20)), so the edge test alone
    // let the ear through and the rest of the ring was lost.
    const int32_t points[44] = {8,  6, 6,  6, 6,  4,  0,  0,  0,  20, 1, 14, 3, 14, 3,
                                16, 1, 16, 1, 14, 0,  20, 20, 20, 20, 0, 0,  0, 6,  4,
                                8,  4, 8,  6, 7,  10, 9,  10, 9,  12, 7, 12, 7, 10};
    const int32_t counts[1] = {22};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.failedRings == 0, "the ring is complete");
    CHECK(Valid(&mesh) && TwiceArea(&mesh) == 776, "the square less three holes");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
}

static void TestPinchedRingNeedsTheLooseTest(void)
{
    // Two loops touching at (3, 2), as a region touching itself at a
    // corner traces: only a diagonal that touches the pinch finishes it.
    const int32_t points[12] = {4, 0, 3, 2, 2, 1, 0, 1, 5, 5, 3, 2};
    const int32_t counts[1] = {6};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = Rings(&memory, points, counts, 1);
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, 20, 100, 100, &mesh) == mnav_success, "built");
    CHECK(mesh.failedRings == 0, "the ring is complete");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
}

static void TestLevelMeshIsPinned(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    mnavTriangleMesh input = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
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
    mnavContourSet set;
    CHECK(mnavBuildContours(&memory, &compact, &regions, cells.border, cells.edgeError,
                            cells.edgeLength, &set) == mnav_success,
          "contours");
    CHECK(mnavMergeHoles(&memory, &set, regions.count) == mnav_success, "merged");
    mnavPolyMesh mesh;
    CHECK(mnavBuildPolyMesh(&memory, &set, def.tileCells, def.limits.tileVertices,
                            def.limits.tilePolygons, &mesh) == mnav_success,
          "polygons");
    CHECK(Valid(&mesh) && mesh.failedRings == 0, "convex, linked, every ring complete");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t v = 0; v < mesh.vertexCount; ++v)
    {
        uint16_t fields[4] = {mesh.vertices[v].x, mesh.vertices[v].y, mesh.vertices[v].z,
                              mesh.removable[v]};
        hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
    }
    for (int32_t p = 0; p < mesh.polygonCount; ++p)
    {
        const mnavPolygon* polygon = &mesh.polygons[p];
        hash = mnavHash64(hash, polygon->vertices, (int32_t)sizeof(polygon->vertices));
        hash = mnavHash64(hash, polygon->neighbors, (int32_t)sizeof(polygon->neighbors));
        hash = mnavHash64(hash, polygon->sides, (int32_t)sizeof(polygon->sides));
        uint32_t tail[3] = {polygon->count, polygon->area, polygon->region};
        hash = mnavHash64(hash, tail, (int32_t)sizeof(tail));
    }
    printf("MNAV_MESH_HASH=%016llx vertices=%d polygons=%d\n", (unsigned long long)hash,
           mesh.vertexCount, mesh.polygonCount);
    CHECK(hash == LEVEL_MESH_HASH, "the pinned hash");
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestSquareIsOneQuadOnFourSides();
    TestConcaveRingBecomesLinkedConvexPolygons();
    TestRingsWeldAndLinkAcrossRegions();
    TestLimitsAreTyped();
    TestTangledRingIsCounted();
    TestBridgedRingWithAHiddenVertex();
    TestPinchedRingNeedsTheLooseTest();
    TestLevelMeshIsPinned();
    return s_failures == 0 ? 0 : 1;
}
