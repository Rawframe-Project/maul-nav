// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The detail mesh's outline: edge samples and their triangulation (N20).

#include "allocator.h"
#include "border_vertices.h"
#include "compact.h"
#include "contour.h"
#include "detail.h"
#include "erode.h"
#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "holes.h"
#include "polymesh.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// Sixteenths of a cell per cell.
#define SUBCELLS 16

// The hash of the level's detail mesh, the same on every platform.
#define LEVEL_DETAIL_HASH 0xeb1d6f8f368537ecull

// A hand field compacted, every span in region 1.
typedef struct Setup
{
    mnavMemory memory;
    mnavCompactField field;
    uint32_t ids[WIDTH * WIDTH * MAX_SPANS];
    mnavRegionMap regions;
} Setup;

static void Prepare(Setup* setup, Field* hand)
{
    setup->memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    CHECK(mnavBuildCompactField(&setup->memory, Pack(hand), HEIGHT, STEP, &setup->field) ==
              mnav_success,
          "compacted");
    for (int32_t i = 0; i < setup->field.spanCount; ++i)
    {
        setup->ids[i] = 1;
    }
    setup->regions = (mnavRegionMap){setup->ids, setup->field.spanCount, 1};
}

// A floor at height 4, and at height top in cell (bx, bz).
static void Bumped(Field* hand, int32_t bx, int32_t bz, int32_t top)
{
    *hand = (Field){0};
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            int32_t h = x == bx && z == bz ? top : 4;
            AddSpan(hand, x, z, h - 1, h, mnav_areaWalkable);
        }
    }
}

// Up to two rectangles at height 4 in region 1, sharing vertices where
// they meet.
typedef struct Rects
{
    mnavMeshVertex vertices[8];
    mnavPolygon polygons[2];
    mnavPolyMesh mesh;
} Rects;

static void AddRect(Rects* rects, int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    mnavPolyMesh* mesh = &rects->mesh;
    const int32_t corners[8] = {x0, z0, x0, z1, x1, z1, x1, z0};
    mnavPolygon* polygon = &rects->polygons[mesh->polygonCount++];
    *polygon = (mnavPolygon){{0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}, {0}, {0}, 4, 1, 1};
    for (int32_t k = 0; k < 4; ++k)
    {
        int32_t found = -1;
        for (int32_t v = 0; v < mesh->vertexCount; ++v)
        {
            found =
                rects->vertices[v].x == corners[2 * k] && rects->vertices[v].z == corners[2 * k + 1]
                    ? v
                    : found;
        }
        if (found < 0)
        {
            found = mesh->vertexCount++;
            rects->vertices[found] =
                (mnavMeshVertex){(uint16_t)corners[2 * k], BASE + 4, (uint16_t)corners[2 * k + 1]};
        }
        polygon->vertices[k] = (uint16_t)found;
    }
}

static void InitRects(Rects* rects)
{
    *rects = (Rects){0};
    rects->mesh.vertices = rects->vertices;
    rects->mesh.polygons = rects->polygons;
    rects->mesh.tileCells = WIDTH;
}

static int32_t OutlineEdges(const mnavDetailMesh* detail, int32_t p)
{
    int32_t count = 0;
    const mnavDetailPart* part = &detail->parts[p];
    for (int32_t t = 0; t < part->triangleCount; ++t)
    {
        uint8_t outline = detail->triangles[part->firstTriangle + t].outline;
        count += (outline & 1) + ((outline >> 1) & 1) + ((outline >> 2) & 1);
    }
    return count;
}

static bool HasVertex(const mnavDetailMesh* detail, int32_t p, int32_t x, int32_t y, int32_t z)
{
    const mnavDetailPart* part = &detail->parts[p];
    for (int32_t v = 0; v < part->vertexCount; ++v)
    {
        const mnavDetailVertex* d = &detail->vertices[part->firstVertex + v];
        if (d->x == x && d->y == y && d->z == z)
        {
            return true;
        }
    }
    return false;
}

// The part's vertices on the line x = lineX, or z = lineZ when lineX is
// negative.
static int32_t OnLine(const mnavDetailMesh* detail, int32_t p, int32_t lineX, int32_t lineZ)
{
    int32_t count = 0;
    const mnavDetailPart* part = &detail->parts[p];
    for (int32_t v = 0; v < part->vertexCount; ++v)
    {
        const mnavDetailVertex* d = &detail->vertices[part->firstVertex + v];
        count += (lineX >= 0 ? d->x == lineX : d->z == lineZ) ? 1 : 0;
    }
    return count;
}

static bool Sound(const mnavDetailMesh* detail, const mnavPolyMesh* mesh);

static void TestFlatSquare(void)
{
    static Field hand;
    Bumped(&hand, -1, -1, 0);
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 6, 6);
    mnavDetailMesh detail;
    mnavDetailSettings none = {0, 16, 1, 0};
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh, none,
                              &detail) == mnav_success,
          "built");
    CHECK(detail.parts[0].vertexCount == 4 && detail.parts[0].triangleCount == 2,
          "no samples: the polygon's two triangles");
    CHECK(OutlineEdges(&detail, 0) == 4, "four outline edges");
    CHECK(HasVertex(&detail, 0, 96, BASE + 4, 0), "vertices in sixteenths");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavDetailSettings every = {16, 16, 1, 0};
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh, every,
                              &detail) == mnav_success,
          "built");
    CHECK(detail.parts[0].vertexCount == 4, "flat samples are dropped");
    CHECK(detail.fallbackHeights == 0, "every sample found a height");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
    CHECK(setup.memory.used == 0, "everything released");
}

static void TestBumpOnAnEdgeIsSampled(void)
{
    static Field hand;
    Bumped(&hand, 3, 0, 8);
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 6, 6);
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 16, 1, 0}, &detail) == mnav_success,
          "built");
    // Samples fall every cell; the bump's and, to hold its sides within
    // the error, its two neighbors are kept.
    CHECK(OnLine(&detail, 0, -1, 0) == 5 && HasVertex(&detail, 0, 48, BASE + 8, 0) &&
              HasVertex(&detail, 0, 32, BASE + 4, 0) && HasVertex(&detail, 0, 64, BASE + 4, 0),
          "on the edge, the bump and its neighbors, nothing else");
    CHECK(OutlineEdges(&detail, 0) == 7, "seven outline edges");
    CHECK(detail.parts[0].vertexCount > 7, "samples inside hold the floor round the bump");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    // Within the maximum error the bump is dropped.
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 64, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(detail.parts[0].vertexCount == 4, "4 cell heights is within 64 sixteenths");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
}

static void TestHillInsideIsSampled(void)
{
    // A hill in the middle of the square; the edges stay flat.
    static Field hand;
    Bumped(&hand, 3, 3, 9);
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 6, 6);
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 16, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(OutlineEdges(&detail, 0) == 4, "the outline unsampled");
    CHECK(HasVertex(&detail, 0, 48, BASE + 9, 48), "the hilltop added first");
    CHECK(detail.cappedPolygons == 0 && Sound(&detail, &rects.mesh), "sound");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    // With samples twice as far apart the grid misses the hill's cell.
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){32, 16, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(detail.parts[0].vertexCount == 4, "nothing to add");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
}

static void TestSharedEdgeIsSampledAlike(void)
{
    static Field hand;
    Bumped(&hand, 3, 2, 8);
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 3, 6);
    AddRect(&rects, 3, 0, 6, 6);
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 16, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(HasVertex(&detail, 0, 48, BASE + 8, 32) && HasVertex(&detail, 1, 48, BASE + 8, 32),
          "both sides hold the bump's sample");
    int32_t onEdge[2] = {0, 0};
    for (int32_t p = 0; p < 2; ++p)
    {
        for (int32_t v = 0; v < detail.parts[p].vertexCount; ++v)
        {
            onEdge[p] += detail.vertices[detail.parts[p].firstVertex + v].x == 48 ? 1 : 0;
        }
    }
    CHECK(onEdge[0] == 5 && onEdge[1] == 5, "five points on the shared edge each side");
    bool alike = true;
    for (int32_t v = 0; v < detail.parts[0].vertexCount; ++v)
    {
        const mnavDetailVertex* d = &detail.vertices[detail.parts[0].firstVertex + v];
        alike = alike && (d->x != 48 || HasVertex(&detail, 1, d->x, d->y, d->z));
    }
    CHECK(alike, "the same points");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
}

static void TestTiesKeepTheFirstFromTheLexicalEnd(void)
{
    // A ridge two cells long on the square's +X edge, which runs from
    // (3, 3) to (3, 0) in polygon order: its two samples stray equally
    // far, and once one is kept the other is within the error. The edge
    // is sampled from its lexically first end, (3, 0), so the sample
    // nearer it is kept, whichever polygon samples the edge.
    static Field hand;
    hand = (Field){0};
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            int32_t h = x == 3 && (z == 1 || z == 2) ? 6 : 4;
            AddSpan(&hand, x, z, h - 1, h, mnav_areaWalkable);
        }
    }
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 3, 3);
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 16, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(OnLine(&detail, 0, 48, -1) == 3 && HasVertex(&detail, 0, 48, BASE + 6, 16),
          "the sample nearer (3, 0)");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
}

static void TestNoHeightsFallBackAndCount(void)
{
    static Field hand;
    hand = (Field){0};
    static Setup setup;
    Prepare(&setup, &hand);
    Rects rects;
    InitRects(&rects);
    AddRect(&rects, 0, 0, 6, 6);
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&setup.memory, &setup.field, &setup.regions, &rects.mesh,
                              (mnavDetailSettings){16, 16, 1, 0}, &detail) == mnav_success,
          "built");
    CHECK(detail.fallbackHeights == 4 * 5 + 5 * 5, "every edge and grid sample counted");
    CHECK(detail.parts[0].vertexCount == 4, "the polygon's own heights, nothing kept");
    mnavReleaseDetailMesh(&setup.memory, &detail);
    mnavReleaseCompactField(&setup.memory, &setup.field);
}

static uint64_t HashDetail(const mnavDetailMesh* detail)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t p = 0; p < detail->partCount; ++p)
    {
        const mnavDetailPart* part = &detail->parts[p];
        int32_t fields[4] = {part->firstVertex, part->firstTriangle, part->vertexCount,
                             part->triangleCount};
        hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
    }
    for (int32_t v = 0; v < detail->vertexCount; ++v)
    {
        const mnavDetailVertex* d = &detail->vertices[v];
        int32_t fields[3] = {d->x, d->y, d->z};
        hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
    }
    for (int32_t t = 0; t < detail->triangleCount; ++t)
    {
        hash = mnavHash64(hash, &detail->triangles[t], (int32_t)sizeof(detail->triangles[t]));
    }
    return hash;
}

// Twice a triangle's signed area on the ground, in sixteenths squared.
static int64_t TriangleArea2(const mnavDetailMesh* detail, const mnavDetailPart* part,
                             const mnavDetailTriangle* t)
{
    const mnavDetailVertex* a = &detail->vertices[part->firstVertex + t->corners[0]];
    const mnavDetailVertex* b = &detail->vertices[part->firstVertex + t->corners[1]];
    const mnavDetailVertex* c = &detail->vertices[part->firstVertex + t->corners[2]];
    return (int64_t)(b->x - a->x) * (c->z - a->z) - (int64_t)(c->x - a->x) * (b->z - a->z);
}

// Every part within its caps with an outline, every triangle wound like
// the polygons, and the triangles of each part covering its polygon to
// within the rounding of edge samples.
static bool Sound(const mnavDetailMesh* detail, const mnavPolyMesh* mesh)
{
    for (int32_t p = 0; p < detail->partCount; ++p)
    {
        const mnavDetailPart* part = &detail->parts[p];
        const mnavPolygon* polygon = &mesh->polygons[p];
        if (part->vertexCount > MNAV_DETAIL_VERTICES ||
            part->triangleCount > 2 * MNAV_DETAIL_VERTICES || OutlineEdges(detail, p) < 3)
        {
            return false;
        }
        int64_t area = 0;
        for (int32_t t = 0; t < part->triangleCount; ++t)
        {
            int64_t a = TriangleArea2(detail, part, &detail->triangles[part->firstTriangle + t]);
            if (a > 0)
            {
                return false;
            }
            area += a;
        }
        int64_t polygonArea = 0;
        for (int32_t i = 0, j = polygon->count - 1; i < polygon->count; j = i++)
        {
            const mnavMeshVertex* a = &mesh->vertices[polygon->vertices[i]];
            const mnavMeshVertex* b = &mesh->vertices[polygon->vertices[j]];
            polygonArea += (int64_t)a->x * b->z - (int64_t)b->x * a->z;
        }
        // Each rounded sample moves the outline by at most half a
        // sixteenth across an edge of at most the tile's side.
        int64_t slack = (int64_t)(part->vertexCount - polygon->count) * mesh->tileCells * SUBCELLS;
        int64_t difference = -area - polygonArea * 256;
        difference = difference < 0 ? -difference : difference;
        if (difference > slack)
        {
            return false;
        }
    }
    return true;
}

static void TestLevelDetailIsPinned(void)
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
    CHECK(mnavRemoveBorderVertices(&memory, &mesh, def.limits.tilePolygons) == mnav_success,
          "border vertices");
    CHECK(mnavLinkPolyMesh(&memory, &mesh) == mnav_success, "linked");
    mnavDetailSettings settings = {cells.detailSample, cells.detailError, 1, cells.border};
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&memory, &compact, &regions, &mesh, settings, &detail) ==
              mnav_success,
          "detail");
    CHECK(Sound(&detail, &mesh) && detail.failedPolygons == 0, "sound");
    uint64_t hash = HashDetail(&detail);
    printf("LEVEL_DETAIL_HASH=%016llx vertices=%d triangles=%d fallbacks=%d\n",
           (unsigned long long)hash, detail.vertexCount, detail.triangleCount,
           detail.fallbackHeights);
    CHECK(hash == LEVEL_DETAIL_HASH, "the pinned hash");
    mnavReleaseDetailMesh(&memory, &detail);
    // Samples every cell meet the per-edge cap on the longest edges.
    settings.sample = SUBCELLS;
    settings.error = 0;
    CHECK(mnavBuildDetailMesh(&memory, &compact, &regions, &mesh, settings, &detail) ==
              mnav_success,
          "dense detail");
    CHECK(Sound(&detail, &mesh) && detail.failedPolygons == 0, "sound when dense");
    int32_t most = 0;
    for (int32_t p = 0; p < detail.partCount; ++p)
    {
        int32_t samples = detail.parts[p].vertexCount - mesh.polygons[p].count;
        most = samples > most ? samples : most;
    }
    printf("dense: vertices=%d triangles=%d most samples in a part=%d\n", detail.vertexCount,
           detail.triangleCount, most);
    mnavReleaseDetailMesh(&memory, &detail);
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

enum
{
    TERRAIN_QUADS = 32,
    TERRAIN_VERTICES = (TERRAIN_QUADS + 1) * (TERRAIN_QUADS + 1),
    TERRAIN_TRIANGLES = TERRAIN_QUADS * TERRAIN_QUADS * 2
};

// Rolling ground over the tile: 1 m quads, heights from 0 to 0.3 m from
// a fixed generator.
static void MakeTerrain(mnavVec3* vertices, int32_t* indices)
{
    uint32_t state = 5;
    for (int32_t z = 0; z <= TERRAIN_QUADS; ++z)
    {
        for (int32_t x = 0; x <= TERRAIN_QUADS; ++x)
        {
            float h = (float)(Next(&state) % 31u) / 100.0f;
            vertices[x + z * (TERRAIN_QUADS + 1)] = (mnavVec3){(float)x, h, (float)z};
        }
    }
    int32_t k = 0;
    for (int32_t z = 0; z < TERRAIN_QUADS; ++z)
    {
        for (int32_t x = 0; x < TERRAIN_QUADS; ++x)
        {
            int32_t a = x + z * (TERRAIN_QUADS + 1);
            int32_t b = a + TERRAIN_QUADS + 1;
            const int32_t quad[6] = {a, b, b + 1, a, b + 1, a + 1};
            for (int32_t q = 0; q < 6; ++q)
            {
                indices[k++] = quad[q];
            }
        }
    }
}

static void TestBumpyGroundReachesTheCap(void)
{
    static mnavVec3 vertices[TERRAIN_VERTICES];
    static int32_t indices[TERRAIN_TRIANGLES * 3];
    MakeTerrain(vertices, indices);
    mnavTriangleMesh input = {vertices, TERRAIN_VERTICES, indices, TERRAIN_TRIANGLES, nullptr};
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
    CHECK(mnavLinkPolyMesh(&memory, &mesh) == mnav_success, "linked");
    mnavDetailSettings settings = {SUBCELLS, 0, 1, cells.border};
    mnavDetailMesh detail;
    CHECK(mnavBuildDetailMesh(&memory, &compact, &regions, &mesh, settings, &detail) ==
              mnav_success,
          "detail");
    CHECK(detail.cappedPolygons > 0, "some polygons reach the cap");
    CHECK(Sound(&detail, &mesh) && detail.failedPolygons == 0, "sound at the cap");
    printf("terrain: polygons=%d vertices=%d triangles=%d capped=%d\n", mesh.polygonCount,
           detail.vertexCount, detail.triangleCount, detail.cappedPolygons);
    mnavReleaseDetailMesh(&memory, &detail);
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestFlatSquare();
    TestBumpOnAnEdgeIsSampled();
    TestHillInsideIsSampled();
    TestSharedEdgeIsSampledAlike();
    TestTiesKeepTheFirstFromTheLexicalEnd();
    TestNoHeightsFallBackAndCount();
    TestLevelDetailIsPinned();
    TestBumpyGroundReachesTheCap();
    return s_failures == 0 ? 0 : 1;
}
