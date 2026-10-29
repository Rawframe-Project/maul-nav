// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Partitioning open space into regions by layers, read from the baked
// tile: its regions and dropped islands, the polygons over each cell and
// the paths between them.

#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

enum
{
    QUADS = 64,
    TILE_ROOM = 1 << 20,
    NO_POLYGON = 0xFF
};

// A flat or sloped quad: from (x0, z0) to (x1, z1), at height y0 along
// x = x0 and y1 along x = x1, of an area.
typedef struct Quad
{
    float x0;
    float z0;
    float x1;
    float z1;
    float y0;
    float y1;
    mnavAreaType area;
} Quad;

static uint8_t s_tile[TILE_ROOM];
static mnavNavmesh* s_navmesh;
static mnavQuery* s_query;
static mnavBakeReport s_report;

// The default def, but an agent of no radius, regions of any size and no
// edge error, so that the polygons cover exactly the cells the regions
// hold, less ledges.
static mnavBakeDef Exact(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    return def;
}

// Bakes tile (0, 0) of the quads, facing up, and loads it in a fresh
// navmesh.
static void Bake(const mnavBakeDef* def, const Quad* quads, int32_t count)
{
    static mnavVec3 vertices[4 * QUADS];
    static int32_t indices[6 * QUADS];
    static mnavAreaType areas[2 * QUADS];
    const int32_t corners[6] = {0, 1, 2, 0, 2, 3};
    for (int32_t q = 0; q < count; ++q)
    {
        const Quad* d = &quads[q];
        vertices[4 * q + 0] = (mnavVec3){d->x0, d->y0, d->z0};
        vertices[4 * q + 1] = (mnavVec3){d->x0, d->y0, d->z1};
        vertices[4 * q + 2] = (mnavVec3){d->x1, d->y1, d->z1};
        vertices[4 * q + 3] = (mnavVec3){d->x1, d->y1, d->z0};
        for (int32_t k = 0; k < 6; ++k)
        {
            indices[6 * q + k] = 4 * q + corners[k];
        }
        areas[2 * q] = d->area;
        areas[2 * q + 1] = d->area;
    }
    const mnavTriangleMesh mesh = {vertices, 4 * count, indices, 2 * count, areas};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBaker* baker = nullptr;
    size_t size = 0;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success &&
              mnavBaker_BakeTile(baker, &input, 0, 0, &s_report) == mnav_success &&
              mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(def, &s_navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavNavmesh_Commit(s_navmesh) == mnav_success,
          "loaded");
}

// The polygon over (x, z) within half a meter of height y, if any.
static mnavNearest Over(double x, double y, double z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(s_navmesh, (mnavPos3){x, y, z}, (mnavVec3){0.01f, 0.5f, 0.01f},
                                  nullptr, &n) == mnav_success,
          "nearest");
    if (!n.over)
    {
        n.polygon = (mnavPolygonId){0, 0, 0};
    }
    return n;
}

// The area of that polygon, or NO_POLYGON.
static mnavAreaType AreaAt(double x, double y, double z)
{
    mnavNearest n = Over(x, y, z);
    mnavAreaType area = NO_POLYGON;
    if (n.polygon.slot != 0)
    {
        CHECK(mnavPolygon_GetArea(s_navmesh, n.polygon, &area) == mnav_success, "area");
    }
    return area;
}

// Whether a path leads from one point to another, each on a polygon.
static bool Reaches(mnavPos3 from, mnavPos3 to)
{
    mnavNearest a = Over(from.x, from.y, from.z);
    mnavNearest b = Over(to.x, to.y, to.z);
    CHECK(a.polygon.slot != 0 && b.polygon.slot != 0, "both on polygons");
    mnavPath path;
    CHECK(mnavQuery_FindPath(s_query, s_navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                             &path) == mnav_success,
          "searched");
    return path.end == mnav_pathFound;
}

static void TestOpenFloorIsOneRegion(void)
{
    const Quad floor = {0, 0, 10, 10, 0, 0, mnav_areaWalkable};
    mnavBakeDef def = Exact();
    Bake(&def, &floor, 1);
    CHECK(s_report.regions == 1, "one region");
}

static void TestAreasSplitRegions(void)
{
    const Quad halves[2] = {{0, 0, 5, 10, 0, 0, mnav_areaWalkable}, {5, 0, 10, 10, 0, 0, 7}};
    mnavBakeDef def = Exact();
    Bake(&def, halves, 2);
    CHECK(s_report.regions == 2, "two areas, two regions");
    CHECK(AreaAt(2.5, 0, 5) == mnav_areaWalkable && AreaAt(7.5, 0, 5) == 7, "split at the area");
    CHECK(Reaches((mnavPos3){2.5, 0, 5}, (mnavPos3){7.5, 0, 5}), "and linked across it");
}

static void TestFloorOverTheWholeTile(void)
{
    // Every row and column of the tile holds a region, its edges too.
    const Quad floor = {-5, -5, 37, 37, 0, 0, mnav_areaWalkable};
    mnavBakeDef def = Exact();
    Bake(&def, &floor, 1);
    int32_t walkable = 0;
    for (int32_t z = 0; z < 128; ++z)
    {
        for (int32_t x = 0; x < 128; ++x)
        {
            walkable += AreaAt(0.25 * x + 0.125, 0, 0.25 * z + 0.125) != NO_POLYGON ? 1 : 0;
        }
    }
    CHECK(s_report.regions == 1 && walkable == 128 * 128, "every cell");
}

static void TestStackedFloorsJoinedByARampAreTwoLayers(void)
{
    // A floor 2.5 m up over x = 0 to 6 m, z = 0 to 6 m, with a ramp from
    // its edge down along x to the ground at 14 m; under the upper floor
    // a lower one, which goes on past it along z and round the ramp to its
    // foot: one linked surface over the cells it covers twice.
    const Quad quads[5] = {{0, 0, 6, 6, 2.5f, 2.5f, mnav_areaWalkable},
                           {6, 0, 14, 6, 2.5f, 0, mnav_areaWalkable},
                           {0, 0, 6, 18, 0, 0, mnav_areaWalkable},
                           {6, 7, 18, 18, 0, 0, mnav_areaWalkable},
                           {14, 0, 18, 7, 0, 0, mnav_areaWalkable}};
    mnavBakeDef def = Exact();
    Bake(&def, quads, 5);
    mnavNearest low = Over(2, 0, 3);
    mnavNearest high = Over(2, 2.5, 3);
    CHECK(low.polygon.slot != 0 && high.polygon.slot != 0 && s_report.regions >= 2,
          "both floors, in different layers");
    CHECK(Reaches((mnavPos3){2, 0, 3}, (mnavPos3){2, 2.5, 3}), "linked by the ramp");
}

static void TestSmallIslandsAreDropped(void)
{
    // An island whose ledge ring leaves 2 by 2 cells, away from the
    // border, and a strip from the border into the tile's first column,
    // where its ledge ring leaves it 2 cells; nothing links them.
    const Quad quads[2] = {{10, 10, 11, 11, 0, 0, mnav_areaWalkable},
                           {-2, 5, 0.5f, 6, 0, 0, mnav_areaWalkable}};
    for (int32_t minRegion = 4; minRegion <= 5; ++minRegion)
    {
        mnavBakeDef def = Exact();
        def.minRegionArea = 0.0625f * (float)minRegion;
        Bake(&def, quads, 2);
        bool kept = minRegion == 4;
        CHECK((AreaAt(10.5, 0, 10.5) != NO_POLYGON) == kept,
              "an island of 4 cells against the minimum");
        CHECK(s_report.droppedRegions == (kept ? 0 : 1), "a dropped island counted");
        CHECK(AreaAt(0.125, 0, 5.375) == mnav_areaWalkable &&
                  AreaAt(0.125, 0, 5.625) == mnav_areaWalkable,
              "a strip on the border is kept");
    }
}

static void TestRowWithMoreSweepsThanCells(void)
{
    // Strips one cell wide along z, 9 of them side by side, each holding
    // four floors 18 levels apart under a solid cap, each strip 3 levels
    // above the one before. A floor shares 14 levels of space with the
    // one beside it, too little to link, though each has 17 of room: a
    // row starts 28 sweeps across 7 inside cells (Recast issue 317 writes
    // past its sweep array here). The strips at the ends are ledges.
    Quad quads[45];
    for (int32_t x = 0; x < 9; ++x)
    {
        for (int32_t k = 0; k < 5; ++k)
        {
            float x0 = 10.0f + 0.25f * (float)x;
            float y = 0.125f * (float)(3 * x + 18 * k);
            quads[5 * x + k] =
                (Quad){x0, 10, x0 + 0.25f, 12.25f, y, y, k < 4 ? mnav_areaWalkable : mnav_areaNone};
        }
    }
    mnavBakeDef def = Exact();
    Bake(&def, quads, 45);
    // Each floor of each inside strip is a region: 4 by 7.
    CHECK(s_report.spans == 28 * 7 && s_report.regions == 28, "twenty-eight strips");
}

static void TestTwoSweepsOverOneRegionStaySeparate(void)
{
    // Blocks of 6 by 6 cells. Row 1 rises across blocks 1 to 5 by levels
    // 0, 0, 1, 7 and 13. Row 2 has a low floor at level 0 over blocks 1
    // to 3, and a high one over blocks 3 to 5 at levels 18, 13 and 19,
    // both stepping down onto row 1; block 3 holds both. Neither may
    // continue row 1's region, or that region would hold two spans of one
    // column.
    const int32_t rowOne[5] = {0, 0, 1, 7, 13};
    const int32_t high[3] = {18, 13, 19};
    Quad quads[13];
    int32_t n = 0;
    for (int32_t b = 0; b < 5; ++b)
    {
        float x0 = 10.0f + 1.5f * (float)b;
        float y = 0.125f * (float)rowOne[b];
        quads[n++] = (Quad){x0, 10.0f, x0 + 1.5f, 11.5f, y, y, mnav_areaWalkable};
        if (b < 3)
        {
            quads[n++] = (Quad){x0, 11.5f, x0 + 1.5f, 13.0f, 0, 0, mnav_areaWalkable};
        }
        if (b >= 2)
        {
            float h = 0.125f * (float)high[b - 2];
            quads[n++] = (Quad){x0, 11.5f, x0 + 1.5f, 13.0f, h, h, mnav_areaWalkable};
        }
    }
    mnavBakeDef def = Exact();
    Bake(&def, quads, n);
    // Block 3 runs from x = 13 to 14.5 m: both floors over its middle.
    // The surface is linked and covers block 3 twice, so it takes two
    // layers; a region holding both would leave no polygon there.
    CHECK(Over(13.875, 0.125, 12.375).polygon.slot != 0 &&
              Over(13.875, 2.375, 12.375).polygon.slot != 0 && s_report.regions == 2,
          "both sweeps, in two layers");
}

static void TestLevelRegionsAreOneLayer(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    const mnavTriangleMesh mesh = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBaker* baker = nullptr;
    size_t size = 0;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBaker_BakeTile(baker, &input, 0, 0, &s_report) == mnav_success &&
              mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &s_navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavNavmesh_Commit(s_navmesh) == mnav_success,
          "loaded");
    // The floor, the ramp and the platform are linked.
    CHECK(Reaches((mnavPos3){31.125, 0.125, 10.125}, (mnavPos3){24.125, 0.625, 18.125}),
          "the ramp joins the floor");
    CHECK(Reaches((mnavPos3){31.125, 0.125, 10.125}, (mnavPos3){28.125, 1.125, 28.125}),
          "so does the platform");
}

int main(void)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    CHECK(mnavCreateQuery(&queryDef, &s_query) == mnav_success, "query");
    TestOpenFloorIsOneRegion();
    TestFloorOverTheWholeTile();
    TestAreasSplitRegions();
    TestStackedFloorsJoinedByARampAreTwoLayers();
    TestSmallIslandsAreDropped();
    TestRowWithMoreSweepsThanCells();
    TestTwoSweepsOverOneRegionStaySeparate();
    TestLevelRegionsAreOneLayer();
    mnavDestroyNavmesh(s_navmesh);
    mnavDestroyQuery(s_query);
    return s_failures == 0 ? 0 : 1;
}
