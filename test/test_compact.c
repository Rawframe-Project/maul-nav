// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The open-space field, its links and erosion, read from the baked tile:
// the space above walkable spans, links by the step and the shared
// headroom, as paths across them, erosion by the radius, and the bake's
// landmarks on the test level.

#include "soup.h"
#include "squares.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

static mnavQuery* s_query;

// The polygon over the center of cell (x, z) of the scene within a
// meter of a level.
static mnavNearest Over(int32_t x, int32_t z, int32_t level)
{
    mnavPos3 center = {(AT + x + 0.5) * 0.25, level * 0.125, (AT + z + 0.5) * 0.25};
    mnavNearest n;
    CHECK(mnavFindNearest(s_navmesh, nullptr, center, (mnavVec3){0.01f, 1.0f, 0.01f}, &n) ==
                  mnav_success &&
              n.polygon.slot != 0 && n.over,
          "a polygon over the cell");
    return n;
}

// Whether a path leads from cell (x0, z0) at a level to cell (x1, z1) at
// another.
static bool Reaches(int32_t x0, int32_t z0, int32_t level0, int32_t x1, int32_t z1, int32_t level1)
{
    mnavNearest a = Over(x0, z0, level0);
    mnavNearest b = Over(x1, z1, level1);
    mnavPath path;
    CHECK(mnavFindPath(s_query, s_navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                       &path) == mnav_success,
          "searched");
    return path.end == mnav_pathFound;
}

// The cells of the scene walkable at a level.
static int32_t CountWalkable(int32_t level)
{
    int32_t n = 0;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            n += At(x, z, level) != NO_POLYGON ? 1 : 0;
        }
    }
    return n;
}

static void TestOpenSpansAreTheSpaceAboveWalkableSpans(void)
{
    // A floor of area 2, a solid square 40 levels up over (4, 4), and a
    // floor of area 5 60 levels up over the whole scene.
    Scene scene = Flat(0, true);
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            scene.area[z][x] = 2;
        }
    }
    scene.roofLevel = 60;
    scene.roofArea = 5;
    Add(&scene, 4, 4, 40, mnav_areaNone);
    Load(&scene);
    CHECK(At(4, 4, 0) == 2, "up to the next solid");
    // Inside each floor's ledge ring, 7 by 7 open spans; none over the
    // solid square.
    CHECK(At(4, 4, 40) == NO_POLYGON && s_report.spans == 2 * 7 * 7,
          "the unwalkable span has no open space");
    CHECK(At(4, 4, 60) == 5, "open to the top; the area is kept");
}

static void TestLinksFollowTheStepAndHeadroom(void)
{
    // Cells 0 to 3 at level 0, cells 4 to 8 a step up, or one more.
    for (int32_t rise = STEP; rise <= STEP + 1; ++rise)
    {
        Scene scene = Flat(0, false);
        for (int32_t z = 0; z < SIDE; ++z)
        {
            for (int32_t x = 4; x < SIDE; ++x)
            {
                scene.level[z][x] = rise;
            }
        }
        Load(&scene);
        bool linked = rise == STEP;
        CHECK(Reaches(2, 4, 0, 6, 4, rise) == linked && Reaches(6, 4, rise, 2, 4, 0) == linked,
              "a step of exactly the step height links; one more does not");
    }
    // Cells 0 to 4 at level 0 under a ceiling, cells 5 to 8 a slab 4
    // levels up: they share the agent's height of space, or one level
    // less, though each has room.
    for (int32_t ceiling = HEIGHT + 4; ceiling <= HEIGHT + 5; ++ceiling)
    {
        Scene scene = Flat(0, false);
        for (int32_t z = 0; z < SIDE; ++z)
        {
            for (int32_t x = 5; x < SIDE; ++x)
            {
                scene.level[z][x] = 4;
            }
        }
        AddCells(&scene, 0, 0, 4, SIDE - 1, ceiling, mnav_areaNone);
        Load(&scene);
        bool linked = ceiling == HEIGHT + 5;
        CHECK(Reaches(2, 4, 0, 6, 4, 4) == linked && Reaches(6, 4, 4, 2, 4, 0) == linked,
              "the agent's height of shared space links; one level less does not");
    }
}

static void TestErosionKeepsTheRadiusFromBoundaries(void)
{
    // The scene's outer ring is a ledge; erosion by r cells takes r more
    // rings of the 7 by 7 cells inside it.
    const int32_t expected[5] = {49, 25, 9, 1, 0};
    for (int32_t radius = 0; radius <= 4; ++radius)
    {
        mnavBakeDef def = Exact();
        def.agent.radius = 0.25f * (float)radius;
        UseDef(&def);
        Scene scene = Flat(0, false);
        Load(&scene);
        CHECK(CountWalkable(0) == expected[radius], "rings eroded by radius");
    }
}

static void TestHoleErodesAround(void)
{
    // An unwalkable cell in the middle, level with the floor: with a
    // radius of one cell, its four straight neighbors go; its diagonal
    // ones are 3 half cells from it, past the threshold of 2.
    mnavBakeDef def = Exact();
    def.agent.radius = 0.25f;
    UseDef(&def);
    Scene scene = Flat(0, false);
    scene.area[4][4] = mnav_areaNone;
    Load(&scene);
    CHECK(CountWalkable(0) == 25 - 1 - 4, "the hole's straight neighbors");
    CHECK(At(3, 3, 0) == mnav_areaWalkable && At(5, 5, 0) == mnav_areaWalkable, "diagonal kept");
}

// A 2D bake skips the walkable filters, so open spans reach the frame's
// edge: an outline past the frame on every side makes every cell of the
// tile walkable, and its border ones link on to the frame's last cells.
static void TestOpenSpansToTheFrameEdge(void)
{
    const mnavVec2 points[4] = {{-5, -5}, {-5, 40}, {40, 40}, {40, -5}};
    const mnavOutline outline = {points, 4, mnav_areaWalkable};
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    size_t size = 0;
    CHECK(mnavCreateBaker(&s_def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, &outline, 1, 0, 0, &report) == mnav_success &&
              mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
    // The tile and its border of 3 cells: 134 by 134 open spans.
    int32_t walkable = 0;
    for (int32_t z = 0; z < 128; ++z)
    {
        for (int32_t x = 0; x < 128; ++x)
        {
            walkable += At(x - AT, z - AT, 0) == mnav_areaWalkable ? 1 : 0;
        }
    }
    CHECK(report.spans == 134 * 134 && walkable == 128 * 128, "every cell walkable");
}

// The area of the polygon over world (x, z) within a meter of height y,
// or NO_POLYGON.
static mnavAreaType AreaAt(double x, double y, double z)
{
    mnavNearest n;
    mnavAreaType area = NO_POLYGON;
    CHECK(mnavFindNearest(s_navmesh, nullptr, (mnavPos3){x, y, z}, (mnavVec3){0.01f, 1.0f, 0.01f},
                          &n) == mnav_success,
          "nearest");
    if (n.polygon.slot != 0 && n.over)
    {
        CHECK(mnavGetArea(s_navmesh, n.polygon, &area) == mnav_success, "area");
    }
    return area;
}

// The bake over the level: the platform and the ramp are walkable, and
// the agent's radius is kept from the platform's drop.
static void TestLevelLandmarks(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    const mnavTriangleMesh mesh = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    size_t size = 0;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTileInput(baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
    // A surface's solid is at least one cell tall, so the platform's top,
    // 1 m or 8 cell heights up, is at 9, 1.125 m; the ramp's cell at z =
    // 18 m spans 4 to 4.5 cell heights, so its floor is 5, 0.625 m.
    CHECK(AreaAt(28.125, 1.125, 28.125) == mnav_areaWalkable, "platform walkable");
    CHECK(AreaAt(24.125, 0.625, 18.125) == mnav_areaWalkable, "ramp walkable");
    // The platform's edge at x = 30 m has no box near it: 1 m in is
    // walkable, 0.25 m in is within the 0.5 m radius of the drop.
    CHECK(AreaAt(29.125, 1.125, 25.125) == mnav_areaWalkable, "platform near its edge");
    CHECK(AreaAt(29.875, 1.125, 25.125) == NO_POLYGON, "eroded at the drop");
}

int main(void)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    CHECK(mnavCreateQuery(&queryDef, &s_query) == mnav_success, "query");
    mnavBakeDef def = Exact();
    UseDef(&def);
    TestOpenSpansAreTheSpaceAboveWalkableSpans();
    TestLinksFollowTheStepAndHeadroom();
    TestErosionKeepsTheRadiusFromBoundaries();
    TestHoleErodesAround();
    def = Exact();
    UseDef(&def);
    TestOpenSpansToTheFrameEdge();
    TestLevelLandmarks();
    Done();
    mnavDestroyQuery(s_query);
    return s_failures == 0 ? 0 : 1;
}
