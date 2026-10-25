// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tracing and simplifying region contours, read from the baked tile's
// polygon edges: walls follow the outlines and holes as simplified, area
// borders are simplified alike from both sides, long walls are split,
// corners take the highest floor around them, and the tile's sides split
// where what lies inside or beyond them changes.

#include "ramp_level.h"
#include "squares.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/debug.h"

#include <stdint.h>
#include <stdlib.h>

enum
{
    LINE_ROOM = 1 << 14
};

static mnavDebugVertex s_lineVertices[2 * LINE_ROOM];
static uint32_t s_lineIndices[2 * LINE_ROOM];

// An edge of the tile's polygons in cells of the scene, from (x0, z0) to
// (x1, z1), at the heights of its ends in cell heights.
typedef struct Line
{
    int32_t x0;
    int32_t z0;
    int32_t x1;
    int32_t z1;
    int32_t y0;
    int32_t y1;
} Line;

static Line s_lines[LINE_ROOM];

// The scene's cells of a world place.
static int32_t Cell(float meters)
{
    return (int32_t)(meters * 4.0f) - AT;
}

// Reads the edges of one kind, each once; returns how many.
static int32_t Lines(mnavDebugKind kind)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0},
                         s_lineVertices,
                         2 * LINE_ROOM,
                         0,
                         nullptr,
                         0,
                         0,
                         s_lineIndices,
                         2 * LINE_ROOM,
                         0};
    // Polygon fills go uncounted into a buffer with no triangle room.
    (void)mnavDebugNavmesh(s_navmesh, 0, 0, 0, 0, &b);
    CHECK(b.vertexCount <= 2 * LINE_ROOM && b.lineCount <= 2 * LINE_ROOM, "the edges fit");
    int32_t count = 0;
    for (int32_t l = 0; l < b.lineCount / 2; ++l)
    {
        const mnavDebugVertex* a = &s_lineVertices[s_lineIndices[2 * l]];
        const mnavDebugVertex* c = &s_lineVertices[s_lineIndices[2 * l + 1]];
        if (a->kind == kind)
        {
            s_lines[count++] = (Line){Cell(a->x),
                                      Cell(a->z),
                                      Cell(c->x),
                                      Cell(c->z),
                                      (int32_t)(a->y * 8.0f),
                                      (int32_t)(c->y * 8.0f)};
        }
    }
    return count;
}

// Whether one of the lines read runs between two places, either way.
static bool HasLine(int32_t count, int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    for (int32_t l = 0; l < count; ++l)
    {
        const Line* line = &s_lines[l];
        if ((line->x0 == x0 && line->z0 == z0 && line->x1 == x1 && line->z1 == z1) ||
            (line->x0 == x1 && line->z0 == z1 && line->x1 == x0 && line->z1 == z0))
        {
            return true;
        }
    }
    return false;
}

static void TestSquareIsFourWalls(void)
{
    // Inside its ledge ring the scene's walkable cells are 1 to 7.
    Scene scene = Flat(0, false);
    Load(&scene);
    int32_t walls = Lines(mnav_debugWall);
    CHECK(walls == 4 && HasLine(walls, 1, 1, 8, 1) && HasLine(walls, 8, 1, 8, 8) &&
              HasLine(walls, 8, 8, 1, 8) && HasLine(walls, 1, 8, 1, 1),
          "the corners of the walkable cells");
    bool level = true;
    for (int32_t l = 0; l < walls; ++l)
    {
        level = level && s_lines[l].y0 == 1 && s_lines[l].y1 == 1;
    }
    CHECK(level, "at the floor's top");
    CHECK(Lines(mnav_debugInnerEdge) == 0 && Lines(mnav_debugTileSide) == 0 &&
              s_report.polygons == 1,
          "one polygon, walls all round");
}

static void TestDroppedRegionHasNoContour(void)
{
    // An island of 3 by 3 squares away from the scene, one walkable cell
    // inside its ledges, under the smallest region: it is dropped, and the
    // scene's square keeps its four walls alone.
    mnavBakeDef def = Exact();
    def.minRegionArea = 0.125f;
    UseDef(&def);
    Scene scene = Flat(0, false);
    AddCells(&scene, 20, 20, 22, 22, 0, mnav_areaWalkable);
    Load(&scene);
    CHECK(s_report.regions == 1 && s_report.droppedRegions == 1, "the island dropped");
    CHECK(Lines(mnav_debugWall) == 4, "the square's walls alone");
}

static void TestHoleIsItsOwnContour(void)
{
    // An unwalkable cell level with the floor at (4, 4). Within an error
    // of half a cell its contour keeps four corners; within a cell it
    // simplifies to its diagonal and goes, and the polygon covers it.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        mnavBakeDef def = Exact();
        def.maxEdgeError = pass == 0 ? 0.125f : 0.25f;
        UseDef(&def);
        Scene scene = Flat(0, false);
        scene.area[4][4] = mnav_areaNone;
        Load(&scene);
        int32_t walls = Lines(mnav_debugWall);
        if (pass == 0)
        {
            CHECK(walls == 8 && HasLine(walls, 4, 4, 5, 4) && HasLine(walls, 5, 4, 5, 5) &&
                      HasLine(walls, 5, 5, 4, 5) && HasLine(walls, 4, 5, 4, 4),
                  "the outline and the hole around cell (4, 4)");
            CHECK(At(4, 4, 0) == NO_POLYGON, "the hole is left out");
        }
        else
        {
            CHECK(walls == 4 && At(4, 4, 0) == mnav_areaWalkable, "the hole simplified away");
        }
    }
}

static void TestRegionsShareTheirEdge(void)
{
    // Cells from x = 4 on of area 7: two regions, meeting along x = 4
    // from z = 1 to 8, where the polygons of each meet the other's.
    mnavBakeDef def = Exact();
    UseDef(&def);
    Scene scene = Flat(0, false);
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 4; x < SIDE; ++x)
        {
            scene.area[z][x] = 7;
        }
    }
    Load(&scene);
    CHECK(s_report.regions == 2, "two regions");
    int32_t inner = Lines(mnav_debugInnerEdge);
    int32_t along = 0;
    for (int32_t l = 0; l < inner; ++l)
    {
        const Line* line = &s_lines[l];
        along += line->x0 == 4 && line->x1 == 4 ? abs(line->z1 - line->z0) : 0;
    }
    CHECK(along == 7, "the shared edge joins the polygons along its length");
    int32_t walls = Lines(mnav_debugWall);
    bool crosses = false;
    for (int32_t l = 0; l < walls; ++l)
    {
        crosses = crosses || (s_lines[l].x0 == 4 && s_lines[l].x1 == 4);
    }
    CHECK(!crosses && At(3, 4, 0) == mnav_areaWalkable && At(4, 4, 0) == 7,
          "no wall between, each side its area");
}

static void TestAreaBordersAreSimplifiedLikeWalls(void)
{
    // Area 7 on the cells with x + z of at least 9: the two regions meet
    // in a staircase of unit steps from (2, 8) to (8, 2). Without error
    // each step is kept; within 0.75 cells the stair, its corners 0.71
    // cells off, is one diagonal. Within half a cell every corner ties
    // as the farthest; both regions pick the same ones, so no wall opens
    // between them.
    const float errors[3] = {0.0f, 0.1875f, 0.125f};
    for (int32_t pass = 0; pass < 3; ++pass)
    {
        mnavBakeDef def = Exact();
        def.maxEdgeError = errors[pass];
        UseDef(&def);
        Scene scene = Flat(0, false);
        for (int32_t z = 0; z < SIDE; ++z)
        {
            for (int32_t x = 0; x < SIDE; ++x)
            {
                scene.area[z][x] = x + z >= 9 ? 7 : mnav_areaWalkable;
            }
        }
        Load(&scene);
        int32_t inner = Lines(mnav_debugInnerEdge);
        if (pass == 0)
        {
            bool steps = true;
            for (int32_t k = 2; k < 8; ++k)
            {
                steps = steps && HasLine(inner, k, 10 - k, k, 9 - k) &&
                        HasLine(inner, k, 9 - k, k + 1, 9 - k);
            }
            CHECK(steps, "every step kept");
        }
        else if (pass == 1)
        {
            CHECK(inner == 1 && HasLine(inner, 2, 8, 8, 2), "one diagonal");
        }
        int32_t walls = Lines(mnav_debugWall);
        bool outside = true;
        for (int32_t l = 0; l < walls; ++l)
        {
            const Line* w = &s_lines[l];
            outside = outside && (w->x0 == w->x1 || w->z0 == w->z1) &&
                      (w->x0 == 1 || w->x0 == 8 || w->z0 == 1 || w->z0 == 8);
        }
        CHECK(outside, "walls only round the outside");
    }
}

static void TestErrorDecidesASmallNotch(void)
{
    // An unwalkable cell in the -Z wall at (4, 1), its inner corners a
    // cell off the wall: kept within half a cell, simplified away within
    // exactly one, and within two.
    const float errors[3] = {0.125f, 0.25f, 0.5f};
    for (int32_t pass = 0; pass < 3; ++pass)
    {
        mnavBakeDef def = Exact();
        def.maxEdgeError = errors[pass];
        UseDef(&def);
        Scene scene = Flat(0, false);
        scene.area[1][4] = mnav_areaNone;
        Load(&scene);
        int32_t walls = Lines(mnav_debugWall);
        CHECK(walls == (pass == 0 ? 8 : 4), "the notch kept or not");
        CHECK(At(4, 1, 0) == (pass == 0 ? NO_POLYGON : mnav_areaWalkable), "the notch's cell");
    }
}

// Cells (1, 1) and (2, 2) left out: the outline walks in at corner (2, 2),
// round the cell and out at the same corner. Simplified hard, the loop
// round the cell goes and the corner is left twice running; the repeat
// is dropped, and the outline still bounds the region.
static void TestCornerHoleLeavesNoRepeat(void)
{
    const float errors[3] = {0.125f, 0.375f, 0.5f};
    for (int32_t pass = 0; pass < 3; ++pass)
    {
        mnavBakeDef def = Exact();
        def.maxEdgeError = errors[pass];
        UseDef(&def);
        Scene scene = Flat(0, false);
        scene.area[1][1] = mnav_areaNone;
        scene.area[2][2] = mnav_areaNone;
        Load(&scene);
        int32_t walls = Lines(mnav_debugWall);
        bool repeats = false;
        for (int32_t l = 0; l < walls; ++l)
        {
            repeats = repeats || (s_lines[l].x0 == s_lines[l].x1 && s_lines[l].z0 == s_lines[l].z1);
        }
        CHECK(walls >= 3 && !repeats && s_report.polygons > 0, "no wall of no length");
    }
}

static void TestLongWallsAreSplit(void)
{
    // Area 7 from x = 4 on: walls of 3, 4 and 7 cells and a border of 7
    // between the areas. At most 2 cells long, the walls of 3 and 4 cells
    // split in two and those of 7 in four; the border is not a wall and
    // stays whole. With no limit nothing is split.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        mnavBakeDef def = Exact();
        def.maxEdgeLength = pass == 0 ? 0.5f : 0.0f;
        UseDef(&def);
        Scene scene = Flat(0, false);
        for (int32_t z = 0; z < SIDE; ++z)
        {
            for (int32_t x = 4; x < SIDE; ++x)
            {
                scene.area[z][x] = 7;
            }
        }
        Load(&scene);
        int32_t walls = Lines(mnav_debugWall);
        bool short_ = true;
        for (int32_t l = 0; l < walls; ++l)
        {
            int32_t dx = s_lines[l].x1 - s_lines[l].x0;
            int32_t dz = s_lines[l].z1 - s_lines[l].z0;
            short_ = short_ && dx * dx + dz * dz <= 4;
        }
        int32_t inner = Lines(mnav_debugInnerEdge);
        CHECK(pass == 0 ? short_ && walls == 16 : walls == 6, "walls split by length");
        CHECK(HasLine(inner, 4, 1, 4, 8), "the border whole");
    }
}

// The top of the floor of cell (x, z), or 0 for no walkable floor.
static int32_t Top(const Scene* scene, int32_t x, int32_t z)
{
    bool inside = x >= 1 && x < SIDE - 1 && z >= 1 && z < SIDE - 1;
    return inside && scene->area[z][x] != mnav_areaNone ? scene->level[z][x] + 1 : 0;
}

// Whether every end of the lines read is as high as the highest floor of
// the cells around it.
static bool EndsAtTheHighestFloor(const Scene* scene, int32_t count)
{
    bool highest = count > 0;
    for (int32_t l = 0; l < count; ++l)
    {
        for (int32_t end = 0; end < 2; ++end)
        {
            int32_t x = end == 0 ? s_lines[l].x0 : s_lines[l].x1;
            int32_t z = end == 0 ? s_lines[l].z0 : s_lines[l].z1;
            int32_t top = 0;
            for (int32_t k = 0; k < 4; ++k)
            {
                int32_t t = Top(scene, x - 1 + k % 2, z - 1 + k / 2);
                top = t > top ? t : top;
            }
            highest = highest && (end == 0 ? s_lines[l].y0 : s_lines[l].y1) == top;
        }
    }
    return highest;
}

static void TestCornersTakeTheHighestFloorAround(void)
{
    // Floors 0 to 3 levels up, all within a step, with a notch at (4, 1),
    // holes at (4, 4) and (2, 5), and area 7 from x = 5 on, or where x + z
    // is at least 9, a staircase: every edge's end, on a wall or between
    // the areas, is as high as the highest floor of the cells around it.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        mnavBakeDef def = Exact();
        UseDef(&def);
        Scene scene = Flat(0, false);
        for (int32_t z = 0; z < SIDE; ++z)
        {
            for (int32_t x = 0; x < SIDE; ++x)
            {
                scene.level[z][x] = (3 * x + 5 * z) % 4;
                bool seven = pass == 0 ? x >= 5 : x + z >= 9;
                scene.area[z][x] = seven ? 7 : mnav_areaWalkable;
            }
        }
        scene.area[1][4] = mnav_areaNone;
        scene.area[4][4] = mnav_areaNone;
        scene.area[5][2] = mnav_areaNone;
        Load(&scene);
        CHECK(EndsAtTheHighestFloor(&scene, Lines(mnav_debugWall)), "walls");
        CHECK(EndsAtTheHighestFloor(&scene, Lines(mnav_debugInnerEdge)), "inner edges");
    }
}

// Whether one of the lines read runs between two places of the tile, in
// its cells, either way.
static bool HasTileLine(int32_t count, int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    return HasLine(count, x0 - AT, z0 - AT, x1 - AT, z1 - AT);
}

static void TestTileSides(void)
{
    // A 2D bake over the whole tile: area 7 from 16 m on, but not beyond
    // the -Z side, a cut from 4 m to 8 m along X reaching past the +Z
    // side, and border cells of area 9 beyond the -Z side up to 8 m.
    const mnavVec2 all[4] = {{-5, -5}, {-5, 40}, {40, 40}, {40, -5}};
    const mnavVec2 seven[4] = {{16, 0}, {16, 40}, {40, 40}, {40, 0}};
    const mnavVec2 cut[4] = {{4, 28}, {4, 40}, {8, 40}, {8, 28}};
    const mnavVec2 nine[4] = {{-5, -5}, {-5, -0.1f}, {8, -0.1f}, {8, -5}};
    const mnavOutline outlines[4] = {
        {all, 4, mnav_areaWalkable}, {seven, 4, 7}, {cut, 4, mnav_areaNone}, {nine, 4, 9}};
    mnavBakeDef def = Exact();
    UseDef(&def);
    size_t size = 0;
    CHECK(mnavBakeTile2D(s_baker, outlines, 4, 0, 0, &s_report) == mnav_success &&
              mnavCopyBakedTile(s_baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
    // The sides split where the areas inside change, at 16 m, where the
    // border's areas change, at 8 m on the -Z side, and where the cut
    // leaves the +Z side; the cut's three edges are walls.
    int32_t sides = Lines(mnav_debugTileSide);
    CHECK(sides == 8 && HasTileLine(sides, 0, 0, 32, 0) && HasTileLine(sides, 32, 0, 64, 0) &&
              HasTileLine(sides, 64, 0, 128, 0) && HasTileLine(sides, 128, 0, 128, 128) &&
              HasTileLine(sides, 0, 0, 0, 128) && HasTileLine(sides, 0, 128, 16, 128) &&
              HasTileLine(sides, 32, 128, 64, 128) && HasTileLine(sides, 64, 128, 128, 128),
          "the tile's sides");
    int32_t walls = Lines(mnav_debugWall);
    CHECK(walls == 3 && HasTileLine(walls, 16, 128, 16, 112) &&
              HasTileLine(walls, 16, 112, 32, 112) && HasTileLine(walls, 32, 112, 32, 128),
          "the cut's walls");
}

static void TestRegionsOfOneAreaMeetOnTheSide(void)
{
    // The ramp level: the floor and the ramp over it are two regions of
    // one area, meeting on the tile's +X side under the ramp's foot,
    // 12.75 m in. The corner there is marked, and the polygons lose it:
    // the ramp's region runs along the side from the tile's corner to the
    // deck's end at 29.5 m as one edge.
    mnavVec3 vertices[RAMP_LEVEL_VERTICES];
    int32_t indices[RAMP_LEVEL_INDICES];
    const mnavTriangleMesh mesh = {vertices, RAMP_LEVEL_VERTICES, indices,
                                   MakeRampLevel(vertices, indices, 0), nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    UseDef(&def);
    size_t size = 0;
    CHECK(mnavBakeTileInput(s_baker, &input, 0, 0, &s_report) == mnav_success &&
              mnavCopyBakedTile(s_baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
    int32_t sides = Lines(mnav_debugTileSide);
    CHECK(s_report.regions == 2 && HasTileLine(sides, 128, 0, 128, 118),
          "one edge across where the regions meet");
}

int main(void)
{
    mnavBakeDef def = Exact();
    UseDef(&def);
    TestSquareIsFourWalls();
    TestDroppedRegionHasNoContour();
    TestHoleIsItsOwnContour();
    TestRegionsShareTheirEdge();
    TestAreaBordersAreSimplifiedLikeWalls();
    TestErrorDecidesASmallNotch();
    TestCornerHoleLeavesNoRepeat();
    TestLongWallsAreSplit();
    TestCornersTakeTheHighestFloorAround();
    TestTileSides();
    TestRegionsOfOneAreaMeetOnTheSide();
    Done();
    return s_failures == 0 ? 0 : 1;
}
