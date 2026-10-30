// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floor heights under one polygon, read from baked tiles through the
// detail surface they give: which spans a polygon reads, how far past it
// the patch reaches, and the search round a sample whose own cell has no
// floor. Samples fall every cell and are kept within no error, so a
// sample off the surface shows as a corner. test_height_patch_hand.c
// covers the cases no bake reaches.

#include "defs_for.h"
#include "ramp_level.h"
#include "squares.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/debug.h"

#include <stdint.h>

enum
{
    DRAW_ROOM = 1 << 14,
    // A deck over the floor with room for the agent between them.
    DECK = 21
};

static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawTriangles[DRAW_ROOM];
// Cells the scene was moved along z from AT, and HasCorner reads it from.
static int32_t s_movedZ;

// Draws every polygon's detail triangles; returns the number of corners.
static int32_t Draw(void)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0},
                         s_drawVertices,
                         DRAW_ROOM,
                         0,
                         s_drawTriangles,
                         DRAW_ROOM,
                         0,
                         nullptr,
                         0,
                         0};
    // Polygon edges go uncounted into a buffer with no line room.
    (void)mnavNavmesh_Debug(s_navmesh, &(mnavTileRange){0, 0, 0, 0}, &b);
    CHECK(b.vertexCount <= DRAW_ROOM && b.triangleCount <= DRAW_ROOM, "the triangles fit");
    return b.vertexCount;
}

// Whether a detail corner lies at (x, y, z): x and z in sixteenths of a
// cell of the scene, y in cell heights.
static bool HasCorner(int64_t x, int64_t y, int64_t z)
{
    int32_t count = Draw();
    for (int32_t i = 0; i < count; ++i)
    {
        const mnavDebugVertex* v = &s_drawVertices[i];
        if ((int64_t)(v->x * 64.0f) - AT * 16 == x && (int64_t)(v->y * 8.0f) == y &&
            (int64_t)(v->z * 64.0f) - (AT + s_movedZ) * 16 == z)
        {
            return true;
        }
    }
    return false;
}

// The exact def within an edge error in sixteenths of a cell, sampled
// every cell within no error.
static void UseError(int32_t error)
{
    mnavBakeDef def = Exact();
    def.maxEdgeError = 0.25f * (float)error / 16.0f;
    def.detailSampleDistance = 0.25f;
    def.detailMaxError = 0.0f;
    UseDef(&def);
}

static void TestPolygonsReadTheirOwnFloor(void)
{
    // A deck over the floor on x = 1 to 5. The floor's polygons read the
    // floor under the deck, and the deck's the deck: every corner lies on
    // one or the other.
    Scene scene = Flat(0, false);
    AddCells(&scene, 1, 0, 5, SIDE - 1, DECK, mnav_areaWalkable);
    UseError(0);
    Load(&scene);
    int32_t count = Draw();
    int32_t floor = 0;
    int32_t deck = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        int64_t y = (int64_t)(s_drawVertices[i].y * 8.0f);
        floor += y == 1 ? 1 : 0;
        deck += y == DECK + 1 ? 1 : 0;
    }
    CHECK(floor > 0 && deck > 0 && floor + deck == count, "the floor or the deck");
}

static void TestNoSampleOnTheSidesFallsBack(void)
{
    // The floor's polygon runs from cell 1 to 8, its +X and +Z sides on
    // the ledge cells 8, which have no floor. Each sample on those sides
    // takes the floor from the first ring round its cell.
    Scene scene = Flat(0, false);
    UseError(0);
    Load(&scene);
    CHECK(s_report.fallbackHeights == 0, "none falls back");
}

static void TestPatchReachesACellPastThePolygon(void)
{
    // Found by a search of scenes: holes at (4, 3) and (4, 6) cut their
    // neighbors as ledges, and within half a cell of edge error a floor
    // polygon still covers (4, 2). Its sample there has no floor of its
    // own; the cells round it hold the floor's top, 1, and a step's, 2,
    // nearest the polygon's mean. The step is a cell of area 7, its own
    // region, outside the floor polygon: the patch reaches it as the cell
    // past the polygon, flooded from the floor, and the step's height is
    // kept. On the -X side the polygon starts at x = 4 and the step is at
    // (3, 2); on the +X side the polygon ends at x = 5 and the step is at
    // (5, 2).
    const int32_t steps[2] = {3, 5};
    const int64_t corners[2] = {74, 70};
    for (int32_t side = 0; side < 2; ++side)
    {
        Scene scene = Flat(0, false);
        scene.level[2][steps[side]] = 1;
        scene.area[2][steps[side]] = 7;
        scene.level[3][4] = NO_FLOOR;
        scene.level[6][4] = NO_FLOOR;
        UseError(8);
        Load(&scene);
        CHECK(HasCorner(corners[side], 2, 37), "the step's height at (4, 2)");
    }
}

// The scene with its cells moved dz cells along z, as squares of its
// more: the grid may then cross the tile's -Z side.
static Scene MovedZ(const Scene* scene, int32_t dz)
{
    Scene moved = *scene;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            if (scene->level[z][x] != NO_FLOOR)
            {
                Add(&moved, x, z + dz, scene->level[z][x], scene->area[z][x]);
            }
            moved.level[z][x] = NO_FLOOR;
        }
    }
    return moved;
}

static void TestPatchReachesACellPastTheTileSide(void)
{
    // Found by a search of scenes: the scene moved so that its row 4 is
    // the tile's first, z = 0. A hole at (3, 4) cuts (4, 4) as a ledge,
    // and within half a cell of edge error a polygon still covers it. Its
    // sample there has no floor of its own; the cells round it hold the
    // floor's top, 1, and across the tile's -Z side, at (4, 3), a step's,
    // 2, nearest the polygon's mean. The patch reaches the row past the
    // tile's side, in its border, and the step's height is kept.
    Scene scene = Flat(0, false);
    scene.level[3][4] = 1;
    scene.level[4][3] = NO_FLOOR;
    s_movedZ = -AT - 4;
    Scene moved = MovedZ(&scene, s_movedZ);
    UseError(8);
    Load(&moved);
    CHECK(HasCorner(75, 2, 74), "the step's height at (4, 4)");
    s_movedZ = 0;
}

static void TestSearchTakesTheNearestHeight(void)
{
    // Found by a search of scenes: a hole at (6, 3) cuts (6, 4) as a
    // ledge, which a polygon covers within half a cell of edge error. The
    // cells round its sample there hold the floor's top, 1, and at (5, 5)
    // a step's, 2, which is the polygon's mean. The step's height is the
    // nearest; a signed difference would take the floor's, below it.
    Scene scene = Flat(0, false);
    scene.level[3][6] = NO_FLOOR;
    scene.level[5][5] = 1;
    UseError(8);
    Load(&scene);
    CHECK(HasCorner(106, 2, 75), "the step's height at (6, 4)");
}

static void TestSearchKeepsTheFirstOnTies(void)
{
    // Found by a search of scenes: a step at (1, 7) one level up and one
    // at (5, 7) two up raise the polygon's mean to 2. The sample at the
    // ledge cell (4, 8) finds, in the row below, the floor's 1 at (3, 7)
    // and (4, 7) and the step's 3 at (5, 7), each 1 from the mean. The
    // first in ring order is the floor's, and the floor is kept flat.
    Scene scene = Flat(0, false);
    scene.level[7][1] = 1;
    scene.level[7][5] = 2;
    UseError(4);
    Load(&scene);
    CHECK(HasCorner(48, 1, 128) && !HasCorner(64, 3, 128), "the floor's height at (4, 8)");
}

static void TestSearchReadsTheWholeRing(void)
{
    // Found by a search of scenes: holes at (4, 4) and (3, 6) leave (4, 5)
    // and (4, 6) as ledges whose only floor within the ring is on their +X
    // side, at (5, 5) and (5, 6). Within two cells of edge error their
    // samples search two rings and find it.
    Scene scene = Flat(0, false);
    scene.level[4][4] = NO_FLOOR;
    scene.level[6][3] = NO_FLOOR;
    scene.level[7][6] = NO_FLOOR;
    UseError(32);
    Load(&scene);
    CHECK(s_report.fallbackHeights == 0, "none falls back");
}

// Bakes the ramp level, its ramp up to z = top and its deck on to z = end,
// turned turns times, and loads its tile.
static void LoadRampLevel(const mnavBakeDef* def, int32_t turns, float top, float end)
{
    mnavVec3 vertices[RAMP_LEVEL_VERTICES];
    int32_t indices[RAMP_LEVEL_INDICES];
    const mnavTriangleMesh mesh = {vertices, RAMP_LEVEL_VERTICES, indices,
                                   MakeRampLevelTo(vertices, indices, turns, top, end), nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    UseDef(def);
    CHECK(mnavBaker_BakeTile(s_baker, &input, 0, 0, &s_report) == mnav_success &&
              mnavBaker_CopyTile(s_baker, s_tile, TILE_ROOM, &s_tileSize) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&s_def);
    CHECK(mnavCreateNavmesh(&meshDef, &s_navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(s_navmesh, s_tile, s_tileSize).result == mnav_success &&
              mnavNavmesh_Commit(s_navmesh) == mnav_success,
          "loaded");
}

// Whether the surface over (x, z) of the level before its turns lies on
// the level's height y there, or at most 0.15 m above.
static bool OnTheLevel(float x, float z, float y, int32_t turns)
{
    TurnOnTile(&x, &z, turns);
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(s_navmesh, (mnavPos3){(double)x, (double)y, (double)z},
                                  (mnavVec3){0.01f, 0.4f, 0.01f}, nullptr, &n) == mnav_success,
          "nearest");
    double off = n.point.y - (double)y;
    return n.over && off >= 0.0 && off <= 0.15;
}

static void TestMixedPolygonsSeedFromTheirVertices(void)
{
    // The ramp level, turned to each of the tile's sides: where the floor
    // and the ramp meet on the side, removing their vertex there leaves
    // polygons of both regions. Their patch is seeded from the span
    // nearest each vertex among the cells round its corner and flooded
    // from there, so their surface follows the floor, the ramp and the
    // deck.
    for (int32_t turns = 0; turns < 4; ++turns)
    {
        mnavBakeDef def = mnavDefaultBakeDef();
        LoadRampLevel(&def, turns, 22.0f, 30.0f);
        bool follows = true;
        for (int32_t k = 0; k < 59; ++k)
        {
            float z = 0.125f + 0.5f * (float)k;
            float y = z < 10.0f ? 0.0f : (z > 22.0f ? 3.0f : (z - 10.0f) * 0.25f);
            follows = follows && OnTheLevel(31.875f, z, y, turns);
        }
        CHECK(follows, "the surface on the floor, the ramp and the deck");
    }
}

static void TestMixedVertexSeedsFromTheFirstCell(void)
{
    // Found by a search of scenes: the ramp level with its ramp 8 m long
    // and its deck 5 m, turned once, within 1.5 m of edge error, sampled
    // every cell within no error. A mixed polygon's vertices find the
    // same floor in several of the cells round their corners, equally
    // near; the first cell seeds the patch, and the surface over the
    // ramp's last meter before the tile's side follows the ramp. Seeding
    // from the last cell floods the floor under the ramp into the patch,
    // and the surface over (30.875, 16.125) falls to the floor.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.maxEdgeError = 1.5f;
    def.detailSampleDistance = 0.25f;
    def.detailMaxError = 0.0f;
    LoadRampLevel(&def, 1, 18.0f, 23.0f);
    bool follows = true;
    for (int32_t i = 0; i < 5; ++i)
    {
        for (int32_t k = 0; k < 28; ++k)
        {
            float x = 30.875f + 0.25f * (float)i;
            float z = 10.625f + 0.25f * (float)k;
            follows = follows && OnTheLevel(x, z, (z - 10.0f) * 0.375f, 1);
        }
    }
    CHECK(follows, "the surface on the ramp");
}

int main(void)
{
    TestPolygonsReadTheirOwnFloor();
    TestNoSampleOnTheSidesFallsBack();
    TestPatchReachesACellPastThePolygon();
    TestPatchReachesACellPastTheTileSide();
    TestSearchTakesTheNearestHeight();
    TestSearchKeepsTheFirstOnTies();
    TestSearchReadsTheWholeRing();
    TestMixedPolygonsSeedFromTheirVertices();
    TestMixedVertexSeedsFromTheFirstCell();
    Done();
    return s_failures == 0 ? 0 : 1;
}
