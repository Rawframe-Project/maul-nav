// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walkable filters on small scenes of flat cell squares, read from
// the baked tile: a step within reach takes the floor's area, too little
// headroom, also below the top of the heights, ledges at drops, holes,
// gaps and steep neighbors, space too low to enter, and the filters'
// order. The ledge cases run in the open,
// where most columns hold one span, and again under a roof over the
// scene, where every column holds two.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// The default agent in cells of 0.125 m: 2 m tall, a step of 0.75 m.
enum
{
    HEIGHT = 16,
    STEP = 6,
    SIDE = 9,
    AT = 40,
    ROOF = 40,
    MORE = 6,
    MOST = SIDE * SIDE + MORE + 1,
    NO_FLOOR = -100,
    NO_POLYGON = 0xFF,
    TILE_ROOM = 1 << 20
};

// A square of SIDE by SIDE cells at cell AT of tile (0, 0), a flat square
// of one cell at each cell's level, a few more squares, and a roof at
// level ROOF over them all, or none. A square at level k fills its cells
// from height k to k + 1, so a span's top is its level plus 1. Squares of
// mnav_areaNone are solid but not walkable.
typedef struct Square
{
    int32_t x;
    int32_t z;
    int32_t level;
    mnavAreaType area;
} Square;

typedef struct Scene
{
    int32_t level[SIDE][SIDE];
    mnavAreaType area[SIDE][SIDE];
    Square more[MORE];
    int32_t moreCount;
    bool roof;
} Scene;

// Every cell's floor at a level, walkable, under a roof or not.
static Scene Flat(int32_t level, bool roof)
{
    Scene scene = {0};
    scene.roof = roof;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            scene.level[z][x] = level;
            scene.area[z][x] = mnav_areaWalkable;
        }
    }
    return scene;
}

static void Add(Scene* scene, int32_t x, int32_t z, int32_t level, mnavAreaType area)
{
    scene->more[scene->moreCount++] = (Square){x, z, level, area};
}

static mnavBakeDef s_def;
static mnavBaker* s_baker;
static mnavNavmesh* s_navmesh;
static uint8_t s_tile[TILE_ROOM];
static mnavVec3 s_vertices[4 * MOST];
static int32_t s_indices[6 * MOST];
static mnavAreaType s_areas[2 * MOST];

// Puts square n of cells sides by sides from a cell.
static void Put(int32_t n, Square s, int32_t sides)
{
    float x0 = (float)(AT + s.x) * 0.25f;
    float z0 = (float)(AT + s.z) * 0.25f;
    float x1 = (float)(AT + s.x + sides) * 0.25f;
    float z1 = (float)(AT + s.z + sides) * 0.25f;
    float y = (float)s.level * 0.125f;
    s_vertices[4 * n + 0] = (mnavVec3){x0, y, z0};
    s_vertices[4 * n + 1] = (mnavVec3){x1, y, z0};
    s_vertices[4 * n + 2] = (mnavVec3){x1, y, z1};
    s_vertices[4 * n + 3] = (mnavVec3){x0, y, z1};
    const int32_t corners[6] = {0, 3, 2, 0, 2, 1};
    for (int32_t k = 0; k < 6; ++k)
    {
        s_indices[6 * n + k] = 4 * n + corners[k];
    }
    s_areas[2 * n] = s.area;
    s_areas[2 * n + 1] = s.area;
}

// Bakes the scene and loads its tile in a fresh navmesh.
static void Load(const Scene* scene)
{
    int32_t n = 0;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            if (scene->level[z][x] != NO_FLOOR)
            {
                Put(n++, (Square){x, z, scene->level[z][x], scene->area[z][x]}, 1);
            }
        }
    }
    for (int32_t k = 0; k < scene->moreCount; ++k)
    {
        Put(n++, scene->more[k], 1);
    }
    if (scene->roof)
    {
        Put(n++, (Square){0, 0, ROOF, mnav_areaNone}, SIDE);
    }
    const mnavTriangleMesh mesh = {s_vertices, 4 * n, s_indices, 2 * n, s_areas};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    size_t size = 0;
    CHECK(mnavBakeTileInput(s_baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavCopyBakedTile(s_baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
}

// The area of the polygon over cell (x, z) of the scene within a meter
// of a level, or NO_POLYGON. A column of these scenes holds one walkable
// span at most, and a polygon's surface over a single raised or lowered
// cell may lie nearer its neighbors' height.
static mnavAreaType At(int32_t x, int32_t z, int32_t level)
{
    mnavPos3 center = {(AT + x + 0.5) * 0.25, level * 0.125, (AT + z + 0.5) * 0.25};
    mnavNearest n;
    mnavAreaType area = NO_POLYGON;
    CHECK(mnavFindNearest(s_navmesh, nullptr, center, (mnavVec3){0.01f, 1.0f, 0.01f}, &n) ==
              mnav_success,
          "nearest");
    if (n.polygon.slot != 0 && n.over)
    {
        CHECK(mnavGetArea(s_navmesh, n.polygon, &area) == mnav_success, "area");
    }
    return area;
}

static void TestLowObstacleWithinAStepTakesTheArea(void)
{
    // A floor of area 3 with curbs on it: 2 levels up at (2, 2), 2 and
    // then 2 more at (5, 2), exactly a step up at (5, 5), and 7 levels up,
    // past the step, at (2, 5).
    Scene scene = Flat(0, false);
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            scene.area[z][x] = 3;
        }
    }
    Add(&scene, 2, 2, 2, mnav_areaNone);
    Add(&scene, 5, 2, 2, mnav_areaNone);
    Add(&scene, 5, 2, 4, mnav_areaNone);
    Add(&scene, 5, 5, STEP, mnav_areaNone);
    Add(&scene, 2, 5, STEP + 1, mnav_areaNone);
    Load(&scene);
    CHECK(At(2, 2, 2) == 3, "the curb takes the floor's area");
    CHECK(At(5, 2, 4) == NO_POLYGON, "obstacles do not climb by induction");
    CHECK(At(5, 5, STEP) == 3, "a curb exactly a step up");
    CHECK(At(2, 5, STEP + 1) == NO_POLYGON, "past the step stays unwalkable");
    CHECK(At(3, 3, 0) == 3, "the floor between");
}

static void TestLowHeadroomIsUnwalkable(void)
{
    // Roofs over (2, 3) one level short of the agent, and over (4, 3) at
    // exactly its height.
    Scene scene = Flat(0, false);
    Add(&scene, 2, 3, HEIGHT, mnav_areaNone);
    Add(&scene, 4, 3, HEIGHT + 1, mnav_areaNone);
    Load(&scene);
    CHECK(At(2, 3, 0) == NO_POLYGON, "one cell short of the agent");
    CHECK(At(4, 3, 0) == mnav_areaWalkable, "exactly the agent's height");
    CHECK(At(3, 3, 0) == mnav_areaWalkable, "open sky");
}

static void TestRoomBelowTheTopOfHeights(void)
{
    // Heights end at level 32767, the most a span's top can be: a floor
    // at level 32766 - HEIGHT leaves the agent's height above it, one
    // level higher does not.
    for (int32_t level = 32766 - HEIGHT; level <= 32767 - HEIGHT; ++level)
    {
        Scene scene = Flat(level, false);
        Load(&scene);
        CHECK(At(3, 3, level) == (level == 32766 - HEIGHT ? mnav_areaWalkable : NO_POLYGON),
              "room below the top of the heights");
    }
}

static void TestFloorEdgeIsALedge(bool roof)
{
    Scene scene = Flat(0, roof);
    Load(&scene);
    CHECK(At(0, 3, 0) == NO_POLYGON, "the edge column");
    CHECK(At(SIDE - 1, SIDE - 1, 0) == NO_POLYGON, "the corner column");
    CHECK(At(1, 1, 0) == mnav_areaWalkable, "one cell in");
    CHECK(At(3, 3, 0) == mnav_areaWalkable, "the middle");
}

static void TestDropPastAStepIsALedge(bool roof)
{
    // Column (3, 3) stands 10 levels above its neighbors.
    Scene scene = Flat(0, roof);
    scene.level[3][3] = 10;
    Load(&scene);
    CHECK(At(3, 3, 10) == NO_POLYGON && At(3, 3, 0) == NO_POLYGON, "the raised cell drops 10");
    CHECK(At(2, 3, 0) == mnav_areaWalkable, "its neighbor only faces a rise");
}

static void TestHoleIsALedge(bool roof)
{
    Scene scene = Flat(0, roof);
    scene.level[3][3] = NO_FLOOR;
    Load(&scene);
    CHECK(At(2, 3, 0) == NO_POLYGON && At(4, 3, 0) == NO_POLYGON && At(3, 2, 0) == NO_POLYGON &&
              At(3, 4, 0) == NO_POLYGON,
          "next to a hole");
    CHECK(At(2, 2, 0) == mnav_areaWalkable && At(4, 4, 0) == mnav_areaWalkable,
          "diagonal to a hole");
}

static void TestGapUnderANeighborIsADrop(bool roof)
{
    // No floor at (4, 3) or (4, 5), but a square over each: at (4, 3)
    // the agent's height above the floor, so the agent fits under it and
    // (3, 3) drops; at (4, 5) one level lower, so (3, 5) does not.
    Scene scene = Flat(0, roof);
    scene.level[3][4] = NO_FLOOR;
    scene.level[5][4] = NO_FLOOR;
    Add(&scene, 4, 3, 1 + HEIGHT, mnav_areaNone);
    Add(&scene, 4, 5, HEIGHT, mnav_areaNone);
    Load(&scene);
    CHECK(At(3, 3, 0) == NO_POLYGON, "room under the neighbor: a drop");
    CHECK(At(3, 5, 0) == mnav_areaWalkable, "no room under the neighbor");
}

static void TestSteepNeighborsAreALedge(bool roof)
{
    // Around (3, 3) at level 8: one neighbor 4 levels down, one 4 up.
    Scene scene = Flat(8, roof);
    scene.level[3][2] = 4;
    scene.level[3][4] = 12;
    Load(&scene);
    CHECK(At(3, 3, 8) == NO_POLYGON, "neighbors 8 levels apart");
    CHECK(At(3, 2, 8) == mnav_areaWalkable && At(2, 3, 4) == mnav_areaWalkable &&
              At(4, 3, 12) == mnav_areaWalkable,
          "4 levels apart is fine");
}

static void TestLedgeIgnoresSpaceTooLow(bool roof)
{
    // Over column (3, 3) a roof 10 levels up leaves the cell no room; its
    // neighbor (2, 3) cannot reach it, so the roof's top is no drop. Over
    // column (3, 5), 7 levels down, a roof as high leaves the agent too
    // little room to step down, so (2, 5) does not drop either.
    Scene scene = Flat(0, roof);
    Add(&scene, 3, 3, 10, mnav_areaNone);
    scene.level[5][3] = -STEP - 1;
    Add(&scene, 3, 5, 10, mnav_areaNone);
    Load(&scene);
    CHECK(At(2, 3, 0) == mnav_areaWalkable, "a roof the agent cannot enter");
    CHECK(At(3, 3, 0) == NO_POLYGON, "the cell under the roof");
    CHECK(At(2, 5, 0) == mnav_areaWalkable, "a drop the agent cannot enter");
}

static void TestFiltersRunInOrder(void)
{
    // A curb 2 levels high under a beam 10 levels above it: low-obstacle
    // makes it walkable, then low-height removes it again.
    Scene scene = Flat(0, false);
    Add(&scene, 3, 3, 2, mnav_areaNone);
    Add(&scene, 3, 3, 12, mnav_areaNone);
    Load(&scene);
    CHECK(At(3, 3, 2) == NO_POLYGON, "a curb under a beam");
    CHECK(At(3, 3, 12) == NO_POLYGON, "the beam is not walkable");
}

static void TestLedgeBoundariesAreTheStep(bool roof)
{
    // A cell raised by exactly the step is walkable; one more is a drop.
    for (int32_t rise = STEP; rise <= STEP + 1; ++rise)
    {
        Scene scene = Flat(0, roof);
        scene.level[3][3] = rise;
        Load(&scene);
        CHECK(At(3, 3, rise) == (rise == STEP ? mnav_areaWalkable : NO_POLYGON), "drop boundary");
    }
    // Reachable neighbors below levels down and spread - below up: spread
    // by exactly the step is walkable; one more is steep.
    const int32_t belows[3] = {1, 3, STEP};
    for (int32_t b = 0; b < 3; ++b)
    {
        for (int32_t spread = STEP; spread <= STEP + 1; ++spread)
        {
            Scene scene = Flat(8, roof);
            scene.level[3][4] = 8 + spread - belows[b];
            scene.level[3][2] = 8 - belows[b];
            Load(&scene);
            CHECK(At(3, 3, 8) == (spread == STEP ? mnav_areaWalkable : NO_POLYGON),
                  "spread boundary");
        }
    }
}

int main(void)
{
    // An agent of no radius, regions of any size and no edge error, so
    // that the polygons cover exactly the cells the filters leave
    // walkable.
    s_def = mnavDefaultBakeDef();
    s_def.agent.radius = 0.0f;
    s_def.minRegionArea = 0.0f;
    s_def.maxEdgeError = 0.0f;
    CHECK(mnavCreateBaker(&s_def, &s_baker).result == mnav_success, "baker");
    TestLowObstacleWithinAStepTakesTheArea();
    TestLowHeadroomIsUnwalkable();
    TestRoomBelowTheTopOfHeights();
    TestFiltersRunInOrder();
    for (int32_t roof = 0; roof < 2; ++roof)
    {
        TestFloorEdgeIsALedge(roof == 1);
        TestDropPastAStepIsALedge(roof == 1);
        TestHoleIsALedge(roof == 1);
        TestGapUnderANeighborIsADrop(roof == 1);
        TestSteepNeighborsAreALedge(roof == 1);
        TestLedgeIgnoresSpaceTooLow(roof == 1);
        TestLedgeBoundariesAreTheStep(roof == 1);
    }
    mnavDestroyNavmesh(s_navmesh);
    mnavDestroyBaker(s_baker);
    return s_failures == 0 ? 0 : 1;
}
