// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walkable filters on small scenes of flat cell squares, read from
// the baked tile: a step within reach takes the floor's area, too little
// headroom, also below the top of the heights, ledges at drops, holes,
// gaps and steep neighbors, space too low to enter, and the filters'
// order. The ledge cases run in the open, where most columns hold one
// span, and again under a roof over the scene, where every column holds
// two.

#include "squares.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

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
    mnavBakeDef def = Exact();
    UseDef(&def);
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
    Done();
    return s_failures == 0 ? 0 : 1;
}
