// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walkable filters on small heightfields built by hand (N13).

#include "filter.h"
#include "heightfield.h"
#include "raster.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    WIDTH = 7,
    MAX_SPANS = 4,
    HEIGHT = 16,
    STEP = 6,
    BASE = MNAV_HEIGHT_OFFSET
};

// A field of WIDTH by WIDTH columns, each with up to MAX_SPANS spans.
typedef struct Field
{
    mnavSpan input[WIDTH * WIDTH][MAX_SPANS];
    int32_t counts[WIDTH * WIDTH];
    uint32_t columns[WIDTH * WIDTH + 1];
    mnavSpan spans[WIDTH * WIDTH * MAX_SPANS];
    mnavHeightfield heightfield;
} Field;

static void AddSpan(Field* field, int32_t x, int32_t z, int32_t bottom, int32_t top,
                    mnavAreaType area)
{
    int32_t column = x + z * WIDTH;
    field->input[column][field->counts[column]++] =
        (mnavSpan){(uint16_t)(BASE + bottom), (uint16_t)(BASE + top), area};
}

// Every column a floor whose top is at height top, walkable.
static void Floor(Field* field, int32_t top)
{
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            AddSpan(field, x, z, top - 1, top, mnav_areaWalkable);
        }
    }
}

static mnavHeightfield* Pack(Field* field)
{
    uint32_t written = 0;
    for (int32_t c = 0; c < WIDTH * WIDTH; ++c)
    {
        field->columns[c] = written;
        for (int32_t k = 0; k < field->counts[c]; ++k)
        {
            field->spans[written++] = field->input[c][k];
        }
    }
    field->columns[WIDTH * WIDTH] = written;
    field->heightfield = (mnavHeightfield){{WIDTH, 0.0f, 0.0f, 0.25f, 0.125f},
                                           field->columns,
                                           field->spans,
                                           (int32_t)written,
                                           (int32_t)written};
    return &field->heightfield;
}

static mnavAreaType Area(const Field* field, int32_t x, int32_t z, int32_t k)
{
    return field->spans[field->columns[x + z * WIDTH] + (uint32_t)k].area;
}

static void TestLowObstacleWithinAStepTakesTheArea(void)
{
    static Field field;
    field = (Field){0};
    // A floor of area 3, a curb 2 cells above it and another 2 above that.
    AddSpan(&field, 0, 0, 0, 10, 3);
    AddSpan(&field, 0, 0, 11, 12, mnav_areaNone);
    AddSpan(&field, 0, 0, 13, 14, mnav_areaNone);
    // A ledge 7 cells above a floor, past the step.
    AddSpan(&field, 1, 0, 0, 10, 3);
    AddSpan(&field, 1, 0, 16, 17, mnav_areaNone);
    mnavFilterLowObstacles(Pack(&field), STEP);
    CHECK(Area(&field, 0, 0, 1) == 3, "the curb takes the floor's area");
    CHECK(Area(&field, 0, 0, 2) == mnav_areaNone, "obstacles do not climb by induction");
    CHECK(Area(&field, 1, 0, 1) == mnav_areaNone, "past the step stays unwalkable");
}

static void TestLowHeadroomIsUnwalkable(void)
{
    static Field field;
    field = (Field){0};
    AddSpan(&field, 0, 0, 0, 10, 1);
    AddSpan(&field, 0, 0, 10 + HEIGHT - 1, 40, mnav_areaNone);
    AddSpan(&field, 1, 0, 0, 10, 1);
    AddSpan(&field, 1, 0, 10 + HEIGHT, 40, mnav_areaNone);
    AddSpan(&field, 2, 0, 0, 10, 1);
    mnavFilterLowHeight(Pack(&field), HEIGHT);
    CHECK(Area(&field, 0, 0, 0) == mnav_areaNone, "one cell short of the agent");
    CHECK(Area(&field, 1, 0, 0) == 1, "exactly the agent's height");
    CHECK(Area(&field, 2, 0, 0) == 1, "open sky");
}

static void TestFieldEdgeIsALedge(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    mnavFilterLedges(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 0, 3, 0) == mnav_areaNone, "the edge column");
    CHECK(Area(&field, 6, 6, 0) == mnav_areaNone, "the corner column");
    CHECK(Area(&field, 1, 1, 0) == 1, "one cell in");
    CHECK(Area(&field, 3, 3, 0) == 1, "the middle");
}

static void TestDropPastAStepIsALedge(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    // Column (3, 3) stands 10 cells above its neighbors.
    field.input[3 + 3 * WIDTH][0] = (mnavSpan){BASE + 109, BASE + 110, 1};
    mnavFilterLedges(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 3, 3, 0) == mnav_areaNone, "the raised cell drops 10");
    CHECK(Area(&field, 2, 3, 0) == 1, "its neighbor only faces a rise");
}

static void TestHoleIsALedge(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    field.counts[3 + 3 * WIDTH] = 0;
    mnavFilterLedges(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 2, 3, 0) == mnav_areaNone, "next to a hole");
    CHECK(Area(&field, 2, 2, 0) == 1, "diagonal to a hole");
}

static void TestSteepNeighborsAreALedge(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    field.input[2 + 3 * WIDTH][0] = (mnavSpan){BASE + 95, BASE + 96, 1};
    field.input[4 + 3 * WIDTH][0] = (mnavSpan){BASE + 103, BASE + 104, 1};
    mnavFilterLedges(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 3, 3, 0) == mnav_areaNone, "neighbors 8 cells apart");
    CHECK(Area(&field, 3, 2, 0) == 1, "4 cells apart is fine");
}

static void TestLedgeIgnoresSpaceTooLow(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    // Over column (3, 3) a roof 10 cells up leaves the cell no room; its
    // neighbor (2, 3) cannot reach it, so the roof's top is no drop.
    AddSpan(&field, 3, 3, 110, 111, mnav_areaNone);
    mnavFilterLedges(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 2, 3, 0) == 1, "a roof the agent cannot enter");
}

static void TestFiltersRunInOrder(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    // A curb 2 cells high under a beam 10 cells above it: low-obstacle
    // makes it walkable, then low-height removes it again.
    AddSpan(&field, 3, 3, 101, 102, mnav_areaNone);
    AddSpan(&field, 3, 3, 112, 120, mnav_areaNone);
    mnavFilterWalkable(Pack(&field), HEIGHT, STEP);
    CHECK(Area(&field, 3, 3, 1) == mnav_areaNone, "a curb under a beam");
    CHECK(Area(&field, 3, 3, 2) == mnav_areaNone, "the beam is not walkable");
}

static void TestLedgeBoundariesAreTheStep(void)
{
    // A cell raised by exactly the step is walkable; one more is a drop.
    for (int32_t rise = STEP; rise <= STEP + 1; ++rise)
    {
        static Field field;
        field = (Field){0};
        Floor(&field, 100);
        field.input[3 + 3 * WIDTH][0] =
            (mnavSpan){(uint16_t)(BASE + 99 + rise), (uint16_t)(BASE + 100 + rise), 1};
        mnavFilterLedges(Pack(&field), HEIGHT, STEP);
        CHECK(Area(&field, 3, 3, 0) == (rise == STEP ? 1 : mnav_areaNone), "drop boundary");
    }
    // Reachable neighbors 3 below and spread - 3 above: spread by exactly
    // the step is walkable; one more is steep.
    for (int32_t spread = STEP; spread <= STEP + 1; ++spread)
    {
        static Field field;
        field = (Field){0};
        Floor(&field, 100);
        int32_t above = spread - 3;
        field.input[4 + 3 * WIDTH][0] =
            (mnavSpan){(uint16_t)(BASE + 99 + above), (uint16_t)(BASE + 100 + above), 1};
        field.input[2 + 3 * WIDTH][0] = (mnavSpan){BASE + 96, BASE + 97, 1};
        mnavFilterLedges(Pack(&field), HEIGHT, STEP);
        CHECK(Area(&field, 3, 3, 0) == (spread == STEP ? 1 : mnav_areaNone), "spread boundary");
    }
}

int main(void)
{
    TestLowObstacleWithinAStepTakesTheArea();
    TestLowHeadroomIsUnwalkable();
    TestFieldEdgeIsALedge();
    TestDropPastAStepIsALedge();
    TestHoleIsALedge();
    TestSteepNeighborsAreALedge();
    TestLedgeIgnoresSpaceTooLow();
    TestFiltersRunInOrder();
    TestLedgeBoundariesAreTheStep();
    return s_failures == 0 ? 0 : 1;
}
