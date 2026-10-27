// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The cuts rasterizing makes along cell lines: a piece splits into the
// corners on each side and the crossings, a corner on the cut goes to
// both, and a cut whose part would pass MNAV_RASTER_CORNERS stops and
// says so.
//
// White-box: a triangle's pieces have at most seven corners in exact
// arithmetic, and no bake makes a cut run out of room, so only a piece
// built by hand reaches the refusal.

#include "raster.h"
#include "test_harness.h"

#include <stdbool.h>
#include <stdint.h>

// A ring of count corners whose u alternates between 0 and 2, so that
// every edge crosses u = 1, w rising along it.
static void Zigzag(int32_t count, mnavRasterPoly* poly, mnavRasterRow* row)
{
    poly->count = count;
    row->count = count;
    for (int32_t k = 0; k < count; ++k)
    {
        float u = k % 2 == 0 ? 0.0f : 2.0f;
        poly->v[k][0] = u;
        poly->v[k][1] = (float)k;
        poly->v[k][2] = (float)k;
        row->u[k] = u;
        row->h[k] = (float)k;
    }
}

static void TestSquare(void)
{
    mnavRasterPoly square = {
        {{0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}}, 4};
    mnavRasterPoly below;
    mnavRasterPoly above;
    CHECK(mnavSplitPoly(&square, 0, 1.0f, &below, &above) && below.count == 4 && above.count == 4,
          "a square cut across leaves four corners on each side");
    CHECK(mnavSplitPoly(&square, 0, 3.0f, &below, &above) && below.count == 4 && above.count == 2,
          "a cut along a side gives its corners to both parts");
}

static void TestRoom(void)
{
    mnavRasterPoly poly;
    mnavRasterRow row;
    mnavRasterPoly polyBelow;
    mnavRasterPoly polyAbove;
    mnavRasterRow rowBelow;
    mnavRasterRow rowAbove;
    // Four crossings and two corners on each side: six each.
    Zigzag(4, &poly, &row);
    CHECK(mnavSplitPoly(&poly, 0, 1.0f, &polyBelow, &polyAbove) && polyBelow.count == 6 &&
              polyAbove.count == 6,
          "a piece of four crossing edges fits");
    CHECK(mnavSplitRow(&row, 1.0f, &rowBelow, &rowAbove) && rowBelow.count == 6 &&
              rowAbove.count == 6,
          "and its row fits");
    // Eight crossings and four corners on each side: twelve, the room.
    Zigzag(8, &poly, &row);
    CHECK(mnavSplitPoly(&poly, 0, 1.0f, &polyBelow, &polyAbove) &&
              polyBelow.count == MNAV_RASTER_CORNERS && polyAbove.count == MNAV_RASTER_CORNERS,
          "a piece filling the room fits");
    CHECK(mnavSplitRow(&row, 1.0f, &rowBelow, &rowAbove) && rowBelow.count == MNAV_RASTER_CORNERS &&
              rowAbove.count == MNAV_RASTER_CORNERS,
          "and its row fits");
    // Ten crossings and five corners on each side: fifteen, past it.
    Zigzag(10, &poly, &row);
    CHECK(!mnavSplitPoly(&poly, 0, 1.0f, &polyBelow, &polyAbove), "a piece past the room stops");
    CHECK(!mnavSplitRow(&row, 1.0f, &rowBelow, &rowAbove), "and its row stops");
}

int main(void)
{
    TestSquare();
    TestRoom();
    return s_failures == 0 ? 0 : 1;
}
