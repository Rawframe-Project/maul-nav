// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The circle clip of the random point near a center: a polygon inside the
// circle stays whole, a circle inside a polygon leaves its 32-gon, a
// circle of no radius leaves nothing, and a clip with too little room
// stops and says so.
//
// White-box: the random point gives the clip room for every corner a
// polygon clipped to the 32-gon has in exact arithmetic, so no public
// call makes it run out.

#include "clip.h"
#include "test_harness.h"

#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>

// A square of side 2 * half about the origin, wound as the navmesh's
// polygons are.
static void Square(double half, mnavPos3 corners[4])
{
    corners[0] = (mnavPos3){-half, 0.0, -half};
    corners[1] = (mnavPos3){-half, 0.0, half};
    corners[2] = (mnavPos3){half, 0.0, half};
    corners[3] = (mnavPos3){half, 0.0, -half};
}

static void TestPolygonInsideCircle(void)
{
    mnavPos3 square[4];
    Square(1.0, square);
    mnavPos3 out[MNAV_CLIP_CORNERS];
    int32_t count =
        mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 10.0, out, MNAV_CLIP_CORNERS);
    bool same = count == 4;
    for (int32_t k = 0; k < count && same; ++k)
    {
        same = out[k].x == square[k].x && out[k].z == square[k].z;
    }
    CHECK(same, "a polygon inside the circle keeps its corners");
}

static void TestCircleInsidePolygon(void)
{
    mnavPos3 square[4];
    Square(100.0, square);
    mnavPos3 out[MNAV_CLIP_CORNERS];
    int32_t count =
        mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 1.0, out, MNAV_CLIP_CORNERS);
    bool onCircle = count == 32;
    for (int32_t k = 0; k < count && onCircle; ++k)
    {
        onCircle = fabs(sqrt(out[k].x * out[k].x + out[k].z * out[k].z) - 1.0) < 1e-12;
    }
    CHECK(onCircle, "a circle inside the polygon leaves its 32-gon");
}

static void TestNoRadius(void)
{
    mnavPos3 square[4];
    Square(1.0, square);
    mnavPos3 out[MNAV_CLIP_CORNERS];
    CHECK(mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 0.0, out, MNAV_CLIP_CORNERS) == 0,
          "a circle of no radius leaves nothing");
}

static void TestTooLittleRoom(void)
{
    mnavPos3 square[4];
    Square(100.0, square);
    mnavPos3 out[MNAV_CLIP_CORNERS];
    // The 32-gon needs 32 corners, and the square 4 to begin with.
    CHECK(mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 1.0, out, 32) == 32,
          "room for the 32-gon is enough");
    CHECK(mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 1.0, out, 31) == -1,
          "one corner less stops the clip");
    CHECK(mnavClipToCircle(square, 4, (mnavPos3){0.0, 0.0, 0.0}, 1.0, out, 3) == -1,
          "room for fewer corners than the polygon has stops it at once");
}

int main(void)
{
    TestPolygonInsideCircle();
    TestCircleInsidePolygon();
    TestNoRadius();
    TestTooLittleRoom();
    return s_failures == 0 ? 0 : 1;
}
