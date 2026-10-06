// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// ORCA's linear programs (mnav-0006) on lines laid out by hand, parallel
// ones included.

#include "orca.h"
#include "test_harness.h"

#include "maul-nav/avoidance.h"

#include <math.h>
#include <stdint.h>

static void TestFacingParallels(void)
{
    // y at least 1 and y at most 0: nothing allowed; the 2D program fails
    // at the second line.
    const mnavLine lines[2] = {{{0.0, 1.0}, {1.0, 0.0}}, {{0.0, 0.0}, {-1.0, 0.0}}};
    mnavPos2 result = {0.0, 0.0};
    CHECK(mnavLinearProgram2(lines, 2, 5.0, (mnavPos2){0.0, 0.5}, false, &result) == 1 &&
              result.y == 1.0,
          "facing parallels leave nothing");
}

static void TestFacingParallelsBalanced(void)
{
    // The same two lines through the 3D program: their bisector is the
    // line halfway between them, and the velocity breaking both least
    // lies on it, y = 0.5.
    const mnavLine lines[2] = {{{0.0, 1.0}, {1.0, 0.0}}, {{0.0, 0.0}, {-1.0, 0.0}}};
    mnavLine scratch[2];
    mnavPos2 result = {0.0, 0.0};
    int32_t failed = mnavLinearProgram2(lines, 2, 5.0, (mnavPos2){0.0, 0.5}, false, &result);
    mnavLinearProgram3(lines, 2, 0, failed, 5.0, scratch, &result);
    CHECK(failed == 1 && result.y == 0.5 && fabs(result.x) <= 5.0, "halfway between them");
}

static void TestSameWayParallels(void)
{
    // y at least 3 and y at least 4 within speed 1: the 3D program finds
    // the velocity breaking them least, straight up at full speed.
    const mnavLine lines[2] = {{{0.0, 3.0}, {1.0, 0.0}}, {{0.0, 4.0}, {1.0, 0.0}}};
    mnavLine scratch[2];
    mnavPos2 result = {0.0, 0.0};
    int32_t failed = mnavLinearProgram2(lines, 2, 1.0, (mnavPos2){0.5, 0.0}, false, &result);
    CHECK(failed == 0, "fails at once");
    mnavLinearProgram3(lines, 2, 0, failed, 1.0, scratch, &result);
    CHECK(isfinite(result.x) && isfinite(result.y) && fabs(result.y - 1.0) < 1e-12 &&
              fabs(result.x) < 1e-6,
          "straight up");
}

static void TestNearestOnALine(void)
{
    // One line, y at least 1, wanting (2, 0) within speed 5: (2, 1).
    const mnavLine line = {{0.0, 1.0}, {1.0, 0.0}};
    mnavPos2 result = {0.0, 0.0};
    CHECK(mnavLinearProgram2(&line, 1, 5.0, (mnavPos2){2.0, 0.0}, false, &result) == 1 &&
              result.x == 2.0 && result.y == 1.0,
          "the nearest allowed");
    CHECK(mnavLinearProgram2(&line, 1, 5.0, (mnavPos2){9.0, 9.0}, false, &result) == 1 &&
              fabs(result.x * result.x + result.y * result.y - 25.0) < 1e-9,
          "too fast: on the circle");
}

static void TestOnePointLeft(void)
{
    // x at most 0, x at least 0, then y at least 0: on the third line the
    // first two leave the single point t = 0, which is feasible: (0, 0).
    const mnavLine lines[3] = {
        {{0.0, 0.0}, {0.0, 1.0}}, {{0.0, 0.0}, {0.0, -1.0}}, {{0.0, 0.0}, {1.0, 0.0}}};
    mnavPos2 result = {0.0, 0.0};
    CHECK(mnavLinearProgram2(lines, 3, 5.0, (mnavPos2){0.0, -1.0}, false, &result) == 3 &&
              result.x == 0.0 && result.y == 0.0,
          "an interval of one point is kept");
}

static void TestTouchingTheSpeed(void)
{
    // y at least 5 within speed 5 leaves one velocity, (0, 5).
    const mnavLine line = {{0.0, 5.0}, {1.0, 0.0}};
    mnavPos2 result = {0.0, 0.0};
    CHECK(mnavLinearProgram2(&line, 1, 5.0, (mnavPos2){0.0, 0.0}, false, &result) == 1 &&
              result.x == 0.0 && result.y == 5.0,
          "a line touching the speed circle is kept");
}

int main(void)
{
    TestFacingParallels();
    TestFacingParallelsBalanced();
    TestSameWayParallels();
    TestNearestOnALine();
    TestOnePointLeft();
    TestTouchingTheSpeed();
    return s_failures == 0 ? 0 : 1;
}
