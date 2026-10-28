// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's own cosine and its conversion of meters to cells, as a
// bake def's cells give them (mnavValidateBakeDef): the cosine of the
// agent's maximum slope, and its height, step height and radius.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t Bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

// The cells of the default def with the agent's maximum slope.
static float Cosine(float degrees)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.maxSlopeDegrees = degrees;
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "valid");
    return cells.cosMaxSlope;
}

// The cells of the default def with a cell size and height and an agent.
static mnavBakeDefResult Cells(float cellSize, float cellHeight, float radius, float height,
                               float stepHeight, mnavBakeCells* cells)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = cellSize;
    def.cellHeight = cellHeight;
    def.agent.radius = radius;
    def.agent.height = height;
    def.agent.stepHeight = stepHeight;
    return mnavValidateBakeDef(&def, cells);
}

static void TestCosineIsCloseToTheTrueValue(void)
{
    // Every tenth of a degree below 90, against the double cosine: within
    // 3 units in the last place of the result. A slope of 90 is refused.
    int worst = 0;
    for (int tenth = 0; tenth < 900; ++tenth)
    {
        float degrees = (float)tenth / 10.0f;
        float ours = Cosine(degrees);
        double truth = cos((double)degrees * 3.14159265358979323846 / 180.0);
        double ulp = (double)nextafterf((float)truth, 2.0f) - (double)(float)truth;
        int error = (int)ceil(fabs((double)ours - truth) / ulp);
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 3, "cosine within 3 ulp");
}

static void TestCosineEndpoints(void)
{
    CHECK(Cosine(0.0f) == 1.0f, "cos 0 is 1");
    CHECK(Cosine(60.0f) > 0.4999999f && Cosine(60.0f) < 0.5000001f, "cos 60");
}

static void TestCosineBitsArePinned(void)
{
    // The bits every platform must produce.
    CHECK(Bits(Cosine(45.0f)) == 0x3F3504F2u, "cos 45 bits");
    CHECK(Bits(Cosine(30.0f)) == 0x3F5DB3D7u, "cos 30 bits");
    CHECK(Bits(Cosine(89.0f)) == 0x3C8EF859u, "cos 89 bits");
}

static void TestCellsSnapNearIntegers(void)
{
    mnavBakeCells cells;
    // 0.9 / 0.3 is 2.99999976 in binary32: a step of 3 cell heights, not 2.
    CHECK(Cells(0.25f, 0.3f, 0.5f, 2.0f, 0.9f, &cells).result == mnav_success &&
              cells.agentStep == 3,
          "step snaps to 3");
    CHECK(Cells(0.25f, 0.15f, 0.5f, 2.0f, 1.8f, &cells).result == mnav_success &&
              cells.agentStep == 12,
          "step snaps to 12");
    CHECK(Cells(0.25f, 0.2f, 0.5f, 2.0f, 0.6f, &cells).result == mnav_success &&
              cells.agentHeight == 10,
          "height stays 10");
}

static void TestCellsRoundByDirection(void)
{
    mnavBakeCells cells;
    CHECK(Cells(0.3f, 0.3f, 0.5f, 2.0f, 0.5f, &cells).result == mnav_success &&
              cells.agentRadius == 2 && cells.agentStep == 1,
          "the radius rounds up, the step down");
    CHECK(Cells(0.3f, 0.3f, 0.0f, 2.0f, 0.0f, &cells).result == mnav_success &&
              cells.agentRadius == 0 && cells.agentStep == 0,
          "zero is zero");
}

static void TestCellsRefuseTooMany(void)
{
    // Heights up to MNAV_MAX_HEIGHT_CELLS cell heights of 0.125 m.
    mnavBakeCells cells;
    mnavBakeDefResult result = Cells(0.25f, 0.125f, 0.5f, 4096.0f, 0.75f, &cells);
    CHECK(result.result == mnav_errorLimit && result.setting == mnav_settingAgentHeight,
          "32768 cell heights past the limit");
    CHECK(Cells(0.25f, 0.125f, 0.5f, 4095.875f, 0.75f, &cells).result == mnav_success &&
              cells.agentHeight == MNAV_MAX_HEIGHT_CELLS,
          "32767 fit");
    // 32767.25 cell heights, rounded up, are 32768.
    result = Cells(0.25f, 0.125f, 0.5f, 4095.90625f, 0.75f, &cells);
    CHECK(result.result == mnav_errorLimit && result.setting == mnav_settingAgentHeight,
          "a quarter past the limit, rounded up, past it");
    result = Cells(0.25f, 0.001f, 0.5f, 2.0f, 3.0e38f, &cells);
    CHECK(result.result == mnav_errorLimit && result.setting == mnav_settingAgentStepHeight,
          "an infinite quotient refused");
}

int main(void)
{
    TestCosineIsCloseToTheTrueValue();
    TestCosineEndpoints();
    TestCosineBitsArePinned();
    TestCellsSnapNearIntegers();
    TestCellsRoundByDirection();
    TestCellsRefuseTooMany();
    return s_failures == 0 ? 0 : 1;
}
