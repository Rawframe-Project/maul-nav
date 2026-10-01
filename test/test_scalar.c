// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's own cosine and its conversion of meters to cells.

#include "scalar.h"
#include "test_harness.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t Bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void TestCosineIsCloseToTheTrueValue(void)
{
    // Every tenth of a degree below 90, against the double cosine: within
    // 3 units in the last place of the result. At 90 the true value is 0,
    // checked below.
    int worst = 0;
    for (int tenth = 0; tenth < 900; ++tenth)
    {
        float degrees = (float)tenth / 10.0f;
        float ours = mnavCosDegrees(degrees);
        double truth = cos((double)degrees * 3.14159265358979323846 / 180.0);
        double ulp = (double)nextafterf((float)truth, 2.0f) - (double)(float)truth;
        int error = (int)ceil(fabs((double)ours - truth) / ulp);
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 3, "cosine within 3 ulp");
}

static void TestCosineEndpoints(void)
{
    CHECK(mnavCosDegrees(0.0f) == 1.0f, "cos 0 is 1");
    CHECK(mnavCosDegrees(90.0f) == 0.0f, "cos 90 is 0");
    CHECK(mnavCosDegrees(60.0f) > 0.4999999f && mnavCosDegrees(60.0f) < 0.5000001f, "cos 60");
}

static void TestCosineBitsArePinned(void)
{
    // The bits every platform must produce (mnav-0001).
    CHECK(Bits(mnavCosDegrees(45.0f)) == 0x3F3504F2u, "cos 45 bits");
    CHECK(Bits(mnavCosDegrees(30.0f)) == 0x3F5DB3D7u, "cos 30 bits");
    CHECK(Bits(mnavCosDegrees(89.0f)) == 0x3C8EF859u, "cos 89 bits");
}

static void TestCellsSnapNearIntegers(void)
{
    int32_t cells = -1;
    // 0.9 / 0.3 is 2.99999976 in binary32: a climb of 3 cells, not 2.
    CHECK(mnavToCells(0.9f, 0.3f, false, 100, &cells) && cells == 3, "climb snaps to 3");
    CHECK(mnavToCells(1.8f, 0.15f, false, 100, &cells) && cells == 12, "climb snaps to 12");
    CHECK(mnavToCells(2.0f, 0.2f, true, 100, &cells) && cells == 10, "height stays 10");
}

static void TestCellsRoundByDirection(void)
{
    int32_t cells = -1;
    CHECK(mnavToCells(0.5f, 0.3f, true, 100, &cells) && cells == 2, "radius rounds up");
    CHECK(mnavToCells(0.5f, 0.3f, false, 100, &cells) && cells == 1, "climb rounds down");
    CHECK(mnavToCells(0.0f, 0.3f, true, 100, &cells) && cells == 0, "zero is zero");
}

static void TestCellsRefuseTooMany(void)
{
    int32_t cells = -1;
    CHECK(!mnavToCells(10.0f, 0.1f, true, 99, &cells), "100 cells past 99");
    CHECK(mnavToCells(9.9f, 0.1f, true, 99, &cells) && cells == 99, "99 cells fit");
    CHECK(!mnavToCells(3.0e38f, 0.001f, true, 99, &cells), "infinite quotient refused");
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
