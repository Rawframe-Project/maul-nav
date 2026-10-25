// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The walkable filters on random heightfields against the plain ledge
// filter, run alone and as mnavFilterWalkable runs them in one pass, and
// the field's edge as a ledge. White-box: the filters run alone have no
// public form, a bake shows only the spans walkable at its end, and the
// field's edge lies in the tile's border, which no tile holds. The
// filters' cases on scenes a host can bake are in test_filter.c.

#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>
#include <string.h>

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

// The ledge filter the plain way: every walkable span against its four
// neighbors' spans, by the rule mnavFilterLedges keeps.
static bool PlainLedge(const mnavHeightfield* f, int32_t x, int32_t z, uint32_t i)
{
    const int32_t dx[4] = {-1, 0, 1, 0};
    const int32_t dz[4] = {0, 1, 0, -1};
    uint32_t end = f->columns[x + z * WIDTH + 1];
    int32_t floor = f->spans[i].top;
    int32_t ceiling = i + 1 < end ? (int32_t)f->spans[i + 1].bottom : 65535;
    int32_t low = floor;
    int32_t high = floor;
    bool drop = false;
    for (int32_t d = 0; d < 4; ++d)
    {
        int32_t nx = x + dx[d];
        int32_t nz = z + dz[d];
        if (nx < 0 || nz < 0 || nx >= WIDTH || nz >= WIDTH)
        {
            return true;
        }
        uint32_t first = f->columns[nx + nz * WIDTH];
        uint32_t last = f->columns[nx + nz * WIDTH + 1];
        int32_t under = first < last ? (int32_t)f->spans[first].bottom : 65535;
        drop = drop || (ceiling < under ? ceiling : under) - floor >= HEIGHT;
        for (uint32_t k = first; k < last; ++k)
        {
            int32_t top = f->spans[k].top;
            int32_t above = k + 1 < last ? (int32_t)f->spans[k + 1].bottom : 65535;
            int32_t room = (ceiling < above ? ceiling : above) - (floor > top ? floor : top);
            if (room < HEIGHT)
            {
                continue;
            }
            drop = drop || top - floor < -STEP;
            if (top - floor >= -STEP && top - floor <= STEP)
            {
                low = top < low ? top : low;
                high = top > high ? top : high;
            }
        }
    }
    return drop || high - low > STEP;
}

static uint32_t s_random = 1;

static int32_t Random(int32_t n)
{
    s_random = s_random * 1664525u + 1013904223u;
    return (int32_t)(s_random >> 8) % n;
}

// A random field: columns of 0 to MAX_SPANS spans apart from each other,
// around height base, with floors a few steps apart and gaps near the
// agent's height, walkable or not.
static void RandomField(Field* field, int32_t base)
{
    *field = (Field){0};
    for (int32_t c = 0; c < WIDTH * WIDTH; ++c)
    {
        int32_t count = Random(10) < 7 ? 1 : Random(MAX_SPANS + 1);
        int32_t top = base + Random(4 * STEP) - 2 * STEP;
        for (int32_t k = 0; k < count; ++k)
        {
            int32_t bottom = top - 1 - Random(3);
            mnavAreaType area = Random(4) == 0 ? mnav_areaNone : (mnavAreaType)(1 + Random(3));
            AddSpan(field, c % WIDTH, c / WIDTH, bottom, top, area);
            top += 2 + Random(2 * HEIGHT);
        }
    }
}

// mnavFilterWalkable, and the three filters run alone, against the low
// obstacle filter, the plain ledge filter and the low height filter in
// turn, on random fields, some near the top of the heights, where a span
// alone in its column has too little room above.
static void TestOnePassAsThreeAndPlain(void)
{
    int32_t wrong = 0;
    for (int32_t run = 0; run < 2000; ++run)
    {
        static Field field;
        static Field alone;
        static Field plain;
        RandomField(&field, run % 5 == 0 ? 32767 - HEIGHT / 2 - Random(HEIGHT) : 100);
        alone = field;
        plain = field;
        mnavFilterWalkable(Pack(&field), HEIGHT, STEP);
        mnavHeightfield* f = Pack(&alone);
        mnavFilterLowObstacles(f, STEP);
        mnavFilterLedges(f, HEIGHT, STEP);
        mnavFilterLowHeight(f, HEIGHT);
        f = Pack(&plain);
        mnavFilterLowObstacles(f, STEP);
        static bool ledges[WIDTH * WIDTH * MAX_SPANS];
        for (int32_t c = 0; c < WIDTH * WIDTH; ++c)
        {
            for (uint32_t i = f->columns[c]; i < f->columns[c + 1]; ++i)
            {
                ledges[i] =
                    f->spans[i].area != mnav_areaNone && PlainLedge(f, c % WIDTH, c / WIDTH, i);
            }
        }
        for (int32_t i = 0; i < f->spanCount; ++i)
        {
            f->spans[i].area = ledges[i] ? mnav_areaNone : f->spans[i].area;
        }
        mnavFilterLowHeight(f, HEIGHT);
        for (int32_t i = 0; i < f->spanCount; ++i)
        {
            wrong += field.spans[i].area != plain.spans[i].area ? 1 : 0;
            wrong += alone.spans[i].area != plain.spans[i].area ? 1 : 0;
        }
    }
    CHECK(wrong == 0, "one pass and the filters alone as the plain filters");
}

int main(void)
{
    TestFieldEdgeIsALedge();
    TestOnePassAsThreeAndPlain();
    return s_failures == 0 ? 0 : 1;
}
