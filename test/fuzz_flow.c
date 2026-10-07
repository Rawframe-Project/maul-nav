// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes grid flow fields (mnav-0007): a grid, a filter, a region and
// goals read from the bytes, now and then out of range. Every call must
// end in a typed status. A field built in budgeted steps must be the one
// built at once; each cell's cost must be its next cell's plus the step
// between them, as the search sums it; after random area and goal changes
// a repaired field must be, bit for bit, a fresh build's; everything is
// given back when destroyed.

#include "counting_allocator.h"

#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    SIDE = 40,
    MOST_GOALS = 4,
    MOST_CHANGES = 12
};

#define DIAGONAL 1.4142135623730951

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

static bool Typed(mnavResult result)
{
    return result == mnav_success || result == mnav_errorInvalid || result == mnav_errorRange ||
           result == mnav_errorLimit || result == mnav_errorStale || result == mnav_errorCapacity;
}

// An area: mostly walkable, some dearer, some blocked, now and then one
// the filter may leave out.
static mnavAreaType Area(Reader* r)
{
    uint8_t b = Byte(r);
    return b < 150 ? mnav_areaWalkable : (b < 200 ? (mnavAreaType)(2 + b % 4) : mnav_areaNone);
}

static mnavCell Cell(Reader* r, int32_t width, int32_t height)
{
    // Now and then outside the grid.
    int32_t x = (int32_t)(Byte(r) % (uint32_t)(width + 2)) - 1;
    int32_t y = (int32_t)(Byte(r) % (uint32_t)(height + 2)) - 1;
    return (mnavCell){x, y};
}

// Begins and continues a field in steps of a budget read from the bytes.
static mnavResult BuildInSteps(mnavFlowField* field, const mnavGrid* grid,
                               const mnavQueryFilter* filter, const mnavFlowRegion* region,
                               const mnavCell* goals, int32_t goalCount, int32_t budget)
{
    mnavResult result = mnavBeginFlowField(field, grid, filter, region, goals, goalCount);
    bool ended = false;
    while (result == mnav_success && !ended)
    {
        result = mnavContinueFlowField(field, grid, budget, &ended);
    }
    return result;
}

// Builds a field at once: mnavBuildFlowField for the whole grid, else
// the work begun and continued with no limit.
static mnavResult BuildAtOnce(mnavFlowField* field, const mnavGrid* grid,
                              const mnavQueryFilter* filter, const mnavFlowRegion* region,
                              const mnavCell* goals, int32_t goalCount)
{
    return region == nullptr
               ? mnavBuildFlowField(field, grid, filter, goals, goalCount)
               : BuildInSteps(field, grid, filter, region, goals, goalCount, INT32_MAX);
}

static bool SameBits(double a, double b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

// Whether two fields agree on every cell of the region, cost and next.
static bool SameFields(const mnavFlowField* a, const mnavFlowField* b, mnavFlowRegion r)
{
    for (int32_t y = r.y; y < r.y + r.height; ++y)
    {
        for (int32_t x = r.x; x < r.x + r.width; ++x)
        {
            mnavFlow fa;
            mnavFlow fb;
            Expect(mnavFlowAt(a, (mnavCell){x, y}, &fa) == mnav_success &&
                   mnavFlowAt(b, (mnavCell){x, y}, &fb) == mnav_success);
            if (!SameBits(fa.cost, fb.cost) || fa.next.x != fb.next.x || fa.next.y != fb.next.y)
            {
                return false;
            }
        }
    }
    return true;
}

// Each cell's cost is its next cell's plus the step between them, summed
// as the search sums it; a cell with no way, or a goal, points at itself.
static void CheckWays(const mnavFlowField* field, const mnavGrid* grid,
                      const mnavQueryFilter* filter, mnavFlowRegion r)
{
    for (int32_t y = r.y; y < r.y + r.height; ++y)
    {
        for (int32_t x = r.x; x < r.x + r.width; ++x)
        {
            mnavFlow f;
            Expect(mnavFlowAt(field, (mnavCell){x, y}, &f) == mnav_success);
            bool home = f.next.x == x && f.next.y == y;
            Expect(f.cost >= 0.0 && (isfinite(f.cost) || home));
            if (home || !isfinite(f.cost))
            {
                Expect(home && (f.cost == 0.0 || !isfinite(f.cost)));
                continue;
            }
            int32_t dx = f.next.x - x;
            int32_t dy = f.next.y - y;
            Expect(dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1);
            mnavFlow next;
            Expect(mnavFlowAt(field, f.next, &next) == mnav_success);
            double length = (dx != 0 && dy != 0 ? DIAGONAL : 1.0) * (double)grid->cellSize;
            double here = (double)filter->costs[grid->areas[y * grid->width + x]];
            double there = (double)filter->costs[grid->areas[f.next.y * grid->width + f.next.x]];
            Expect(SameBits(f.cost, next.cost + length * (here + there) * 0.5));
        }
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    Reader r = {data, size, 0};
    static mnavAreaType areas[SIDE * SIDE];
    int32_t width = 1 + Byte(&r) % SIDE;
    int32_t height = 1 + Byte(&r) % SIDE;
    uint8_t sizeByte = Byte(&r);
    float cellSize = sizeByte == 0 ? 0.0f : 0.25f + (float)(sizeByte % 16) * 0.25f;
    for (int32_t c = 0; c < width * height; ++c)
    {
        areas[c] = Area(&r);
    }
    mnavGrid grid = {areas, width, height, cellSize};
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    uint8_t leave = Byte(&r);
    if (leave % 3 == 0)
    {
        filter.areas &= ~((uint64_t)1 << (2 + leave % 4));
    }
    for (int32_t a = 2; a < 6; ++a)
    {
        filter.costs[a] = 1.0f + (float)(Byte(&r) % 8) * 0.5f;
    }
    if (Byte(&r) == 255)
    {
        filter.costs[3] = -1.0f;
    }
    // The whole grid, or a window of it, now and then not within it.
    mnavFlowRegion region = {0, 0, width, height};
    const mnavFlowRegion* window = nullptr;
    if (Byte(&r) % 2 == 1)
    {
        region.x = Byte(&r) % (uint32_t)width;
        region.y = Byte(&r) % (uint32_t)height;
        region.width = 1 + Byte(&r) % (uint32_t)(width - region.x + 1);
        region.height = 1 + Byte(&r) % (uint32_t)(height - region.y + 1);
        window = &region;
    }
    int32_t goalCount = Byte(&r) % (MOST_GOALS + 1);
    mnavCell goals[MOST_GOALS];
    for (int32_t g = 0; g < goalCount; ++g)
    {
        goals[g] = Cell(&r, width, height);
    }
    int32_t budget = 1 + Byte(&r) % 64;

    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.allocator = CountingAllocator();
    def.cells = Byte(&r) == 0 ? 8 : SIDE * SIDE;
    mnavFlowField* stepped = nullptr;
    mnavFlowField* whole = nullptr;
    Expect(mnavCreateFlowField(&def, &stepped) == mnav_success &&
           mnavCreateFlowField(&def, &whole) == mnav_success);
    mnavResult a = BuildInSteps(stepped, &grid, &filter, window, goals, goalCount, budget);
    mnavResult b = BuildAtOnce(whole, &grid, &filter, window, goals, goalCount);
    Expect(Typed(a) && a == b);
    if (a == mnav_success)
    {
        Expect(SameFields(stepped, whole, region));
        CheckWays(whole, &grid, &filter, region);

        // Areas and goals change; the repaired field is the rebuilt one.
        int32_t changeCount = Byte(&r) % (MOST_CHANGES + 1);
        mnavCell changed[MOST_CHANGES];
        for (int32_t k = 0; k < changeCount; ++k)
        {
            mnavCell c = {Byte(&r) % (uint32_t)width, Byte(&r) % (uint32_t)height};
            changed[k] = c;
            areas[c.y * width + c.x] = Area(&r);
        }
        int32_t newGoalCount = Byte(&r) % (MOST_GOALS + 1);
        mnavCell newGoals[MOST_GOALS];
        for (int32_t g = 0; g < newGoalCount; ++g)
        {
            newGoals[g] = (mnavCell){Byte(&r) % (uint32_t)width, Byte(&r) % (uint32_t)height};
        }
        mnavResult u =
            mnavUpdateFlowField(stepped, &grid, newGoals, newGoalCount, changed, changeCount);
        Expect(Typed(u));
        bool ended = u != mnav_success;
        while (!ended)
        {
            Expect(mnavContinueFlowField(stepped, &grid, budget, &ended) == mnav_success);
        }
        Expect(BuildAtOnce(whole, &grid, &filter, window, newGoals, newGoalCount) == mnav_success);
        if (u == mnav_success)
        {
            Expect(SameFields(stepped, whole, region));
            CheckWays(stepped, &grid, &filter, region);
        }
        // A repair before the work ends is stale.
        Expect(mnavBeginFlowField(whole, &grid, &filter, window, goals, goalCount) ==
                   mnav_success &&
               mnavUpdateFlowField(whole, &grid, goals, goalCount, nullptr, 0) == mnav_errorStale);
    }
    mnavDestroyFlowField(stepped);
    mnavDestroyFlowField(whole);
    Expect(s_held == 0);
    return 0;
}
