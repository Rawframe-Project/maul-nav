// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields (mnav-0007): Dijkstra's search from every goal at once over
// a grid, with the grid path's steps. The step rule is symmetric, so the
// cost found from the goals to a cell is the cost of the cell's way to
// them. The open list is a binary heap by cost, then by cell index, with
// each cell's place in it kept for lowering its cost.

#include "maul-nav/flow.h"

#include "allocator.h"
#include "draw.h"
#include "query_filter.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>

// Marks a def built by mnavDefaultFlowFieldDef.
#define FLOW_DEF_COOKIE 0x4E415646u

// The diagonal's length in cells, rounded once to binary64.
#define DIAGONAL 1.4142135623730951

// A cell's place in the heap when it is not in it: never reached, or done.
#define NOT_OPEN (-1)
#define DONE     (-2)

struct mnavFlowField
{
    mnavMemory memory;
    mnavFlowFieldDef def;
    // Per cell: the cost to the goals, the next cell's index and the place
    // in the heap.
    double* costs;
    int32_t* next;
    int32_t* places;
    int32_t* heap;
    int32_t heapCount;
    // The grid last built, 0 by 0 when none is.
    int32_t width;
    int32_t height;
};

// A search: the field, the grid and the filter it uses.
typedef struct Search
{
    mnavFlowField* field;
    const mnavGrid* grid;
    const mnavQueryFilter* filter;
} Search;

mnavFlowFieldDef mnavDefaultFlowFieldDef(void)
{
    return (mnavFlowFieldDef){FLOW_DEF_COOKIE, {0}, 65536};
}

mnavResult mnavCreateFlowField(const mnavFlowFieldDef* def, mnavFlowField** fieldOut)
{
    if (fieldOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *fieldOut = nullptr;
    if (def == nullptr || def->cookie != FLOW_DEF_COOKIE ||
        (def->allocator.alloc == nullptr) != (def->allocator.free == nullptr))
    {
        return mnav_errorInvalid;
    }
    if (def->cells < 1 || def->cells > MNAV_MAX_FLOW_CELLS)
    {
        return mnav_errorRange;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavFlowField* f = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavFlowField), alignof(mnavFlowField), (void**)&f);
    if (result != mnav_success)
    {
        return result;
    }
    *f = (mnavFlowField){0};
    f->memory = memory;
    f->def = *def;
    size_t cells = (size_t)def->cells;
    result = mnavAllocate(&f->memory, cells, sizeof(double), alignof(double), (void**)&f->costs);
    if (result == mnav_success)
    {
        result =
            mnavAllocate(&f->memory, cells, sizeof(int32_t), alignof(int32_t), (void**)&f->next);
    }
    if (result == mnav_success)
    {
        result =
            mnavAllocate(&f->memory, cells, sizeof(int32_t), alignof(int32_t), (void**)&f->places);
    }
    if (result == mnav_success)
    {
        result =
            mnavAllocate(&f->memory, cells, sizeof(int32_t), alignof(int32_t), (void**)&f->heap);
    }
    if (result != mnav_success)
    {
        mnavDestroyFlowField(f);
        return result;
    }
    *fieldOut = f;
    return mnav_success;
}

void mnavDestroyFlowField(mnavFlowField* field)
{
    if (field == nullptr)
    {
        return;
    }
    mnavMemory memory = field->memory;
    size_t cells = (size_t)field->def.cells;
    mnavRelease(&memory, field->costs, cells, sizeof(double), alignof(double));
    mnavRelease(&memory, field->next, cells, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, field->places, cells, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, field->heap, cells, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, field, 1, sizeof(mnavFlowField), alignof(mnavFlowField));
}

static bool Open(const Search* s, int32_t x, int32_t y)
{
    if (x < 0 || y < 0 || x >= s->grid->width || y >= s->grid->height)
    {
        return false;
    }
    mnavAreaType area = s->grid->areas[(size_t)y * (size_t)s->grid->width + (size_t)x];
    return area != mnav_areaNone && mnavIncludes(s->filter, area);
}

static double AreaCost(const Search* s, int32_t cell)
{
    return (double)s->filter->costs[s->grid->areas[cell]];
}

// The heap, by cost, then by cell index.
static bool Sooner(const mnavFlowField* f, int32_t a, int32_t b)
{
    return f->costs[a] != f->costs[b] ? f->costs[a] < f->costs[b] : a < b;
}

static void Place(mnavFlowField* f, int32_t at, int32_t cell)
{
    f->heap[at] = cell;
    f->places[cell] = at;
}

static void SiftUp(mnavFlowField* f, int32_t at)
{
    int32_t cell = f->heap[at];
    while (at > 0)
    {
        int32_t up = (at - 1) / 2;
        if (!Sooner(f, cell, f->heap[up]))
        {
            break;
        }
        Place(f, at, f->heap[up]);
        at = up;
    }
    Place(f, at, cell);
}

static int32_t Pop(mnavFlowField* f)
{
    int32_t top = f->heap[0];
    int32_t last = f->heap[--f->heapCount];
    int32_t at = 0;
    while (f->heapCount > 0)
    {
        int32_t child = 2 * at + 1;
        if (child >= f->heapCount)
        {
            break;
        }
        if (child + 1 < f->heapCount && Sooner(f, f->heap[child + 1], f->heap[child]))
        {
            child += 1;
        }
        if (!Sooner(f, f->heap[child], last))
        {
            break;
        }
        Place(f, at, f->heap[child]);
        at = child;
    }
    if (f->heapCount > 0)
    {
        Place(f, at, last);
    }
    f->places[top] = DONE;
    return top;
}

// Gives a cell a lower cost by way of the next cell given.
static void Lower(mnavFlowField* f, int32_t cell, double cost, int32_t next)
{
    f->costs[cell] = cost;
    f->next[cell] = next;
    if (f->places[cell] == NOT_OPEN)
    {
        Place(f, f->heapCount++, cell);
    }
    SiftUp(f, f->places[cell]);
}

// The eight directions, in the order a cell's neighbours are tried.
static const mnavCell s_steps[8] = {{1, 0}, {0, 1},  {-1, 0},  {0, -1},
                                    {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

// Reaches the neighbours of a cell done: each may step to it.
static void Expand(const Search* s, int32_t cell)
{
    mnavFlowField* f = s->field;
    int32_t width = s->grid->width;
    int32_t x = cell % width;
    int32_t y = cell / width;
    double here = AreaCost(s, cell);
    for (int32_t d = 0; d < 8; ++d)
    {
        int32_t dx = s_steps[d].x;
        int32_t dy = s_steps[d].y;
        bool diagonal = dx != 0 && dy != 0;
        if (!Open(s, x + dx, y + dy) || (diagonal && !(Open(s, x + dx, y) && Open(s, x, y + dy))))
        {
            continue;
        }
        int32_t other = (y + dy) * width + (x + dx);
        if (f->places[other] == DONE)
        {
            continue;
        }
        double length = (diagonal ? DIAGONAL : 1.0) * (double)s->grid->cellSize;
        double cost = f->costs[cell] + length * (AreaCost(s, other) + here) * 0.5;
        if (cost < f->costs[other])
        {
            Lower(f, other, cost, cell);
        }
    }
}

static bool GoodGrid(const mnavGrid* grid)
{
    return grid->areas != nullptr && grid->width >= 1 && grid->width <= MNAV_MAX_GRID_SIDE &&
           grid->height >= 1 && grid->height <= MNAV_MAX_GRID_SIDE && isfinite(grid->cellSize) &&
           grid->cellSize > 0.0f;
}

static bool GoodGoals(const mnavGrid* grid, const mnavCell* goals, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        if (goals[i].x < 0 || goals[i].y < 0 || goals[i].x >= grid->width ||
            goals[i].y >= grid->height)
        {
            return false;
        }
    }
    return true;
}

static mnavResult Check(mnavFlowField* field, const mnavGrid* grid, const mnavCell* goals,
                        int32_t goalCount)
{
    if (field == nullptr || grid == nullptr || goalCount < 0 ||
        (goalCount > 0 && goals == nullptr) || !GoodGrid(grid) ||
        !GoodGoals(grid, goals, goalCount))
    {
        return mnav_errorInvalid;
    }
    if ((int64_t)grid->width * (int64_t)grid->height > (int64_t)field->def.cells)
    {
        return mnav_errorLimit;
    }
    return mnav_success;
}

mnavResult mnavBuildFlowField(mnavFlowField* field, const mnavGrid* grid,
                              const mnavQueryFilter* filter, const mnavCell* goals,
                              int32_t goalCount)
{
    if (field != nullptr)
    {
        field->width = 0;
        field->height = 0;
    }
    mnavResult result = Check(field, grid, goals, goalCount);
    const mnavQueryFilter* usable = nullptr;
    if (result == mnav_success)
    {
        result = mnavCheckFilter(filter, &usable);
    }
    if (result != mnav_success)
    {
        return result;
    }
    Search s = {field, grid, usable};
    int32_t cells = grid->width * grid->height;
    for (int32_t c = 0; c < cells; ++c)
    {
        field->costs[c] = (double)INFINITY;
        field->next[c] = c;
        field->places[c] = NOT_OPEN;
    }
    field->heapCount = 0;
    for (int32_t i = 0; i < goalCount; ++i)
    {
        int32_t cell = goals[i].y * grid->width + goals[i].x;
        if (Open(&s, goals[i].x, goals[i].y) && field->costs[cell] != 0.0)
        {
            Lower(field, cell, 0.0, cell);
        }
    }
    while (field->heapCount > 0)
    {
        Expand(&s, Pop(field));
    }
    field->width = grid->width;
    field->height = grid->height;
    return mnav_success;
}

mnavResult mnavFlowAt(const mnavFlowField* field, mnavCell cell, mnavFlow* flowOut)
{
    if (field == nullptr || flowOut == nullptr || cell.x < 0 || cell.y < 0 ||
        cell.x >= field->width || cell.y >= field->height)
    {
        return mnav_errorInvalid;
    }
    int32_t at = cell.y * field->width + cell.x;
    int32_t next = field->next[at];
    *flowOut = (mnavFlow){field->costs[at], {next % field->width, next / field->width}};
    return mnav_success;
}

mnavResult mnavDebugFlowField(const mnavFlowField* field, double cellSize, double height,
                              mnavDebugBuffer* buffer)
{
    if (field == nullptr || !mnavGoodBuffer(buffer) || !isfinite(cellSize) || cellSize <= 0.0 ||
        !isfinite(height))
    {
        return mnav_errorInvalid;
    }
    int32_t cells = field->width * field->height;
    for (int32_t c = 0; c < cells; ++c)
    {
        int32_t next = field->next[c];
        if (next == c)
        {
            continue;
        }
        int32_t column = c % field->width;
        int32_t row = c / field->width;
        int32_t nextColumn = next % field->width;
        int32_t nextRow = next / field->width;
        double x = ((double)column + 0.5) * cellSize;
        double z = ((double)row + 0.5) * cellSize;
        double dx = (double)(nextColumn - column);
        double dz = (double)(nextRow - row);
        double scale = 0.4 * cellSize / sqrt(dx * dx + dz * dz);
        dx *= scale;
        dz *= scale;
        mnavPos3 tail = {x - dx, height, z - dz};
        mnavPos3 head = {x + dx, height, z + dz};
        mnavDrawLine(buffer, tail, head, mnav_debugFlow, 0);
        // Barbs back from the head, a quarter turned either way.
        mnavDrawLine(buffer, head,
                     (mnavPos3){head.x - 0.5 * (dx - dz), height, head.z - 0.5 * (dz + dx)},
                     mnav_debugFlow, 0);
        mnavDrawLine(buffer, head,
                     (mnavPos3){head.x - 0.5 * (dx + dz), height, head.z - 0.5 * (dz - dx)},
                     mnav_debugFlow, 0);
    }
    return mnavDrawResult(buffer);
}
