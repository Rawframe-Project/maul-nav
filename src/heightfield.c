// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A tile's solid heightfield, merged from a sorted set of fragments.

#include "heightfield.h"

#include "allocator.h"
#include "outline.h"
#include "raster.h"

#include "maul-nav/bake.h"

#include <string.h>

static mnavResult Collect(mnavMemory* memory, const mnavBakeDef* def, const mnavBakeCells* cells,
                          const mnavTriangleMesh* meshes, int32_t meshCount,
                          const mnavTileFrame* frame, mnavFragmentList* list)
{
    int32_t touching = 0;
    for (int32_t m = 0; m < meshCount; ++m)
    {
        const mnavTriangleMesh* mesh = &meshes[m];
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            const int32_t* index = mesh->indices + (size_t)t * 3;
            const mnavVec3 corners[3] = {mesh->vertices[index[0]], mesh->vertices[index[1]],
                                         mesh->vertices[index[2]]};
            mnavAreaType given = mesh->areas != nullptr ? mesh->areas[t] : mnav_areaWalkable;
            int32_t area = mnavTriangleArea(corners, given, cells->cosMaxSlope);
            if (area < 0 || !mnavTriangleTouchesTile(frame, corners))
            {
                continue;
            }
            if (++touching > def->limits.tileTriangles)
            {
                return mnav_errorLimit;
            }
            mnavResult result =
                mnavRasterizeTriangle(memory, frame, corners, (mnavAreaType)area, list);
            if (result != mnav_success)
            {
                return result;
            }
        }
    }
    return mnav_success;
}

static bool Before(const mnavFragment* a, const mnavFragment* b)
{
    if (a->bottom != b->bottom)
    {
        return a->bottom < b->bottom;
    }
    if (a->top != b->top)
    {
        return a->top < b->top;
    }
    return a->area < b->area;
}

// Orders fragments by column with a counting pass, then each column by
// bottom, top and area. columns receives each column's start in sorted.
static void Sort(const mnavFragmentList* list, int32_t width, uint32_t* columns,
                 mnavFragment* sorted)
{
    int32_t columnCount = width * width;
    memset(columns, 0, ((size_t)columnCount + 1) * sizeof(uint32_t));
    for (int32_t i = 0; i < list->count; ++i)
    {
        columns[list->items[i].x + list->items[i].z * width + 1] += 1;
    }
    for (int32_t c = 0; c < columnCount; ++c)
    {
        columns[c + 1] += columns[c];
    }
    for (int32_t i = 0; i < list->count; ++i)
    {
        const mnavFragment* fragment = &list->items[i];
        int32_t column = fragment->x + fragment->z * width;
        // The column's start serves as its cursor and is restored below.
        sorted[columns[column]++] = *fragment;
    }
    for (int32_t c = columnCount; c > 0; --c)
    {
        columns[c] = columns[c - 1];
    }
    columns[0] = 0;
    for (int32_t c = 0; c < columnCount; ++c)
    {
        for (uint32_t i = columns[c] + 1; i < columns[c + 1]; ++i)
        {
            mnavFragment item = sorted[i];
            uint32_t j = i;
            while (j > columns[c] && Before(&item, &sorted[j - 1]))
            {
                sorted[j] = sorted[j - 1];
                --j;
            }
            sorted[j] = item;
        }
    }
}

// Merges one column's sorted fragments, first to end, into spans written
// from out; returns the number written.
static uint32_t MergeColumn(const mnavFragment* fragments, uint32_t first, uint32_t end,
                            int32_t step, mnavSpan* out)
{
    uint32_t written = 0;
    uint32_t i = first;
    while (i < end)
    {
        uint32_t runEnd = i + 1;
        uint16_t top = fragments[i].top;
        while (runEnd < end && fragments[runEnd].bottom <= top)
        {
            top = fragments[runEnd].top > top ? fragments[runEnd].top : top;
            ++runEnd;
        }
        mnavAreaType area = mnav_areaNone;
        for (uint32_t k = i; k < runEnd; ++k)
        {
            if ((int32_t)fragments[k].top + step >= (int32_t)top && fragments[k].area > area)
            {
                area = fragments[k].area;
            }
        }
        out[written++] = (mnavSpan){fragments[i].bottom, top, area};
        i = runEnd;
    }
    return written;
}

static mnavResult Merge(mnavMemory* memory, const mnavFragment* sorted, int32_t count, int32_t step,
                        mnavHeightfield* heightfield)
{
    mnavResult result = mnavAllocate(memory, (size_t)count, sizeof(mnavSpan), alignof(mnavSpan),
                                     (void**)&heightfield->spans);
    if (result != mnav_success)
    {
        return result;
    }
    int32_t columnCount = heightfield->frame.width * heightfield->frame.width;
    uint32_t* columns = heightfield->columns;
    uint32_t written = 0;
    uint32_t first = columns[0];
    for (int32_t c = 0; c < columnCount; ++c)
    {
        uint32_t end = columns[c + 1];
        columns[c] = written;
        written += MergeColumn(sorted, first, end, step, heightfield->spans + written);
        first = end;
    }
    columns[columnCount] = written;
    heightfield->spanCount = (int32_t)written;
    // The span array keeps the fragment count's size: merging only shrinks
    // and the bytes are released with the heightfield.
    heightfield->spanCapacity = count;
    return mnav_success;
}

static mnavResult Finish(mnavMemory* memory, const mnavBakeCells* cells, mnavResult result,
                         mnavFragmentList* list, mnavHeightfield* heightfield);

mnavResult mnavBuildHeightfield(mnavMemory* memory, const mnavBakeDef* def,
                                const mnavBakeCells* cells, const mnavTriangleMesh* meshes,
                                int32_t meshCount, int32_t tileX, int32_t tileZ,
                                mnavHeightfield* heightfield)
{
    *heightfield = (mnavHeightfield){0};
    if (!mnavMakeTileFrame(def, cells, tileX, tileZ, &heightfield->frame))
    {
        return mnav_errorRange;
    }
    mnavFragmentList list = {nullptr, 0, 0, def->limits.tileSpans};
    mnavResult result = Collect(memory, def, cells, meshes, meshCount, &heightfield->frame, &list);
    return Finish(memory, cells, result, &list, heightfield);
}

mnavResult mnavBuildHeightfield2D(mnavMemory* memory, const mnavBakeDef* def,
                                  const mnavBakeCells* cells, const mnavOutline* outlines,
                                  int32_t outlineCount, int32_t tileX, int32_t tileZ,
                                  mnavHeightfield* heightfield)
{
    *heightfield = (mnavHeightfield){0};
    if (!mnavMakeTileFrame(def, cells, tileX, tileZ, &heightfield->frame))
    {
        return mnav_errorRange;
    }
    mnavFragmentList list = {nullptr, 0, 0, def->limits.tileSpans};
    mnavResult result =
        mnavCollectOutlines(memory, def, &heightfield->frame, outlines, outlineCount, &list);
    return Finish(memory, cells, result, &list, heightfield);
}

// Sorts and merges the collected fragments into spans, when collecting
// succeeded, and releases them.
static mnavResult Finish(mnavMemory* memory, const mnavBakeCells* cells, mnavResult result,
                         mnavFragmentList* list, mnavHeightfield* heightfield)
{
    int32_t width = heightfield->frame.width;
    mnavFragment* sorted = nullptr;
    if (result == mnav_success)
    {
        result = mnavAllocate(memory, (size_t)width * (size_t)width + 1, sizeof(uint32_t),
                              alignof(uint32_t), (void**)&heightfield->columns);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(memory, (size_t)list->count, sizeof(mnavFragment),
                              alignof(mnavFragment), (void**)&sorted);
    }
    if (result == mnav_success)
    {
        Sort(list, width, heightfield->columns, sorted);
        mnavReleaseFragments(memory, list);
        result = Merge(memory, sorted, list->count, cells->agentStep, heightfield);
    }
    mnavRelease(memory, sorted, (size_t)list->count, sizeof(mnavFragment), alignof(mnavFragment));
    mnavReleaseFragments(memory, list);
    if (result != mnav_success)
    {
        mnavReleaseHeightfield(memory, heightfield);
    }
    return result;
}

void mnavReleaseHeightfield(mnavMemory* memory, mnavHeightfield* heightfield)
{
    size_t columnCount = (size_t)heightfield->frame.width * (size_t)heightfield->frame.width + 1;
    if (heightfield->columns != nullptr)
    {
        mnavRelease(memory, heightfield->columns, columnCount, sizeof(uint32_t), alignof(uint32_t));
    }
    mnavRelease(memory, heightfield->spans, (size_t)heightfield->spanCapacity, sizeof(mnavSpan),
                alignof(mnavSpan));
    heightfield->columns = nullptr;
    heightfield->spans = nullptr;
    heightfield->spanCount = 0;
    heightfield->spanCapacity = 0;
}
