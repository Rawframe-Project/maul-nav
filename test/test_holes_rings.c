// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Merging holes into hand-built contour sets, for what no bake reaches:
// a hole without an outline is dropped and counted. test_holes.c bakes
// the merges bakes reach.

#include "allocator.h"
#include "contour.h"
#include "holes.h"
#include "test_harness.h"

#include <stdint.h>

// A hole from (5, 5) to (7, 7) of region 1, with no outline.
static const mnavContourVertex s_vertices[4] = {
    {5, 0, 5, 0, 0},
    {7, 0, 5, 0, 0},
    {7, 0, 7, 0, 0},
    {5, 0, 7, 0, 0},
};

static void TestHoleWithoutOutlineIsDropped(void)
{
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = {0};
    CHECK(mnavReserve(&memory, (void**)&set.vertices, &set.vertexCapacity, 0, 4,
                      sizeof(mnavContourVertex), alignof(mnavContourVertex)) == mnav_success &&
              mnavReserve(&memory, (void**)&set.contours, &set.capacity, 0, 1, sizeof(mnavContour),
                          alignof(mnavContour)) == mnav_success,
          "room");
    for (int32_t i = 0; i < 4; ++i)
    {
        set.vertices[i] = s_vertices[i];
    }
    set.vertexCount = 4;
    set.contours[set.count++] = (mnavContour){0, 4, 1, 1, true};
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 0 && set.droppedHoles == 1, "dropped and counted");
    mnavReleaseContours(&memory, &set);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestHoleWithoutOutlineIsDropped();
    return s_failures == 0 ? 0 : 1;
}
