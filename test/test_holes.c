// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Merging hole contours into their outlines (N17).

#include "allocator.h"
#include "compact.h"
#include "contour.h"
#include "erode.h"
#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "holes.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hash of the level's merged contours, the same on every platform.
#define LEVEL_MERGED_HASH 0x8d5043009d7438ccull

static int64_t TwiceArea(const mnavContourSet* set, int32_t contour)
{
    const mnavContourVertex* v = set->vertices + set->contours[contour].first;
    int32_t count = set->contours[contour].count;
    int64_t area = 0;
    for (int32_t i = 0, j = count - 1; i < count; j = i++)
    {
        area += (int64_t)v[i].x * v[j].z - (int64_t)v[j].x * v[i].z;
    }
    return area;
}

// Builds the contours of a hand field's regions and merges their holes.
static void Build(Field* field, mnavMemory* memory, mnavCompactField* compact,
                  mnavRegionMap* regions, mnavContourSet* set)
{
    *memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    CHECK(mnavBuildCompactField(memory, Pack(field), HEIGHT, STEP, compact) == mnav_success,
          "compact field");
    CHECK(mnavBuildRegions(memory, compact, 1, 0, regions) == mnav_success, "regions");
    CHECK(mnavBuildContours(memory, compact, regions, 1, 0.5f, 0, set) == mnav_success, "contours");
    CHECK(mnavMergeHoles(memory, set, regions->count) == mnav_success, "merged");
}

static void TestHolesMergeIntoOneContour(void)
{
    // The inside floor of 25 cells less a hole of 4, then less two holes
    // of 1.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        static Field field;
        field = (Field){0};
        for (int32_t z = 1; z <= 5; ++z)
        {
            for (int32_t x = 1; x <= 5; ++x)
            {
                bool hole = pass == 0 ? (x >= 3 && x <= 4 && z >= 3 && z <= 4)
                                      : ((x == 2 && z == 2) || (x == 4 && z == 4));
                if (!hole)
                {
                    AddSpan(&field, x, z, 99, 100, 1);
                }
            }
        }
        mnavMemory memory;
        mnavCompactField compact;
        mnavRegionMap regions;
        mnavContourSet set;
        Build(&field, &memory, &compact, &regions, &set);
        CHECK(set.count == 1, "one contour left");
        CHECK(!set.contours[0].hole && set.droppedHoles == 0, "no hole left or dropped");
        int32_t holes = pass == 0 ? 1 : 2;
        CHECK(set.contours[0].count == 4 + 6 * holes, "four corners and a bridged ring per hole");
        CHECK(TwiceArea(&set, 0) == 2 * (pass == 0 ? 21 : 23), "the area less the holes");
        mnavReleaseContours(&memory, &set);
        mnavReleaseRegions(&memory, &regions);
        mnavReleaseCompactField(&memory, &compact);
        CHECK(memory.used == 0, "everything released");
    }
}

// A contour set built by hand: an outline from (0, 0) to (10, 10) and a
// hole from (5, 5) to (7, 7), both of region 1.
static mnavContourVertex s_vertices[8] = {
    {0, 0, 0, 0, 0}, {0, 0, 10, 0, 0}, {10, 0, 10, 0, 0}, {10, 0, 0, 0, 0},
    {5, 0, 5, 0, 0}, {7, 0, 5, 0, 0},  {7, 0, 7, 0, 0},   {5, 0, 7, 0, 0},
};

static mnavContourSet HandSet(mnavMemory* memory, bool withOutline)
{
    mnavContourSet set = {0};
    CHECK(mnavReserve(memory, (void**)&set.vertices, &set.vertexCapacity, 0, 8,
                      sizeof(mnavContourVertex), alignof(mnavContourVertex)) == mnav_success,
          "vertices");
    CHECK(mnavReserve(memory, (void**)&set.contours, &set.capacity, 0, 2, sizeof(mnavContour),
                      alignof(mnavContour)) == mnav_success,
          "contours");
    for (int32_t i = 0; i < 8; ++i)
    {
        set.vertices[i] = s_vertices[i];
    }
    set.vertexCount = 8;
    if (withOutline)
    {
        set.contours[set.count++] = (mnavContour){0, 4, 1, 1, false};
    }
    set.contours[set.count++] = (mnavContour){4, 4, 1, 1, true};
    return set;
}

static void TestTiedBridgesTakeTheLowestOutlineVertex(void)
{
    // The hole's lowest-left vertex (5, 5) is 50 squared cells from all
    // four outline corners; the bridge goes to the first, (0, 0).
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = HandSet(&memory, true);
    CHECK(TwiceArea(&set, 0) > 0 && TwiceArea(&set, 1) < 0, "an outline and a hole");
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 1 && set.contours[0].count == 10, "one contour of 10");
    const mnavContourVertex* v = set.vertices + set.contours[0].first;
    CHECK(v[0].x == 0 && v[0].z == 0 && v[4].x == 0 && v[4].z == 0, "bridged at (0, 0)");
    CHECK(v[5].x == 5 && v[5].z == 5 && v[9].x == 5 && v[9].z == 5, "to (5, 5)");
    CHECK(TwiceArea(&set, 0) == 200 - 8, "the outline less the hole");
    mnavReleaseContours(&memory, &set);
    CHECK(memory.used == 0, "everything released");
}

static void TestHoleWithoutOutlineIsDropped(void)
{
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = HandSet(&memory, false);
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 0 && set.droppedHoles == 1, "dropped and counted");
    mnavReleaseContours(&memory, &set);
}

static void TestLevelMergesEveryHole(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    mnavTriangleMesh mesh = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    mnavMemory memory = mnavMakeMemory(def.allocator, def.limits.memoryBytes);
    mnavHeightfield heightfield;
    CHECK(mnavBuildHeightfield(&memory, &def, &cells, &mesh, 1, 0, 0, &heightfield) == mnav_success,
          "rasterized");
    mnavFilterWalkable(&heightfield, cells.agentHeight, cells.agentStep);
    mnavCompactField compact;
    CHECK(mnavBuildCompactField(&memory, &heightfield, cells.agentHeight, cells.agentStep,
                                &compact) == mnav_success,
          "compacted");
    CHECK(mnavErode(&memory, &compact, cells.agentRadius) == mnav_success, "eroded");
    mnavRegionMap regions;
    CHECK(mnavBuildRegions(&memory, &compact, cells.border, cells.minRegion, &regions) ==
              mnav_success,
          "regions");
    mnavContourSet set;
    CHECK(mnavBuildContours(&memory, &compact, &regions, cells.border, cells.edgeError,
                            cells.edgeLength, &set) == mnav_success,
          "contours");
    CHECK(mnavMergeHoles(&memory, &set, regions.count) == mnav_success, "merged");
    bool noHoles = set.droppedHoles == 0;
    for (int32_t c = 0; c < set.count; ++c)
    {
        noHoles = noHoles && !set.contours[c].hole && TwiceArea(&set, c) > 0;
    }
    CHECK(noHoles, "every hole merged into an outline");
    // Field by field: the struct's padding is not part of the result.
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < set.vertexCount; ++i)
    {
        const mnavContourVertex* v = &set.vertices[i];
        int32_t fields[5] = {v->x, v->y, v->z, (int32_t)v->neighbor, v->flags};
        hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
    }
    printf("MNAV_MERGED_HASH=%016llx contours=%d vertices=%d\n", (unsigned long long)hash,
           set.count, set.vertexCount);
    CHECK(hash == LEVEL_MERGED_HASH, "the pinned hash");
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestHolesMergeIntoOneContour();
    TestTiedBridgesTakeTheLowestOutlineVertex();
    TestHoleWithoutOutlineIsDropped();
    TestLevelMergesEveryHole();
    return s_failures == 0 ? 0 : 1;
}
