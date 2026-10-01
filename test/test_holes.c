// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Merging hole contours into their outlines.

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

// A set from an outline square of side size and the given holes, each a
// square of side 2 by its lowest-left corner, all of region 1.
static mnavContourSet SquareWithHoles(mnavMemory* memory, int32_t size, const int32_t* corners,
                                      int32_t holeCount)
{
    mnavContourSet set = {0};
    int32_t vertexCount = 4 + 4 * holeCount;
    CHECK(mnavReserve(memory, (void**)&set.vertices, &set.vertexCapacity, 0, vertexCount,
                      sizeof(mnavContourVertex), alignof(mnavContourVertex)) == mnav_success,
          "vertices");
    CHECK(mnavReserve(memory, (void**)&set.contours, &set.capacity, 0, 1 + holeCount,
                      sizeof(mnavContour), alignof(mnavContour)) == mnav_success,
          "contours");
    const int32_t outline[8] = {0, 0, 0, size, size, size, size, 0};
    for (int32_t k = 0; k < 4; ++k)
    {
        set.vertices[k] = (mnavContourVertex){outline[2 * k], 0, outline[2 * k + 1], 0, 0};
    }
    set.contours[set.count++] = (mnavContour){0, 4, 1, 1, false};
    for (int32_t h = 0; h < holeCount; ++h)
    {
        int32_t x = corners[2 * h];
        int32_t z = corners[2 * h + 1];
        // Wound the other way from the outline.
        const int32_t ring[8] = {x, z, x + 2, z, x + 2, z + 2, x, z + 2};
        for (int32_t k = 0; k < 4; ++k)
        {
            set.vertices[4 + 4 * h + k] =
                (mnavContourVertex){ring[2 * k], 0, ring[2 * k + 1], 0, 0};
        }
        set.contours[set.count++] = (mnavContour){4 + 4 * h, 4, 1, 1, true};
    }
    set.vertexCount = vertexCount;
    return set;
}

// Pairs of edges of a contour that cross at a point inside both.
static int32_t ProperCrossings(const mnavContourSet* set, int32_t contour)
{
    const mnavContourVertex* v = set->vertices + set->contours[contour].first;
    int32_t n = set->contours[contour].count;
    int32_t crossings = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        for (int32_t j = i + 1; j < n; ++j)
        {
            const mnavContourVertex* a = &v[i];
            const mnavContourVertex* b = &v[(i + 1) % n];
            const mnavContourVertex* c = &v[j];
            const mnavContourVertex* d = &v[(j + 1) % n];
            int64_t abc =
                (int64_t)(b->x - a->x) * (c->z - a->z) - (int64_t)(c->x - a->x) * (b->z - a->z);
            int64_t abd =
                (int64_t)(b->x - a->x) * (d->z - a->z) - (int64_t)(d->x - a->x) * (b->z - a->z);
            int64_t cda =
                (int64_t)(d->x - c->x) * (a->z - c->z) - (int64_t)(a->x - c->x) * (d->z - c->z);
            int64_t cdb =
                (int64_t)(d->x - c->x) * (b->z - c->z) - (int64_t)(b->x - c->x) * (d->z - c->z);
            crossings += ((abc < 0 && abd > 0) || (abc > 0 && abd < 0)) &&
                         ((cda < 0 && cdb > 0) || (cda > 0 && cdb < 0));
        }
    }
    return crossings;
}

static void TestBridgeAvoidsAWaitingHole(void)
{
    // The first hole's nearest corner, (20, 0), lies behind the second
    // hole; the bridge must go elsewhere and nothing may cross.
    const int32_t corners[4] = {14, 4, 16, 1};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = SquareWithHoles(&memory, 20, corners, 2);
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 1 && set.droppedHoles == 0, "both holes bridged");
    CHECK(ProperCrossings(&set, 0) == 0, "no edges cross");
    CHECK(TwiceArea(&set, 0) == 800 - 16, "the square less both holes");
    mnavReleaseContours(&memory, &set);
}

static void TestManyHolesNeverCross(void)
{
    // A grid of 25 holes: later bridges must not cross earlier ones,
    // which are part of the outline by then.
    int32_t corners[50];
    for (int32_t i = 0; i < 25; ++i)
    {
        corners[2 * i] = 2 + 5 * (i % 5) + (i / 5) % 2;
        corners[2 * i + 1] = 2 + 5 * (i / 5) + (i % 5) % 3;
    }
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = SquareWithHoles(&memory, 30, corners, 25);
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 1 && set.droppedHoles == 0, "every hole bridged");
    CHECK(ProperCrossings(&set, 0) == 0, "no edges cross");
    CHECK(TwiceArea(&set, 0) == 1800 - 25 * 8, "the square less every hole");
    mnavReleaseContours(&memory, &set);
}

static void TestBridgeNeverCrossesAnEarlierBridge(void)
{
    // Found by a search: the last hole's nearest candidate lies across a
    // bridge made earlier, which is part of the outline by then.
    const int32_t corners[6] = {16, 5, 15, 1, 12, 9};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = SquareWithHoles(&memory, 20, corners, 3);
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 1 && set.droppedHoles == 0, "every hole bridged");
    CHECK(ProperCrossings(&set, 0) == 0, "no edges cross");
    mnavReleaseContours(&memory, &set);
}

static void TestSecondBridgeTakesTheCopyFacingIt(void)
{
    // Both holes are nearest to corner (0, 0), 65 squared cells away,
    // nearer than to each other: after the first bridge (0, 0) appears
    // twice, and the second hole, below that bridge, must leave from the
    // copy between the -Z wall and the bridge.
    const int32_t corners[4] = {1, 8, 8, 1};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavContourSet set = SquareWithHoles(&memory, 40, corners, 2);
    CHECK(mnavMergeHoles(&memory, &set, 1) == mnav_success, "merged");
    CHECK(set.count == 1 && set.droppedHoles == 0, "both holes bridged");
    const mnavContourVertex* v = set.vertices + set.contours[0].first;
    int32_t n = set.contours[0].count;
    bool facing = false;
    for (int32_t k = 0; k < n; ++k)
    {
        const mnavContourVertex* prev = &v[(k + n - 1) % n];
        const mnavContourVertex* next = &v[(k + 1) % n];
        if (v[k].x == 0 && v[k].z == 0 && next->x == 8 && next->z == 1)
        {
            facing = prev->x == 40 && prev->z == 0;
        }
    }
    CHECK(facing, "the bridge to (8, 1) leaves after the -Z wall");
    mnavReleaseContours(&memory, &set);
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
    TestBridgeAvoidsAWaitingHole();
    TestSecondBridgeTakesTheCopyFacingIt();
    TestManyHolesNeverCross();
    TestBridgeNeverCrossesAnEarlierBridge();
    TestLevelMergesEveryHole();
    return s_failures == 0 ? 0 : 1;
}
