// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Rasterizing triangles into a tile's heightfield (N12).

#include "allocator.h"
#include "heightfield.h"
#include "raster.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct Bake
{
    mnavBakeDef def;
    mnavBakeCells cells;
    mnavMemory memory;
    mnavHeightfield field;
} Bake;

static Bake MakeBake(void)
{
    Bake bake = {0};
    bake.def = mnavDefaultBakeDef();
    return bake;
}

static mnavResult Build(Bake* bake, const mnavTriangleMesh* meshes, int32_t count, int32_t tileX,
                        int32_t tileZ)
{
    if (mnavValidateBakeDef(&bake->def, &bake->cells).result != mnav_success)
    {
        return mnav_errorInvalid;
    }
    bake->memory = mnavMakeMemory(bake->def.allocator, bake->def.limits.memoryBytes);
    return mnavBuildHeightfield(&bake->memory, &bake->def, &bake->cells, meshes, count, tileX,
                                tileZ, &bake->field);
}

static void Release(Bake* bake)
{
    mnavReleaseHeightfield(&bake->memory, &bake->field);
}

static int32_t ColumnSpans(const mnavHeightfield* field, int32_t x, int32_t z)
{
    int32_t column = x + z * field->frame.width;
    return (int32_t)(field->columns[column + 1] - field->columns[column]);
}

static const mnavSpan* ColumnSpan(const mnavHeightfield* field, int32_t x, int32_t z, int32_t k)
{
    return &field->spans[field->columns[x + z * field->frame.width] + (uint32_t)k];
}

// A 64-bit FNV-1a over the heightfield's columns and spans, field by field.
static uint64_t HashField(const mnavHeightfield* field)
{
    uint64_t hash = 0xCBF29CE484222325ull;
    int32_t columnCount = field->frame.width * field->frame.width;
    for (int32_t c = 0; c <= columnCount; ++c)
    {
        uint32_t value = field->columns[c];
        for (int32_t b = 0; b < 4; ++b)
        {
            hash = (hash ^ ((value >> (8 * b)) & 0xFFu)) * 0x100000001B3ull;
        }
    }
    for (int32_t i = 0; i < field->spanCount; ++i)
    {
        const mnavSpan* span = &field->spans[i];
        uint8_t bytes[5] = {(uint8_t)span->bottom, (uint8_t)(span->bottom >> 8), (uint8_t)span->top,
                            (uint8_t)(span->top >> 8), span->area};
        for (int32_t b = 0; b < 5; ++b)
        {
            hash = (hash ^ bytes[b]) * 0x100000001B3ull;
        }
    }
    return hash;
}

// A floor from (0, 0) to (10, 10) at height 0, counter-clockwise from above.
static const mnavVec3 s_floor[4] = {{0, 0, 0}, {0, 0, 10}, {10, 0, 10}, {10, 0, 0}};
static const int32_t s_floorIndices[6] = {0, 1, 2, 0, 2, 3};

static void TestFloorFillsItsCells(void)
{
    Bake bake = MakeBake();
    mnavTriangleMesh mesh = {s_floor, 4, s_floorIndices, 2, nullptr};
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_success, "built");
    // The tile starts 5 border cells before the origin, so the floor's
    // 0.25 m cells are 5 to 44 on both axes.
    CHECK(bake.field.frame.width == 138, "128 cells and two borders of 5");
    int32_t inside = 0;
    int32_t outside = 0;
    for (int32_t z = 0; z < bake.field.frame.width; ++z)
    {
        for (int32_t x = 0; x < bake.field.frame.width; ++x)
        {
            bool covered = x >= 5 && x <= 44 && z >= 5 && z <= 44;
            int32_t count = ColumnSpans(&bake.field, x, z);
            if (covered && count == 1)
            {
                const mnavSpan* span = ColumnSpan(&bake.field, x, z, 0);
                inside += span->bottom == MNAV_HEIGHT_OFFSET &&
                          span->top == MNAV_HEIGHT_OFFSET + 1 && span->area == mnav_areaWalkable;
            }
            outside += !covered && count != 0;
        }
    }
    CHECK(inside == 40 * 40, "every covered cell has one walkable span");
    CHECK(outside == 0, "no span outside the floor");
    CHECK(bake.field.spanCount == 1600, "1600 spans");
    Release(&bake);
    CHECK(bake.memory.used == 0, "everything released");
}

static void TestSlopesAndFacing(void)
{
    mnavVec3 corners[3] = {{0, 0, 0}, {0, 0, 1}, {1, 0, 0}};
    CHECK(mnavTriangleArea(corners, 5, 0.7f) == 5, "flat and up is walkable");
    mnavVec3 flipped[3] = {corners[0], corners[2], corners[1]};
    CHECK(mnavTriangleArea(flipped, 5, 0.7f) == mnav_areaNone, "facing down is solid only");
    // A 50 degree ramp against a 45 degree limit, and a 40 degree one.
    mnavVec3 steep[3] = {{0, 0, 0}, {0, 1.19175f, 1}, {1, 0, 0}};
    mnavVec3 gentle[3] = {{0, 0, 0}, {0, 0.83910f, 1}, {1, 0, 0}};
    CHECK(mnavTriangleArea(steep, 5, 0.70710677f) == mnav_areaNone, "50 degrees is too steep");
    CHECK(mnavTriangleArea(gentle, 5, 0.70710677f) == 5, "40 degrees is walkable");
    mnavVec3 line[3] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    CHECK(mnavTriangleArea(line, 5, 0.7f) == -1, "a degenerate triangle has no area");
    CHECK(mnavTriangleArea(corners, mnav_areaNone, 0.7f) == mnav_areaNone, "unwalkable stays");
}

static void TestStackedFloorsKeepSeparateSpans(void)
{
    mnavVec3 vertices[8];
    for (int32_t i = 0; i < 4; ++i)
    {
        vertices[i] = s_floor[i];
        vertices[i + 4] = (mnavVec3){s_floor[i].x, 3.0f, s_floor[i].z};
    }
    const int32_t indices[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    Bake bake = MakeBake();
    mnavTriangleMesh mesh = {vertices, 8, indices, 4, nullptr};
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_success, "built");
    CHECK(ColumnSpans(&bake.field, 10, 10) == 2, "two floors, two spans");
    CHECK(ColumnSpan(&bake.field, 10, 10, 0)->bottom == MNAV_HEIGHT_OFFSET, "lower first");
    CHECK(ColumnSpan(&bake.field, 10, 10, 1)->bottom == MNAV_HEIGHT_OFFSET + 24, "3 m is 24");
    Release(&bake);
}

static void TestMergedAreaDoesNotDependOnOrder(void)
{
    // Research 03's case: a floor of area 2 whose top is 3 cell heights
    // below a floor of area 1, with a step of 2: the merged span's top is
    // area 1's, and area 2's top lies past the step, so area 1 wins in
    // either order. With a step of 6 both tops count and 2 wins.
    mnavVec3 vertices[6] = {{0, 0, 0},      {0, 0, 10},      {10, 0, 10},
                            {0, 0.375f, 0}, {0, 0.375f, 10}, {10, 0.375f, 10}};
    const int32_t lowFirst[6] = {0, 1, 2, 3, 4, 5};
    const int32_t highFirst[6] = {3, 4, 5, 0, 1, 2};
    const mnavAreaType areasLowFirst[2] = {2, 1};
    const mnavAreaType areasHighFirst[2] = {1, 2};
    // A wall from 0 up to the upper floor at x = 5 m joins the two into
    // one solid.
    mnavVec3 wall[3] = {{4, 0, 5}, {5, 0.375f, 5}, {6, 0, 5}};
    const int32_t wallIndices[3] = {0, 1, 2};
    const mnavAreaType wallArea[1] = {mnav_areaNone};
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        uint64_t hashes[2] = {0, 0};
        mnavAreaType areas[2] = {0, 0};
        for (int32_t order = 0; order < 2; ++order)
        {
            Bake bake = MakeBake();
            bake.def.agent.stepHeight = pass == 0 ? 0.25f : 0.75f;
            mnavTriangleMesh meshes[2] = {
                {vertices, 6, order == 0 ? lowFirst : highFirst, 2,
                 order == 0 ? areasLowFirst : areasHighFirst},
                {wall, 3, wallIndices, 1, wallArea},
            };
            CHECK(Build(&bake, meshes, 2, 0, 0) == mnav_success, "built");
            hashes[order] = HashField(&bake.field);
            // Cell (25, 25) lies under the wall at z = 5 m, x = 5 m.
            CHECK(ColumnSpans(&bake.field, 25, 25) == 1, "one merged span");
            areas[order] = ColumnSpan(&bake.field, 25, 25, 0)->area;
            Release(&bake);
        }
        CHECK(hashes[0] == hashes[1], "same heightfield in either order");
        CHECK(areas[0] == (pass == 0 ? 1 : 2), "area by the step rule");
    }
}

static void TestBoundaryEdgeLeavesNoSliver(void)
{
    // A floor from x = 0 to 10 m ends exactly on a cell boundary; column
    // 45 must stay empty, and a wall on that boundary goes to column 45.
    Bake bake = MakeBake();
    mnavTriangleMesh mesh = {s_floor, 4, s_floorIndices, 2, nullptr};
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_success, "built");
    CHECK(ColumnSpans(&bake.field, 45, 20) == 0, "no sliver past the edge");
    Release(&bake);
    mnavVec3 wall[3] = {{10, 0, 4}, {10, 2, 4.1f}, {10, 0, 4.2f}};
    const int32_t indices[3] = {0, 1, 2};
    mnavTriangleMesh wallMesh = {wall, 3, indices, 1, nullptr};
    bake = MakeBake();
    CHECK(Build(&bake, &wallMesh, 1, 0, 0) == mnav_success, "built");
    CHECK(ColumnSpans(&bake.field, 45, 21) == 1, "the wall is in column 45");
    CHECK(ColumnSpans(&bake.field, 44, 21) == 0, "and not in column 44");
    CHECK(ColumnSpan(&bake.field, 45, 21, 0)->top == MNAV_HEIGHT_OFFSET + 16, "2 m tall");
    Release(&bake);
}

static void TestNeighborTileSeesOnlyItsBorder(void)
{
    // Tile (1, 0) starts at x = 32 m less a 1.25 m border: the floor
    // reaches only tile (0, 0); a floor at 30 to 40 m reaches both.
    Bake bake = MakeBake();
    mnavTriangleMesh mesh = {s_floor, 4, s_floorIndices, 2, nullptr};
    CHECK(Build(&bake, &mesh, 1, 1, 0) == mnav_success, "built");
    CHECK(bake.field.spanCount == 0, "nothing in the neighbor");
    Release(&bake);
    mnavVec3 shifted[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        shifted[i] = (mnavVec3){s_floor[i].x + 30.0f, 0.0f, s_floor[i].z};
    }
    mnavTriangleMesh across = {shifted, 4, s_floorIndices, 2, nullptr};
    bake = MakeBake();
    CHECK(Build(&bake, &across, 1, 1, 0) == mnav_success, "built");
    // From 30.75 m (cell 0 of tile 1) to 40 m: 37 cells by 40.
    CHECK(bake.field.spanCount == 37 * 40, "the neighbor holds its share");
    Release(&bake);
}

static void TestLimitsAreTyped(void)
{
    mnavTriangleMesh mesh = {s_floor, 4, s_floorIndices, 2, nullptr};
    Bake bake = MakeBake();
    // 1600 cells, and the 40 on the diagonal get a fragment from each
    // triangle.
    bake.def.limits.tileSpans = 1639;
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_errorLimit, "fragments past the limit");
    CHECK(bake.memory.used == 0, "nothing held after a refusal");
    bake = MakeBake();
    bake.def.limits.tileSpans = 1640;
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_success, "fragments at the limit");
    Release(&bake);
    bake = MakeBake();
    bake.def.limits.tileTriangles = 1;
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_errorLimit, "triangles past the limit");
    bake = MakeBake();
    bake.def.limits.memoryBytes = 4096;
    CHECK(Build(&bake, &mesh, 1, 0, 0) == mnav_errorLimit, "memory past the limit");
    CHECK(bake.memory.used == 0, "nothing held after the memory limit");
    bake = MakeBake();
    CHECK(Build(&bake, &mesh, 1, 1 << 20, 0) == mnav_errorRange, "tile past the extent");
}

static void* FailingAlloc(size_t size, size_t alignment, void* context)
{
    (void)alignment;
    int32_t* left = context;
    if (*left == 0)
    {
        return nullptr;
    }
    *left -= 1;
    return malloc(size);
}

static void ForwardFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    free(memory);
}

static void TestAllocatorFailureIsCapacity(void)
{
    mnavTriangleMesh mesh = {s_floor, 4, s_floorIndices, 2, nullptr};
    for (int32_t allowed = 0; allowed < 4; ++allowed)
    {
        int32_t left = allowed;
        Bake bake = MakeBake();
        bake.def.allocator = (mnavAllocator){FailingAlloc, ForwardFree, &left};
        mnavResult result = Build(&bake, &mesh, 1, 0, 0);
        CHECK(result == mnav_errorCapacity, "a failed allocation is capacity");
        CHECK(bake.memory.used == 0, "nothing counted after a failure");
    }
}

// The hash of the soup below, the same on every platform (mnav-0001).
#define SOUP_HASH 0xe955dec170776113ull

static void TestSoupDependsOnlyOnTheTriangleSet(void)
{
    static mnavVec3 vertices[SOUP_TRIANGLES * 3];
    static mnavAreaType areas[SOUP_TRIANGLES];
    static int32_t forward[SOUP_TRIANGLES * 3];
    static int32_t backward[SOUP_TRIANGLES * 3];
    static mnavAreaType backwardAreas[SOUP_TRIANGLES];
    MakeSoup(vertices, areas);
    for (int32_t t = 0; t < SOUP_TRIANGLES; ++t)
    {
        int32_t r = SOUP_TRIANGLES - 1 - t;
        for (int32_t k = 0; k < 3; ++k)
        {
            forward[t * 3 + k] = t * 3 + k;
            // Reversed order, and each triangle's corners rotated.
            backward[r * 3 + k] = t * 3 + (k + 1) % 3;
        }
        backwardAreas[r] = areas[t];
    }
    Bake bake = MakeBake();
    mnavTriangleMesh one = {vertices, SOUP_TRIANGLES * 3, forward, SOUP_TRIANGLES, areas};
    CHECK(Build(&bake, &one, 1, 0, 0) == mnav_success, "built forward");
    uint64_t hash = HashField(&bake.field);
    int32_t spans = bake.field.spanCount;
    uint64_t peak = bake.memory.peak;
    Release(&bake);
    // The same set, reversed and split across two meshes.
    int32_t half = SOUP_TRIANGLES / 2;
    mnavTriangleMesh two[2] = {
        {vertices, SOUP_TRIANGLES * 3, backward, half, backwardAreas},
        {vertices, SOUP_TRIANGLES * 3, backward + half * 3, SOUP_TRIANGLES - half,
         backwardAreas + half},
    };
    bake = MakeBake();
    CHECK(Build(&bake, two, 2, 0, 0) == mnav_success, "built backward");
    CHECK(HashField(&bake.field) == hash, "the same heightfield from the same set");
    Release(&bake);
    printf("MNAV_RASTER_HASH=%016llx spans=%d peak=%llu\n", (unsigned long long)hash, spans,
           (unsigned long long)peak);
    CHECK(hash == SOUP_HASH, "the pinned hash");
}

int main(void)
{
    TestFloorFillsItsCells();
    TestSlopesAndFacing();
    TestStackedFloorsKeepSeparateSpans();
    TestMergedAreaDoesNotDependOnOrder();
    TestBoundaryEdgeLeavesNoSliver();
    TestNeighborTileSeesOnlyItsBorder();
    TestLimitsAreTyped();
    TestAllocatorFailureIsCapacity();
    TestSoupDependsOnlyOnTheTriangleSet();
    return s_failures == 0 ? 0 : 1;
}
