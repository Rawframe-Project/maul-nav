// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The open-space field, its links, and erosion (N14).

#include "allocator.h"
#include "compact.h"
#include "erode.h"
#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hashes of the soup's and the level's eroded open-space fields, the
// same on every platform (mnav-0001).
#define SOUP_OPEN_HASH  0x2af461f001197c4full
#define LEVEL_OPEN_HASH 0xd6c58bf04b95682bull

static mnavMemory s_memory;

static mnavCompactField Compact(Field* field)
{
    s_memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    mnavCompactField compact;
    mnavResult result = mnavBuildCompactField(&s_memory, Pack(field), HEIGHT, STEP, &compact);
    CHECK(result == mnav_success, "compact field built");
    return compact;
}

static uint32_t SpanAt(const mnavCompactField* compact, int32_t x, int32_t z, int32_t k)
{
    return compact->columns[x + z * WIDTH] + (uint32_t)k;
}

static void TestOpenSpansAreTheSpaceAboveWalkableSpans(void)
{
    static Field field;
    field = (Field){0};
    AddSpan(&field, 0, 0, 0, 10, 2);
    AddSpan(&field, 0, 0, 40, 41, mnav_areaNone);
    AddSpan(&field, 0, 0, 60, 70, 5);
    mnavCompactField compact = Compact(&field);
    CHECK(compact.spanCount == 2, "the unwalkable span has no open space");
    const mnavOpenSpan* low = &compact.spans[SpanAt(&compact, 0, 0, 0)];
    const mnavOpenSpan* high = &compact.spans[SpanAt(&compact, 0, 0, 1)];
    CHECK(low->floor == BASE + 10 && low->height == 30, "up to the next solid");
    CHECK(high->floor == BASE + 70 && high->height == 65535 - (BASE + 70), "open to the top");
    CHECK(compact.areas[SpanAt(&compact, 0, 0, 1)] == 5, "the area is kept");
    mnavReleaseCompactField(&s_memory, &compact);
    CHECK(s_memory.used == 0, "everything released");
}

static void TestLinksFollowTheStepAndHeadroom(void)
{
    static Field field;
    field = (Field){0};
    AddSpan(&field, 1, 1, 0, 100, 1);
    // +X: a step of exactly the step height; -X: one more.
    AddSpan(&field, 2, 1, 0, 100 + STEP, 1);
    AddSpan(&field, 0, 1, 0, 100 + STEP + 1, 1);
    // +Z: a floor at the same height under a roof too low to share.
    AddSpan(&field, 1, 2, 0, 100, 1);
    AddSpan(&field, 1, 2, 100 + HEIGHT - 1, 200, mnav_areaNone);
    // -Z: a low layer out of reach and two reachable ones; the lower of
    // the reachable two is the link.
    AddSpan(&field, 1, 0, 0, 50, 1);
    AddSpan(&field, 1, 0, 70, 98, 1);
    AddSpan(&field, 1, 0, 120, 121, 1);
    mnavCompactField compact = Compact(&field);
    const mnavOpenSpan* span = &compact.spans[SpanAt(&compact, 1, 1, 0)];
    CHECK(span->links[2] == 0, "a step of exactly the step height links");
    CHECK(span->links[0] == MNAV_NO_LINK, "one more does not");
    CHECK(span->links[1] == MNAV_NO_LINK, "too little shared space");
    CHECK(span->links[3] == 1, "the lowest reachable layer");
    mnavReleaseCompactField(&s_memory, &compact);
}

static int32_t CountWalkableOpen(const mnavCompactField* compact)
{
    int32_t count = 0;
    for (int32_t i = 0; i < compact->spanCount; ++i)
    {
        count += compact->areas[i] != mnav_areaNone;
    }
    return count;
}

static void TestErosionKeepsTheRadiusFromBoundaries(void)
{
    // A 7 by 7 floor: the field's edge is a boundary. Chamfer distances in
    // half cells are 0 on the edge, 2 one cell in, 4 two in, 6 at the
    // middle.
    const int32_t expected[5] = {49, 25, 9, 1, 0};
    for (int32_t radius = 0; radius <= 4; ++radius)
    {
        static Field field;
        field = (Field){0};
        Floor(&field, 100);
        mnavCompactField compact = Compact(&field);
        CHECK(mnavErode(&s_memory, &compact, radius) == mnav_success, "eroded");
        CHECK(CountWalkableOpen(&compact) == expected[radius], "rings eroded by radius");
        mnavReleaseCompactField(&s_memory, &compact);
        CHECK(s_memory.used == 0, "erosion released its distances");
    }
}

static void TestHoleErodesAround(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    field.input[3 + 3 * WIDTH][0].area = mnav_areaNone;
    mnavCompactField compact = Compact(&field);
    CHECK(mnavErode(&s_memory, &compact, 1) == mnav_success, "eroded");
    // The edge ring and the four straight neighbors of the hole go; the
    // diagonal neighbors are 3 half cells from it, past the threshold 2.
    CHECK(CountWalkableOpen(&compact) == 25 - 1 - 4, "the hole's straight neighbors");
    CHECK(compact.areas[SpanAt(&compact, 2, 2, 0)] == mnav_areaWalkable, "diagonal kept");
    mnavReleaseCompactField(&s_memory, &compact);
}

static uint64_t HashOpen(const mnavCompactField* compact)
{
    uint64_t hash = 0xCBF29CE484222325ull;
    for (int32_t i = 0; i < compact->spanCount; ++i)
    {
        const mnavOpenSpan* span = &compact->spans[i];
        uint16_t words[6] = {span->floor,    span->height,   span->links[0],
                             span->links[1], span->links[2], span->links[3]};
        for (int32_t w = 0; w < 6; ++w)
        {
            hash = (hash ^ (words[w] & 0xFFu)) * 0x100000001B3ull;
            hash = (hash ^ (uint32_t)(words[w] >> 8)) * 0x100000001B3ull;
        }
        hash = (hash ^ compact->areas[i]) * 0x100000001B3ull;
    }
    return hash;
}

// The area of the open span with the given floor in the column at world
// (x, z), or -1 when the column has none.
static int32_t AreaAt(const mnavCompactField* compact, float x, float z, int32_t floor)
{
    int32_t cx = (int32_t)((x - compact->frame.minX) / compact->frame.cellSize);
    int32_t cz = (int32_t)((z - compact->frame.minZ) / compact->frame.cellSize);
    int32_t column = cx + cz * compact->frame.width;
    for (uint32_t i = compact->columns[column]; i < compact->columns[column + 1]; ++i)
    {
        if (compact->spans[i].floor == MNAV_HEIGHT_OFFSET + floor)
        {
            return compact->areas[i];
        }
    }
    return -1;
}

// Runs the bake's stages so far over one mesh and returns the hash of
// the eroded open-space field; checks the level's landmarks when level.
static uint64_t Pipeline(const mnavTriangleMesh* mesh, const char* name, bool level)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    CHECK(mnavValidateTriangleMesh(&def, mesh).result == mnav_success, "mesh");
    mnavMemory memory = mnavMakeMemory(def.allocator, def.limits.memoryBytes);
    mnavHeightfield heightfield;
    CHECK(mnavBuildHeightfield(&memory, &def, &cells, mesh, 1, 0, 0, &heightfield) == mnav_success,
          "rasterized");
    mnavFilterWalkable(&heightfield, cells.agentHeight, cells.agentStep);
    mnavCompactField compact;
    CHECK(mnavBuildCompactField(&memory, &heightfield, cells.agentHeight, cells.agentStep,
                                &compact) == mnav_success,
          "compacted");
    CHECK(mnavErode(&memory, &compact, cells.agentRadius) == mnav_success, "eroded");
    if (level)
    {
        // A surface's solid is at least one cell tall, so the platform's top,
        // 1 m or 8 cell heights up, has its floor at 9; the ramp's cell at
        // z = 18 m spans 4 to 4.5 cell heights, so its floor is 5.
        CHECK(AreaAt(&compact, 28.0f, 28.0f, 9) == mnav_areaWalkable, "platform walkable");
        CHECK(AreaAt(&compact, 24.0f, 18.0f, 5) == mnav_areaWalkable, "ramp walkable");
        // The platform's edge at x = 30 m has no box near it: 1 m in is
        // walkable, 0.25 m in is within the 0.5 m radius of the drop.
        CHECK(AreaAt(&compact, 29.0f, 25.0f, 9) == mnav_areaWalkable, "platform near its edge");
        CHECK(AreaAt(&compact, 29.75f, 25.0f, 9) == mnav_areaNone, "eroded at the drop");
    }
    uint64_t hash = HashOpen(&compact);
    printf("MNAV_OPEN_HASH_%s=%016llx spans=%d walkable=%d\n", name, (unsigned long long)hash,
           compact.spanCount, CountWalkableOpen(&compact));
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
    return hash;
}

static void TestPipelineHashesArePinned(void)
{
    static mnavVec3 vertices[SOUP_TRIANGLES * 3];
    static mnavAreaType areas[SOUP_TRIANGLES];
    static int32_t indices[SOUP_TRIANGLES * 3];
    MakeSoup(vertices, areas);
    for (int32_t i = 0; i < SOUP_TRIANGLES * 3; ++i)
    {
        indices[i] = i;
    }
    mnavTriangleMesh soup = {vertices, SOUP_TRIANGLES * 3, indices, SOUP_TRIANGLES, areas};
    CHECK(Pipeline(&soup, "SOUP", false) == SOUP_OPEN_HASH, "the soup's pinned hash");
    static mnavVec3 levelVertices[LEVEL_VERTICES];
    static int32_t levelIndices[LEVEL_TRIANGLES * 3];
    MakeLevel(levelVertices, levelIndices);
    mnavTriangleMesh level = {levelVertices, LEVEL_VERTICES, levelIndices, LEVEL_TRIANGLES,
                              nullptr};
    CHECK(Pipeline(&level, "LEVEL", true) == LEVEL_OPEN_HASH, "the level's pinned hash");
}

int main(void)
{
    TestOpenSpansAreTheSpaceAboveWalkableSpans();
    TestLinksFollowTheStepAndHeadroom();
    TestErosionKeepsTheRadiusFromBoundaries();
    TestHoleErodesAround();
    TestPipelineHashesArePinned();
    return s_failures == 0 ? 0 : 1;
}
