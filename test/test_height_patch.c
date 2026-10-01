// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floor heights under one polygon.

#include "allocator.h"
#include "compact.h"
#include "hand_field.h"
#include "height_patch.h"
#include "polymesh.h"
#include "region.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// A deck high above the floor: the agent fits between them.
enum
{
    DECK = 4 + HEIGHT + 4
};

// A compact field from a hand field, with region ids by floor height:
// floors below DECK are region 1 left of x = 4 and 2 from it, but none
// at x = 0; decks are 3.
typedef struct Setup
{
    mnavMemory memory;
    mnavCompactField field;
    uint32_t ids[WIDTH * WIDTH * MAX_SPANS];
    mnavRegionMap regions;
} Setup;

static void Prepare(Setup* setup, Field* hand)
{
    setup->memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    CHECK(mnavBuildCompactField(&setup->memory, Pack(hand), HEIGHT, STEP, &setup->field) ==
              mnav_success,
          "compacted");
    for (int32_t c = 0; c < WIDTH * WIDTH; ++c)
    {
        for (uint32_t i = setup->field.columns[c]; i < setup->field.columns[c + 1]; ++i)
        {
            bool deck = setup->field.spans[i].floor >= BASE + DECK;
            setup->ids[i] = deck ? 3u : (c % WIDTH < 4 ? 1u : 2u);
            // The floor's first column is eroded: no region, the id that
            // also marks mixed polygons.
            setup->ids[i] = !deck && c % WIDTH == 0 ? 0u : setup->ids[i];
        }
    }
    setup->regions = (mnavRegionMap){setup->ids, setup->field.spanCount, 3};
}

// A mesh of one square polygon from (x0, z0) to (x1, z1) at height y.
typedef struct Square
{
    mnavMeshVertex vertices[4];
    mnavPolygon polygon;
    mnavPolyMesh mesh;
} Square;

static void MakeSquare(Square* square, int32_t x0, int32_t z0, int32_t x1, int32_t z1, int32_t y,
                       uint32_t region)
{
    uint16_t h = (uint16_t)(BASE + y);
    square->vertices[0] = (mnavMeshVertex){(uint16_t)x0, h, (uint16_t)z0};
    square->vertices[1] = (mnavMeshVertex){(uint16_t)x0, h, (uint16_t)z1};
    square->vertices[2] = (mnavMeshVertex){(uint16_t)x1, h, (uint16_t)z1};
    square->vertices[3] = (mnavMeshVertex){(uint16_t)x1, h, (uint16_t)z0};
    square->polygon = (mnavPolygon){{0, 1, 2, 3, 0xFFFF, 0xFFFF}, {0}, {0}, 4, 1, region};
    square->mesh = (mnavPolyMesh){0};
    square->mesh.vertices = square->vertices;
    square->mesh.vertexCount = 4;
    square->mesh.polygons = &square->polygon;
    square->mesh.polygonCount = 1;
    square->mesh.tileCells = WIDTH;
}

static uint16_t At(const mnavHeightPatch* patch, int32_t x, int32_t z)
{
    return patch->heights[(x - patch->minX) + (z - patch->minZ) * patch->width];
}

static void TestRegionHeightsAndTheFloodBeyond(void)
{
    // Floor 4 left of x = 4 (region 1), 6 from it (region 2).
    static Field hand;
    hand = (Field){0};
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            int32_t top = x < 4 ? 4 : 6;
            AddSpan(&hand, x, z, top - 1, top, mnav_areaWalkable);
        }
    }
    static Setup setup;
    Prepare(&setup, &hand);
    Square square;
    MakeSquare(&square, 1, 1, 4, 4, 4, 1);
    mnavHeightPatch patch = {0};
    CHECK(mnavFillHeightPatch(&setup.memory, &setup.field, &setup.regions, 0, &square.mesh, 0,
                              &patch) == mnav_success,
          "filled");
    CHECK(patch.minX == 0 && patch.minZ == 0 && patch.width == 5 && patch.depth == 5,
          "the polygon's cells and one more each side");
    CHECK(At(&patch, 2, 2) == BASE + 4 && At(&patch, 0, 0) == BASE + 4, "the region's floor");
    CHECK(At(&patch, 4, 2) == BASE + 6 && At(&patch, 4, 4) == BASE + 6, "flooded beyond it");
    mnavReleaseHeightPatch(&setup.memory, &patch);
    mnavReleaseCompactField(&setup.memory, &setup.field);
    CHECK(setup.memory.used == 0, "everything released");
}

static void TestDeckOverFloor(void)
{
    // A floor at 4 everywhere and a deck over it on x = 1 to 3.
    static Field hand;
    hand = (Field){0};
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            AddSpan(&hand, x, z, 3, 4, mnav_areaWalkable);
            if (x >= 1 && x <= 3)
            {
                AddSpan(&hand, x, z, DECK - 1, DECK, mnav_areaWalkable);
            }
        }
    }
    static Setup setup;
    Prepare(&setup, &hand);
    Square square;
    mnavHeightPatch patch = {0};
    // The floor's polygon reads the floor under the deck.
    MakeSquare(&square, 1, 1, 3, 3, 4, 1);
    CHECK(mnavFillHeightPatch(&setup.memory, &setup.field, &setup.regions, 0, &square.mesh, 0,
                              &patch) == mnav_success,
          "floor");
    CHECK(At(&patch, 2, 2) == BASE + 4, "the floor, not the deck");
    // A polygon of mixed regions at the deck's height floods from the
    // deck's spans at its vertices.
    MakeSquare(&square, 1, 1, 3, 3, DECK, MNAV_MIXED_REGION);
    CHECK(mnavFillHeightPatch(&setup.memory, &setup.field, &setup.regions, 0, &square.mesh, 0,
                              &patch) == mnav_success,
          "deck");
    CHECK(At(&patch, 2, 2) == BASE + DECK && At(&patch, 1, 1) == BASE + DECK,
          "the deck, not the floor");
    // A vertex halfway between floor and deck takes the lower span.
    MakeSquare(&square, 1, 1, 3, 3, (4 + DECK) / 2, MNAV_MIXED_REGION);
    CHECK(mnavFillHeightPatch(&setup.memory, &setup.field, &setup.regions, 0, &square.mesh, 0,
                              &patch) == mnav_success,
          "halfway");
    CHECK(At(&patch, 2, 2) == BASE + 4, "the first span on ties");
    // A region with no span under the polygon floods from its vertices too.
    MakeSquare(&square, 1, 1, 3, 3, 4, 7);
    CHECK(mnavFillHeightPatch(&setup.memory, &setup.field, &setup.regions, 0, &square.mesh, 0,
                              &patch) == mnav_success,
          "absent region");
    CHECK(At(&patch, 2, 2) == BASE + 4, "the floor nearest the vertices");
    mnavReleaseHeightPatch(&setup.memory, &patch);
    mnavReleaseCompactField(&setup.memory, &setup.field);
    CHECK(setup.memory.used == 0, "everything released");
}

static void TestLookupSearchesRings(void)
{
    mnavHeightPatch patch = {0};
    uint16_t heights[25];
    for (int32_t i = 0; i < 25; ++i)
    {
        heights[i] = MNAV_UNSET_HEIGHT;
    }
    patch.heights = heights;
    patch.width = 5;
    patch.depth = 5;
    heights[2 + 2 * 5] = 100;
    uint16_t h = 0;
    CHECK(mnavPatchHeight(&patch, 2, 2, 0, 0, &h) && h == 100, "a set cell");
    CHECK(mnavPatchHeight(&patch, -3, 9, 0, 0, &h) == false && h == MNAV_UNSET_HEIGHT,
          "clamped to an unset corner");
    heights[2 + 2 * 5] = MNAV_UNSET_HEIGHT;
    heights[1 + 1 * 5] = 90;
    heights[3 + 3 * 5] = 110;
    heights[4 + 4 * 5] = 101;
    CHECK(mnavPatchHeight(&patch, 2, 2, 104, 2, &h) && h == 110, "the nearest in ring 1");
    CHECK(mnavPatchHeight(&patch, 2, 2, 100, 2, &h) && h == 90, "the first on ties");
    CHECK(mnavPatchHeight(&patch, 2, 2, 100, 0, &h) == false, "nothing within radius 0");
    heights[1 + 1 * 5] = MNAV_UNSET_HEIGHT;
    heights[3 + 3 * 5] = MNAV_UNSET_HEIGHT;
    CHECK(mnavPatchHeight(&patch, 2, 2, 0, 1, &h) == false, "nothing in ring 1");
    CHECK(mnavPatchHeight(&patch, 2, 2, 0, 2, &h) && h == 101, "ring 2");
}

int main(void)
{
    TestRegionHeightsAndTheFloodBeyond();
    TestDeckOverFloor();
    TestLookupSearchesRings();
    return s_failures == 0 ? 0 : 1;
}
