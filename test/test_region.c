// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Partitioning open space into regions by layers (N15).

#include "allocator.h"
#include "compact.h"
#include "erode.h"
#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hash of the level's regions, the same on every platform.
#define LEVEL_REGION_HASH 0xa25f1ac8d6793faaull

typedef struct Built
{
    mnavMemory memory;
    mnavCompactField compact;
    mnavRegionMap map;
} Built;

static void Build(Field* field, int32_t minRegion, Built* built)
{
    built->memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    CHECK(mnavBuildCompactField(&built->memory, Pack(field), HEIGHT, STEP, &built->compact) ==
              mnav_success,
          "compact field");
    CHECK(mnavBuildRegions(&built->memory, &built->compact, 1, minRegion, &built->map) ==
              mnav_success,
          "regions");
}

static void Release(Built* built)
{
    mnavReleaseRegions(&built->memory, &built->map);
    mnavReleaseCompactField(&built->memory, &built->compact);
    CHECK(built->memory.used == 0, "everything released");
}

static uint32_t RegionAt(const Built* built, int32_t x, int32_t z, int32_t k)
{
    return built->map.ids[built->compact.columns[x + z * WIDTH] + (uint32_t)k];
}

// No region holds two spans of one column: a layer never overlaps itself.
// Border regions are one per side and may.
static bool NoColumnRepeatsARegion(const mnavCompactField* compact, const mnavRegionMap* map)
{
    int32_t columns = compact->frame.width * compact->frame.width;
    for (int32_t c = 0; c < columns; ++c)
    {
        for (uint32_t i = compact->columns[c]; i < compact->columns[c + 1]; ++i)
        {
            for (uint32_t k = i + 1; k < compact->columns[c + 1]; ++k)
            {
                bool region = map->ids[i] != 0 && (map->ids[i] & MNAV_BORDER_REGION) == 0;
                if (region && map->ids[i] == map->ids[k])
                {
                    return false;
                }
            }
        }
    }
    return true;
}

static void TestOpenFloorIsOneRegion(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    Built built;
    Build(&field, 0, &built);
    CHECK(built.map.count == 1, "one region");
    CHECK(RegionAt(&built, 3, 3, 0) == 1, "the inside is region 1");
    CHECK(RegionAt(&built, 0, 3, 0) == (MNAV_BORDER_REGION | 1), "the -X border");
    CHECK(RegionAt(&built, 6, 3, 0) == (MNAV_BORDER_REGION | 2), "the +X border");
    CHECK(RegionAt(&built, 3, 0, 0) == (MNAV_BORDER_REGION | 3), "the -Z border");
    CHECK(RegionAt(&built, 0, 6, 0) == (MNAV_BORDER_REGION | 4), "a +Z corner");
    Release(&built);
}

static void TestAreasSplitRegions(void)
{
    static Field field;
    field = (Field){0};
    Floor(&field, 100);
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 4; x < WIDTH; ++x)
        {
            field.input[x + z * WIDTH][0].area = 7;
        }
    }
    Built built;
    Build(&field, 0, &built);
    CHECK(built.map.count == 2, "two areas, two regions");
    CHECK(RegionAt(&built, 2, 3, 0) != RegionAt(&built, 4, 3, 0), "split at the area");
    Release(&built);
}

static void TestStackedFloorsJoinedByARampAreTwoLayers(void)
{
    // A lower floor at 100 over x 1 to 5, z 1 to 2; a ramp up the +X side
    // (106, 112) to an upper floor at 118 over x 1 to 4, z 1 to 4 that
    // stands above the lower one. Everything is linked, one area.
    static Field field;
    field = (Field){0};
    for (int32_t z = 1; z <= 2; ++z)
    {
        for (int32_t x = 1; x <= 5; ++x)
        {
            AddSpan(&field, x, z, 99, 100, 1);
        }
    }
    AddSpan(&field, 5, 3, 105, 106, 1);
    AddSpan(&field, 5, 4, 111, 112, 1);
    for (int32_t z = 1; z <= 4; ++z)
    {
        for (int32_t x = 1; x <= 4; ++x)
        {
            AddSpan(&field, x, z, 117, 118, 1);
        }
    }
    Built built;
    Build(&field, 0, &built);
    CHECK(RegionAt(&built, 2, 1, 0) != 0 && RegionAt(&built, 2, 1, 1) != 0, "both floors");
    CHECK(RegionAt(&built, 2, 1, 0) != RegionAt(&built, 2, 1, 1), "different layers");
    CHECK(NoColumnRepeatsARegion(&built.compact, &built.map), "no layer overlaps itself");
    Release(&built);
}

static void TestSmallIslandsAreDropped(void)
{
    // A 2 by 2 island in the middle, away from the border, and a strip
    // along the border; nothing links them.
    for (int32_t minRegion = 4; minRegion <= 5; ++minRegion)
    {
        static Field field;
        field = (Field){0};
        for (int32_t z = 2; z <= 3; ++z)
        {
            for (int32_t x = 3; x <= 4; ++x)
            {
                AddSpan(&field, x, z, 99, 100, 1);
            }
        }
        for (int32_t z = 0; z < WIDTH; ++z)
        {
            AddSpan(&field, 1, z, 99, 100, 1);
            AddSpan(&field, 0, z, 99, 100, 1);
        }
        Built built;
        Build(&field, minRegion, &built);
        bool kept = RegionAt(&built, 3, 2, 0) != 0;
        CHECK(kept == (minRegion == 4), "an island of 4 cells against the minimum");
        CHECK(RegionAt(&built, 1, 3, 0) != 0, "a strip on the border is kept");
        Release(&built);
    }
}

static void TestRowWithMoreSweepsThanCells(void)
{
    // Every inside column holds four floors, and no floor links to the
    // floor beside it, so a row starts 20 sweeps across 5 cells (Recast
    // issue 317 writes past its sweep array here).
    static Field field;
    field = (Field){0};
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            for (int32_t k = 0; k < 4; ++k)
            {
                int32_t top = 100 + 40 * k + 7 * x;
                AddSpan(&field, x, z, top - 1, top, 1);
            }
        }
    }
    Built built;
    Build(&field, 0, &built);
    CHECK(NoColumnRepeatsARegion(&built.compact, &built.map), "no layer overlaps itself");
    // Each floor of each inside column is a strip along Z: 4 by 5.
    CHECK(built.map.count == 20, "twenty strips");
    Release(&built);
}

// The region of the open span with the given floor in the column at
// world (x, z), or MNAV_BORDER_REGION when the column has none.
static uint32_t RegionAtWorld(const mnavCompactField* compact, const mnavRegionMap* map, float x,
                              float z, int32_t floor)
{
    int32_t cx = (int32_t)((x - compact->frame.minX) / compact->frame.cellSize);
    int32_t cz = (int32_t)((z - compact->frame.minZ) / compact->frame.cellSize);
    int32_t column = cx + cz * compact->frame.width;
    for (uint32_t i = compact->columns[column]; i < compact->columns[column + 1]; ++i)
    {
        if (compact->spans[i].floor == MNAV_HEIGHT_OFFSET + floor)
        {
            return map->ids[i];
        }
    }
    return MNAV_BORDER_REGION;
}

static uint64_t HashRegions(const mnavRegionMap* map)
{
    uint64_t hash = MNAV_HASH_INIT;
    hash = mnavHash64(hash, &map->count, (int32_t)sizeof(map->count));
    return mnavHash64(hash, map->ids, map->spanCount * (int32_t)sizeof(uint32_t));
}

static void TestLevelRegionsArePinned(void)
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
    mnavRegionMap map;
    CHECK(mnavBuildRegions(&memory, &compact, cells.border, cells.minRegion, &map) == mnav_success,
          "regions");
    CHECK(NoColumnRepeatsARegion(&compact, &map), "no layer overlaps itself");
    // The floor, the ramp and the platform are linked and never share a
    // column: one layer.
    uint32_t floor = RegionAtWorld(&compact, &map, 31.0f, 10.0f, 1);
    CHECK(floor != 0 && (floor & MNAV_BORDER_REGION) == 0, "the floor has a region");
    CHECK(RegionAtWorld(&compact, &map, 24.0f, 18.0f, 5) == floor, "the ramp joins it");
    CHECK(RegionAtWorld(&compact, &map, 28.0f, 28.0f, 9) == floor, "so does the platform");
    uint64_t hash = HashRegions(&map);
    printf("MNAV_REGION_HASH=%016llx regions=%u peak=%llu\n", (unsigned long long)hash, map.count,
           (unsigned long long)memory.peak);
    CHECK(hash == LEVEL_REGION_HASH, "the pinned hash");
    mnavReleaseRegions(&memory, &map);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestOpenFloorIsOneRegion();
    TestAreasSplitRegions();
    TestStackedFloorsJoinedByARampAreTwoLayers();
    TestSmallIslandsAreDropped();
    TestRowWithMoreSweepsThanCells();
    TestLevelRegionsArePinned();
    return s_failures == 0 ? 0 : 1;
}
