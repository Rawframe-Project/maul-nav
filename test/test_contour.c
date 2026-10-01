// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tracing and simplifying region contours (N16).

#include "allocator.h"
#include "compact.h"
#include "contour.h"
#include "erode.h"
#include "filter.h"
#include "hand_field.h"
#include "heightfield.h"
#include "region.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// The hash of the level's contours, the same on every platform.
#define LEVEL_CONTOUR_HASH 0xa766dce23dae2b9aull

typedef struct Built
{
    mnavMemory memory;
    mnavCompactField compact;
    mnavRegionMap regions;
    mnavContourSet set;
} Built;

static void Build(Field* field, float edgeError, int32_t edgeLength, Built* built)
{
    built->memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    CHECK(mnavBuildCompactField(&built->memory, Pack(field), HEIGHT, STEP, &built->compact) ==
              mnav_success,
          "compact field");
    CHECK(mnavBuildRegions(&built->memory, &built->compact, 1, 0, &built->regions) == mnav_success,
          "regions");
    CHECK(mnavBuildContours(&built->memory, &built->compact, &built->regions, 1, edgeError,
                            edgeLength, &built->set) == mnav_success,
          "contours");
}

static void Release(Built* built)
{
    mnavReleaseContours(&built->memory, &built->set);
    mnavReleaseRegions(&built->memory, &built->regions);
    mnavReleaseCompactField(&built->memory, &built->compact);
    CHECK(built->memory.used == 0, "everything released");
}

// A floor at 100 over the inside cells x and z from 1 to 5.
static void InsideFloor(Field* field)
{
    *field = (Field){0};
    for (int32_t z = 1; z <= 5; ++z)
    {
        for (int32_t x = 1; x <= 5; ++x)
        {
            AddSpan(field, x, z, 99, 100, 1);
        }
    }
}

static const mnavContourVertex* Vertex(const Built* built, int32_t contour, int32_t k)
{
    const mnavContour* c = &built->set.contours[contour];
    return &built->set.vertices[c->first + k % c->count];
}

static bool HasVertex(const Built* built, int32_t contour, int32_t x, int32_t z)
{
    for (int32_t k = 0; k < built->set.contours[contour].count; ++k)
    {
        if (Vertex(built, contour, k)->x == x && Vertex(built, contour, k)->z == z)
        {
            return true;
        }
    }
    return false;
}

static void TestSquareIsFourCorners(void)
{
    static Field field;
    InsideFloor(&field);
    Built built;
    Build(&field, 1.0f, 0, &built);
    CHECK(built.set.count == 1, "one contour");
    CHECK(built.set.contours[0].count == 4, "four corners");
    CHECK(!built.set.contours[0].hole, "an outline");
    CHECK(HasVertex(&built, 0, 0, 0) && HasVertex(&built, 0, 5, 0) && HasVertex(&built, 0, 5, 5) &&
              HasVertex(&built, 0, 0, 5),
          "the corners of the inside, without the border");
    CHECK(Vertex(&built, 0, 0)->y == BASE + 100, "at the floor's height");
    CHECK(Vertex(&built, 0, 0)->neighbor == 0, "walls all round");
    Release(&built);
}

static void TestHoleIsItsOwnContour(void)
{
    static Field field;
    InsideFloor(&field);
    field.counts[3 + 3 * WIDTH] = 0;
    // Within an error of 1 cell a one-cell hole would simplify to its
    // diagonal; half a cell keeps it.
    Built built;
    Build(&field, 0.5f, 0, &built);
    CHECK(built.set.count == 2, "an outline and a hole");
    int32_t holes = built.set.contours[0].hole + built.set.contours[1].hole;
    CHECK(holes == 1, "one hole");
    int32_t hole = built.set.contours[0].hole ? 0 : 1;
    CHECK(built.set.contours[hole].count == 4, "the hole is one cell");
    CHECK(HasVertex(&built, hole, 2, 2) && HasVertex(&built, hole, 3, 3), "around cell 2, 2");
    Release(&built);
}

static void TestRegionsAgreeOnTheirSharedEdge(void)
{
    static Field field;
    InsideFloor(&field);
    for (int32_t z = 1; z <= 5; ++z)
    {
        for (int32_t x = 4; x <= 5; ++x)
        {
            field.input[x + z * WIDTH][0].area = 7;
        }
    }
    Built built;
    Build(&field, 1.0f, 0, &built);
    CHECK(built.set.count == 2, "two regions, two contours");
    // The shared edge runs along x = 3 from z = 0 to 5 in both.
    for (int32_t c = 0; c < 2; ++c)
    {
        CHECK(HasVertex(&built, c, 3, 0) && HasVertex(&built, c, 3, 5), "the shared edge's ends");
        uint32_t other = built.set.contours[1 - c].region;
        bool across = false;
        for (int32_t k = 0; k < built.set.contours[c].count; ++k)
        {
            const mnavContourVertex* v = Vertex(&built, c, k);
            across = across || (v->neighbor == other && (v->flags & MNAV_EDGE_AREA_BORDER) != 0);
        }
        CHECK(across, "an edge names the other region and the area border");
    }
    Release(&built);
}

static void TestErrorDecidesASmallNotch(void)
{
    // A one-cell notch in the -Z wall: kept at an error of 0.5 cells,
    // simplified away at 2.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        static Field field;
        InsideFloor(&field);
        field.counts[3 + 1 * WIDTH] = 0;
        Built built;
        Build(&field, pass == 0 ? 0.5f : 2.0f, 0, &built);
        CHECK(built.set.count == 1, "one contour");
        CHECK(built.set.contours[0].count == (pass == 0 ? 8 : 4), "notch kept or not");
        Release(&built);
    }
}

static void TestLongWallsAreSplit(void)
{
    static Field field;
    InsideFloor(&field);
    Built built;
    Build(&field, 1.0f, 2, &built);
    const mnavContour* contour = &built.set.contours[0];
    bool short_ = true;
    for (int32_t k = 0; k < contour->count; ++k)
    {
        const mnavContourVertex* a = Vertex(&built, 0, k);
        const mnavContourVertex* b = Vertex(&built, 0, k + 1);
        int32_t dx = b->x - a->x;
        int32_t dz = b->z - a->z;
        short_ = short_ && dx * dx + dz * dz <= 4;
    }
    CHECK(short_, "no wall longer than 2 cells");
    CHECK(contour->count == 12, "each 5-cell wall in three");
    Release(&built);
}

static uint64_t CellKey(uint32_t area, uint32_t region)
{
    return ((uint64_t)area << 32) | region;
}

static void TestTileBorderCornerRule(void)
{
    uint64_t side = CellKey(1, MNAV_BORDER_REGION | 3);
    uint64_t other = CellKey(1, MNAV_BORDER_REGION | 1);
    // Two cells of one border side, then two inside cells of one area,
    // in any rotation.
    uint64_t keys[4] = {side, side, CellKey(1, 5), CellKey(1, 6)};
    CHECK(mnavIsTileBorderCorner(keys), "two border cells and two inside cells");
    uint64_t rotated[4] = {CellKey(1, 6), side, side, CellKey(1, 5)};
    CHECK(mnavIsTileBorderCorner(rotated), "rotated");
    uint64_t areas[4] = {side, side, CellKey(1, 5), CellKey(2, 6)};
    CHECK(!mnavIsTileBorderCorner(areas), "inside cells of two areas");
    uint64_t corner[4] = {side, other, CellKey(1, 5), CellKey(1, 5)};
    CHECK(!mnavIsTileBorderCorner(corner), "two border sides: the tile's corner");
    uint64_t missing[4] = {side, side, CellKey(1, 5), 0};
    CHECK(!mnavIsTileBorderCorner(missing), "a missing cell");
}

static void TestLevelContoursArePinned(void)
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
    int32_t holes = 0;
    bool inside = true;
    for (int32_t c = 0; c < set.count; ++c)
    {
        holes += set.contours[c].hole;
        for (int32_t k = 0; k < set.contours[c].count; ++k)
        {
            const mnavContourVertex* v = &set.vertices[set.contours[c].first + k];
            inside =
                inside && v->x >= 0 && v->x <= def.tileCells && v->z >= 0 && v->z <= def.tileCells;
        }
    }
    CHECK(inside, "every vertex within the tile");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t c = 0; c < set.count; ++c)
    {
        const mnavContour* contour = &set.contours[c];
        int32_t header[4] = {contour->count, (int32_t)contour->region, contour->area,
                             contour->hole};
        hash = mnavHash64(hash, header, (int32_t)sizeof(header));
        for (int32_t k = 0; k < contour->count; ++k)
        {
            const mnavContourVertex* v = &set.vertices[contour->first + k];
            int32_t fields[5] = {v->x, v->y, v->z, (int32_t)v->neighbor, v->flags};
            hash = mnavHash64(hash, fields, (int32_t)sizeof(fields));
        }
    }
    printf("MNAV_CONTOUR_HASH=%016llx contours=%d holes=%d vertices=%d\n", (unsigned long long)hash,
           set.count, holes, set.vertexCount);
    CHECK(hash == LEVEL_CONTOUR_HASH, "the pinned hash");
    mnavReleaseContours(&memory, &set);
    mnavReleaseRegions(&memory, &regions);
    mnavReleaseCompactField(&memory, &compact);
    mnavReleaseHeightfield(&memory, &heightfield);
    CHECK(memory.used == 0, "everything released");
}

int main(void)
{
    TestSquareIsFourCorners();
    TestHoleIsItsOwnContour();
    TestRegionsAgreeOnTheirSharedEdge();
    TestErrorDecidesASmallNotch();
    TestLongWallsAreSplit();
    TestTileBorderCornerRule();
    TestLevelContoursArePinned();
    return s_failures == 0 ? 0 : 1;
}
