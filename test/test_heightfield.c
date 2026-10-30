// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Rasterizing triangles into a tile's heightfield, read from the baked
// tile: the cells a floor fills, slopes and facing, stacked floors, the
// area of merged spans, cell and tile edges, limits, refused memory, and
// one tile from one triangle set in any order.

#include "counting_allocator.h"
#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

enum
{
    TILE_ROOM = 1 << 20,
    DRAW_ROOM = 1 << 18,
    NO_POLYGON = 0xFF
};

static uint8_t s_tile[TILE_ROOM];
static mnavNavmesh* s_navmesh;

// The default def, but an agent of no radius, regions of any size and no
// edge error: the polygons cover exactly the cells the heightfield leaves
// walkable, less ledges.
static mnavBakeDef Exact(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    return def;
}

// Bakes tile (tileX, tileZ) of the meshes with a baker of the def and, on
// success, loads it in a fresh navmesh; returns the first failing call's
// result.
static mnavResult Bake(const mnavBakeDef* def, const mnavTriangleMesh* meshes, int32_t count,
                       int32_t tileX, int32_t tileZ, mnavBakeReport* report)
{
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    mnavBaker* baker = nullptr;
    mnavResult result = mnavCreateBaker(def, &baker).result;
    if (result != mnav_success)
    {
        return result;
    }
    const mnavBakeInput input = {meshes, count, nullptr, 0, nullptr, 0, nullptr};
    result = mnavBaker_BakeTile(baker, &input, tileX, tileZ, report);
    if (result == mnav_success)
    {
        // The navmesh takes the default allocator, which never refuses.
        mnavNavmeshDef navmeshDef = mnavDefaultNavmeshDef();
        navmeshDef.bake = *def;
        size_t size = 0;
        CHECK(mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success &&
                  mnavCreateNavmesh(&navmeshDef, &s_navmesh).result == mnav_success &&
                  mnavNavmesh_StageTile(s_navmesh, s_tile, size).result == mnav_success &&
                  mnavNavmesh_Commit(s_navmesh) == mnav_success,
              "loaded");
    }
    mnavDestroyBaker(baker);
    return result;
}

// The polygon over the center of cell (x, z), counted from the origin,
// within a meter of height y.
static mnavNearest Over(int32_t x, int32_t z, double y)
{
    mnavPos3 center = {(x + 0.5) * 0.25, y, (z + 0.5) * 0.25};
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(s_navmesh, center, (mnavVec3){0.01f, 1.0f, 0.01f}, nullptr, &n) ==
              mnav_success,
          "nearest");
    if (!n.over)
    {
        n.polygon = (mnavPolygonId){0, 0, 0};
    }
    return n;
}

// Its area, or NO_POLYGON.
static mnavAreaType At(int32_t x, int32_t z, double y)
{
    mnavNearest n = Over(x, z, y);
    mnavAreaType area = NO_POLYGON;
    if (n.polygon.slot != 0)
    {
        CHECK(mnavPolygon_GetArea(s_navmesh, n.polygon, &area) == mnav_success, "area");
    }
    return area;
}

// The cells from (x0, z0) to (x1, z1) with a polygon over them at height
// y.
static int32_t Walkable(int32_t x0, int32_t z0, int32_t x1, int32_t z1, double y)
{
    int32_t n = 0;
    for (int32_t z = z0; z <= z1; ++z)
    {
        for (int32_t x = x0; x <= x1; ++x)
        {
            n += At(x, z, y) != NO_POLYGON ? 1 : 0;
        }
    }
    return n;
}

static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawTriangles[3 * DRAW_ROOM];
static uint32_t s_drawLines[2 * DRAW_ROOM];

// The hash of the loaded tile as mnavNavmesh_Debug draws it: its polygons,
// detail triangles and edges, but not the fingerprint of its input.
static uint64_t DrawHash(void)
{
    mnavDebugBuffer buffer = {{0, 0, 0}, s_drawVertices, DRAW_ROOM, 0, s_drawTriangles, DRAW_ROOM,
                              0,         s_drawLines,    DRAW_ROOM, 0};
    CHECK(mnavNavmesh_Debug(s_navmesh, &(mnavTileRange){0, 0, 1, 1}, &buffer) == mnav_success,
          "drawn");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < buffer.vertexCount; ++i)
    {
        const mnavDebugVertex* v = &s_drawVertices[i];
        uint32_t words[5];
        memcpy(&words[0], &v->x, 4);
        memcpy(&words[1], &v->y, 4);
        memcpy(&words[2], &v->z, 4);
        words[3] = (uint32_t)v->kind;
        words[4] = v->value;
        hash = mnavHash64(hash, words, (int32_t)sizeof(words));
    }
    hash = mnavHash64(hash, s_drawTriangles, buffer.triangleCount * 3 * (int32_t)sizeof(uint32_t));
    return mnavHash64(hash, s_drawLines, buffer.lineCount * 2 * (int32_t)sizeof(uint32_t));
}

// A floor from (0, 0) to (10, 10) at height 0, counter-clockwise from above.
static const mnavVec3 s_floor[4] = {{0, 0, 0}, {0, 0, 10}, {10, 0, 10}, {10, 0, 0}};
static const int32_t s_floorIndices[6] = {0, 1, 2, 0, 2, 3};
static const mnavTriangleMesh s_floorMesh = {s_floor, 4, s_floorIndices, 2, nullptr};

static void TestFloorFillsItsCells(void)
{
    // The floor's 0.25 m cells are 0 to 39 on both axes; its outer ring
    // is a ledge, so cells 1 to 38 are walkable, at the top of the span
    // from height 0 to 1, 0.125 m.
    mnavBakeDef def = Exact();
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(Walkable(1, 1, 38, 38, 0.0) == 38 * 38, "every inner cell");
    CHECK(Walkable(0, 0, 127, 127, 0.0) == 38 * 38, "nothing more");
    CHECK(Over(20, 20, 0.0).point.y == 0.125, "at the span's top");
    // A floor between two cell heights tops at the higher.
    const float heights[2] = {0.05f, 0.2f};
    for (int32_t k = 0; k < 2; ++k)
    {
        mnavVec3 raised[4];
        for (int32_t i = 0; i < 4; ++i)
        {
            raised[i] = (mnavVec3){s_floor[i].x, heights[k], s_floor[i].z};
        }
        const mnavTriangleMesh mesh = {raised, 4, s_floorIndices, 2, nullptr};
        CHECK(Bake(&def, &mesh, 1, 0, 0, nullptr) == mnav_success &&
                  Over(20, 20, (double)heights[k]).point.y == (k == 0 ? 0.125 : 0.25),
              "a floor between cell heights");
    }
}

static void TestSlopesAndFacing(void)
{
    mnavBakeDef def = Exact();
    mnavBakeReport report;
    // Facing down, the floor is solid only.
    const int32_t down[6] = {0, 2, 1, 0, 3, 2};
    const mnavTriangleMesh flipped = {s_floor, 4, down, 2, nullptr};
    CHECK(Bake(&def, &flipped, 1, 0, 0, &report) == mnav_success && report.polygons == 0,
          "facing down is solid only");
    // Given no area, it stays unwalkable.
    const mnavAreaType none[2] = {mnav_areaNone, mnav_areaNone};
    const mnavTriangleMesh unwalkable = {s_floor, 4, s_floorIndices, 2, none};
    CHECK(Bake(&def, &unwalkable, 1, 0, 0, &report) == mnav_success && report.polygons == 0,
          "unwalkable stays");
    // Ramps 4 m long rising along z: 50 degrees from x = 2 to 6 m against
    // a 45 degree limit, and 40 degrees from x = 12 to 16 m.
    const mnavVec3 ramps[8] = {{2, 0, 2},  {2, 4.767f, 6},   {6, 4.767f, 6},   {6, 0, 2},
                               {12, 0, 2}, {12, 3.3564f, 6}, {16, 3.3564f, 6}, {16, 0, 2}};
    const int32_t rampIndices[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    const mnavTriangleMesh rampMesh = {ramps, 8, rampIndices, 4, nullptr};
    CHECK(Bake(&def, &rampMesh, 1, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(At(16, 16, 2.38) == NO_POLYGON, "50 degrees is too steep");
    CHECK(At(56, 16, 1.68) == mnav_areaWalkable, "40 degrees is walkable");
    // A degenerate triangle 1 m over the floor, along x = 5 m: it has no
    // area, so it leaves the floor under it as it was.
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, &report) == mnav_success, "baked");
    uint64_t plain = DrawHash();
    uint64_t fingerprint = report.fingerprint;
    const mnavVec3 withLine[7] = {s_floor[0], s_floor[1], s_floor[2], s_floor[3],
                                  {5, 1, 2},  {5, 1, 5},  {5, 1, 8}};
    const int32_t lineIndices[9] = {0, 1, 2, 0, 2, 3, 4, 5, 6};
    const mnavTriangleMesh lineMesh = {withLine, 7, lineIndices, 3, nullptr};
    CHECK(Bake(&def, &lineMesh, 1, 0, 0, &report) == mnav_success && DrawHash() == plain &&
              report.fingerprint == fingerprint,
          "a degenerate triangle has no area");
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
    const mnavTriangleMesh mesh = {vertices, 8, indices, 4, nullptr};
    mnavBakeDef def = Exact();
    CHECK(Bake(&def, &mesh, 1, 0, 0, nullptr) == mnav_success, "baked");
    mnavNearest low = Over(10, 10, 0.0);
    mnavNearest high = Over(10, 10, 3.0);
    CHECK(low.polygon.slot != 0 && high.polygon.slot != 0 &&
              memcmp(&low.polygon, &high.polygon, sizeof(low.polygon)) != 0,
          "two floors, two polygons");
    // 3 m is 24 cell heights: the upper span runs from 24 to 25.
    CHECK(low.point.y == 0.125 && high.point.y == 3.125, "each at its own height");
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
    // one solid in the cells beside x = 5 m, z = 5 m.
    mnavVec3 wall[3] = {{4, 0, 5}, {5, 0.375f, 5}, {6, 0, 5}};
    const int32_t wallIndices[3] = {0, 1, 2};
    const mnavAreaType wallArea[1] = {mnav_areaNone};
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        uint64_t hashes[2] = {0, 0};
        mnavAreaType areas[2] = {0, 0};
        for (int32_t order = 0; order < 2; ++order)
        {
            mnavBakeDef def = Exact();
            def.agent.stepHeight = pass == 0 ? 0.25f : 0.75f;
            mnavTriangleMesh meshes[2] = {
                {vertices, 6, order == 0 ? lowFirst : highFirst, 2,
                 order == 0 ? areasLowFirst : areasHighFirst},
                {wall, 3, wallIndices, 1, wallArea},
            };
            CHECK(Bake(&def, meshes, 2, 0, 0, nullptr) == mnav_success, "baked");
            hashes[order] = DrawHash();
            // Cell (19, 20) lies under the wall's top, at x = 4.75 m.
            areas[order] = At(19, 20, 0.375);
        }
        CHECK(hashes[0] == hashes[1], "the same tile in either order");
        CHECK(areas[0] == (pass == 0 ? 1 : 2) && areas[1] == areas[0], "area by the step rule");
    }
}

static void TestBoundaryEdgeLeavesNoSliver(void)
{
    // A floor from x = 0 to 10 m ends exactly on a cell boundary: cell 40
    // stays empty, so cell 39 is the floor's ledge.
    mnavBakeDef def = Exact();
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(At(39, 20, 0.0) == NO_POLYGON && At(38, 20, 0.0) == mnav_areaWalkable,
          "no sliver past the edge");
    // A wall 2 m tall on that boundary, over a floor 20 m wide, stands in
    // cell 40, not 39.
    const mnavVec3 vertices[7] = {{0, 0, 0},  {0, 0, 10},    {20, 0, 10},  {20, 0, 0},
                                  {10, 0, 4}, {10, 2, 4.1f}, {10, 0, 4.2f}};
    const int32_t indices[9] = {0, 1, 2, 0, 2, 3, 4, 5, 6};
    const mnavTriangleMesh mesh = {vertices, 7, indices, 3, nullptr};
    CHECK(Bake(&def, &mesh, 1, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(At(40, 16, 0.0) == NO_POLYGON, "the wall is in cell 40");
    CHECK(At(39, 16, 0.0) == mnav_areaWalkable, "and not in cell 39");
}

// The fingerprint of tile (0, 0) of the floor with a wall standing on the
// plane x = at, from z = 1 to 2 m.
static uint64_t WallFingerprint(float at)
{
    const mnavVec3 vertices[7] = {s_floor[0], s_floor[1],    s_floor[2], s_floor[3],
                                  {at, 0, 1}, {at, 2, 1.5f}, {at, 0, 2}};
    const int32_t indices[9] = {0, 1, 2, 0, 2, 3, 4, 5, 6};
    const mnavTriangleMesh mesh = {vertices, 7, indices, 3, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeReport report;
    CHECK(Bake(&def, &mesh, 1, 0, 0, &report) == mnav_success, "baked");
    return report.fingerprint;
}

static void TestTriangleOnTheTileEdges(void)
{
    // The fingerprint holds the triangles that reach the tile and its
    // border of 5 cells: x = -1.25 to 33.25 m, exact in floats. A wall
    // exactly on the near edge reaches it; one exactly on the far edge
    // does not; one a float below the far edge does.
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeReport report;
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, &report) == mnav_success, "baked");
    uint64_t floor = report.fingerprint;
    CHECK(WallFingerprint(-1.25f) != floor, "a wall on the near edge is in");
    CHECK(WallFingerprint(nextafterf(-1.25f, -2.0f)) == floor, "a float before it is out");
    CHECK(WallFingerprint(-1.5f) == floor, "a cell before it is out");
    CHECK(WallFingerprint(33.25f) == floor, "a wall on the far edge is out");
    CHECK(WallFingerprint(nextafterf(33.25f, 0.0f)) != floor, "a float before it is in");
    CHECK(WallFingerprint(33.75f) == floor && WallFingerprint(500.0f) == floor, "past it is out");
    // A triangle across the tile, its corners all outside it.
    const mnavVec3 across[3] = {{-50, 0, -50}, {-50, 0, 200}, {200, 0, -50}};
    const int32_t indices[3] = {0, 1, 2};
    const mnavTriangleMesh mesh = {across, 3, indices, 1, nullptr};
    def = Exact();
    CHECK(Bake(&def, &mesh, 1, 0, 0, &report) == mnav_success &&
              Walkable(0, 0, 127, 127, 0.0) == 128 * 128,
          "a triangle across the tile is in");
}

static void TestNeighborTileSeesOnlyItsBorder(void)
{
    // Tile (1, 0) starts at x = 32 m less its border: the floor reaches
    // only tile (0, 0); a floor at 30 to 40 m reaches both.
    mnavBakeDef def = Exact();
    mnavBakeReport report;
    mnavBakeReport empty;
    const mnavTriangleMesh nothing = {s_floor, 4, s_floorIndices, 0, nullptr};
    CHECK(Bake(&def, &nothing, 1, 1, 0, &empty) == mnav_success, "baked");
    CHECK(Bake(&def, &s_floorMesh, 1, 1, 0, &report) == mnav_success && report.polygons == 0 &&
              report.fingerprint == empty.fingerprint,
          "nothing in the neighbor");
    mnavVec3 shifted[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        shifted[i] = (mnavVec3){s_floor[i].x + 30.0f, 0.0f, s_floor[i].z};
    }
    const mnavTriangleMesh across = {shifted, 4, s_floorIndices, 2, nullptr};
    CHECK(Bake(&def, &across, 1, 1, 0, nullptr) == mnav_success, "baked");
    // Tile 1's cells 128 to 159 hold the floor up to 40 m, its last cell
    // a ledge; the floor goes on past the tile's near side.
    CHECK(Walkable(128, 1, 158, 38, 0.0) == 31 * 38 && Walkable(128, 0, 255, 127, 0.0) == 31 * 38,
          "the neighbor holds its share");
}

static void TestLimitsAreTyped(void)
{
    // 1600 cells, the 40 on the diagonal a fragment from each triangle,
    // and the border's none.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tileSpans = 1639;
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_errorCapacity,
          "fragments past the limit");
    def.limits.tileSpans = 1640;
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_success, "fragments at the limit");
    def = mnavDefaultBakeDef();
    def.limits.tileTriangles = 1;
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_errorCapacity,
          "triangles past the limit");
    def = mnavDefaultBakeDef();
    CHECK(Bake(&def, &s_floorMesh, 1, 1 << 20, 0, nullptr) == mnav_errorLimit,
          "tile past the extent");
}

// Allocations left before the allocator refuses, or -1 for no end.
static int32_t s_allowed = -1;

static void* RefusingAlloc(size_t size, size_t alignment, void* context)
{
    if (s_allowed == 0)
    {
        return nullptr;
    }
    s_allowed -= s_allowed > 0 ? 1 : 0;
    return CountingAlloc(size, alignment, context);
}

static void TestMemoryLimitAndRefusedMemory(void)
{
    // A memory limit too small for the heightfield.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    def.limits.memoryBytes = 4096;
    CHECK(Bake(&def, &s_floorMesh, 1, 0, 0, nullptr) == mnav_errorCapacity,
          "memory past the limit");
    CHECK(s_held == 0, "nothing held after the memory limit");
    // Each allocation of the baker and the bake refused in turn.
    def = mnavDefaultBakeDef();
    def.allocator = (mnavAllocator){RefusingAlloc, CountingFree, nullptr};
    mnavResult result = mnav_errorCapacity;
    int32_t allowed = 0;
    for (; result == mnav_errorCapacity && allowed < 256; ++allowed)
    {
        s_allowed = allowed;
        result = Bake(&def, &s_floorMesh, 1, 0, 0, nullptr);
        mnavDestroyNavmesh(s_navmesh);
        s_navmesh = nullptr;
        CHECK(s_held == 0, "nothing held after a refusal");
    }
    s_allowed = -1;
    CHECK(result == mnav_success && allowed > 1, "every refusal is capacity, until enough");
}

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
    mnavBakeDef def = Exact();
    mnavBakeReport report;
    const mnavTriangleMesh one = {vertices, SOUP_TRIANGLES * 3, forward, SOUP_TRIANGLES, areas};
    CHECK(Bake(&def, &one, 1, 0, 0, &report) == mnav_success, "baked forward");
    uint64_t hash = DrawHash();
    // The same set, reversed and split across two meshes.
    int32_t half = SOUP_TRIANGLES / 2;
    const mnavTriangleMesh two[2] = {
        {vertices, SOUP_TRIANGLES * 3, backward, half, backwardAreas},
        {vertices, SOUP_TRIANGLES * 3, backward + half * 3, SOUP_TRIANGLES - half,
         backwardAreas + half},
    };
    CHECK(Bake(&def, two, 2, 0, 0, nullptr) == mnav_success, "baked backward");
    CHECK(report.polygons > 0 && DrawHash() == hash, "the same tile from the same set");
}

static void TestFloorToTheFrameEnd(void)
{
    // With the default def the tile and its border of 5 cells run from
    // -1.25 to 33.25 m. A floor from -5 to 33.3 m covers them and a
    // fifth of a cell past: its rows and columns end on the frame's
    // last cell, and the whole tile is walkable.
    const mnavVec3 vertices[4] = {{-5, 0, -5}, {-5, 0, 33.3f}, {33.3f, 0, 33.3f}, {33.3f, 0, -5}};
    const mnavTriangleMesh mesh = {vertices, 4, s_floorIndices, 2, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeReport report;
    CHECK(Bake(&def, &mesh, 1, 0, 0, &report) == mnav_success && report.polygons > 0 &&
              Walkable(0, 0, 127, 127, 0.0) == 128 * 128,
          "a floor to the frame's end");
}

int main(void)
{
    TestFloorFillsItsCells();
    TestSlopesAndFacing();
    TestStackedFloorsKeepSeparateSpans();
    TestMergedAreaDoesNotDependOnOrder();
    TestBoundaryEdgeLeavesNoSliver();
    TestTriangleOnTheTileEdges();
    TestNeighborTileSeesOnlyItsBorder();
    TestLimitsAreTyped();
    TestMemoryLimitAndRefusedMemory();
    TestSoupDependsOnlyOnTheTriangleSet();
    TestFloorToTheFrameEnd();
    mnavDestroyNavmesh(s_navmesh);
    return s_failures == 0 ? 0 : 1;
}
