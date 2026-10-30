// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The bake def's check and conversion, and the triangle mesh check.

#include "test_harness.h"

#include "maul-nav/bake.h"

#include <math.h>
#include <stdint.h>

static int Refuses(mnavBakeDef def, mnavBakeSetting setting)
{
    mnavBakeDefResult result = mnavValidateBakeDef(&def, nullptr);
    return result.result == mnav_errorInvalid && result.setting == setting;
}

// A setting past a bound the library names, MNAV_MIN_ or MNAV_MAX_.
static int Bounds(mnavBakeDef def, mnavBakeSetting setting)
{
    mnavBakeDefResult result = mnavValidateBakeDef(&def, nullptr);
    return result.result == mnav_errorLimit && result.setting == setting;
}

static void TestDefaultDefIsValid(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells = {0};
    mnavBakeDefResult result = mnavValidateBakeDef(&def, &cells);
    CHECK(result.result == mnav_success && result.setting == mnav_settingNone, "default valid");
    CHECK(cells.agentHeight == 16, "2 m over 0.125 m is 16 cells");
    CHECK(cells.agentStep == 6, "0.75 m over 0.125 m is 6 cells");
    CHECK(cells.agentRadius == 2, "0.5 m over 0.25 m is 2 cells");
    CHECK(cells.border == 5, "border is the radius and 3");
    CHECK(cells.tileSize == 32.0f, "128 cells of 0.25 m");
    CHECK(cells.minRegion == 32, "2 square meters of 0.0625 is 32 cells");
    CHECK(cells.edgeError == 1.2f, "0.3 m is 1.2 cells");
    CHECK(cells.edgeLength == 48, "12 m is 48 cells");
    CHECK(cells.detailSample == 96, "1.5 m is 96 sixteenths of a cell");
    CHECK(cells.detailError == 16, "0.125 m is 16 sixteenths of a cell height");
    mnavBakeDef fine = mnavDefaultBakeDef();
    fine.detailSampleDistance = 0.1f;
    CHECK(mnavValidateBakeDef(&fine, &cells).result == mnav_success && cells.detailSample == 16,
          "a sample distance under a cell is one cell");
    fine.detailSampleDistance = 0.0f;
    CHECK(mnavValidateBakeDef(&fine, &cells).result == mnav_success && cells.detailSample == 0,
          "0 for no samples");
    CHECK(cells.cosMaxSlope > 0.707106f && cells.cosMaxSlope < 0.707107f, "cos 45");
}

static void TestDefWithoutCookieIsRefused(void)
{
    mnavBakeDef def = {0};
    CHECK(Refuses(def, mnav_settingCookie), "zeroed def");
    mnavBakeDefResult result = mnavValidateBakeDef(nullptr, nullptr);
    CHECK(result.result == mnav_errorInvalid && result.setting == mnav_settingNone, "NULL def");
}

static void TestGridSettingsAreChecked(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator.context = &def;
    CHECK(mnavValidateBakeDef(&def, nullptr).result == mnav_success, "context alone is fine");
    mnavBakeDef broken = def;
    broken.origin.y = (double)NAN;
    CHECK(Refuses(broken, mnav_settingOrigin), "NaN origin");
    broken = def;
    broken.origin.x = (double)INFINITY;
    CHECK(Refuses(broken, mnav_settingOrigin), "infinite origin");
    broken = def;
    broken.cellSize = 0.0005f;
    CHECK(Bounds(broken, mnav_settingCellSize), "cell too small");
    broken = def;
    broken.cellSize = NAN;
    CHECK(Refuses(broken, mnav_settingCellSize), "NaN cell");
    // An infinity passes the published bound; a NaN is wrong.
    broken.cellSize = INFINITY;
    CHECK(Bounds(broken, mnav_settingCellSize), "infinite cell");
    broken = def;
    broken.cellHeight = 11.0f;
    CHECK(Bounds(broken, mnav_settingCellHeight), "cell too tall");
    broken = def;
    broken.tileCells = 15;
    CHECK(Bounds(broken, mnav_settingTileCells), "tile too small");
    broken = def;
    broken.tileCells = 1025;
    CHECK(Bounds(broken, mnav_settingTileCells), "tile too large");
}

static void TestAllocatorWithOneFunctionIsRefused(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator.free = (void (*)(void*, size_t, size_t, void*))1;
    CHECK(Refuses(def, mnav_settingAllocator), "free without alloc");
}

static void TestAgentSettingsAreChecked(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeDef broken = def;
    broken.agent.radius = -0.1f;
    CHECK(Refuses(broken, mnav_settingAgentRadius), "negative radius");
    broken = def;
    broken.agent.radius = 31.5f;
    CHECK(Refuses(broken, mnav_settingAgentRadius), "border wider than a tile");
    broken = def;
    broken.agent.height = 0.0f;
    CHECK(Refuses(broken, mnav_settingAgentHeight), "zero height");
    broken = def;
    broken.agent.height = 5000.0f;
    CHECK(Bounds(broken, mnav_settingAgentHeight), "height past the cell range");
    broken.agent.height = INFINITY;
    CHECK(Bounds(broken, mnav_settingAgentHeight), "infinite height");
    broken = def;
    broken.agent.stepHeight = NAN;
    CHECK(Refuses(broken, mnav_settingAgentStepHeight), "NaN step");
    broken = def;
    broken.agent.maxSlopeDegrees = 90.0f;
    CHECK(Refuses(broken, mnav_settingAgentMaxSlope), "90 degrees");
    broken = def;
    broken.agent.maxSlopeDegrees = -1.0f;
    CHECK(Refuses(broken, mnav_settingAgentMaxSlope), "negative slope");
    broken = def;
    broken.minRegionArea = -1.0f;
    CHECK(Refuses(broken, mnav_settingMinRegionArea), "negative region area");
    broken = def;
    broken.minRegionArea = NAN;
    CHECK(Refuses(broken, mnav_settingMinRegionArea), "NaN region area");
    broken = def;
    broken.minRegionArea = 1.0e30f;
    CHECK(Refuses(broken, mnav_settingMinRegionArea), "region area past any tile");
    broken = def;
    broken.maxEdgeError = -0.1f;
    CHECK(Refuses(broken, mnav_settingMaxEdgeError), "negative edge error");
    broken.maxEdgeError = MNAV_MAX_CELL_SIZE * MNAV_MAX_TILE_CELLS * 2.0f;
    CHECK(Bounds(broken, mnav_settingMaxEdgeError), "edge error past the bound");
    broken.maxEdgeError = INFINITY;
    CHECK(Bounds(broken, mnav_settingMaxEdgeError), "infinite edge error");
    broken = def;
    broken.maxEdgeLength = NAN;
    CHECK(Refuses(broken, mnav_settingMaxEdgeLength), "NaN edge length");
    broken = def;
    broken.detailSampleDistance = -1.0f;
    CHECK(Refuses(broken, mnav_settingDetailSampleDistance), "negative sample distance");
    broken = def;
    broken.detailSampleDistance = INFINITY;
    CHECK(Refuses(broken, mnav_settingDetailSampleDistance), "infinite sample distance");
    broken = def;
    broken.detailMaxError = NAN;
    CHECK(Refuses(broken, mnav_settingDetailMaxError), "NaN detail error");
}

static void TestMetersSnapToCells(void)
{
    // Recast's demo settings: a 0.9 m climb over 0.3 m cells is 3 cells.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = 0.3f;
    def.cellHeight = 0.3f;
    def.agent.stepHeight = 0.9f;
    def.agent.height = 1.8f;
    def.agent.radius = 0.6f;
    mnavBakeCells cells = {0};
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "valid");
    CHECK(cells.agentStep == 3, "climb of 3 cells");
    CHECK(cells.agentHeight == 6, "height of 6 cells");
    CHECK(cells.agentRadius == 2, "radius of 2 cells");
}

static void TestLimitsAreChecked(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeDef broken = def;
    broken.limits.inputTriangles = 0;
    CHECK(Refuses(broken, mnav_settingInputTriangles), "zero input triangles");
    broken = def;
    broken.limits.tileVertices = MNAV_MAX_TILE_VERTICES + 1;
    CHECK(Bounds(broken, mnav_settingTileVertices), "vertices past the cap");
    broken = def;
    broken.limits.tiles = -1;
    CHECK(Refuses(broken, mnav_settingTiles), "negative tiles");
    broken = def;
    broken.limits.links = 0;
    CHECK(Refuses(broken, mnav_settingLinks), "no links");
    broken = def;
    broken.limits.links = MNAV_MAX_LINKS + 1;
    CHECK(Bounds(broken, mnav_settingLinks), "links past the cap");
    broken = def;
    broken.limits.memoryBytes = 0;
    CHECK(Refuses(broken, mnav_settingMemoryBytes), "zero memory");
}

static const mnavVec3 s_square[4] = {{0, 0, 0}, {10, 0, 0}, {10, 0, 10}, {0, 0, 10}};
static const int32_t s_squareIndices[6] = {0, 1, 2, 0, 2, 3};

static mnavTriangleMesh Square(void)
{
    return (mnavTriangleMesh){s_square, 4, s_squareIndices, 2, nullptr};
}

static mnavInputResult ValidateMesh(const mnavBakeDef* def, const mnavTriangleMesh* mesh)
{
    return mnavValidateBakeInput(def, &(mnavBakeInput){mesh, 1, nullptr, 0, nullptr, 0, nullptr});
}

// Whether the mesh's check gives the result and element; a refusal names
// the mesh too.
static int MeshFails(const mnavTriangleMesh* mesh, mnavResult result, mnavInputElement element,
                     int32_t index)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavInputResult got = ValidateMesh(&def, mesh);
    bool named = result == mnav_success ? got.kind == mnav_inputNone && got.index == -1
                                        : got.kind == mnav_inputMesh && got.index == 0;
    return got.result == result && named && got.element == element && got.elementIndex == index;
}

static void TestValidMeshPasses(void)
{
    mnavTriangleMesh mesh = Square();
    CHECK(MeshFails(&mesh, mnav_success, mnav_elementNone, -1), "square is valid");
    mnavTriangleMesh empty = {nullptr, 0, nullptr, 0, nullptr};
    CHECK(MeshFails(&empty, mnav_success, mnav_elementNone, -1), "empty mesh is valid");
    // A degenerate triangle is accepted: it contributes nothing.
    const int32_t degenerate[3] = {1, 1, 1};
    mnavTriangleMesh flat = {s_square, 4, degenerate, 1, nullptr};
    CHECK(MeshFails(&flat, mnav_success, mnav_elementNone, -1), "degenerate accepted");
}

static void TestMeshArgumentsAreChecked(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavTriangleMesh mesh = Square();
    CHECK(ValidateMesh(nullptr, &mesh).result == mnav_errorInvalid, "NULL def");
    mnavInputResult none = mnavValidateBakeInput(&def, nullptr);
    CHECK(none.result == mnav_errorInvalid && none.kind == mnav_inputNone && none.index == -1,
          "NULL input");
    mnavBakeDef broken = def;
    broken.cookie = 0;
    CHECK(ValidateMesh(&broken, &mesh).result == mnav_errorInvalid, "invalid def");
    mnavTriangleMesh bad = mesh;
    bad.vertexCount = -1;
    CHECK(MeshFails(&bad, mnav_errorInvalid, mnav_elementNone, -1), "negative vertices");
    bad = mesh;
    bad.indices = nullptr;
    CHECK(MeshFails(&bad, mnav_errorInvalid, mnav_elementNone, -1), "missing indices");
    bad = mesh;
    bad.vertices = nullptr;
    CHECK(MeshFails(&bad, mnav_errorInvalid, mnav_elementNone, -1), "missing vertices");
}

static void TestHostileVerticesAreNamed(void)
{
    mnavVec3 vertices[4] = {{0, 0, 0}, {10, 0, 0}, {10, 0, 10}, {0, 0, 10}};
    mnavTriangleMesh mesh = {vertices, 4, s_squareIndices, 2, nullptr};
    vertices[2].y = NAN;
    CHECK(MeshFails(&mesh, mnav_errorInvalid, mnav_elementVertex, 2), "NaN vertex");
    vertices[2].y = 0.0f;
    vertices[1].x = -INFINITY;
    CHECK(MeshFails(&mesh, mnav_errorLimit, mnav_elementVertex, 1),
          "infinite vertex, past the extent");
    vertices[1].x = 10.0f;
    // 0.25 m cells reach 2^22 cells, 1,048,576 m, from the origin.
    vertices[3].z = 1048576.0f;
    CHECK(MeshFails(&mesh, mnav_success, mnav_elementNone, -1), "at the ground extent");
    vertices[3].z = 1048577.0f;
    CHECK(MeshFails(&mesh, mnav_errorLimit, mnav_elementVertex, 3), "past the ground extent");
    vertices[3].z = 10.0f;
    // 0.125 m cell heights reach 32,767 cells, 4,095.875 m, up or down.
    vertices[0].y = -4095.875f;
    CHECK(MeshFails(&mesh, mnav_success, mnav_elementNone, -1), "at the height extent");
    vertices[0].y = -4096.0f;
    CHECK(MeshFails(&mesh, mnav_errorLimit, mnav_elementVertex, 0), "past the height extent");
}

static void TestHostileTrianglesAreNamed(void)
{
    int32_t indices[6] = {0, 1, 2, 0, 2, 3};
    mnavAreaType areas[2] = {mnav_areaWalkable, 7};
    mnavTriangleMesh mesh = {s_square, 4, indices, 2, areas};
    CHECK(MeshFails(&mesh, mnav_success, mnav_elementNone, -1), "areas accepted");
    areas[1] = MNAV_AREA_TYPES;
    CHECK(MeshFails(&mesh, mnav_errorInvalid, mnav_elementTriangle, 1), "area past 63");
    areas[1] = mnav_areaNone;
    indices[5] = 4;
    CHECK(MeshFails(&mesh, mnav_errorInvalid, mnav_elementTriangle, 1), "index past vertices");
    indices[5] = 3;
    indices[0] = -1;
    CHECK(MeshFails(&mesh, mnav_errorInvalid, mnav_elementTriangle, 0), "negative index");
}

static void TestTriangleLimitIsTyped(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.inputTriangles = 1;
    mnavTriangleMesh mesh = Square();
    mnavInputResult result = ValidateMesh(&def, &mesh);
    CHECK(result.result == mnav_errorCapacity && result.kind == mnav_inputMesh &&
              result.element == mnav_elementNone,
          "limit hit");
    def.limits.inputTriangles = 2;
    CHECK(ValidateMesh(&def, &mesh).result == mnav_success, "limit met");
}

int main(void)
{
    TestDefaultDefIsValid();
    TestDefWithoutCookieIsRefused();
    TestGridSettingsAreChecked();
    TestAllocatorWithOneFunctionIsRefused();
    TestAgentSettingsAreChecked();
    TestMetersSnapToCells();
    TestLimitsAreChecked();
    TestValidMeshPasses();
    TestMeshArgumentsAreChecked();
    TestHostileVerticesAreNamed();
    TestHostileTrianglesAreNamed();
    TestTriangleLimitIsTyped();
    return s_failures == 0 ? 0 : 1;
}
