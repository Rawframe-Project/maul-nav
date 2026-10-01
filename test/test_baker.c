// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The public bake call: a baker made from a def bakes one tile per call,
// reports what it did, and hands the tile's bytes over (mnav-0003).

#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

enum
{
    TILE_CAPACITY = 1 << 16
};

static mnavVec3 s_vertices[LEVEL_VERTICES];
static int32_t s_indices[LEVEL_TRIANGLES * 3];

static mnavTriangleMesh Level(void)
{
    MakeLevel(s_vertices, s_indices);
    return (mnavTriangleMesh){s_vertices, LEVEL_VERTICES, s_indices, LEVEL_TRIANGLES, nullptr};
}

// Bakes tile (x, z) and copies its bytes; returns the bake's result.
static mnavResult BakeCopy(mnavBaker* baker, const mnavTriangleMesh* meshes, int32_t count,
                           int32_t x, int32_t z, uint8_t* bytes, size_t* size,
                           mnavBakeReport* report)
{
    mnavResult result = mnavBakeTile(baker, meshes, count, x, z, report);
    if (result == mnav_success)
    {
        CHECK(mnavCopyBakedTile(baker, bytes, TILE_CAPACITY, size) == mnav_success, "copied");
    }
    return result;
}

static void TestCreateChecksTheDef(void)
{
    mnavBaker* baker = (mnavBaker*)&baker;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = -1.0f;
    mnavBakeDefResult result = mnavCreateBaker(&def, &baker);
    CHECK(result.result == mnav_errorInvalid && result.setting == mnav_settingCellSize &&
              baker == nullptr,
          "an invalid def names its setting");
    CHECK(mnavCreateBaker(nullptr, &baker).result == mnav_errorInvalid, "no def");
    def = mnavDefaultBakeDef();
    CHECK(mnavCreateBaker(&def, nullptr).result == mnav_errorInvalid, "nowhere to put it");
    def.limits.memoryBytes = 8;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_errorLimit, "too little memory");
    mnavDestroyBaker(nullptr);
}

static void TestLevelBakesAndCopies(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    size_t size = 0;
    CHECK(mnavCopyBakedTile(baker, nullptr, 0, &size) == mnav_errorInvalid, "nothing baked yet");
    mnavTriangleMesh level = Level();
    mnavBakeReport report;
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.result == mnav_success && report.stage == mnav_stageDone && report.mesh == -1,
          "done");
    CHECK(report.triangles > 0 && report.spans > 0 && report.regions == 3 &&
              report.polygons == 54 && report.vertices > 0 && report.detailTriangles > 0,
          "counts");
    CHECK(report.droppedHoles == 0 && report.partialRings == 0 && report.cappedDetail == 0,
          "nothing given up");
    CHECK(report.memoryPeak > 0 && report.memoryPeak <= def.limits.memoryBytes, "memory");
    CHECK(mnavCopyBakedTile(baker, nullptr, 0, &size) == mnav_errorCapacity &&
              size == report.tileBytes,
          "a size query");
    static uint8_t small[16];
    CHECK(mnavCopyBakedTile(baker, small, sizeof(small), &size) == mnav_errorCapacity, "too small");
    CHECK(mnavCopyBakedTile(baker, nullptr, 4, &size) == mnav_errorInvalid, "no buffer");
    static uint8_t bytes[TILE_CAPACITY];
    CHECK(mnavCopyBakedTile(baker, bytes, sizeof(bytes), nullptr) == mnav_success, "copied");
    CHECK(memcmp(bytes, "MNAV", 4) == 0 && bytes[4] == MNAV_TILE_FORMAT, "a tile");
    printf("level: %zu bytes, %d polygons, %d detail triangles, peak %llu bytes\n", size,
           report.polygons, report.detailTriangles, (unsigned long long)report.memoryPeak);
    mnavDestroyBaker(baker);
}

static void TestBakesDoNotDependOnWhatCameBefore(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    static uint8_t first[TILE_CAPACITY];
    static uint8_t again[TILE_CAPACITY];
    size_t firstSize = 0;
    size_t againSize = 0;
    mnavBakeReport report;
    CHECK(BakeCopy(baker, &level, 1, 0, 0, first, &firstSize, &report) == mnav_success, "first");
    uint64_t peak = report.memoryPeak;
    CHECK(mnavBakeTile(baker, &level, 1, 1, 0, &report) == mnav_success, "a neighbor between");
    CHECK(BakeCopy(baker, &level, 1, 0, 0, again, &againSize, &report) == mnav_success, "again");
    CHECK(againSize == firstSize && memcmp(first, again, firstSize) == 0, "the same bytes");
    CHECK(report.memoryPeak == peak, "the same memory, nothing kept from before");
    CHECK(mnavBakeTile(baker, nullptr, 0, 0, 0, &report) == mnav_success &&
              report.memoryPeak < peak,
          "an empty tile's peak is its own");
    mnavBaker* other = nullptr;
    CHECK(mnavCreateBaker(&def, &other).result == mnav_success, "a second baker");
    CHECK(BakeCopy(other, &level, 1, 0, 0, again, &againSize, &report) == mnav_success, "baked");
    CHECK(againSize == firstSize && memcmp(first, again, firstSize) == 0, "the same bytes");
    mnavDestroyBaker(other);
    mnavDestroyBaker(baker);
}

static void TestFingerprintFollowsTheTilesInput(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    mnavBakeReport report;
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_success, "baked");
    uint64_t base = report.fingerprint;
    int32_t triangles = report.triangles;
    // A triangle far beyond the tile and its border.
    mnavVec3 far[3] = {{200.0f, 0.0f, 200.0f}, {200.0f, 0.0f, 201.0f}, {201.0f, 0.0f, 200.0f}};
    const int32_t one[3] = {0, 1, 2};
    mnavTriangleMesh meshes[2] = {level, {far, 3, one, 1, nullptr}};
    CHECK(mnavBakeTile(baker, meshes, 2, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.fingerprint == base && report.triangles == triangles,
          "far geometry leaves the fingerprint");
    // The same triangle on the tile.
    mnavVec3 near[3] = {{10.0f, 0.5f, 10.0f}, {10.0f, 0.5f, 11.0f}, {11.0f, 0.5f, 10.0f}};
    meshes[1].vertices = near;
    CHECK(mnavBakeTile(baker, meshes, 2, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.fingerprint != base && report.triangles == triangles + 1,
          "geometry on the tile moves it");
    uint64_t withNear = report.fingerprint;
    // The same triangle with another area type.
    const mnavAreaType kind[1] = {2};
    meshes[1].areas = kind;
    CHECK(mnavBakeTile(baker, meshes, 2, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.fingerprint != withNear && report.fingerprint != base, "its area type moves it");
    // Another setting.
    mnavBakeDef coarse = def;
    coarse.detailMaxError = 0.25f;
    mnavBaker* other = nullptr;
    CHECK(mnavCreateBaker(&coarse, &other).result == mnav_success, "created");
    CHECK(mnavBakeTile(other, &level, 1, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.fingerprint != base, "a setting moves it");
    CHECK(mnavBakeTile(other, &level, 1, 1, 0, &report) == mnav_success &&
              report.fingerprint != base,
          "so does the tile's place");
    mnavDestroyBaker(other);
    mnavDestroyBaker(baker);
}

static void TestDetailSearchesAsFarAsWallsStray(void)
{
    // Walls simplified within 2 m of the cells leave edge samples up to 8
    // cells from any height; the lookup searches that far.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.maxEdgeError = 2.0f;
    def.detailSampleDistance = 0.25f;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    mnavBakeReport report;
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_success, "baked");
    CHECK(report.fallbackHeights == 0, "every sample found a height");
    mnavDestroyBaker(baker);
}

static void TestBadInputIsNamed(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    mnavVec3 bad[3] = {{0.0f, 0.0f, 0.0f}, {NAN, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
    const int32_t one[3] = {0, 1, 2};
    mnavTriangleMesh meshes[2] = {level, {bad, 3, one, 1, nullptr}};
    mnavBakeReport report;
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_success, "a good tile first");
    CHECK(mnavBakeTile(baker, meshes, 2, 0, 0, &report) == mnav_errorInvalid, "refused");
    CHECK(report.stage == mnav_stageInput && report.mesh == 1 &&
              report.input.element == mnav_elementVertex && report.input.index == 1,
          "the second mesh's second vertex");
    size_t size = 0;
    CHECK(mnavCopyBakedTile(baker, nullptr, 0, &size) == mnav_errorInvalid,
          "no tile held after a failure");
    CHECK(mnavBakeTile(baker, meshes, -1, 0, 0, &report) == mnav_errorInvalid, "a negative count");
    CHECK(mnavBakeTile(baker, nullptr, 1, 0, 0, &report) == mnav_errorInvalid, "no meshes");
    CHECK(mnavBakeTile(baker, nullptr, 0, 0, 0, &report) == mnav_success && report.polygons == 0,
          "no meshes, an empty tile");
    CHECK(mnavBakeTile(baker, &level, 1, 1 << 20, 0, &report) == mnav_errorRange,
          "a tile past the extent");
    CHECK(mnavBakeTile(nullptr, &level, 1, 0, 0, &report) == mnav_errorInvalid &&
              report.result == mnav_errorInvalid,
          "no baker");
    mnavDestroyBaker(baker);
}

static void TestLimitsEndInAStage(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tilePolygons = 10;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    mnavBakeReport report;
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_errorLimit &&
              report.stage == mnav_stagePolygons,
          "the polygon limit, in the polygon stage");
    mnavDestroyBaker(baker);
    def = mnavDefaultBakeDef();
    def.limits.memoryBytes = 65536;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    CHECK(mnavBakeTile(baker, &level, 1, 0, 0, &report) == mnav_errorLimit &&
              report.stage == mnav_stageRasterize && report.memoryPeak <= 65536,
          "the memory limit, early");
    CHECK(mnavBakeTile(baker, nullptr, 0, 0, 0, &report) == mnav_errorLimit,
          "even an empty tile's columns do not fit");
    mnavDestroyBaker(baker);
    def = mnavDefaultBakeDef();
    def.limits.inputTriangles = 10;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh meshes[2] = {level, level};
    meshes[0].triangleCount = 6;
    meshes[1].triangleCount = 6;
    CHECK(mnavBakeTile(baker, meshes, 2, 0, 0, &report) == mnav_errorLimit &&
              report.stage == mnav_stageInput,
          "the input limit counts every mesh");
    mnavDestroyBaker(baker);
}

int main(void)
{
    TestCreateChecksTheDef();
    TestLevelBakesAndCopies();
    TestBakesDoNotDependOnWhatCameBefore();
    TestFingerprintFollowsTheTilesInput();
    TestDetailSearchesAsFarAsWallsStray();
    TestBadInputIsNamed();
    TestLimitsEndInAStage();
    return s_failures == 0 ? 0 : 1;
}
