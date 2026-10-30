// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The public bake call: a baker made from a def bakes one tile per call,
// reports what it did, and hands the tile's bytes over, built field by
// field so that no padding of the def or the input reaches them.

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
    mnavResult result = mnavBaker_BakeTile(
        baker, &(mnavBakeInput){meshes, count, nullptr, 0, nullptr, 0, nullptr}, x, z, report);
    if (result == mnav_success)
    {
        CHECK(mnavBaker_CopyTile(baker, bytes, TILE_CAPACITY, size) == mnav_success, "copied");
    }
    return result;
}

static void TestCreateChecksTheDef(void)
{
    mnavBaker* baker = (mnavBaker*)&baker;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = -1.0f;
    mnavBakeDefResult result = mnavCreateBaker(&def, &baker);
    CHECK(result.result == mnav_errorLimit && result.setting == mnav_settingCellSize &&
              baker == nullptr,
          "an invalid def names its setting");
    CHECK(mnavCreateBaker(nullptr, &baker).result == mnav_errorInvalid, "no def");
    def = mnavDefaultBakeDef();
    CHECK(mnavCreateBaker(&def, nullptr).result == mnav_errorInvalid, "nowhere to put it");
    def.limits.memoryBytes = 8;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_errorCapacity, "too little memory");
    mnavDestroyBaker(nullptr);
}

static void TestLevelBakesAndCopies(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    size_t size = 0;
    CHECK(mnavBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorState &&
              mnavBaker_GetMisuse(baker) == 0,
          "nothing baked yet: a state, not misuse");
    mnavTriangleMesh level = Level();
    mnavBakeReport report;
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    CHECK(report.result == mnav_success && report.stage == mnav_stageDone &&
              report.input.kind == mnav_inputNone,
          "done");
    CHECK(report.triangles > 0 && report.spans > 0 && report.regions == 3 &&
              report.polygons == 54 && report.vertices > 0 && report.detailTriangles > 0,
          "counts");
    CHECK(report.droppedHoles == 0 && report.partialRings == 0 && report.cappedDetail == 0,
          "nothing given up");
    CHECK(report.memoryPeak > 0 && report.memoryPeak <= def.limits.memoryBytes, "memory");
    CHECK(mnavBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorCapacity &&
              size == report.tileBytes,
          "a size query");
    static uint8_t small[16];
    CHECK(mnavBaker_CopyTile(baker, small, sizeof(small), &size) == mnav_errorCapacity,
          "too small");
    CHECK(mnavBaker_CopyTile(baker, nullptr, 4, &size) == mnav_errorInvalid, "no buffer");
    static uint8_t bytes[TILE_CAPACITY];
    CHECK(mnavBaker_CopyTile(baker, bytes, sizeof(bytes), nullptr) == mnav_success, "copied");
    CHECK(memcmp(bytes, "MNAV", 4) == 0 && bytes[4] == MNAV_TILE_FORMAT, "a tile");
    printf("level: %zu bytes, %d polygons, %d detail triangles, peak %llu bytes\n", size,
           report.polygons, report.detailTriangles, (unsigned long long)report.memoryPeak);
    mnavDestroyBaker(baker);
}

static void TestFlatFloorAtNoSlope(void)
{
    // The steepest walkable slope is walkable: with 0 degrees allowed, a
    // floor with no slope at all is still floor.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.maxSlopeDegrees = 0.0f;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    const mnavVec3 floor[4] = {
        {-1.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 33.0f}, {33.0f, 0.0f, 33.0f}, {33.0f, 0.0f, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    const mnavTriangleMesh mesh = {floor, 4, quad, 2, nullptr};
    mnavBakeReport report;
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&mesh, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success &&
              report.polygons > 0,
          "a flat floor is walkable at a slope of 0");
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
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 1,
                             0, &report) == mnav_success,
          "a neighbor between");
    CHECK(BakeCopy(baker, &level, 1, 0, 0, again, &againSize, &report) == mnav_success, "again");
    CHECK(againSize == firstSize && memcmp(first, again, firstSize) == 0, "the same bytes");
    CHECK(report.memoryPeak == peak, "the same memory, nothing kept from before");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){nullptr, 0, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_success &&
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
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    uint64_t base = report.fingerprint;
    int32_t triangles = report.triangles;
    // A triangle far beyond the tile and its border.
    mnavVec3 far[3] = {{200.0f, 0.0f, 200.0f}, {200.0f, 0.0f, 201.0f}, {201.0f, 0.0f, 200.0f}};
    const int32_t one[3] = {0, 1, 2};
    mnavTriangleMesh meshes[2] = {level, {far, 3, one, 1, nullptr}};
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, 2, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    CHECK(report.fingerprint == base && report.triangles == triangles,
          "far geometry leaves the fingerprint");
    // The same triangle on the tile.
    mnavVec3 near[3] = {{10.0f, 0.5f, 10.0f}, {10.0f, 0.5f, 11.0f}, {11.0f, 0.5f, 10.0f}};
    meshes[1].vertices = near;
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, 2, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    CHECK(report.fingerprint != base && report.triangles == triangles + 1,
          "geometry on the tile moves it");
    uint64_t withNear = report.fingerprint;
    // The same triangle with another area type.
    const mnavAreaType kind[1] = {2};
    meshes[1].areas = kind;
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, 2, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    CHECK(report.fingerprint != withNear && report.fingerprint != base, "its area type moves it");
    // Another setting.
    mnavBakeDef coarse = def;
    coarse.detailMaxError = 0.25f;
    mnavBaker* other = nullptr;
    CHECK(mnavCreateBaker(&coarse, &other).result == mnav_success, "created");
    CHECK(mnavBaker_BakeTile(other, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
    CHECK(report.fingerprint != base, "a setting moves it");
    CHECK(mnavBaker_BakeTile(other, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 1,
                             0, &report) == mnav_success &&
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
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "baked");
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
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_success,
          "a good tile first");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, 2, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_errorInvalid,
          "refused");
    CHECK(report.stage == mnav_stageInput && report.input.kind == mnav_inputMesh &&
              report.input.index == 1 && report.input.element == mnav_elementVertex &&
              report.input.elementIndex == 1,
          "the second mesh's second vertex");
    size_t size = 0;
    uint64_t misuse = mnavBaker_GetMisuse(baker);
    CHECK(mnavBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorState &&
              mnavBaker_GetMisuse(baker) == misuse,
          "no tile held after a failure");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, -1, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_errorInvalid,
          "a negative count");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){nullptr, 1, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_errorInvalid,
          "no meshes");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){nullptr, 0, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_success &&
              report.polygons == 0,
          "no meshes, an empty tile");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr},
                             1 << 20, 0, &report) == mnav_errorLimit,
          "a tile past the extent");
    CHECK(mnavBaker_BakeTile(nullptr, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_errorInvalid &&
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
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_errorCapacity &&
              report.stage == mnav_stagePolygons,
          "the polygon limit, in the polygon stage");
    mnavDestroyBaker(baker);
    def = mnavDefaultBakeDef();
    def.limits.memoryBytes = 65536;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_errorCapacity &&
              report.stage == mnav_stageRasterize && report.memoryPeak <= 65536,
          "the memory limit, early");
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){nullptr, 0, nullptr, 0, nullptr, 0, nullptr},
                             0, 0, &report) == mnav_errorCapacity,
          "even an empty tile's columns do not fit");
    mnavDestroyBaker(baker);
    def = mnavDefaultBakeDef();
    def.limits.inputTriangles = 10;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh meshes[2] = {level, level};
    meshes[0].triangleCount = 6;
    meshes[1].triangleCount = 6;
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){meshes, 2, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &report) == mnav_errorCapacity &&
              report.stage == mnav_stageInput,
          "the input limit counts every mesh");
    mnavDestroyBaker(baker);
}

// A clock that moves 10 ticks at each read, counting its reads.
static uint64_t TenTicks(void* context)
{
    uint64_t* reads = context;
    *reads += 1;
    return *reads * 10;
}

static uint8_t s_timed[TILE_CAPACITY];
static uint8_t s_untimed[TILE_CAPACITY];

static void TestStagesAreTimedAndMeasured(void)
{
    mnavTriangleMesh level = Level();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* plain = nullptr;
    CHECK(mnavCreateBaker(&def, &plain).result == mnav_success, "created");
    uint64_t reads = 0;
    def.clock = (mnavClock){TenTicks, &reads};
    mnavBaker* timed = nullptr;
    CHECK(mnavCreateBaker(&def, &timed).result == mnav_success, "created with a clock");
    mnavBakeReport a;
    mnavBakeReport b;
    size_t sizeA = 0;
    size_t sizeB = 0;
    CHECK(BakeCopy(plain, &level, 1, 0, 0, s_untimed, &sizeA, &a) == mnav_success &&
              BakeCopy(timed, &level, 1, 0, 0, s_timed, &sizeB, &b) == mnav_success,
          "baked both ways");
    CHECK(sizeA == sizeB && memcmp(s_untimed, s_timed, sizeA) == 0 &&
              a.fingerprint == b.fingerprint,
          "the clock never shapes the tile");
    // Read once at the start and once as each of the 12 stages ends.
    uint64_t largest = 0;
    bool timedRight = reads == 13;
    bool untimed = true;
    for (int32_t s = 0; s < MNAV_BAKE_STAGES; ++s)
    {
        timedRight = timedRight && b.stageTicks[s] == 10;
        untimed = untimed && a.stageTicks[s] == 0;
        largest = b.stageMemory[s] > largest ? b.stageMemory[s] : largest;
        CHECK(a.stageMemory[s] == b.stageMemory[s], "the same memory either way");
    }
    CHECK(timedRight && untimed, "every stage 10 ticks with the clock, 0 without");
    CHECK(largest == b.memoryPeak && b.stageMemory[mnav_stageRasterize] > 0 &&
              b.stageMemory[mnav_stageDetail] > 0,
          "the peak is the largest stage's");
    // Each stage's own peak: the rasterizer's fragments are gone before
    // the compact field is built, and the regions' scratch before the
    // contours.
    CHECK(b.stageMemory[mnav_stageCompact] < b.stageMemory[mnav_stageRasterize] &&
              b.stageMemory[mnav_stageContours] < b.stageMemory[mnav_stageRegions],
          "a stage's peak is its own, not the bake's so far");
    // A bake ending in the polygon stage times the stages up to it only.
    mnavDestroyBaker(timed);
    def.limits.tilePolygons = 10;
    CHECK(mnavCreateBaker(&def, &timed).result == mnav_success, "created");
    reads = 0;
    CHECK(mnavBaker_BakeTile(timed, &(mnavBakeInput){&level, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, &b) == mnav_errorCapacity &&
              b.stage == mnav_stagePolygons,
          "ended in the polygon stage");
    bool upTo = true;
    for (int32_t s = 0; s < MNAV_BAKE_STAGES; ++s)
    {
        bool ran = s <= mnav_stagePolygons;
        upTo = upTo && b.stageTicks[s] == (ran ? 10u : 0u) && (b.stageMemory[s] > 0) == ran;
    }
    CHECK(upTo && reads == (uint64_t)mnav_stagePolygons + 2, "only the stages that ran");
    mnavDestroyBaker(timed);
    mnavDestroyBaker(plain);
}

// Along each axis from tile (0, 0), the last tile within the input's
// extent, found by bisection: it bakes, and the next one is refused. Each
// bake's stages make the same tile frame the baker checked first.
static void TestTilesToTheExtent(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    const int32_t dx[4] = {1, -1, 0, 0};
    const int32_t dz[4] = {0, 0, 1, -1};
    for (int32_t d = 0; d < 4; ++d)
    {
        int32_t in = 0;
        int32_t out = 1 << 20;
        CHECK(mnavBaker_BakeTile(baker,
                                 &(mnavBakeInput){nullptr, 0, nullptr, 0, nullptr, 0, nullptr},
                                 out * dx[d], out * dz[d], nullptr) == mnav_errorLimit,
              "far past the extent");
        while (out - in > 1)
        {
            int32_t mid = in + (out - in) / 2;
            mnavResult result = mnavBaker_BakeTile(
                baker, &(mnavBakeInput){nullptr, 0, nullptr, 0, nullptr, 0, nullptr}, mid * dx[d],
                mid * dz[d], nullptr);
            CHECK(result == mnav_success || result == mnav_errorLimit, "baked or refused");
            in = result == mnav_success ? mid : in;
            out = result == mnav_success ? out : mid;
        }
        // 4,194,304 cells of extent and a tile's most, 1,024, in tiles of
        // 128 cells less the border.
        CHECK(in == (dx[d] + dz[d] > 0 ? 32776 : 32775), "the last tile within the extent");
    }
    mnavDestroyBaker(baker);
}

// A def over a byte pattern, every field then set one by one to the
// default's, so only its padding keeps the pattern.
static void DefOver(mnavBakeDef* def, int pattern)
{
    const mnavBakeDef d = mnavDefaultBakeDef();
    memset(def, pattern, sizeof(*def));
    def->cookie = d.cookie;
    def->version = d.version;
    def->allocator.alloc = d.allocator.alloc;
    def->allocator.free = d.allocator.free;
    def->allocator.context = d.allocator.context;
    def->origin.x = d.origin.x;
    def->origin.y = d.origin.y;
    def->origin.z = d.origin.z;
    def->cellSize = d.cellSize;
    def->cellHeight = d.cellHeight;
    def->tileCells = d.tileCells;
    def->agent.radius = d.agent.radius;
    def->agent.height = d.agent.height;
    def->agent.stepHeight = d.agent.stepHeight;
    def->agent.maxSlopeDegrees = d.agent.maxSlopeDegrees;
    def->minRegionArea = d.minRegionArea;
    def->maxEdgeError = d.maxEdgeError;
    def->maxEdgeLength = d.maxEdgeLength;
    def->detailSampleDistance = d.detailSampleDistance;
    def->detailMaxError = d.detailMaxError;
    def->limits.inputTriangles = d.limits.inputTriangles;
    def->limits.tileTriangles = d.limits.tileTriangles;
    def->limits.tileSpans = d.limits.tileSpans;
    def->limits.tilePolygons = d.limits.tilePolygons;
    def->limits.tileVertices = d.limits.tileVertices;
    def->limits.tileLinks = d.limits.tileLinks;
    def->limits.tiles = d.limits.tiles;
    def->limits.links = d.limits.links;
    def->limits.memoryBytes = d.limits.memoryBytes;
    def->tier = d.tier;
    def->clock.now = d.clock.now;
    def->clock.context = d.clock.context;
}

// Bakes the level and a volume over it from structs filled with a byte
// pattern and then set field by field; copies the tile's bytes.
static size_t BakeOver(int pattern, uint8_t* bytes)
{
    mnavBakeDef def;
    DefOver(&def, pattern);
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    mnavTriangleMesh level = Level();
    mnavTriangleMesh mesh;
    memset(&mesh, pattern, sizeof(mesh));
    mesh.vertices = level.vertices;
    mesh.vertexCount = level.vertexCount;
    mesh.indices = level.indices;
    mesh.triangleCount = level.triangleCount;
    mesh.areas = level.areas;
    static const mnavVec2 square[4] = {{2.0f, 2.0f}, {6.0f, 2.0f}, {6.0f, 6.0f}, {2.0f, 6.0f}};
    mnavBakeVolume volume;
    memset(&volume, pattern, sizeof(volume));
    volume.points = square;
    volume.pointCount = 4;
    volume.minY = -10.0f;
    volume.maxY = 10.0f;
    volume.kind = mnav_volumeArea;
    volume.area = (mnavAreaType)2;
    mnavBakeInput input;
    memset(&input, pattern, sizeof(input));
    input.meshes = &mesh;
    input.meshCount = 1;
    input.terrains = nullptr;
    input.terrainCount = 0;
    input.volumes = &volume;
    input.volumeCount = 1;
    input.index = nullptr;
    size_t size = 0;
    CHECK(mnavBaker_BakeTile(baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavBaker_CopyTile(baker, bytes, TILE_CAPACITY, &size) == mnav_success,
          "baked and copied");
    mnavDestroyBaker(baker);
    return size;
}

// Bakes a floor's outline from structs filled with a byte pattern and then
// set field by field; copies the tile's bytes.
static size_t Bake2DOver(int pattern, uint8_t* bytes)
{
    mnavBakeDef def;
    DefOver(&def, pattern);
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "created");
    static const mnavVec2 floor[4] = {{1.0f, 1.0f}, {7.0f, 1.0f}, {7.0f, 7.0f}, {1.0f, 7.0f}};
    mnavOutline outline;
    memset(&outline, pattern, sizeof(outline));
    outline.points = floor;
    outline.pointCount = 4;
    outline.area = (mnavAreaType)1;
    mnavBake2DInput input;
    memset(&input, pattern, sizeof(input));
    input.outlines = &outline;
    input.outlineCount = 1;
    input.index = nullptr;
    size_t size = 0;
    CHECK(mnavBaker_BakeTile2D(baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavBaker_CopyTile(baker, bytes, TILE_CAPACITY, &size) == mnav_success,
          "baked and copied");
    mnavDestroyBaker(baker);
    return size;
}

// Tile bytes are built field by field: the padding of the def and the
// input never reaches them.
static void TestPaddingNeverReachesTheTile(void)
{
    static uint8_t zeros[TILE_CAPACITY];
    static uint8_t ones[TILE_CAPACITY];
    size_t a = BakeOver(0x00, zeros);
    size_t b = BakeOver(0xFF, ones);
    CHECK(a > 0 && a == b && memcmp(zeros, ones, a) == 0, "the same bytes over both patterns");
    a = Bake2DOver(0x00, zeros);
    b = Bake2DOver(0xFF, ones);
    CHECK(a > 0 && a == b && memcmp(zeros, ones, a) == 0, "the same 2D bytes over both patterns");
}

int main(void)
{
    TestCreateChecksTheDef();
    TestLevelBakesAndCopies();
    TestFlatFloorAtNoSlope();
    TestBakesDoNotDependOnWhatCameBefore();
    TestFingerprintFollowsTheTilesInput();
    TestDetailSearchesAsFarAsWallsStray();
    TestBadInputIsNamed();
    TestLimitsEndInAStage();
    TestStagesAreTimedAndMeasured();
    TestTilesToTheExtent();
    TestPaddingNeverReachesTheTile();
    return s_failures == 0 ? 0 : 1;
}
