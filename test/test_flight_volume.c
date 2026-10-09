// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flight volumes through the public API (mnav-0015): the def's checks;
// tiles baked, copied, staged and committed, every voxel's openness as
// the octree has it; the bytes pinned; every truncated or corrupted tile
// refused, naming its section; tiles of other settings refused; staging,
// removal, replacement and a refused commit that changes nothing; every
// byte given back.

#include "bytes.h"
#include "counting_allocator.h"
#include "flight.h"
#include "flight_def.h"
#include "flight_tile.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/navmesh.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The hash of the test world's four flight tiles' bytes, the same on
// every platform.
#define FLIGHT_BYTES_HASH 0x4e9857a26cabe6ceull

// Where the header keeps its fields.
enum
{
    AT_GENERATOR = 8,
    AT_FINGERPRINT = 16,
    AT_HASH = 24,
    AT_PLACE = 36,
    AT_NODES = 96,
    TILE_MOST = 1 << 20
};

static uint8_t s_bytes[4][TILE_MOST];
static size_t s_sizes[4];
static uint64_t s_fingerprints[4];

// The def the tests bake with: 0.5 m voxels, tiles of 64 (32 m, as the
// test world's), from 2 m below the ground to 20 m up.
static mnavFlightDef Def(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.allocator = CountingAllocator();
    def.voxelSize = 0.5f;
    def.tileVoxels = 64;
    def.floor = -2.0f;
    def.ceiling = 20.0f;
    return def;
}

// Bakes tile t of the test world into s_bytes; returns its bytes' hash.
static uint64_t BakeOne(mnavFlightBaker* baker, const mnavBakeInput* input, int32_t t)
{
    mnavFlightBakeReport report;
    CHECK(mnavBakeFlightTile(baker, input, t % 2, t / 2, &report) == mnav_success, "baked");
    CHECK(report.result == mnav_success && report.mesh == -1 && report.triangles > 0 &&
              report.spans > 0 && report.cubes == 1 && report.nodes > 0 && report.leaves > 0 &&
              report.memoryPeak > 0,
          "the report");
    size_t size = 0;
    CHECK(mnavCopyFlightTile(baker, s_bytes[t], TILE_MOST, &size) == mnav_success, "copied");
    CHECK(size == report.tileBytes, "the size reported");
    s_sizes[t] = size;
    s_fingerprints[t] = report.fingerprint;
    printf("flight tile %d: %zu bytes, %d nodes, %d leaves\n", t, size, report.nodes,
           report.leaves);
    return mnavHash64(MNAV_HASH_INIT, s_bytes[t], (int32_t)size);
}

// Bakes the test world's four tiles into s_bytes, and one again.
static void BakeAll(void)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        uint64_t one = BakeOne(baker, &input, t);
        hash = mnavHash64(hash, &one, (int32_t)sizeof(one));
    }
    printf("FLIGHT_BYTES_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FLIGHT_BYTES_HASH, "the pinned bytes");
    // The same bake makes the same bytes.
    CHECK(mnavBakeFlightTile(baker, &input, 0, 0, nullptr) == mnav_success, "again");
    size_t size = 0;
    static uint8_t again[TILE_MOST];
    CHECK(mnavCopyFlightTile(baker, again, TILE_MOST, &size) == mnav_success, "copied again");
    CHECK(size == s_sizes[0] && memcmp(again, s_bytes[0], size) == 0, "the same bytes");
    mnavDestroyFlightBaker(baker);
}

static mnavFlightVolume* Volume(const mnavFlightDef* def)
{
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(def, &volume).result == mnav_success, "volume");
    return volume;
}

// The voxels of tile t whose openness the volume answers otherwise than
// the octree; openOut receives the open ones.
static int32_t Wrong(const mnavFlightVolume* volume, const mnavFlightTile* tile, int32_t t,
                     int32_t floorVoxel, int32_t* openOut)
{
    int32_t wrong = 0;
    int32_t layers = tile->cubeCount * tile->side;
    int32_t column = t % 2;
    int32_t row = t / 2;
    double x0 = (double)(column * tile->side);
    double z0 = (double)(row * tile->side);
    for (int32_t z = 0; z < tile->side; ++z)
    {
        for (int32_t y = 0; y < layers; ++y)
        {
            for (int32_t x = 0; x < tile->side; ++x)
            {
                mnavPos3 p = {(x0 + x + 0.5) * 0.5, (floorVoxel + y + 0.5) * 0.5,
                              (z0 + z + 0.5) * 0.5};
                bool got = false;
                wrong += mnavIsFlightOpen(volume, p, &got) != mnav_success ? 1 : 0;
                wrong += got == mnavFlightSolid(tile, x, y, z) ? 1 : 0;
                *openOut += got ? 1 : 0;
            }
        }
    }
    return wrong;
}

// Checks every voxel of a committed tile's openness against the octree
// decoded from its bytes.
static void CheckVoxels(const mnavFlightVolume* volume, int32_t t)
{
    mnavFlightDef def = Def();
    mnavFlightShape shape;
    CHECK(mnavCheckFlightDef(&def, &shape).result == mnav_success, "shape");
    mnavMemory memory = mnavMakeMemory(def.allocator, UINT64_MAX);
    mnavFlightTile tile;
    uint64_t fingerprint = 0;
    CHECK(mnavDecodeFlightTile(&memory, &def, &shape, s_bytes[t], s_sizes[t], &tile, &fingerprint)
                  .result == mnav_success,
          "decoded");
    int32_t open = 0;
    int32_t wrong = Wrong(volume, &tile, t, shape.floorVoxel, &open);
    printf("tile %d: %d open voxels, %d wrong\n", t, open, wrong);
    CHECK(wrong == 0 && open > 0, "every voxel as the octree has it");
    mnavReleaseFlightTile(&memory, &tile);
}

// Points past the committed tiles, and past the volume's height.
static void CheckOutside(const mnavFlightVolume* volume)
{
    bool open = true;
    uint64_t fingerprint = 1;
    CHECK(mnavGetFlightTile(volume, 2, 0, &fingerprint) == mnav_errorNotLoaded && fingerprint == 0,
          "no tile there");
    const mnavPos3 away[4] = {
        {70.0, 5.0, 1.0}, {-1.0, 5.0, 1.0}, {1.0, 5.0, -0.01}, {1.0, 5.0, 64.5}};
    int32_t wrong = 0;
    for (int32_t k = 0; k < 4; ++k)
    {
        wrong += mnavIsFlightOpen(volume, away[k], &open) == mnav_errorNotLoaded ? 0 : 1;
    }
    const mnavPos3 out[4] = {
        {1.0, -2.01, 1.0}, {1.0, 30.0, 1.0}, {(double)NAN, 5.0, 1.0}, {1.0, 5.0, 1e300}};
    for (int32_t k = 0; k < 4; ++k)
    {
        wrong += mnavIsFlightOpen(volume, out[k], &open) == mnav_errorRange && !open ? 0 : 1;
    }
    CHECK(wrong == 0, "past the tiles, below, above or past the volume");
    CHECK(mnavIsFlightOpen(volume, (mnavPos3){1.0, 29.99, 1.0}, &open) == mnav_success,
          "the top of the last cube");
    CHECK(mnavIsFlightOpen(volume, (mnavPos3){1.0, -1.99, 1.0}, &open) == mnav_success && !open,
          "under the ground");
}

static void TestBakeStageAndQuery(void)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    bool open = true;
    mnavPos3 inside = {1.0, 5.0, 1.0};
    CHECK(mnavIsFlightOpen(volume, inside, &open) == mnav_errorNotLoaded && !open,
          "nothing loaded");
    int32_t staged = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        staged += mnavStageFlightTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
    }
    CHECK(staged == 4, "staged");
    CHECK(mnavIsFlightOpen(volume, inside, &open) == mnav_errorNotLoaded,
          "not seen before the commit");
    CHECK(mnavCommitFlight(volume) == mnav_success, "committed");
    for (int32_t t = 0; t < 4; ++t)
    {
        uint64_t fingerprint = 0;
        CHECK(mnavGetFlightTile(volume, t % 2, t / 2, &fingerprint) == mnav_success &&
                  fingerprint == s_fingerprints[t],
              "the fingerprint");
        CheckVoxels(volume, t);
    }
    CheckOutside(volume);
    CHECK(mnavIsFlightOpen(nullptr, inside, &open) == mnav_errorInvalid &&
              mnavIsFlightOpen(volume, inside, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavDestroyFlightVolume(volume);
}

static uint8_t s_bad[TILE_MOST];

// Stages bad bytes on a fresh volume of the test def.
static mnavTileResult StageBad(size_t size)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    mnavTileResult result = mnavStageFlightTile(volume, s_bad, size);
    mnavDestroyFlightVolume(volume);
    return result;
}

// Writes the payload's hash into the header again, so that a change in
// the payload reaches the structural checks.
static void Rehash(size_t size)
{
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, s_bad + MNAV_FLIGHT_HEADER_BYTES,
                               (int32_t)(size - MNAV_FLIGHT_HEADER_BYTES));
    mnavByteWriter w = {s_bad + AT_HASH};
    mnavPutU64(&w, hash);
}

// Whether tile 1 with byte i flipped is refused as it should be: but in
// the generator, the fingerprint and the place, which any value may hold;
// as another version in the version; as past the limit, or malformed, in
// the counts.
static bool FlipRefused(size_t i, size_t size)
{
    memcpy(s_bad, s_bytes[1], size);
    s_bad[i] ^= 0x5A;
    bool free = (i >= AT_GENERATOR && i < AT_GENERATOR + 6) ||
                (i >= AT_FINGERPRINT && i < AT_FINGERPRINT + 8) ||
                (i >= AT_PLACE && i < AT_PLACE + 8);
    bool count = i >= AT_NODES && i < AT_NODES + 8;
    bool version = i == 4 || i == 5;
    mnavTileResult r = StageBad(size);
    if (free)
    {
        return r.result == mnav_success;
    }
    return r.result == (version ? mnav_errorVersion : mnav_errorInvalid) ||
           (count && r.result == mnav_errorLimit);
}

static void TestTruncatedAndFlippedBytes(void)
{
    size_t size = s_sizes[1];
    int32_t wrong = 0;
    for (size_t n = 0; n < size; n += n < 256 ? 1 : 97)
    {
        memcpy(s_bad, s_bytes[1], n);
        wrong += StageBad(n).result != mnav_errorInvalid ? 1 : 0;
    }
    CHECK(wrong == 0, "every truncation refused");
    for (size_t i = 0; i < size; i += i < 256 ? 1 : 13)
    {
        wrong += FlipRefused(i, size) ? 0 : 1;
    }
    CHECK(wrong == 0, "every flipped byte refused but those no check reads");
    memcpy(s_bad, s_bytes[1], size);
    s_bad[size - 1] ^= 1;
    CHECK(StageBad(size).section == mnav_tilePayload, "the payload's hash");
    memcpy(s_bad, s_bytes[1], size);
    CHECK(StageBad(size + 1).section == mnav_tilePayload, "a byte past the payload");
}

// The tile's first root, node and leaf, and where they start.
static void TestPayloadStructure(void)
{
    size_t size = s_sizes[1];
    size_t roots = MNAV_FLIGHT_HEADER_BYTES;
    size_t nodes = roots + 1;
    mnavByteReader r = {s_bytes[1] + AT_NODES, 8, false};
    size_t nodeCount = (size_t)mnavGetU32(&r);
    size_t leaves = nodes + 2 * nodeCount;
    const struct
    {
        size_t at;
        uint8_t value;
        mnavTileSection section;
        int32_t index;
    } cases[] = {
        // A root class past mixed, and an empty and a solid root left
        // with nodes to spare.
        {roots, 3, mnav_tileFlightRoots, 0},
        {roots, 0, mnav_tileFlightNodes, 0},
        {roots, 1, mnav_tileFlightNodes, 0},
        // A child both mixed and solid; no child mixed or solid.
        {nodes + 1, 0xFF, mnav_tileFlightNodes, 0},
        {nodes, 0, mnav_tileFlightNodes, 0},
        // A leaf of no solid voxel.
        {leaves, 0, mnav_tileFlightLeaves, 0},
    };
    int32_t wrong = 0;
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k)
    {
        memcpy(s_bad, s_bytes[1], size);
        s_bad[cases[k].at] = cases[k].value;
        if (k == 4)
        {
            s_bad[nodes + 1] = 0;
        }
        if (k == 5)
        {
            memset(s_bad + leaves, 0, 8);
        }
        Rehash(size);
        mnavTileResult got = StageBad(size);
        bool right = got.result == mnav_errorInvalid && got.section == cases[k].section &&
                     got.index == cases[k].index;
        printf("case %zu: section %d, element %d\n", k, got.section, got.index);
        wrong += right ? 0 : 1;
    }
    // Every leaf solid.
    memcpy(s_bad, s_bytes[1], size);
    memset(s_bad + leaves, 0xFF, 8);
    Rehash(size);
    mnavTileResult got = StageBad(size);
    wrong += got.section == mnav_tileFlightLeaves && got.index == 0 ? 0 : 1;
    // A node of no mixed child and every child solid.
    memcpy(s_bad, s_bytes[1], size);
    s_bad[nodes] = 0;
    s_bad[nodes + 1] = 0xFF;
    Rehash(size);
    got = StageBad(size);
    wrong += got.section == mnav_tileFlightNodes && got.index == 0 ? 0 : 1;
    // A spare leaf: one more declared and stored than the octree uses.
    memcpy(s_bad, s_bytes[1], size);
    mnavByteReader leafCount = {s_bytes[1] + AT_NODES + 4, 4, false};
    mnavByteWriter w = {s_bad + AT_NODES + 4};
    mnavPutU32(&w, mnavGetU32(&leafCount) + 1);
    w.at = s_bad + 32;
    mnavPutU32(&w, size + 8 - MNAV_FLIGHT_HEADER_BYTES);
    w.at = s_bad + size;
    mnavPutU64(&w, 1);
    Rehash(size + 8);
    got = StageBad(size + 8);
    wrong += got.section == mnav_tileFlightLeaves && got.index == -1 ? 0 : 1;
    CHECK(wrong == 0, "every broken structure refused in its section");
}

static void TestOtherSettingsAndLimits(void)
{
    const struct
    {
        int32_t change;
        mnavResult result;
    } cases[] = {{0, mnav_errorInvalid}, {1, mnav_errorInvalid}, {2, mnav_errorInvalid},
                 {3, mnav_errorInvalid}, {4, mnav_errorInvalid}, {5, mnav_errorInvalid},
                 {6, mnav_errorLimit},   {7, mnav_errorLimit}};
    int32_t wrong = 0;
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k)
    {
        mnavFlightDef def = Def();
        switch (cases[k].change)
        {
        case 0:
            def.radius = 1.0f;
            break;
        case 1:
            def.ceiling = 40.0f;
            break;
        case 2:
            def.groundBelow = false;
            break;
        case 3:
            def.origin.y = 0.25;
            break;
        case 4:
            def.voxelSize = 0.25f;
            def.tileVoxels = 128;
            break;
        case 5:
            def.floor = -1.0f;
            break;
        case 6:
            def.limits.tileNodes = 1;
            break;
        default:
            def.limits.tileLeaves = 1;
            break;
        }
        mnavFlightVolume* volume = Volume(&def);
        mnavTileResult got = mnavStageFlightTile(volume, s_bytes[1], s_sizes[1]);
        wrong += got.result == cases[k].result && got.section == mnav_tileHeader ? 0 : 1;
        mnavDestroyFlightVolume(volume);
    }
    CHECK(wrong == 0, "a tile of other settings or past the limits refused");
    memcpy(s_bad, s_bytes[1], s_sizes[1]);
    s_bad[4] = MNAV_FLIGHT_FORMAT + 1;
    CHECK(StageBad(s_sizes[1]).result == mnav_errorVersion, "another format version");
    s_bad[0] = 'X';
    CHECK(StageBad(s_sizes[1]).result == mnav_errorInvalid, "another magic");
}

// Staged twice at a place: the last one counts.
static void StageTwice(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavStageFlightTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavStageFlightTileRemoval(volume, 0, 0) == mnav_success &&
              mnavStageFlightTile(volume, s_bytes[1], s_sizes[1]).result == mnav_success &&
              mnavCommitFlight(volume) == mnav_success,
          "staged");
    CHECK(mnavGetFlightTile(volume, 0, 0, &f) == mnav_errorNotLoaded, "the removal staged last");
    CHECK(mnavGetFlightTile(volume, 1, 0, &f) == mnav_success && f == s_fingerprints[1],
          "the tile staged");
}

// Past the tiles limit of 2: nothing applies and the staged changes stay,
// so that with a removal staged too the commit fits.
static void PastTheTilesLimit(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavStageFlightTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavStageFlightTile(volume, s_bytes[2], s_sizes[2]).result == mnav_success,
          "two more");
    CHECK(mnavCommitFlight(volume) == mnav_errorLimit, "past the tiles limit");
    CHECK(mnavGetFlightTile(volume, 0, 0, &f) == mnav_errorNotLoaded &&
              mnavGetFlightTile(volume, 1, 0, &f) == mnav_success,
          "nothing applied");
    CHECK(mnavStageFlightTileRemoval(volume, 1, 0) == mnav_success &&
              mnavCommitFlight(volume) == mnav_success,
          "with a removal it fits");
    CHECK(mnavGetFlightTile(volume, 0, 0, &f) == mnav_success && f == s_fingerprints[0] &&
              mnavGetFlightTile(volume, 0, 1, &f) == mnav_success && f == s_fingerprints[2] &&
              mnavGetFlightTile(volume, 1, 0, &f) == mnav_errorNotLoaded,
          "all of it applied");
}

// A replacement, a removal of nothing, an empty commit, then every tile
// removed.
static void ReplaceAndEmpty(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavStageFlightTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavStageFlightTileRemoval(volume, 5, 5) == mnav_success &&
              mnavCommitFlight(volume) == mnav_success && mnavCommitFlight(volume) == mnav_success,
          "replaced");
    CHECK(mnavGetFlightTile(volume, 0, 0, &f) == mnav_success, "still there");
    CHECK(mnavStageFlightTileRemoval(volume, 0, 0) == mnav_success &&
              mnavStageFlightTileRemoval(volume, 0, 1) == mnav_success &&
              mnavCommitFlight(volume) == mnav_success,
          "everything removed");
    CHECK(mnavGetFlightTile(volume, 0, 1, &f) == mnav_errorNotLoaded &&
              mnavGetFlightTile(volume, 0, 0, &f) == mnav_errorNotLoaded,
          "empty");
}

static void TestStagingAndRemoval(void)
{
    mnavFlightDef def = Def();
    def.limits.tiles = 2;
    mnavFlightVolume* volume = Volume(&def);
    StageTwice(volume);
    PastTheTilesLimit(volume);
    ReplaceAndEmpty(volume);
    uint64_t f = 0;
    CHECK(mnavStageFlightTile(nullptr, s_bytes[0], s_sizes[0]).result == mnav_errorInvalid &&
              mnavStageFlightTile(volume, nullptr, 0).result == mnav_errorInvalid &&
              mnavStageFlightTileRemoval(nullptr, 0, 0) == mnav_errorInvalid &&
              mnavCommitFlight(nullptr) == mnav_errorInvalid &&
              mnavGetFlightTile(nullptr, 0, 0, &f) == mnav_errorInvalid &&
              mnavGetFlightTile(volume, 0, 0, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavDestroyFlightVolume(volume);
    mnavDestroyFlightVolume(nullptr);
}

// An allocator that refuses its s_failAt-th call from now, counting.
static int32_t s_failAt;

static void* Failing(size_t size, size_t alignment, void* context)
{
    if (--s_failAt == 0)
    {
        return nullptr;
    }
    return CountingAlloc(size, alignment, context);
}

// One run of staging and commits on an allocator that refuses its
// failAt-th call; returns whether every call succeeded.
static bool RunFailing(int32_t failAt)
{
    mnavFlightDef def = Def();
    def.allocator = (mnavAllocator){Failing, CountingFree, nullptr};
    s_failAt = failAt;
    mnavFlightVolume* volume = nullptr;
    if (mnavCreateFlightVolume(&def, &volume).result != mnav_success)
    {
        return false;
    }
    bool all = true;
    uint64_t f = 0;
    for (int32_t round = 0; round < 2; ++round)
    {
        bool staged = true;
        for (int32_t t = 0; t < 3; ++t)
        {
            staged = mnavStageFlightTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
            all &= staged;
        }
        all &= mnavStageFlightTileRemoval(volume, 1, 1) == mnav_success;
        bool before = mnavGetFlightTile(volume, 0, 1, &f) == mnav_success;
        mnavResult committed = mnavCommitFlight(volume);
        all &= committed == mnav_success;
        bool after = mnavGetFlightTile(volume, 0, 1, &f) == mnav_success;
        // A failed commit changes nothing; a successful one installs the
        // last tile if it was staged.
        CHECK(after == (committed == mnav_success ? staged || before : before), "all or nothing");
    }
    mnavDestroyFlightVolume(volume);
    CHECK(s_held == 0, "every byte given back after a refused allocation");
    return all;
}

static void TestRefusedMemory(void)
{
    int32_t failAt = 1;
    while (!RunFailing(failAt) && failAt < 200)
    {
        ++failAt;
    }
    printf("every allocation refused in turn: %d runs\n", failAt);
    CHECK(failAt > 3 && failAt < 200, "a run with no refusal");
}

static void TestDefIsChecked(void)
{
    const mnavFlightDef good = mnavDefaultFlightDef();
    CHECK(mnavValidateFlightDef(&good).result == mnav_success, "the default");
    CHECK(mnavValidateFlightDef(nullptr).result == mnav_errorInvalid, "NULL");
    int32_t wrong = 0;
    for (int32_t k = 0; k < 26; ++k)
    {
        mnavFlightDef d = good;
        mnavFlightSetting expect = mnav_flightSettingNone;
        switch (k)
        {
        case 0:
            d.cookie = 0;
            expect = mnav_flightSettingCookie;
            break;
        case 1:
            d.allocator.alloc = CountingAlloc;
            expect = mnav_flightSettingAllocator;
            break;
        case 2:
            d.origin.x = (double)NAN;
            expect = mnav_flightSettingOrigin;
            break;
        case 3:
            d.voxelSize = 0.0f;
            expect = mnav_flightSettingVoxelSize;
            break;
        case 4:
            d.voxelSize = 11.0f;
            expect = mnav_flightSettingVoxelSize;
            break;
        case 5:
            d.voxelSize = NAN;
            expect = mnav_flightSettingVoxelSize;
            break;
        case 6:
            d.tileVoxels = 8;
            expect = mnav_flightSettingTileVoxels;
            break;
        case 7:
            d.tileVoxels = 48;
            expect = mnav_flightSettingTileVoxels;
            break;
        case 8:
            d.tileVoxels = 1024;
            expect = mnav_flightSettingTileVoxels;
            break;
        case 9:
            d.floor = NAN;
            expect = mnav_flightSettingFloor;
            break;
        case 10:
            d.floor = -40000.0f;
            expect = mnav_flightSettingFloor;
            break;
        case 11:
            d.ceiling = d.floor;
            expect = mnav_flightSettingCeiling;
            break;
        case 12:
            d.ceiling = INFINITY;
            expect = mnav_flightSettingCeiling;
            break;
        case 13:
            d.ceiling = 32760.0f;
            expect = mnav_flightSettingCeiling;
            break;
        case 14:
            d.radius = -0.5f;
            expect = mnav_flightSettingRadius;
            break;
        case 15:
            d.radius = NAN;
            expect = mnav_flightSettingRadius;
            break;
        case 16:
            d.radius = 30.0f;
            expect = mnav_flightSettingRadius;
            break;
        case 17:
            d.limits.inputTriangles = 0;
            expect = mnav_flightSettingInputTriangles;
            break;
        case 18:
            d.limits.tileSpans = MNAV_MAX_TILE_SPANS + 1;
            expect = mnav_flightSettingTileSpans;
            break;
        case 19:
            d.limits.tileNodes = 0;
            expect = mnav_flightSettingTileNodes;
            break;
        case 20:
            d.limits.tileNodes = MNAV_MAX_FLIGHT_TILE_NODES + 1;
            expect = mnav_flightSettingTileNodes;
            break;
        case 21:
            d.limits.tileLeaves = -1;
            expect = mnav_flightSettingTileLeaves;
            break;
        case 22:
            d.limits.tiles = 0;
            expect = mnav_flightSettingTiles;
            break;
        case 23:
            d.limits.memoryBytes = 0;
            expect = mnav_flightSettingMemoryBytes;
            break;
        case 24:
            d.radius = 29.0f;
            break;
        default:
            d.floor = -32767.0f;
            d.ceiling = -32700.0f;
            break;
        }
        mnavFlightDefResult got = mnavValidateFlightDef(&d);
        mnavResult want = expect == mnav_flightSettingNone ? mnav_success : mnav_errorInvalid;
        if (got.result != want || got.setting != expect)
        {
            printf("def case %d: %d, setting %d\n", k, got.result, got.setting);
            ++wrong;
        }
    }
    CHECK(wrong == 0, "every setting checked");
    mnavFlightBaker* baker = nullptr;
    mnavFlightVolume* volume = nullptr;
    mnavFlightDef bad = good;
    bad.cookie = 1;
    CHECK(mnavCreateFlightBaker(&bad, &baker).setting == mnav_flightSettingCookie &&
              baker == nullptr &&
              mnavCreateFlightVolume(&bad, &volume).result == mnav_errorInvalid &&
              volume == nullptr,
          "a bad def makes nothing");
    CHECK(mnavCreateFlightBaker(&good, nullptr).result == mnav_errorInvalid &&
              mnavCreateFlightVolume(&good, nullptr).result == mnav_errorInvalid,
          "NULL outputs");
}

// Copying with no tile, into too little memory and with NULL arguments.
static void CopyRefusals(mnavFlightBaker* baker, const mnavBakeInput* input)
{
    size_t size = 7;
    CHECK(mnavCopyFlightTile(baker, nullptr, 0, &size) == mnav_errorInvalid && size == 7,
          "no tile yet");
    CHECK(mnavBakeFlightTile(baker, input, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(mnavCopyFlightTile(baker, nullptr, 0, &size) == mnav_errorCapacity && size == s_sizes[0],
          "the size asked for");
    uint8_t small[16];
    CHECK(mnavCopyFlightTile(baker, small, sizeof(small), nullptr) == mnav_errorCapacity,
          "too small");
    CHECK(mnavCopyFlightTile(baker, nullptr, 4, &size) == mnav_errorInvalid &&
              mnavCopyFlightTile(nullptr, small, 4, &size) == mnav_errorInvalid,
          "NULL arguments");
}

// Input refused: a vertex not a number, the mesh and its check named;
// missing arrays; a tile past the extent.
static void InputRefusals(mnavFlightBaker* baker, const mnavBakeInput* input)
{
    mnavVec3 vertices[3] = {{0, 0, 0}, {1, NAN, 0}, {0, 0, 1}};
    int32_t indices[3] = {0, 1, 2};
    mnavTriangleMesh meshes[2] = {input->meshes[0], {vertices, 3, indices, 1, nullptr}};
    mnavBakeInput two = {meshes, 2, nullptr, 0, nullptr, 0, nullptr};
    mnavFlightBakeReport report;
    CHECK(mnavBakeFlightTile(baker, &two, 0, 0, &report) == mnav_errorInvalid &&
              report.result == mnav_errorInvalid && report.mesh == 1 &&
              report.input.result == mnav_errorInvalid,
          "a mesh refused");
    size_t size = 0;
    CHECK(mnavCopyFlightTile(baker, nullptr, 0, &size) == mnav_errorInvalid,
          "a failed bake holds no tile");
    mnavBakeInput broken = {nullptr, 1, nullptr, 0, nullptr, 0, nullptr};
    CHECK(mnavBakeFlightTile(baker, &broken, 0, 0, &report) == mnav_errorInvalid &&
              mnavBakeFlightTile(baker, nullptr, 0, 0, &report) == mnav_errorInvalid &&
              mnavBakeFlightTile(nullptr, input, 0, 0, &report) == mnav_errorInvalid,
          "NULL input");
    CHECK(mnavBakeFlightTile(baker, input, 1 << 20, 0, &report) == mnav_errorRange,
          "a tile past the extent");
}

// A bake of tile (0, 0) with a def.
static mnavResult BakeLimited(mnavFlightDef def, const mnavBakeInput* input)
{
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "a small baker");
    mnavResult result = mnavBakeFlightTile(baker, input, 0, 0, nullptr);
    mnavDestroyFlightBaker(baker);
    return result;
}

static void TestBakerRefusals(void)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    CopyRefusals(baker, &input);
    InputRefusals(baker, &input);
    mnavDestroyFlightBaker(baker);
    mnavDestroyFlightBaker(nullptr);
    mnavFlightDef leaves = def;
    leaves.limits.tileLeaves = 4;
    mnavFlightDef nodes = def;
    nodes.limits.tileNodes = 4;
    mnavFlightDef triangles = def;
    triangles.limits.inputTriangles = 3;
    mnavFlightDef memory = def;
    memory.limits.memoryBytes = 65536;
    CHECK(BakeLimited(leaves, &input) == mnav_errorLimit &&
              BakeLimited(nodes, &input) == mnav_errorLimit &&
              BakeLimited(triangles, &input) == mnav_errorLimit &&
              BakeLimited(memory, &input) == mnav_errorLimit,
          "past each limit");
}

int main(void)
{
    TestDefIsChecked();
    BakeAll();
    TestBakeStageAndQuery();
    TestTruncatedAndFlippedBytes();
    TestPayloadStructure();
    TestOtherSettingsAndLimits();
    TestStagingAndRemoval();
    TestRefusedMemory();
    TestBakerRefusals();
    CHECK(s_held == 0, "every byte given back");
    return s_failures == 0 ? 0 : 1;
}
