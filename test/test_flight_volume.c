// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flight volumes through the public API: the def's checks; tiles baked,
// copied, staged and committed; the bytes pinned, with no padding of the
// def or the input in them; every truncated or corrupted tile refused,
// naming its section; tiles of other settings refused; staging, removal,
// replacement and a refused commit that changes nothing; every byte given
// back. Every voxel against the octree, the walk and the payload's
// structure are test_flight_octree.c's.

#include "counting_allocator.h"
#include "flight_scene.h"
#include "float_hosts.h"
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
#define FLIGHT_BYTES_HASH 0xb591d2b1cd797091ull

// The test world, made beforehand, for WorldHash.
static mnavTriangleMesh s_world;

// The test world's four tiles baked by a baker of their own: the hash of
// their bytes' hashes.
static uint64_t WorldHash(void)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    mnavBakeInput input = {&s_world, 1, nullptr, 0, nullptr, 0, nullptr};
    static uint8_t bytes[TILE_MOST];
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        size_t size = 0;
        CHECK(mnavFlightBaker_BakeTile(baker, &input, t % 2, t / 2, nullptr) == mnav_success &&
                  mnavFlightBaker_CopyTile(baker, bytes, TILE_MOST, &size) == mnav_success,
              "baked");
        uint64_t one = mnavHash64(MNAV_HASH_INIT, bytes, (int32_t)size);
        hash = mnavHash64(hash, &one, (int32_t)sizeof(one));
    }
    mnavDestroyFlightBaker(baker);
    return hash;
}

// Bakes the test world's four tiles into s_bytes, and one again.
static void BakeAll(void)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    s_world = world;
    mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        uint64_t one = BakeOne(baker, &input, t);
        hash = mnavHash64(hash, &one, (int32_t)sizeof(one));
    }
    printf("FLIGHT_BYTES_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FLIGHT_BYTES_HASH, "the pinned bytes");
    CHECK(EveryFloatHost(WorldHash, FLIGHT_BYTES_HASH),
          "the pinned bytes under every host setting");
    // The same bake makes the same bytes.
    CHECK(mnavFlightBaker_BakeTile(baker, &input, 0, 0, nullptr) == mnav_success, "again");
    size_t size = 0;
    static uint8_t again[TILE_MOST];
    CHECK(mnavFlightBaker_CopyTile(baker, again, TILE_MOST, &size) == mnav_success, "copied again");
    CHECK(size == s_sizes[0] && memcmp(again, s_bytes[0], size) == 0, "the same bytes");
    mnavDestroyFlightBaker(baker);
}

// The hash of every voxel's openness across the test world's four
// tiles, the same on every platform.
#define FLIGHT_OPEN_HASH 0x5dd49b48fe158415ull

// Every voxel's openness across the four committed tiles, from the floor
// at -2 m to the top of the one cube of 64 voxels, as one hash.
static uint64_t OpenHash(const mnavFlightVolume* volume, int32_t* openOut)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t z = 0; z < 128; ++z)
    {
        for (int32_t y = 0; y < 64; ++y)
        {
            uint8_t row[128];
            for (int32_t x = 0; x < 128; ++x)
            {
                mnavPos3 p = {(x + 0.5) * 0.5, -2.0 + (y + 0.5) * 0.5, (z + 0.5) * 0.5};
                bool open = false;
                CHECK(mnavFlightVolume_IsOpen(volume, p, &open) == mnav_success, "answered");
                row[x] = open ? 1 : 0;
                *openOut += open ? 1 : 0;
            }
            hash = mnavHash64(hash, row, (int32_t)sizeof(row));
        }
    }
    return hash;
}

// Points past the committed tiles, and past the volume's height.
static void CheckOutside(const mnavFlightVolume* volume)
{
    bool open = true;
    uint64_t fingerprint = 1;
    CHECK(mnavFlightVolume_GetTile(volume, 2, 0, &fingerprint) == mnav_errorNotLoaded &&
              fingerprint == 0,
          "no tile there");
    const mnavPos3 away[4] = {
        {70.0, 5.0, 1.0}, {-1.0, 5.0, 1.0}, {1.0, 5.0, -0.01}, {1.0, 5.0, 64.5}};
    int32_t wrong = 0;
    for (int32_t k = 0; k < 4; ++k)
    {
        wrong += mnavFlightVolume_IsOpen(volume, away[k], &open) == mnav_errorNotLoaded ? 0 : 1;
    }
    const mnavPos3 out[5] = {{1.0, -2.01, 1.0},
                             {1.0, 30.0, 1.0},
                             {(double)NAN, 5.0, 1.0},
                             {1.0, 5.0, 1e300},
                             {(double)INFINITY, 5.0, 1.0}};
    // Below the floor and above the ceiling are outside the volume, NaN is
    // wrong; past MNAV_MAX_FLIGHT_VOXELS, an infinity included, is a bound.
    const mnavResult refused[5] = {mnav_errorOutside, mnav_errorOutside, mnav_errorInvalid,
                                   mnav_errorLimit, mnav_errorLimit};
    for (int32_t k = 0; k < 5; ++k)
    {
        wrong += mnavFlightVolume_IsOpen(volume, out[k], &open) == refused[k] && !open ? 0 : 1;
    }
    CHECK(wrong == 0, "past the tiles, below, above or past the volume");
    CHECK(mnavFlightVolume_IsOpen(volume, (mnavPos3){1.0, 29.99, 1.0}, &open) == mnav_success,
          "the top of the last cube");
    CHECK(mnavFlightVolume_IsOpen(volume, (mnavPos3){1.0, -1.99, 1.0}, &open) == mnav_success &&
              !open,
          "under the ground");
}

static void TestBakeStageAndQuery(void)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    bool open = true;
    mnavPos3 inside = {1.0, 5.0, 1.0};
    CHECK(mnavFlightVolume_IsOpen(volume, inside, &open) == mnav_errorNotLoaded && !open,
          "nothing loaded");
    int32_t staged = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        staged += mnavFlightVolume_StageTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
    }
    CHECK(staged == 4, "staged");
    CHECK(mnavFlightVolume_IsOpen(volume, inside, &open) == mnav_errorNotLoaded,
          "not seen before the commit");
    CHECK(mnavFlightVolume_Commit(volume) == mnav_success, "committed");
    for (int32_t t = 0; t < 4; ++t)
    {
        uint64_t fingerprint = 0;
        CHECK(mnavFlightVolume_GetTile(volume, t % 2, t / 2, &fingerprint) == mnav_success &&
                  fingerprint == s_fingerprints[t],
              "the fingerprint");
    }
    int32_t openVoxels = 0;
    uint64_t hash = OpenHash(volume, &openVoxels);
    printf("FLIGHT_OPEN_HASH=%016llx open=%d\n", (unsigned long long)hash, openVoxels);
    CHECK(hash == FLIGHT_OPEN_HASH && openVoxels > 0, "every voxel's openness pinned");
    CheckOutside(volume);
    CHECK(mnavFlightVolume_IsOpen(nullptr, inside, &open) == mnav_errorInvalid &&
              mnavFlightVolume_IsOpen(volume, inside, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavDestroyFlightVolume(volume);
}

// Whether tile 1 with byte i flipped is refused as it should be: but in
// the generation and the fingerprint, which any value may hold; as
// another version in the version. The payload's hash covers the rest of
// the bytes, the place and the fields' tags among them.
static bool FlipRefused(size_t i, size_t size)
{
    memcpy(s_bad, s_bytes[1], size);
    s_bad[i] ^= 0x5A;
    bool free = (i >= AT_GENERATOR && i < AT_GENERATOR + 2) ||
                (i >= AT_FINGERPRINT && i < AT_FINGERPRINT + 8);
    bool version = i == 4 || i == 5;
    mnavTileResult r = StageBad(size);
    if (free)
    {
        return r.result == mnav_success;
    }
    return r.result == (version ? mnav_errorVersion : mnav_errorFormat);
}

static void TestTruncatedAndFlippedBytes(void)
{
    size_t size = s_sizes[1];
    int32_t wrong = 0;
    for (size_t n = 0; n < size; n += n < 256 ? 1 : 97)
    {
        memcpy(s_bad, s_bytes[1], n);
        wrong += StageBad(n).result != mnav_errorFormat ? 1 : 0;
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

static void TestOtherSettingsAndLimits(void)
{
    const struct
    {
        int32_t change;
        mnavResult result;
    } cases[] = {{0, mnav_errorFormat}, {1, mnav_errorFormat}, {2, mnav_errorFormat},
                 {3, mnav_errorFormat}, {4, mnav_errorFormat}, {5, mnav_errorFormat},
                 {6, mnav_errorFormat}, {7, mnav_errorFormat}};
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
            def.enableGroundFill = false;
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
        mnavTileResult got = mnavFlightVolume_StageTile(volume, s_bytes[1], s_sizes[1]);
        wrong += got.result == cases[k].result && got.section == mnav_tileHeader ? 0 : 1;
        mnavDestroyFlightVolume(volume);
    }
    CHECK(wrong == 0, "a tile of other settings or past the limits refused");
    memcpy(s_bad, s_bytes[1], s_sizes[1]);
    s_bad[4] = MNAV_FLIGHT_FORMAT + 1;
    CHECK(StageBad(s_sizes[1]).result == mnav_errorVersion, "another format version");
    s_bad[0] = 'X';
    CHECK(StageBad(s_sizes[1]).result == mnav_errorFormat, "another magic");
}

// Staged twice at a place: the last one counts.
static void StageTwice(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavFlightVolume_StageTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavFlightVolume_StageTileRemoval(volume, 0, 0) == mnav_success &&
              mnavFlightVolume_StageTile(volume, s_bytes[1], s_sizes[1]).result == mnav_success &&
              mnavFlightVolume_Commit(volume) == mnav_success,
          "staged");
    CHECK(mnavFlightVolume_GetTile(volume, 0, 0, &f) == mnav_errorNotLoaded,
          "the removal staged last");
    CHECK(mnavFlightVolume_GetTile(volume, 1, 0, &f) == mnav_success && f == s_fingerprints[1],
          "the tile staged");
}

// Past the tiles limit of 2: nothing applies and the staged changes stay,
// so that with a removal staged too the commit fits.
static void PastTheTilesLimit(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavFlightVolume_StageTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavFlightVolume_StageTile(volume, s_bytes[2], s_sizes[2]).result == mnav_success,
          "two more");
    CHECK(mnavFlightVolume_Commit(volume) == mnav_errorCapacity, "past the tiles limit");
    CHECK(mnavFlightVolume_GetTile(volume, 0, 0, &f) == mnav_errorNotLoaded &&
              mnavFlightVolume_GetTile(volume, 1, 0, &f) == mnav_success,
          "nothing applied");
    CHECK(mnavFlightVolume_StageTileRemoval(volume, 1, 0) == mnav_success &&
              mnavFlightVolume_Commit(volume) == mnav_success,
          "with a removal it fits");
    CHECK(mnavFlightVolume_GetTile(volume, 0, 0, &f) == mnav_success && f == s_fingerprints[0] &&
              mnavFlightVolume_GetTile(volume, 0, 1, &f) == mnav_success &&
              f == s_fingerprints[2] &&
              mnavFlightVolume_GetTile(volume, 1, 0, &f) == mnav_errorNotLoaded,
          "all of it applied");
}

// A replacement, a removal of nothing, an empty commit, then every tile
// removed.
static void ReplaceAndEmpty(mnavFlightVolume* volume)
{
    uint64_t f = 0;
    CHECK(mnavFlightVolume_StageTile(volume, s_bytes[0], s_sizes[0]).result == mnav_success &&
              mnavFlightVolume_StageTileRemoval(volume, 5, 5) == mnav_success &&
              mnavFlightVolume_Commit(volume) == mnav_success &&
              mnavFlightVolume_Commit(volume) == mnav_success,
          "replaced");
    CHECK(mnavFlightVolume_GetTile(volume, 0, 0, &f) == mnav_success, "still there");
    CHECK(mnavFlightVolume_StageTileRemoval(volume, 0, 0) == mnav_success &&
              mnavFlightVolume_StageTileRemoval(volume, 0, 1) == mnav_success &&
              mnavFlightVolume_Commit(volume) == mnav_success,
          "everything removed");
    CHECK(mnavFlightVolume_GetTile(volume, 0, 1, &f) == mnav_errorNotLoaded &&
              mnavFlightVolume_GetTile(volume, 0, 0, &f) == mnav_errorNotLoaded,
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
    CHECK(mnavFlightVolume_StageTile(nullptr, s_bytes[0], s_sizes[0]).result == mnav_errorInvalid &&
              mnavFlightVolume_StageTile(volume, nullptr, 0).result == mnav_errorInvalid &&
              mnavFlightVolume_StageTileRemoval(nullptr, 0, 0) == mnav_errorInvalid &&
              mnavFlightVolume_Commit(nullptr) == mnav_errorInvalid &&
              mnavFlightVolume_GetTile(nullptr, 0, 0, &f) == mnav_errorInvalid &&
              mnavFlightVolume_GetTile(volume, 0, 0, nullptr) == mnav_errorInvalid,
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
            staged =
                mnavFlightVolume_StageTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
            all &= staged;
        }
        all &= mnavFlightVolume_StageTileRemoval(volume, 1, 1) == mnav_success;
        bool before = mnavFlightVolume_GetTile(volume, 0, 1, &f) == mnav_success;
        mnavResult committed = mnavFlightVolume_Commit(volume);
        all &= committed == mnav_success;
        bool after = mnavFlightVolume_GetTile(volume, 0, 1, &f) == mnav_success;
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
        // Past a bound the library names, MNAV_MIN_ or MNAV_MAX_.
        bool bound = false;
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
            bound = true;
            break;
        case 4:
            d.voxelSize = 11.0f;
            expect = mnav_flightSettingVoxelSize;
            bound = true;
            break;
        case 5:
            d.voxelSize = NAN;
            expect = mnav_flightSettingVoxelSize;
            break;
        case 6:
            d.tileVoxels = 8;
            expect = mnav_flightSettingTileVoxels;
            bound = true;
            break;
        case 7:
            d.tileVoxels = 48;
            expect = mnav_flightSettingTileVoxels;
            break;
        case 8:
            d.tileVoxels = 1024;
            expect = mnav_flightSettingTileVoxels;
            bound = true;
            break;
        case 9:
            d.floor = NAN;
            expect = mnav_flightSettingFloor;
            break;
        case 10:
            d.floor = -40000.0f;
            expect = mnav_flightSettingFloor;
            bound = true;
            break;
        case 11:
            d.ceiling = d.floor;
            expect = mnav_flightSettingCeiling;
            break;
        case 12:
            d.ceiling = INFINITY;
            expect = mnav_flightSettingCeiling;
            bound = true;
            break;
        case 13:
            d.ceiling = 32760.0f;
            expect = mnav_flightSettingCeiling;
            bound = true;
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
            bound = true;
            break;
        case 19:
            d.limits.tileNodes = 0;
            expect = mnav_flightSettingTileNodes;
            break;
        case 20:
            d.limits.tileNodes = MNAV_MAX_FLIGHT_TILE_NODES + 1;
            expect = mnav_flightSettingTileNodes;
            bound = true;
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
        mnavResult want = expect == mnav_flightSettingNone ? mnav_success
                          : bound                          ? mnav_errorLimit
                                                           : mnav_errorInvalid;
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
    CHECK(mnavFlightBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorState && size == 0 &&
              mnavFlightBaker_GetMisuse(baker) == 0,
          "no tile yet: a state, not misuse, the size zeroed");
    CHECK(mnavFlightBaker_BakeTile(baker, input, 0, 0, nullptr) == mnav_success, "baked");
    CHECK(mnavFlightBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorCapacity &&
              size == s_sizes[0],
          "the size asked for");
    uint8_t small[16];
    CHECK(mnavFlightBaker_CopyTile(baker, small, sizeof(small), nullptr) == mnav_errorCapacity,
          "too small");
    CHECK(mnavFlightBaker_CopyTile(baker, nullptr, 4, &size) == mnav_errorInvalid &&
              mnavFlightBaker_CopyTile(nullptr, small, 4, &size) == mnav_errorInvalid,
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
    CHECK(mnavFlightBaker_BakeTile(baker, &two, 0, 0, &report) == mnav_errorInvalid &&
              report.result == mnav_errorInvalid && report.input.kind == mnav_inputMesh &&
              report.input.index == 1 && report.input.result == mnav_errorInvalid,
          "a mesh refused");
    size_t size = 0;
    CHECK(mnavFlightBaker_CopyTile(baker, nullptr, 0, &size) == mnav_errorState,
          "a failed bake holds no tile");
    mnavBakeInput broken = {nullptr, 1, nullptr, 0, nullptr, 0, nullptr};
    CHECK(mnavFlightBaker_BakeTile(baker, &broken, 0, 0, &report) == mnav_errorInvalid &&
              mnavFlightBaker_BakeTile(baker, nullptr, 0, 0, &report) == mnav_errorInvalid &&
              mnavFlightBaker_BakeTile(nullptr, input, 0, 0, &report) == mnav_errorInvalid,
          "NULL input");
    CHECK(mnavFlightBaker_BakeTile(baker, input, 1 << 20, 0, &report) == mnav_errorLimit,
          "a tile past the extent");
}

// Along each axis from tile (0, 0), the last tile within the input's
// extent, found by bisection: it bakes, and the next one is refused. Each
// bake's heightfield makes the same tile frame the baker checked first.
static void TilesToTheExtent(mnavFlightBaker* baker)
{
    const mnavBakeInput none = {nullptr, 0, nullptr, 0, nullptr, 0, nullptr};
    const int32_t dx[4] = {1, -1, 0, 0};
    const int32_t dz[4] = {0, 0, 1, -1};
    for (int32_t d = 0; d < 4; ++d)
    {
        int32_t in = 0;
        int32_t out = 1 << 20;
        CHECK(mnavFlightBaker_BakeTile(baker, &none, out * dx[d], out * dz[d], nullptr) ==
                  mnav_errorLimit,
              "far past the extent");
        while (out - in > 1)
        {
            int32_t mid = in + (out - in) / 2;
            mnavResult result =
                mnavFlightBaker_BakeTile(baker, &none, mid * dx[d], mid * dz[d], nullptr);
            CHECK(result == mnav_success || result == mnav_errorLimit, "baked or refused");
            in = result == mnav_success ? mid : in;
            out = result == mnav_success ? out : mid;
        }
        // 4,194,304 cells of extent and a tile's most, 1,024, in tiles of
        // 64 voxels less the border.
        CHECK(in == (dx[d] + dz[d] > 0 ? 65552 : 65551), "the last tile within the extent");
    }
}

// A bake of tile (0, 0) with a def.
static mnavResult BakeLimited(mnavFlightDef def, const mnavBakeInput* input)
{
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "a small baker");
    mnavResult result = mnavFlightBaker_BakeTile(baker, input, 0, 0, nullptr);
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
    TilesToTheExtent(baker);
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
    CHECK(BakeLimited(leaves, &input) == mnav_errorCapacity &&
              BakeLimited(nodes, &input) == mnav_errorCapacity &&
              BakeLimited(triangles, &input) == mnav_errorCapacity &&
              BakeLimited(memory, &input) == mnav_errorCapacity,
          "past each limit");
}

// A field of tile 1: its tag and payload.
typedef struct Piece
{
    char tag[4];
    const uint8_t* bytes;
    size_t size;
} Piece;

static void PutLittle32(uint8_t* at, size_t value)
{
    for (int32_t k = 0; k < 4; ++k)
    {
        at[k] = (uint8_t)(value >> (8 * k));
    }
}

// Writes tile 1's header and the pieces as fields into s_bad, each padded
// with pad bytes, seals it and stages it.
static mnavTileResult Fields(const Piece* pieces, int32_t count, uint8_t pad)
{
    memcpy(s_bad, s_bytes[1], FIELDS_START);
    PutLittle32(s_bad + AT_FIELDS, (size_t)count);
    size_t at = FIELDS_START;
    for (int32_t k = 0; k < count; ++k)
    {
        memcpy(s_bad + at, pieces[k].tag, 4);
        PutLittle32(s_bad + at + 4, pieces[k].size);
        memcpy(s_bad + at + 8, pieces[k].bytes, pieces[k].size);
        at += 8 + pieces[k].size;
        while (at % 8 != 0)
        {
            s_bad[at++] = pad;
        }
    }
    PutLittle32(s_bad + AT_PAYLOAD, at - FIELDS_START);
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, s_bad + FIELDS_START, (int32_t)(at - FIELDS_START));
    for (int32_t k = 0; k < 8; ++k)
    {
        s_bad[AT_HASH + k] = (uint8_t)(hash >> (8 * k));
    }
    return StageBad(at);
}

static bool Is(mnavTileResult r, mnavResult result, mnavTileSection section)
{
    return r.result == result && r.section == section;
}

static void TestFields(void)
{
    // Tile 1's fields, in order of their names.
    const char* tags[4] = {"FLHT", "LEAF", "NODE", "ROOT"};
    const mnavTileSection sections[4] = {mnav_tileHeader, mnav_tileFlightLeaves,
                                         mnav_tileFlightNodes, mnav_tileFlightRoots};
    const size_t records[4] = {56, 8, 2, 1};
    Piece pieces[5];
    for (int32_t k = 0; k < 4; ++k)
    {
        size_t length = 0;
        size_t at = FlightFieldAt(s_bytes[1], tags[k], &length);
        CHECK(at > 0, "tile 1 has every field");
        pieces[k] = (Piece){{0}, s_bytes[1] + at, length};
        memcpy(pieces[k].tag, tags[k], 4);
    }
    CHECK(Fields(pieces, 4, 0).result == mnav_success, "reassembled, it loads");
    // A tag a later version may write.
    const uint8_t later[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    pieces[4] = (Piece){{'Z', 'O', 'N', 'E'}, later, 8};
    mnavTileResult r = Fields(pieces, 5, 0);
    CHECK(r.result == mnav_errorVersion && r.section == mnav_tilePayload && r.index == 4,
          "an unknown tag");
    // No FLHT; ROOT before NODE.
    CHECK(Is(Fields(pieces + 1, 3, 0), mnav_errorFormat, mnav_tileHeader), "no FLHT");
    Piece swapped[4] = {pieces[0], pieces[1], pieces[3], pieces[2]};
    CHECK(Is(Fields(swapped, 4, 0), mnav_errorFormat, mnav_tileFlightNodes), "NODE after ROOT");
    // ROOT, a single byte, is padded: a padding byte not 0.
    CHECK(Is(Fields(pieces, 4, 1), mnav_errorFormat, mnav_tileFlightRoots), "a padding byte not 0");
    // Each field a byte short of whole records and a record short, and
    // present with no bytes; FLHT and ROOT, of fixed length, a record long.
    static uint8_t longer[TILE_MOST];
    for (int32_t k = 0; k < 4; ++k)
    {
        size_t whole = pieces[k].size;
        if (records[k] > 1)
        {
            pieces[k].size = whole - 1;
            CHECK(Is(Fields(pieces, 4, 0), mnav_errorFormat, sections[k]),
                  "a field a byte short of whole records");
        }
        pieces[k].size = whole - records[k];
        CHECK(Fields(pieces, 4, 0).result == mnav_errorFormat, "a field a record short");
        pieces[k].size = 0;
        CHECK(Is(Fields(pieces, 4, 0), mnav_errorFormat, sections[k]), "a field of no bytes");
        pieces[k].size = whole;
    }
    // Each field a byte long.
    for (int32_t k = 0; k < 4; ++k)
    {
        Piece whole = pieces[k];
        CHECK(whole.size < sizeof longer, "room for a byte more");
        memcpy(longer, whole.bytes, whole.size);
        longer[whole.size] = 0;
        pieces[k].bytes = longer;
        pieces[k].size = whole.size + 1;
        CHECK(Is(Fields(pieces, 4, 0), mnav_errorFormat, sections[k]), "a field a byte long");
        pieces[k] = whole;
    }
    for (int32_t k = 0; k < 4; k += 3)
    {
        Piece whole = pieces[k];
        memcpy(longer, whole.bytes, whole.size);
        memset(longer + whole.size, 0, records[k]);
        pieces[k].bytes = longer;
        pieces[k].size = whole.size + records[k];
        CHECK(Is(Fields(pieces, 4, 0), mnav_errorFormat, sections[k]),
              "a fixed field a record long");
        pieces[k] = whole;
    }
    // Every cube empty: no NODE or LEAF, which read as none.
    const uint8_t empty = 0;
    const Piece open[2] = {pieces[0], {{'R', 'O', 'O', 'T'}, &empty, 1}};
    CHECK(Fields(open, 2, 0).result == mnav_success, "open air loads without NODE or LEAF");
}

static void TestEmptyFieldsAreLeftOut(void)
{
    // A tile baked from nothing over no ground: every cube empty, so the
    // writer leaves NODE and LEAF out.
    mnavFlightDef def = Def();
    def.enableGroundFill = false;
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    const mnavBakeInput nothing = {nullptr, 0, nullptr, 0, nullptr, 0, nullptr};
    size_t size = 0;
    CHECK(mnavFlightBaker_BakeTile(baker, &nothing, 0, 0, nullptr) == mnav_success &&
              mnavFlightBaker_CopyTile(baker, s_bad, TILE_MOST, &size) == mnav_success,
          "baked");
    mnavDestroyFlightBaker(baker);
    size_t length = 0;
    CHECK(Little32(s_bad + AT_FIELDS) == 2 && FlightFieldAt(s_bad, "NODE", &length) == 0 &&
              FlightFieldAt(s_bad, "LEAF", &length) == 0,
          "FLHT and ROOT only");
    mnavFlightVolume* volume = Volume(&def);
    CHECK(mnavFlightVolume_StageTile(volume, s_bad, size).result == mnav_success, "it loads");
    mnavDestroyFlightVolume(volume);
}

// The test world's def over a byte pattern, every field then set one by
// one, so only its padding keeps the pattern.
static void DefOver(mnavFlightDef* def, int pattern)
{
    const mnavFlightDef d = Def();
    memset(def, pattern, sizeof(*def));
    def->cookie = d.cookie;
    def->version = d.version;
    def->allocator.alloc = d.allocator.alloc;
    def->allocator.free = d.allocator.free;
    def->allocator.context = d.allocator.context;
    def->origin.x = d.origin.x;
    def->origin.y = d.origin.y;
    def->origin.z = d.origin.z;
    def->voxelSize = d.voxelSize;
    def->tileVoxels = d.tileVoxels;
    def->floor = d.floor;
    def->ceiling = d.ceiling;
    def->radius = d.radius;
    def->enableGroundFill = d.enableGroundFill;
    def->limits.inputTriangles = d.limits.inputTriangles;
    def->limits.tileSpans = d.limits.tileSpans;
    def->limits.tileNodes = d.limits.tileNodes;
    def->limits.tileLeaves = d.limits.tileLeaves;
    def->limits.tiles = d.limits.tiles;
    def->limits.memoryBytes = d.limits.memoryBytes;
}

// Flight tile bytes are built field by field: the padding of the def and
// the input never reaches them. Tile 0 baked from structs over two byte
// patterns gives the bytes BakeAll pinned.
static void TestPaddingNeverReachesTheTile(void)
{
    static uint8_t bytes[TILE_MOST];
    for (int pattern = 0x00; pattern <= 0xFF; pattern += 0xFF)
    {
        mnavFlightDef def;
        DefOver(&def, pattern);
        mnavFlightBaker* baker = nullptr;
        CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
        mnavTriangleMesh mesh;
        memset(&mesh, pattern, sizeof(mesh));
        mesh.vertices = s_world.vertices;
        mesh.vertexCount = s_world.vertexCount;
        mesh.indices = s_world.indices;
        mesh.triangleCount = s_world.triangleCount;
        mesh.areas = s_world.areas;
        mnavBakeInput input;
        memset(&input, pattern, sizeof(input));
        input.meshes = &mesh;
        input.meshCount = 1;
        input.terrains = nullptr;
        input.terrainCount = 0;
        input.volumes = nullptr;
        input.volumeCount = 0;
        input.index = nullptr;
        size_t size = 0;
        CHECK(mnavFlightBaker_BakeTile(baker, &input, 0, 0, nullptr) == mnav_success &&
                  mnavFlightBaker_CopyTile(baker, bytes, TILE_MOST, &size) == mnav_success,
              "baked and copied");
        CHECK(size == s_sizes[0] && memcmp(bytes, s_bytes[0], size) == 0,
              "the pinned bytes over the pattern");
        mnavDestroyFlightBaker(baker);
    }
}

int main(void)
{
    TestDefIsChecked();
    BakeAll();
    TestBakeStageAndQuery();
    TestTruncatedAndFlippedBytes();
    TestFields();
    TestEmptyFieldsAreLeftOut();
    TestOtherSettingsAndLimits();
    TestStagingAndRemoval();
    TestRefusedMemory();
    TestBakerRefusals();
    TestPaddingNeverReachesTheTile();
    CHECK(s_held == 0, "every byte given back");
    return s_failures == 0 ? 0 : 1;
}
