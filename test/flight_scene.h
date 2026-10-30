// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The flight volume suites' scene: the test world's four flight tiles
// baked with one def, their bytes, sizes and fingerprints, and staging
// bad bytes on a fresh volume. Public calls only.

#ifndef MAUL_NAV_TEST_FLIGHT_SCENE_H
#define MAUL_NAV_TEST_FLIGHT_SCENE_H

#include "counting_allocator.h"
#include "defs_for.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Where the header keeps its values, and where the fields start.
enum
{
    AT_GENERATOR = 6,
    AT_FINGERPRINT = 8,
    AT_HASH = 16,
    AT_PAYLOAD = 24,
    AT_FIELDS = 28,
    FIELDS_START = 32,
    TILE_MOST = 1 << 20
};

static inline uint32_t Little32(const uint8_t* at)
{
    return (uint32_t)at[0] | (uint32_t)at[1] << 8 | (uint32_t)at[2] << 16 | (uint32_t)at[3] << 24;
}

// Where the field of a tag starts its payload in a well-formed tile, and
// its length; 0 for both when the tile has none.
static inline size_t FlightFieldAt(const uint8_t* bytes, const char tag[4], size_t* length)
{
    size_t at = FIELDS_START;
    for (uint32_t k = Little32(bytes + AT_FIELDS); k > 0; --k)
    {
        size_t size = Little32(bytes + at + 4);
        if (memcmp(bytes + at, tag, 4) == 0)
        {
            *length = size;
            return at + 8;
        }
        at += 8 + (size + 7) / 8 * 8;
    }
    *length = 0;
    return 0;
}

static uint8_t s_bytes[4][TILE_MOST];
static size_t s_sizes[4];
static uint64_t s_fingerprints[4];
static uint8_t s_bad[TILE_MOST];

// The def the tests bake with: 0.5 m voxels, tiles of 64 (32 m, as the
// test world's), from 2 m below the ground to 20 m up.
static inline mnavFlightDef Def(void)
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
static inline uint64_t BakeOne(mnavFlightBaker* baker, const mnavBakeInput* input, int32_t t)
{
    mnavFlightBakeReport report;
    CHECK(mnavFlightBaker_BakeTile(baker, input, t % 2, t / 2, &report) == mnav_success, "baked");
    CHECK(report.result == mnav_success && report.input.kind == mnav_inputNone &&
              report.triangleCount > 0 && report.spanCount > 0 && report.cubeCount == 1 &&
              report.nodeCount > 0 && report.leafCount > 0 && report.memoryPeak > 0,
          "the report");
    size_t size = 0;
    CHECK(mnavFlightBaker_CopyTile(baker, s_bytes[t], TILE_MOST, &size) == mnav_success, "copied");
    CHECK(size == report.tileBytes, "the size reported");
    s_sizes[t] = size;
    s_fingerprints[t] = report.fingerprint;
    printf("flight tile %d: %zu bytes, %d nodes, %d leaves\n", t, size, report.nodeCount,
           report.leafCount);
    return mnavHash64(MNAV_HASH_INIT, s_bytes[t], (int64_t)size);
}

static inline mnavFlightVolume* Volume(const mnavFlightDef* def)
{
    mnavFlightVolume* volume = nullptr;
    mnavFlightVolumeDef volumeDef = FlightVolumeDefFor(def);
    CHECK(mnavCreateFlightVolume(&volumeDef, &volume).result == mnav_success, "volume");
    return volume;
}

// Stages bad bytes on a fresh volume of the test def.
static inline mnavTileResult StageBad(size_t size)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    mnavTileResult result = mnavFlightVolume_StageTile(volume, s_bad, size);
    mnavDestroyFlightVolume(volume);
    return result;
}

#endif // MAUL_NAV_TEST_FLIGHT_SCENE_H
