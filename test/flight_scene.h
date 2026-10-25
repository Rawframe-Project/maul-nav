// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The flight volume suites' scene: the test world's four flight tiles
// baked with one def, their bytes, sizes and fingerprints, and staging
// bad bytes on a fresh volume. Public calls only.

#ifndef MAUL_NAV_TEST_FLIGHT_SCENE_H
#define MAUL_NAV_TEST_FLIGHT_SCENE_H

#include "counting_allocator.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

static inline mnavFlightVolume* Volume(const mnavFlightDef* def)
{
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(def, &volume).result == mnav_success, "volume");
    return volume;
}

// Stages bad bytes on a fresh volume of the test def.
static inline mnavTileResult StageBad(size_t size)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    mnavTileResult result = mnavStageFlightTile(volume, s_bad, size);
    mnavDestroyFlightVolume(volume);
    return result;
}

#endif // MAUL_NAV_TEST_FLIGHT_SCENE_H
