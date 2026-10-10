// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Tiles 1.0.0 wrote, loaded by this version (mnav-0003, mnav-0015): every
// 1.x reads tile format 1 and flight tile format 1. The tiles load, commit
// and answer queries; the answers are pinned, and move only with a line
// under Changed or Fixed, as any result does. A failure to load is never
// fixed by rebaking the stored tiles.

#include "stored_1_0_0.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// The hash of every stored byte, so that the data is never changed by
// accident.
#define STORED_HASH 0x3f5acc6411547c0full

// What queries on the stored tiles answer.
#define GROUND_PATH_HASH  0xc2eb19d847441002ull
#define TERRAIN_PATH_HASH 0xd85379505d6a2ccaull
#define FLIGHT_PATH_HASH  0xf43eb1b51126ae6bull

static const uint8_t* const s_grounds[4] = {s_ground0, s_ground1, s_ground2, s_ground3};
static const size_t s_groundSizes[4] = {sizeof(s_ground0), sizeof(s_ground1), sizeof(s_ground2),
                                        sizeof(s_ground3)};
static const uint8_t* const s_flights[4] = {s_flight0, s_flight1, s_flight2, s_flight3};
static const size_t s_flightSizes[4] = {sizeof(s_flight0), sizeof(s_flight1), sizeof(s_flight2),
                                        sizeof(s_flight3)};

// The def the ground tiles were baked with, written out, so that a later
// default does not change what the test loads.
static mnavBakeDef GroundDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.cellSize = 0.25f;
    def.cellHeight = 0.125f;
    def.tileCells = 128;
    def.agent.radius = 0.5f;
    def.agent.height = 2.0f;
    def.agent.stepHeight = 0.75f;
    def.agent.maxSlopeDegrees = 45.0f;
    return def;
}

static mnavFlightDef FlightDef(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.voxelSize = 0.5f;
    def.tileVoxels = 64;
    def.floor = -2.0f;
    def.ceiling = 20.0f;
    def.radius = 0.5f;
    def.groundBelow = true;
    return def;
}

static uint64_t HashPoints(uint64_t hash, const mnavPos3* points, int32_t count)
{
    return mnavHash64(hash, points, (int32_t)((size_t)count * sizeof(mnavPos3)));
}

static void TestBytes(void)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        hash = mnavHash64(hash, s_grounds[t], (int32_t)s_groundSizes[t]);
        hash = mnavHash64(hash, s_flights[t], (int32_t)s_flightSizes[t]);
    }
    hash = mnavHash64(hash, s_terrain, (int32_t)sizeof(s_terrain));
    printf("STORED_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == STORED_HASH, "the stored bytes");
}

// Finds a path between two points on a navmesh; returns its hash, 0 when
// none was found.
static uint64_t Path(const mnavNavmesh* navmesh, mnavPos3 from, mnavPos3 to)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "a query");
    const mnavVec3 box = {2.0f, 4.0f, 2.0f};
    mnavNearest start;
    mnavNearest end;
    mnavPath path;
    uint64_t hash = 0;
    if (mnavFindNearest(navmesh, nullptr, from, box, &start) == mnav_success &&
        mnavFindNearest(navmesh, nullptr, to, box, &end) == mnav_success &&
        mnavFindPath(query, navmesh, nullptr, start.polygon, start.point, end.polygon, end.point,
                     &path) == mnav_success &&
        path.end == mnav_pathFound)
    {
        hash = HashPoints(MNAV_HASH_INIT, path.points, path.pointCount);
        hash = mnavHash64(hash, &path.length, (int32_t)sizeof(path.length));
        printf("a path of %d polygons and %d points, %.6f m\n", path.polygonCount, path.pointCount,
               path.length);
    }
    mnavDestroyQuery(query);
    return hash;
}

static void TestGround(void)
{
    mnavBakeDef def = GroundDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_grounds[t], s_groundSizes[t]).result == mnav_success,
              "a ground tile of 1.0.0 staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    uint64_t hash = Path(navmesh, (mnavPos3){2.0, 0.0, 2.0}, (mnavPos3){62.0, 0.0, 62.0});
    printf("GROUND_PATH_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == GROUND_PATH_HASH, "the path across the four tiles");
    mnavDestroyNavmesh(navmesh);
}

static void TestTerrain(void)
{
    mnavBakeDef def = GroundDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    CHECK(mnavStageTile(navmesh, s_terrain, sizeof(s_terrain)).result == mnav_success,
          "the terrain tile of 1.0.0 staged");
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    // Up the slope, round the plateau.
    uint64_t hash = Path(navmesh, (mnavPos3){1.0, 1.5, 12.0}, (mnavPos3){30.0, 5.0, 12.0});
    mnavNearest on;
    const mnavVec3 box = {1.0f, 4.0f, 1.0f};
    double height = 0.0;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){10.0, 3.0, 25.0}, box, &on) ==
                  mnav_success &&
              mnavGetHeight(navmesh, on.polygon, 10.0, 25.0, &height) == mnav_success,
          "a height on the slope");
    printf("height at (10, 25): %.9f\n", height);
    hash = mnavHash64(hash, &height, (int32_t)sizeof(height));
    printf("TERRAIN_PATH_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == TERRAIN_PATH_HASH, "the path and the height on the terrain");
    mnavDestroyNavmesh(navmesh);
}

static void TestFlight(void)
{
    mnavFlightDef def = FlightDef();
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(&def, &volume).result == mnav_success, "a flight volume");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageFlightTile(volume, s_flights[t], s_flightSizes[t]).result == mnav_success,
              "a flight tile of 1.0.0 staged");
    }
    CHECK(mnavCommitFlight(volume) == mnav_success, "committed");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "a query");
    mnavFlightPath path;
    uint64_t hash = 0;
    if (mnavFindFlightPath(query, volume, (mnavPos3){2.0, 1.0, 2.0}, (mnavPos3){62.0, 1.0, 62.0},
                           &path) == mnav_success &&
        path.end == mnav_pathFound)
    {
        hash = HashPoints(MNAV_HASH_INIT, path.points, path.pointCount);
        hash = mnavHash64(hash, &path.length, (int32_t)sizeof(path.length));
        printf("a flight path of %d points, %.6f m\n", path.pointCount, path.length);
    }
    bool open = true;
    CHECK(mnavIsFlightOpen(volume, (mnavPos3){31.0, 0.5, 11.0}, &open) == mnav_success && !open,
          "inside a box, closed");
    printf("FLIGHT_PATH_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FLIGHT_PATH_HASH, "the flight path across the four tiles");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

int main(void)
{
    TestBytes();
    TestGround();
    TestTerrain();
    TestFlight();
    return s_failures == 0 ? 0 : 1;
}
