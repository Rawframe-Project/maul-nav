// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Stored tiles. Tiles in the current formats (stored_tiles.h) load, commit
// and answer queries, and the answers are pinned. Tiles in an older format
// (stored_format1.h) are refused with mnav_errorVersion: a library reads
// only the formats it writes, and a host rebakes from source when the
// format rises.
//
// stored_tiles.h is written by `test_stored --write > test/stored_tiles.h`
// in the pull request that raises MNAV_TILE_FORMAT or MNAV_FLIGHT_FORMAT,
// and only then.

#include "stored_format1.h"
#include "stored_tiles.h"
#include "terrain.h"
#include "test_harness.h"
#include "trench.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The hash of every stored byte, so that the data is never changed by
// accident.
#define STORED_HASH 0x37a4d4d4d060c265ull

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

// The def the ground tiles are baked with, written out, so that a later
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

static void PrintBytes(const char* name, const uint8_t* bytes, size_t size)
{
    printf("[[maybe_unused]] static const uint8_t %s[%zu] = {\n", name, size);
    for (size_t i = 0; i < size; ++i)
    {
        printf("%s0x%02x,%s", i % 16 == 0 ? "    " : " ", bytes[i],
               i % 16 == 15 || i + 1 == size ? "\n" : "");
    }
    printf("};\n\n");
}

// Writes stored_tiles.h to stdout: test/world.h's four tiles, tile (0, 0)
// of test/terrain.h's terrain, test/trench.h's two tiles and the world's
// four flight tiles, baked with GroundDef and FlightDef.
static int Write(void)
{
    static uint8_t bytes[1 << 20];
    printf("// SPDX-License-Identifier: MIT\n"
           "// Copyright (c) 2026 Sirac Ozmen\n"
           "//\n"
           "// Tiles in tile format %d and flight tile format %d, written by\n"
           "// `test_stored --write`; see test_stored.c.\n"
           "//\n"
           "// - s_ground0 to s_ground3: test/world.h's four tiles, baked with\n"
           "//   test_stored.c's GroundDef.\n"
           "// - s_terrain: tile (0, 0) of test/terrain.h's slope with a plateau,\n"
           "//   baked with the same def.\n"
           "// - s_trench0 and s_trench1: test/trench.h's two tiles, baked with the\n"
           "//   same def.\n"
           "// - s_flight0 to s_flight3: the world's four flight tiles, baked with\n"
           "//   test_stored.c's FlightDef.\n"
           "\n"
           "#ifndef MAUL_NAV_TEST_STORED_TILES_H\n"
           "#define MAUL_NAV_TEST_STORED_TILES_H\n"
           "\n"
           "#include <stdint.h>\n"
           "\n",
           MNAV_TILE_FORMAT, MNAV_FLIGHT_FORMAT);
    mnavBakeDef def = GroundDef();
    mnavBaker* baker = nullptr;
    mnavFlightDef flightDef = FlightDef();
    mnavFlightBaker* flightBaker = nullptr;
    if (mnavCreateBaker(&def, &baker).result != mnav_success ||
        mnavCreateFlightBaker(&flightDef, &flightBaker).result != mnav_success)
    {
        return 1;
    }
    mnavTriangleMesh world = World();
    const mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    char name[16];
    size_t size = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        if (mnavBakeTile(baker, &world, 1, t & 1, t >> 1, nullptr) != mnav_success ||
            mnavCopyBakedTile(baker, bytes, sizeof(bytes), &size) != mnav_success)
        {
            return 1;
        }
        snprintf(name, sizeof(name), "s_ground%d", t);
        PrintBytes(name, bytes, size);
    }
    const mnavTerrain terrain = Terrain();
    const mnavBakeInput terrainInput = {nullptr, 0, &terrain, 1, nullptr, 0, nullptr};
    if (mnavBakeTileInput(baker, &terrainInput, 0, 0, nullptr) != mnav_success ||
        mnavCopyBakedTile(baker, bytes, sizeof(bytes), &size) != mnav_success)
    {
        return 1;
    }
    PrintBytes("s_terrain", bytes, size);
    const mnavTriangleMesh trench = Trench();
    for (int32_t t = 0; t < 2; ++t)
    {
        if (mnavBakeTile(baker, &trench, 1, t, 0, nullptr) != mnav_success ||
            mnavCopyBakedTile(baker, bytes, sizeof(bytes), &size) != mnav_success)
        {
            return 1;
        }
        snprintf(name, sizeof(name), "s_trench%d", t);
        PrintBytes(name, bytes, size);
    }
    for (int32_t t = 0; t < 4; ++t)
    {
        if (mnavBakeFlightTile(flightBaker, &input, t & 1, t >> 1, nullptr) != mnav_success ||
            mnavCopyFlightTile(flightBaker, bytes, sizeof(bytes), &size) != mnav_success)
        {
            return 1;
        }
        snprintf(name, sizeof(name), "s_flight%d", t);
        PrintBytes(name, bytes, size);
    }
    printf("#endif // MAUL_NAV_TEST_STORED_TILES_H\n");
    mnavDestroyFlightBaker(flightBaker);
    mnavDestroyBaker(baker);
    return 0;
}

static uint64_t HashPoints(uint64_t hash, const mnavPos3* points, int32_t count)
{
    return mnavHash64(hash, points, (int32_t)((size_t)count * sizeof(mnavPos3)));
}

static void TestFormat1(void)
{
    const uint8_t* const grounds[2] = {s_ground100, s_ground101};
    const size_t groundSizes[2] = {sizeof(s_ground100), sizeof(s_ground101)};
    const uint8_t* const flights[2] = {s_flight100, s_flight101};
    const size_t flightSizes[2] = {sizeof(s_flight100), sizeof(s_flight101)};
    mnavBakeDef def = GroundDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    mnavFlightDef flightDef = FlightDef();
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(&flightDef, &volume).result == mnav_success, "a flight volume");
    for (int32_t k = 0; k < 2; ++k)
    {
        mnavTileResult got = mnavStageTile(navmesh, grounds[k], groundSizes[k]);
        CHECK(got.result == mnav_errorVersion && got.section == mnav_tileHeader,
              "a format 1 tile refused as another version");
        got = mnavStageFlightTile(volume, flights[k], flightSizes[k]);
        CHECK(got.result == mnav_errorVersion && got.section == mnav_tileHeader,
              "a format 1 flight tile refused as another version");
    }
    mnavDestroyFlightVolume(volume);
    mnavDestroyNavmesh(navmesh);
}

static void TestBytes(void)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(s_grounds[t][4] == MNAV_TILE_FORMAT && s_flights[t][4] == MNAV_FLIGHT_FORMAT,
              "stored in the current formats");
        hash = mnavHash64(hash, s_grounds[t], (int32_t)s_groundSizes[t]);
        hash = mnavHash64(hash, s_flights[t], (int32_t)s_flightSizes[t]);
    }
    CHECK(s_terrain[4] == MNAV_TILE_FORMAT, "the terrain tile in the current format");
    hash = mnavHash64(hash, s_terrain, (int32_t)sizeof(s_terrain));
    CHECK(s_trench0[4] == MNAV_TILE_FORMAT && s_trench1[4] == MNAV_TILE_FORMAT,
          "the trench tiles in the current format");
    hash = mnavHash64(hash, s_trench0, (int32_t)sizeof(s_trench0));
    hash = mnavHash64(hash, s_trench1, (int32_t)sizeof(s_trench1));
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
              "a stored ground tile staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    uint64_t hash = Path(navmesh, (mnavPos3){2.0, 0.0, 2.0}, (mnavPos3){62.0, 0.0, 62.0});
    printf("GROUND_PATH_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == GROUND_PATH_HASH, "the path across the four tiles");
    mnavDestroyNavmesh(navmesh);
}

// A path up the slope and round the plateau, and a height on the slope,
// on the stored terrain tile.
static void TestTerrain(void)
{
    mnavBakeDef def = GroundDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    CHECK(mnavStageTile(navmesh, s_terrain, sizeof(s_terrain)).result == mnav_success,
          "the stored terrain tile staged");
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
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
              "a stored flight tile staged");
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

int main(int argc, char** argv)
{
    if (argc == 2 && strcmp(argv[1], "--write") == 0)
    {
        return Write();
    }
    TestFormat1();
    TestBytes();
    TestGround();
    TestTerrain();
    TestFlight();
    return s_failures == 0 ? 0 : 1;
}
