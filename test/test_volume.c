// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bake volumes: include, exclude and area volumes, their heights, order
// and erosion, read cell by cell from the baked tile; the checks; the
// fingerprint; the pinned tile under every host floating-point setting.

#include "defs_for.h"
#include "float_hosts.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of a tile baked with every kind of volume, the same on every
// platform.
#define VOLUME_HASH 0x8a9824b224b7cae5ull

enum
{
    TILE_ROOM = 1 << 20
};

static uint8_t s_tile[TILE_ROOM];

// Ground at height 0 from -4 to 36 m, and a platform 3 m up over 20 to
// 28 m, both walkable, the gap under the platform tall enough to walk.
static const mnavVec3 s_vertices[8] = {{-4, 0, -4}, {36, 0, -4}, {36, 0, 36}, {-4, 0, 36},
                                       {20, 3, 20}, {28, 3, 20}, {28, 3, 28}, {20, 3, 28}};
static const int32_t s_indices[12] = {0, 3, 2, 0, 2, 1, 4, 7, 6, 4, 6, 5};
static const mnavTriangleMesh s_mesh = {s_vertices, 8, s_indices, 4, nullptr};

// The same ground with a slab 0.5 m up over 8 to 12 m, x and z, and a
// ceiling 2.25 m up over x = 4 to 8 m, z = 4 to 16 m, beside it. Under
// the ceiling and on the slab are walkable, but an agent 2 m tall cannot
// pass between them: 2.25 - 0.5 m is too low.
static const mnavVec3 s_lowVertices[12] = {
    {-4, 0, -4},    {36, 0, -4},   {36, 0, 36},   {-4, 0, 36},   {8, 0.5f, 8},   {12, 0.5f, 8},
    {12, 0.5f, 12}, {8, 0.5f, 12}, {4, 2.25f, 4}, {8, 2.25f, 4}, {8, 2.25f, 16}, {4, 2.25f, 16}};
static const int32_t s_lowIndices[18] = {0, 3, 2, 0, 2, 1, 4, 7, 6, 4, 6, 5, 8, 11, 10, 8, 10, 9};
static const mnavTriangleMesh s_low = {s_lowVertices, 12, s_lowIndices, 6, nullptr};

// A square ring from (x0, z0) to (x1, z1).
typedef struct Square
{
    mnavVec2 points[4];
} Square;

static Square MakeSquare(float x0, float z0, float x1, float z1)
{
    return (Square){{{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}}};
}

static mnavBakeVolume Volume(const Square* s, float minY, float maxY, mnavVolumeKind kind,
                             mnavAreaType area)
{
    return (mnavBakeVolume){s->points, 4, minY, maxY, kind, area};
}

// The tile's cells are 0.25 m from the origin, so a square with
// whole-meter corners holds 16 cells per meter squared: one from 22 to
// 26 m covers cells 88 to 103.
enum
{
    CELLS = 128,
    NO_POLYGON = 0xFF,
    GROUND = 0,
    PLATFORM = 1
};

// What the baked tile holds over each cell's center at the ground and at
// the platform: the area of the polygon there, or NO_POLYGON. Baked with
// no edge error, the polygons cover exactly the cells the volumes and
// erosion leave walkable.
typedef struct Map
{
    uint8_t at[2][CELLS][CELLS];
} Map;

static Map s_map;

// Bakes tile (0, 0) of a mesh with the volumes and reads its map back
// through a navmesh holding it.
static const Map* BakeMesh(mnavBaker* baker, const mnavTriangleMesh* mesh,
                           const mnavBakeVolume* volumes, int32_t count)
{
    const mnavBakeInput input = {mesh, 1, nullptr, 0, volumes, count, nullptr};
    size_t size = 0;
    CHECK(mnavBaker_BakeTile(baker, &input, 0, 0, nullptr) == mnav_success &&
              mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_tile, size).result == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "loaded");
    for (int32_t level = 0; level < 2; ++level)
    {
        for (int32_t z = 0; z < CELLS; ++z)
        {
            for (int32_t x = 0; x < CELLS; ++x)
            {
                mnavPos3 center = {0.125 + 0.25 * x, level == 0 ? 0.0 : 3.0, 0.125 + 0.25 * z};
                mnavNearest n;
                mnavAreaType area = NO_POLYGON;
                CHECK(mnavNavmesh_FindNearest(navmesh, center, (mnavVec3){0.01f, 1.0f, 0.01f},
                                              nullptr, &n) == mnav_success,
                      "nearest");
                if (n.polygon.slot != 0 && n.over)
                {
                    CHECK(mnavPolygon_GetArea(navmesh, n.polygon, &area) == mnav_success, "area");
                }
                s_map.at[level][z][x] = area;
            }
        }
    }
    mnavDestroyNavmesh(navmesh);
    return &s_map;
}

static const Map* Bake(mnavBaker* baker, const mnavBakeVolume* volumes, int32_t count)
{
    return BakeMesh(baker, &s_mesh, volumes, count);
}

// The cells of a level holding a value.
static int32_t Count(const Map* m, int32_t level, uint8_t value)
{
    int32_t n = 0;
    for (int32_t z = 0; z < CELLS; ++z)
    {
        for (int32_t x = 0; x < CELLS; ++x)
        {
            n += m->at[level][z][x] == value ? 1 : 0;
        }
    }
    return n;
}

// Whether every cell from (x0, z0) to (x1, z1) of a level holds a value.
static bool Cells(const Map* m, int32_t level, int32_t x0, int32_t z0, int32_t x1, int32_t z1,
                  uint8_t value)
{
    bool all = true;
    for (int32_t z = z0; z <= z1; ++z)
    {
        for (int32_t x = x0; x <= x1; ++x)
        {
            all = all && m->at[level][z][x] == value;
        }
    }
    return all;
}

// The platform's 32 cells a side lose their rim as a ledge and two more
// cells, the agent's radius, to erosion: cells 83 to 108 stay.
static void TestField(mnavBaker* cells)
{
    const Map* m = Bake(cells, nullptr, 0);
    CHECK(Count(m, GROUND, mnav_areaWalkable) == CELLS * CELLS,
          "the ground, wider than the tile, is walkable everywhere");
    CHECK(Count(m, PLATFORM, mnav_areaWalkable) == 26 * 26 &&
              Cells(m, PLATFORM, 83, 83, 108, 108, mnav_areaWalkable),
          "the platform less its rim and the radius");
}

// Carved before erosion, an excluded square of cells x0 to x1 and z0 to
// z1 widens by the radius, two cells, along each side; erosion's
// distances of 2 a straight step and 3 a diagonal one keep only the
// widened square's four corner cells. Whether the cells up to three
// past the square are just those.
static bool Hole(const Map* m, int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    bool exact = true;
    for (int32_t z = z0 - 3; z <= z1 + 3; ++z)
    {
        for (int32_t x = x0 - 3; x <= x1 + 3; ++x)
        {
            int32_t dx = x < x0 ? x0 - x : x > x1 ? x - x1 : 0;
            int32_t dz = z < z0 ? z0 - z : z > z1 ? z - z1 : 0;
            bool gone = dx <= 2 && dz <= 2 && !(dx == 2 && dz == 2);
            exact = exact && (m->at[GROUND][z][x] == NO_POLYGON) == gone;
        }
    }
    return exact;
}

static void TestKinds(mnavBaker* cells)
{
    // Excluded below the platform: the ground goes, the platform stays.
    Square under = MakeSquare(22, 22, 26, 26);
    mnavBakeVolume exclude = Volume(&under, -1.0f, 1.0f, mnav_volumeExclude, 0);
    const Map* m = Bake(cells, &exclude, 1);
    CHECK(Hole(m, 88, 88, 103, 103) && Count(m, GROUND, NO_POLYGON) == 20 * 20 - 4 &&
              Count(m, PLATFORM, mnav_areaWalkable) == 26 * 26,
          "an exclude volume takes the ground within its heights");
    // An area volume over the platform's heights alone.
    Square over = MakeSquare(20, 20, 24, 24);
    mnavBakeVolume high = Volume(&over, 2.0f, 4.0f, mnav_volumeArea, 7);
    m = Bake(cells, &high, 1);
    CHECK(Count(m, PLATFORM, 7) == 13 * 13 && Cells(m, PLATFORM, 83, 83, 95, 95, 7) &&
              Count(m, GROUND, 7) == 0,
          "an area volume marks the platform");
    // Two overlapping area volumes: the last wins; excluded cells stay out.
    Square a = MakeSquare(0, 0, 8, 8);
    Square b = MakeSquare(4, 0, 12, 8);
    Square c = MakeSquare(22, 22, 23, 23);
    const mnavBakeVolume areas[4] = {exclude, Volume(&a, -1, 1, mnav_volumeArea, 3),
                                     Volume(&b, -1, 1, mnav_volumeArea, 4),
                                     Volume(&c, -1, 1, mnav_volumeArea, 5)};
    m = Bake(cells, areas, 4);
    CHECK(Count(m, GROUND, 3) == 16 * 32 && Cells(m, GROUND, 0, 0, 15, 31, 3) &&
              Count(m, GROUND, 4) == 32 * 32 && Cells(m, GROUND, 16, 0, 47, 31, 4) &&
              Count(m, GROUND, 5) == 0 && Count(m, PLATFORM, 5) == 0,
          "the last volume wins; no walkable ground is made");
    // A ring of many points before a square: the scan has room for the
    // most points of any.
    mnavVec2 strip[24];
    for (int32_t k = 0; k < 24; ++k)
    {
        // A zigzag strip from x = 2 to 13, between z = 2 and 6.
        strip[k] = k < 12 ? (mnavVec2){2.0f + (float)k, k % 2 == 0 ? 2.0f : 3.0f}
                          : (mnavVec2){13.0f - (float)(k - 12), k % 2 == 0 ? 6.0f : 5.0f};
    }
    Square small = MakeSquare(20, 2, 24, 6);
    const mnavBakeVolume two[2] = {{strip, 24, -1, 1, mnav_volumeExclude, 0},
                                   Volume(&small, -1, 1, mnav_volumeExclude, 0)};
    m = Bake(cells, two, 2);
    // The strip lies within x = 2 to 13 and z = 2 to 6, cells 8 to 51
    // and 8 to 23, and holds all of z = 3 to 5, cells 12 to 19; each
    // widens by the radius.
    CHECK(Cells(m, GROUND, 6, 12, 53, 19, NO_POLYGON) &&
              Cells(m, GROUND, 8, 10, 51, 21, NO_POLYGON) && Hole(m, 80, 8, 95, 23) &&
              Count(m, GROUND, NO_POLYGON) <= 48 * 20 + 20 * 20 - 4,
          "a many-pointed ring, then a square");
    // Include volumes: ground outside every one goes, and erosion takes
    // the radius inside each.
    Square left = MakeSquare(0, 0, 4, 4);
    Square right = MakeSquare(8, 0, 12, 4);
    Square far = MakeSquare(500, 500, 504, 504);
    const mnavBakeVolume includes[3] = {Volume(&left, -1, 1, mnav_volumeInclude, 0),
                                        Volume(&right, -1, 1, mnav_volumeInclude, 0),
                                        Volume(&far, -1, 1, mnav_volumeExclude, 0)};
    m = Bake(cells, includes, 3);
    CHECK(Count(m, GROUND, mnav_areaWalkable) == 2 * 12 * 12 &&
              Cells(m, GROUND, 2, 2, 13, 13, mnav_areaWalkable) &&
              Cells(m, GROUND, 34, 2, 45, 13, mnav_areaWalkable) &&
              Count(m, PLATFORM, mnav_areaWalkable) == 0,
          "only the ground two include volumes hold");
    // An exclude volume missing the tile changes nothing.
    m = Bake(cells, &includes[2], 1);
    CHECK(Count(m, GROUND, mnav_areaWalkable) == CELLS * CELLS &&
              Count(m, PLATFORM, mnav_areaWalkable) == 26 * 26,
          "an exclude volume missing the tile changes nothing");
}

// Marked after erosion, an area does not shrink.
static void TestErosion(mnavBaker* cells)
{
    Square hole = MakeSquare(8, 8, 12, 12);
    mnavBakeVolume exclude = Volume(&hole, -1, 1, mnav_volumeExclude, 0);
    const Map* m = Bake(cells, &exclude, 1);
    CHECK(Hole(m, 32, 32, 47, 47) && Count(m, GROUND, NO_POLYGON) == 20 * 20 - 4,
          "the hole widened by the radius");
    mnavBakeVolume mark = Volume(&hole, -1, 1, mnav_volumeArea, 9);
    m = Bake(cells, &mark, 1);
    CHECK(Count(m, GROUND, 9) == 16 * 16 && Cells(m, GROUND, 32, 32, 47, 47, 9) &&
              Count(m, GROUND, NO_POLYGON) == 0,
          "an area keeps its size");
    // Erosion also starts where two walkable cells do not link: along x
    // = 8 m from z = 8 to 12 m, cells 31 and 32 and 32 to 47. The radius
    // takes one cell more on each side and at each end.
    m = BakeMesh(cells, &s_low, nullptr, 0);
    CHECK(Cells(m, GROUND, 30, 31, 33, 48, NO_POLYGON) && Count(m, GROUND, NO_POLYGON) == 4 * 18,
          "erosion from cells too low to pass between");
}

// Whether two input checks' outcomes are the same.
static bool SameInput(mnavInputResult a, mnavInputResult b)
{
    return a.result == b.result && a.kind == b.kind && a.index == b.index &&
           a.element == b.element && a.elementIndex == b.elementIndex;
}

// Bakes the mesh with two copies of a volume the bake refuses; the
// whole-input check refuses it the same way.
static mnavBakeReport Refused(mnavBaker* baker, mnavBakeVolume volume, mnavResult result)
{
    const mnavBakeVolume volumes[2] = {volume, volume};
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, volumes, 2, nullptr};
    mnavBakeReport report;
    CHECK(mnavBaker_BakeTile(baker, &input, 0, 0, &report) == result, "refused");
    mnavBakeDef def = mnavDefaultBakeDef();
    CHECK(SameInput(mnavValidateBakeInput(&def, &input), report.input), "checked alike");
    return report;
}

// Whether a report names the first volume.
static bool FirstVolume(mnavBakeReport report)
{
    return report.input.kind == mnav_inputVolume && report.input.index == 0;
}

static void TestChecks(mnavBaker* baker)
{
    Square s = MakeSquare(0, 0, 4, 4);
    mnavBakeVolume good = Volume(&s, -1, 1, mnav_volumeArea, 3);
    mnavBakeVolume v = good;
    v.pointCount = 2;
    mnavBakeReport r = Refused(baker, v, mnav_errorInvalid);
    CHECK(FirstVolume(r) && r.input.element == mnav_elementNone, "two points, the volume named");
    v = good;
    v.kind = 3;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "a kind out of range");
    v = good;
    v.area = mnav_areaNone;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "an area volume of no area");
    v = good;
    v.area = MNAV_AREA_TYPES;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "an area out of range");
    v = good;
    v.minY = 2.0f;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "heights backward");
    v = good;
    v.maxY = (float)INFINITY;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "an endless height");
    v = good;
    v.points = nullptr;
    CHECK(FirstVolume(Refused(baker, v, mnav_errorInvalid)), "no points");
    mnavVec2 points[4] = {{0, 0}, {4, 0}, {(float)NAN, 4}, {0, 4}};
    v = good;
    v.points = points;
    r = Refused(baker, v, mnav_errorInvalid);
    CHECK(r.input.element == mnav_elementPoint && r.input.elementIndex == 2, "a NaN point");
    points[2] = (mnavVec2){1e9f, 4};
    r = Refused(baker, v, mnav_errorLimit);
    CHECK(r.input.element == mnav_elementPoint && r.input.elementIndex == 2, "a point too far");
    points[2] = (mnavVec2){4, -1e9f};
    r = Refused(baker, v, mnav_errorLimit);
    CHECK(r.input.element == mnav_elementPoint && r.input.elementIndex == 2,
          "a point too far along Z");
    // After a terrain too.
    const float heights[4] = {0, 0, 0, 0};
    const mnavTerrain terrain = {{0, 0, 0}, 1, 1, 2, 2, heights, nullptr};
    v = good;
    v.pointCount = 2;
    const mnavBakeInput withTerrain = {&s_mesh, 1, &terrain, 1, &v, 1, nullptr};
    CHECK(mnavBaker_BakeTile(baker, &withTerrain, 0, 0, &r) == mnav_errorInvalid && FirstVolume(r),
          "a volume named by its own index, after a mesh and a terrain");
    // An exclude or include volume's area is unused.
    v = good;
    v.kind = mnav_volumeExclude;
    v.area = mnav_areaNone;
    const mnavBakeInput fine = {&s_mesh, 1, nullptr, 0, &v, 1, nullptr};
    CHECK(mnavBaker_BakeTile(baker, &fine, 0, 0, nullptr) == mnav_success, "an exclude volume");
    const mnavBakeInput missing = {&s_mesh, 1, nullptr, 0, nullptr, 1, nullptr};
    CHECK(mnavBaker_BakeTile(baker, &missing, 0, 0, nullptr) == mnav_errorInvalid,
          "volumes missing");
}

static uint64_t BakeHash(mnavBaker* baker, const mnavBakeVolume* volumes, int32_t count,
                         mnavBakeReport* report)
{
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, volumes, count, nullptr};
    size_t size = 0;
    CHECK(mnavBaker_BakeTile(baker, &input, 0, 0, report) == mnav_success &&
              mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    return mnavHash64(MNAV_HASH_INIT, s_tile, (int64_t)size);
}

// The baker and the three volumes of the pinned tile, made beforehand,
// for VolumeHash.
static mnavBaker* s_volumeBaker;
static mnavBakeVolume s_pinned[3];

static uint64_t VolumeHash(void)
{
    mnavBakeReport report;
    return BakeHash(s_volumeBaker, s_pinned, 3, &report);
}

static void TestBake(mnavBaker* baker)
{
    mnavBakeReport plain;
    uint64_t none = BakeHash(baker, nullptr, 0, &plain);
    Square far = MakeSquare(500, 500, 504, 504);
    mnavBakeVolume away = Volume(&far, -1, 1, mnav_volumeExclude, 0);
    mnavBakeReport report;
    CHECK(BakeHash(baker, &away, 1, &report) == none && report.fingerprint == plain.fingerprint,
          "an exclude volume missing the tile: the same tile");
    away.kind = mnav_volumeInclude;
    CHECK(BakeHash(baker, &away, 1, &report) != none && report.polygons == 0,
          "an include volume missing the tile: an empty one");
    Square inside = MakeSquare(2, 2, 18, 18);
    Square hole = MakeSquare(8, 8, 12, 12);
    Square mud = MakeSquare(2, 12, 18, 18);
    const mnavBakeVolume all[3] = {Volume(&inside, -1, 1, mnav_volumeInclude, 0),
                                   Volume(&hole, -1, 1, mnav_volumeExclude, 0),
                                   Volume(&mud, -1, 1, mnav_volumeArea, 6)};
    uint64_t hash = BakeHash(baker, all, 3, &report);
    printf("volumes: %d polygons; VOLUME_HASH=%016llx\n", report.polygons,
           (unsigned long long)hash);
    CHECK(report.polygons > 2 && report.fingerprint != plain.fingerprint, "a shaped tile");
    CHECK(hash == VOLUME_HASH, "the pinned hash");
    s_volumeBaker = baker;
    memcpy(s_pinned, all, sizeof(s_pinned));
    CHECK(EveryFloatHost(VolumeHash, VOLUME_HASH), "the pinned hash under every host setting");
    // Volume points count as input triangles.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.inputTriangles = 4 + 3 * 4 - 1;
    mnavBaker* small = nullptr;
    CHECK(mnavCreateBaker(&def, &small).result == mnav_success, "baker");
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, all, 3, nullptr};
    CHECK(mnavBaker_BakeTile(small, &input, 0, 0, &report) == mnav_errorCapacity &&
              report.input.kind == mnav_inputNone,
          "points over the input limit");
    mnavVec2 many[16];
    for (int32_t k = 0; k < 16; ++k)
    {
        many[k] = (mnavVec2){(float)(k % 2), (float)k};
    }
    const mnavBakeVolume big = {many, 16, -1, 1, mnav_volumeExclude, 0};
    const mnavBakeInput alone = {nullptr, 0, nullptr, 0, &big, 1, nullptr};
    CHECK(mnavBaker_BakeTile(small, &alone, 0, 0, &report) == mnav_errorCapacity &&
              report.input.kind == mnav_inputVolume && report.input.index == 0,
          "one volume over the input limit, named");
    mnavDestroyBaker(small);
    def.limits.inputTriangles += 1;
    CHECK(mnavCreateBaker(&def, &small).result == mnav_success &&
              mnavBaker_BakeTile(small, &input, 0, 0, &report) == mnav_success,
          "points at the input limit");
    mnavDestroyBaker(small);
}

int main(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    // A baker of no edge error, for the maps.
    mnavBakeDef exact = mnavDefaultBakeDef();
    exact.maxEdgeError = 0.0f;
    mnavBaker* cells = nullptr;
    CHECK(mnavCreateBaker(&exact, &cells).result == mnav_success, "baker");
    TestField(cells);
    TestKinds(cells);
    TestErosion(cells);
    mnavDestroyBaker(cells);
    TestChecks(baker);
    TestBake(baker);
    mnavDestroyBaker(baker);
    return s_failures == 0 ? 0 : 1;
}
