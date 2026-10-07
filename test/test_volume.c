// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bake volumes (mnav-0003): include, exclude and area volumes on the
// open-space field, their heights, order and erosion; the checks; the
// fingerprint.

#include "allocator.h"
#include "compact.h"
#include "erode.h"
#include "filter.h"
#include "heightfield.h"
#include "test_harness.h"
#include "volume.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of a tile baked with every kind of volume, the same on every
// platform.
#define VOLUME_HASH 0xcf40bf48619544c3ull

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

typedef struct Field
{
    mnavBakeDef def;
    mnavBakeCells cells;
    mnavMemory memory;
    mnavHeightfield heightfield;
    mnavCompactField compact;
} Field;

static void Build(Field* f)
{
    f->def = mnavDefaultBakeDef();
    CHECK(mnavValidateBakeDef(&f->def, &f->cells).result == mnav_success, "def");
    f->memory = mnavMakeMemory(f->def.allocator, f->def.limits.memoryBytes);
    CHECK(mnavBuildHeightfield(&f->memory, &f->def, &f->cells, &s_mesh, 1, 0, 0, &f->heightfield) ==
              mnav_success,
          "rasterized");
    mnavFilterWalkable(&f->heightfield, f->cells.agentHeight, f->cells.agentStep);
    CHECK(mnavBuildCompactField(&f->memory, &f->heightfield, f->cells.agentHeight,
                                f->cells.agentStep, &f->compact) == mnav_success,
          "compacted");
}

static void Release(Field* f)
{
    mnavReleaseCompactField(&f->memory, &f->compact);
    mnavReleaseHeightfield(&f->memory, &f->heightfield);
    CHECK(f->memory.used == 0, "all released");
}

// The spans of an area, and of an area at the ground or the platform.
static int32_t Count(const Field* f, mnavAreaType area, int32_t level)
{
    int32_t n = 0;
    for (int32_t i = 0; i < f->compact.spanCount; ++i)
    {
        bool high = f->compact.spans[i].floor > MNAV_HEIGHT_OFFSET + 8;
        n += f->compact.areas[i] == area && (level < 0 || high == (level == 1)) ? 1 : 0;
    }
    return n;
}

// The ground's cell edges fall on whole quarter meters from -1.25 m, so
// a square with whole-meter corners holds 16 cells per meter squared.
static void TestKinds(void)
{
    Field f;
    Build(&f);
    int32_t ground = Count(&f, mnav_areaWalkable, 0);
    int32_t platform = Count(&f, mnav_areaWalkable, 1);
    // The field's outer ring and the platform's rim are ledges.
    CHECK(ground == 136 * 136 && platform == 30 * 30, "the field");
    // Excluded below the platform: the ground goes, the platform stays.
    Square under = MakeSquare(22, 22, 26, 26);
    mnavBakeVolume exclude = Volume(&under, -1.0f, 1.0f, mnav_volumeExclude, 0);
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, &exclude, 1) == mnav_success &&
              Count(&f, mnav_areaNone, 0) == 256 && Count(&f, mnav_areaNone, 1) == 0,
          "an exclude volume takes the ground within its heights");
    // An area volume over the platform's heights alone.
    Square over = MakeSquare(20, 20, 24, 24);
    mnavBakeVolume high = Volume(&over, 2.0f, 4.0f, mnav_volumeArea, 7);
    CHECK(mnavMarkVolumes(&f.memory, &f.compact, &high, 1) == mnav_success &&
              Count(&f, 7, 1) == 15 * 15 && Count(&f, 7, 0) == 0,
          "an area volume marks the platform");
    // Two overlapping area volumes: the last wins; excluded spans stay out.
    Square a = MakeSquare(0, 0, 8, 8);
    Square b = MakeSquare(4, 0, 12, 8);
    Square c = MakeSquare(22, 22, 23, 23);
    const mnavBakeVolume areas[3] = {Volume(&a, -1, 1, mnav_volumeArea, 3),
                                     Volume(&b, -1, 1, mnav_volumeArea, 4),
                                     Volume(&c, -1, 1, mnav_volumeArea, 5)};
    CHECK(mnavMarkVolumes(&f.memory, &f.compact, areas, 3) == mnav_success &&
              Count(&f, 3, 0) == 16 * 32 && Count(&f, 4, 0) == 32 * 32 && Count(&f, 5, -1) == 0,
          "the last volume wins; no walkable ground is made");
    Release(&f);
    // A ring of many points before a square: the scan has room for the
    // most points of any.
    Build(&f);
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
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, two, 2) == mnav_success &&
              Count(&f, mnav_areaNone, 0) > 256 + 11 * 2 * 16,
          "a many-pointed ring, then a square");
    Release(&f);
    // Include volumes: ground outside every one goes.
    Build(&f);
    Square left = MakeSquare(0, 0, 4, 4);
    Square right = MakeSquare(8, 0, 12, 4);
    Square far = MakeSquare(500, 500, 504, 504);
    const mnavBakeVolume includes[3] = {Volume(&left, -1, 1, mnav_volumeInclude, 0),
                                        Volume(&right, -1, 1, mnav_volumeInclude, 0),
                                        Volume(&far, -1, 1, mnav_volumeExclude, 0)};
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, includes, 3) == mnav_success &&
              Count(&f, mnav_areaWalkable, -1) == 2 * 256,
          "only the ground two include volumes hold");
    Release(&f);
    // An include volume missing the tile leaves nothing.
    Build(&f);
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, &includes[2], 1) == mnav_success &&
              Count(&f, mnav_areaWalkable, -1) == ground + platform,
          "an exclude volume missing the tile changes nothing");
    mnavBakeVolume away = Volume(&far, -1, 1, mnav_volumeInclude, 0);
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, &away, 1) == mnav_success &&
              Count(&f, mnav_areaWalkable, -1) == 0,
          "an include volume missing the tile leaves nothing");
    Release(&f);
}

// Carved before erosion, an excluded square widens by the agent's radius;
// marked after, an area does not shrink.
static void TestErosion(void)
{
    Field f;
    Build(&f);
    Square hole = MakeSquare(8, 8, 12, 12);
    mnavBakeVolume exclude = Volume(&hole, -1, 1, mnav_volumeExclude, 0);
    CHECK(mnavCarveVolumes(&f.memory, &f.compact, &exclude, 1) == mnav_success &&
              mnavErode(&f.memory, &f.compact, f.cells.agentRadius) == mnav_success,
          "carved and eroded");
    // 16 by 16 cells and the radius, 2 cells, along each side; the
    // corners rounded at most.
    int32_t gone = 0;
    for (int32_t i = 0; i < f.compact.spanCount; ++i)
    {
        gone += f.compact.areas[i] == mnav_areaNone ? 1 : 0;
    }
    Release(&f);
    Build(&f);
    CHECK(mnavErode(&f.memory, &f.compact, f.cells.agentRadius) == mnav_success, "eroded");
    int32_t edges = Count(&f, mnav_areaNone, -1);
    mnavBakeVolume mark = Volume(&hole, -1, 1, mnav_volumeArea, 9);
    CHECK(mnavMarkVolumes(&f.memory, &f.compact, &mark, 1) == mnav_success &&
              Count(&f, 9, -1) == 256,
          "an area keeps its size");
    printf("erosion: %d spans gone with the hole, %d without\n", gone, edges);
    CHECK(gone - edges >= 16 * 16 + 4 * 16 * 2 && gone - edges <= 20 * 20,
          "the hole widened by the radius");
    Release(&f);
}

static mnavBakeReport Refused(mnavBaker* baker, mnavBakeVolume volume, mnavResult result)
{
    const mnavBakeVolume volumes[2] = {volume, volume};
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, volumes, 2, nullptr};
    mnavBakeReport report;
    CHECK(mnavBakeTileInput(baker, &input, 0, 0, &report) == result, "refused");
    return report;
}

static void TestChecks(mnavBaker* baker)
{
    Square s = MakeSquare(0, 0, 4, 4);
    mnavBakeVolume good = Volume(&s, -1, 1, mnav_volumeArea, 3);
    mnavBakeVolume v = good;
    v.pointCount = 2;
    mnavBakeReport r = Refused(baker, v, mnav_errorInvalid);
    CHECK(r.mesh == 1 && r.input.element == mnav_elementNone, "two points: after the mesh");
    v = good;
    v.kind = 3;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "a kind out of range");
    v = good;
    v.area = mnav_areaNone;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "an area volume of no area");
    v = good;
    v.area = MNAV_AREA_TYPES;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "an area out of range");
    v = good;
    v.minY = 2.0f;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "heights backward");
    v = good;
    v.maxY = (float)INFINITY;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "an endless height");
    v = good;
    v.points = nullptr;
    CHECK(Refused(baker, v, mnav_errorInvalid).mesh == 1, "no points");
    mnavVec2 points[4] = {{0, 0}, {4, 0}, {(float)NAN, 4}, {0, 4}};
    v = good;
    v.points = points;
    r = Refused(baker, v, mnav_errorInvalid);
    CHECK(r.input.element == mnav_elementPoint && r.input.index == 2, "a NaN point");
    points[2] = (mnavVec2){1e9f, 4};
    r = Refused(baker, v, mnav_errorRange);
    CHECK(r.input.element == mnav_elementPoint && r.input.index == 2, "a point too far");
    points[2] = (mnavVec2){4, -1e9f};
    r = Refused(baker, v, mnav_errorRange);
    CHECK(r.input.element == mnav_elementPoint && r.input.index == 2, "a point too far along Z");
    // After a terrain too.
    const float heights[4] = {0, 0, 0, 0};
    const mnavTerrain terrain = {{0, 0, 0}, 1, 1, 2, 2, heights, nullptr};
    v = good;
    v.pointCount = 2;
    const mnavBakeInput withTerrain = {&s_mesh, 1, &terrain, 1, &v, 1, nullptr};
    CHECK(mnavBakeTileInput(baker, &withTerrain, 0, 0, &r) == mnav_errorInvalid && r.mesh == 2,
          "a volume named after the meshes and terrains");
    // An exclude or include volume's area is unused.
    v = good;
    v.kind = mnav_volumeExclude;
    v.area = mnav_areaNone;
    const mnavBakeInput fine = {&s_mesh, 1, nullptr, 0, &v, 1, nullptr};
    CHECK(mnavBakeTileInput(baker, &fine, 0, 0, nullptr) == mnav_success, "an exclude volume");
    const mnavBakeInput missing = {&s_mesh, 1, nullptr, 0, nullptr, 1, nullptr};
    CHECK(mnavBakeTileInput(baker, &missing, 0, 0, nullptr) == mnav_errorInvalid,
          "volumes missing");
}

static uint64_t BakeHash(mnavBaker* baker, const mnavBakeVolume* volumes, int32_t count,
                         mnavBakeReport* report)
{
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, volumes, count, nullptr};
    size_t size = 0;
    CHECK(mnavBakeTileInput(baker, &input, 0, 0, report) == mnav_success &&
              mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    return mnavHash64(MNAV_HASH_INIT, s_tile, (int32_t)size);
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
    // Volume points count as input triangles.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.inputTriangles = 4 + 3 * 4 - 1;
    mnavBaker* small = nullptr;
    CHECK(mnavCreateBaker(&def, &small).result == mnav_success, "baker");
    const mnavBakeInput input = {&s_mesh, 1, nullptr, 0, all, 3, nullptr};
    CHECK(mnavBakeTileInput(small, &input, 0, 0, &report) == mnav_errorLimit && report.mesh == -1,
          "points over the input limit");
    mnavVec2 many[16];
    for (int32_t k = 0; k < 16; ++k)
    {
        many[k] = (mnavVec2){(float)(k % 2), (float)k};
    }
    const mnavBakeVolume big = {many, 16, -1, 1, mnav_volumeExclude, 0};
    const mnavBakeInput alone = {nullptr, 0, nullptr, 0, &big, 1, nullptr};
    CHECK(mnavBakeTileInput(small, &alone, 0, 0, &report) == mnav_errorLimit && report.mesh == 0,
          "one volume over the input limit, named");
    mnavDestroyBaker(small);
    def.limits.inputTriangles += 1;
    CHECK(mnavCreateBaker(&def, &small).result == mnav_success &&
              mnavBakeTileInput(small, &input, 0, 0, &report) == mnav_success,
          "points at the input limit");
    mnavDestroyBaker(small);
}

int main(void)
{
    TestKinds();
    TestErosion();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    TestChecks(baker);
    TestBake(baker);
    mnavDestroyBaker(baker);
    return s_failures == 0 ? 0 : 1;
}
