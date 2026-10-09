// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tile cache (mnav-0016): a tile rebuilt from the cache with obstacles
// is, byte for byte, the tile a full bake gives with the obstacles
// appended to its volumes, on meshes, terrains and indexed input, with
// include, exclude and area volumes; the refusals; the limits.

#include "counting_allocator.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

enum
{
    MOST_VOLUMES = 32,
    CRATES = 16
};

static uint8_t s_expected[TILE_CAPACITY];
static uint8_t s_rebuilt[TILE_CAPACITY];

// Squares the volumes use, from (x0, z0) to (x1, z1).
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

// A terrain over the world, hills rising through its floor in places.
static float s_heights[9 * 9];

static mnavTerrain Terrain(void)
{
    for (int32_t z = 0; z < 9; ++z)
    {
        for (int32_t x = 0; x < 9; ++x)
        {
            s_heights[z * 9 + x] = 0.6f * sinf((float)x * 1.3f) * cosf((float)z * 0.9f) - 0.2f;
        }
    }
    return (mnavTerrain){{-1.0, 0.0, -1.0}, 8.25f, 8.25f, 9, 9, s_heights, nullptr};
}

// The input's volumes and then the obstacles.
static mnavBakeVolume s_appended[MOST_VOLUMES];

// Bakes the input with the obstacles appended, and rebuilds the cached
// tile with them; true when the bytes and the report's counts agree.
static bool Alike(mnavBaker* baker, mnavTileCache* cache, const mnavBakeInput* input, int32_t tileX,
                  int32_t tileZ, const mnavBakeVolume* obstacles, int32_t count)
{
    mnavBakeInput full = *input;
    if (input->volumeCount > 0)
    {
        memcpy(s_appended, input->volumes, (size_t)input->volumeCount * sizeof(mnavBakeVolume));
    }
    if (count > 0)
    {
        memcpy(&s_appended[input->volumeCount], obstacles, (size_t)count * sizeof(mnavBakeVolume));
    }
    full.volumes = s_appended;
    full.volumeCount = input->volumeCount + count;
    mnavBakeReport baked;
    mnavBakeReport rebuilt;
    size_t expected = 0;
    size_t size = 0;
    if (mnavBakeTileInput(baker, &full, tileX, tileZ, &baked) != mnav_success ||
        mnavCopyBakedTile(baker, s_expected, TILE_CAPACITY, &expected) != mnav_success ||
        mnavRebuildTile(baker, cache, tileX, tileZ, obstacles, count, &rebuilt) != mnav_success ||
        mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &size) != mnav_success)
    {
        return false;
    }
    return size == expected && memcmp(s_rebuilt, s_expected, size) == 0 &&
           rebuilt.fingerprint == baked.fingerprint && rebuilt.triangles == baked.triangles &&
           rebuilt.spans == baked.spans && rebuilt.polygons == baked.polygons &&
           rebuilt.tileBytes == baked.tileBytes && rebuilt.stage == mnav_stageDone;
}

// The obstacle sets each scene is rebuilt with.
static Square s_crate;
static Square s_seam;
static Square s_mud;
static Square s_far;
static Square s_pen;
static Square s_crates[CRATES];

typedef struct Obstacles
{
    mnavBakeVolume list[CRATES];
    int32_t count;
} Obstacles;

static int32_t MakeObstacles(Obstacles* sets)
{
    s_crate = MakeSquare(14, 14, 17, 17);
    s_seam = MakeSquare(30.5f, 14, 33.5f, 50);
    s_mud = MakeSquare(2, 40, 60, 46);
    s_far = MakeSquare(500, 500, 504, 504);
    s_pen = MakeSquare(4, 4, 60, 28);
    int32_t n = 0;
    sets[n++] = (Obstacles){{{0}}, 0};
    sets[n++] = (Obstacles){{Volume(&s_crate, -1, 2, mnav_volumeExclude, 0)}, 1};
    sets[n++] = (Obstacles){{Volume(&s_seam, -1, 2, mnav_volumeExclude, 0)}, 1};
    sets[n++] = (Obstacles){{Volume(&s_mud, -1, 2, mnav_volumeArea, 9)}, 1};
    sets[n++] = (Obstacles){{Volume(&s_far, -1, 2, mnav_volumeExclude, 0)}, 1};
    sets[n++] = (Obstacles){{Volume(&s_pen, -1, 2, mnav_volumeInclude, 0)}, 1};
    // Above the floor's heights: it shapes nothing but the fingerprint.
    sets[n++] = (Obstacles){{Volume(&s_crate, 3, 4, mnav_volumeExclude, 0)}, 1};
    Obstacles* many = &sets[n++];
    many->count = CRATES;
    for (int32_t k = 0; k < CRATES; ++k)
    {
        float x = 3.0f + 3.7f * (float)(k % 4) * 4.0f;
        float z = 3.0f + 3.7f * (float)(k / 4) * 4.0f;
        s_crates[k] = MakeSquare(x, z, x + 1.5f, z + 1.0f);
        many->list[k] =
            Volume(&s_crates[k], -1, 2, k % 3 == 2 ? mnav_volumeArea : mnav_volumeExclude,
                   (mnavAreaType)(k % 3 == 2 ? 4 : 0));
    }
    return n;
}

// Caches the world's four tiles from the input, each alike a plain bake,
// and rebuilds each with every obstacle set; counts the rebuilds alike.
static int32_t Scene(mnavBaker* baker, mnavTileCache* cache, const mnavBakeInput* input)
{
    Obstacles sets[10];
    int32_t setCount = MakeObstacles(sets);
    int32_t alike = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        size_t size = 0;
        size_t cached = 0;
        CHECK(mnavBakeTileInput(baker, input, t & 1, t >> 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_expected, TILE_CAPACITY, &size) == mnav_success &&
                  mnavBakeTileCached(baker, cache, input, t & 1, t >> 1, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &cached) == mnav_success &&
                  size == cached && memcmp(s_expected, s_rebuilt, size) == 0,
              "a cached bake gives a plain bake's bytes");
        for (int32_t k = 0; k < setCount; ++k)
        {
            alike += Alike(baker, cache, input, t & 1, t >> 1, sets[k].list, sets[k].count) ? 1 : 0;
        }
    }
    CHECK(alike == 4 * setCount, "every rebuild alike a full bake");
    return alike;
}

static void TestAlike(mnavBaker* baker, mnavTileCache* cache)
{
    mnavTriangleMesh world = World();
    mnavTerrain terrain = Terrain();
    Square strip = MakeSquare(10, -2, 20, 66);
    Square hole = MakeSquare(40, 40, 44, 44);
    Square most = MakeSquare(1, 1, 63, 63);
    Square far = MakeSquare(500, 500, 504, 504);
    const mnavBakeVolume volumes[3] = {Volume(&strip, -1, 2, mnav_volumeArea, 5),
                                       Volume(&hole, -1, 2, mnav_volumeExclude, 0),
                                       Volume(&most, -1, 2, mnav_volumeInclude, 0)};
    const mnavBakeVolume outside[2] = {Volume(&strip, -1, 2, mnav_volumeArea, 5),
                                       Volume(&far, -1, 2, mnav_volumeInclude, 0)};
    const mnavBakeInput scenes[] = {
        {&world, 1, nullptr, 0, nullptr, 0, nullptr},
        {&world, 1, nullptr, 0, volumes, 3, nullptr},
        {nullptr, 0, &terrain, 1, volumes, 2, nullptr},
        {&world, 1, &terrain, 1, volumes, 3, nullptr},
        // An include out of every tile keeps them all empty, rebuilt too.
        {&world, 1, nullptr, 0, outside, 2, nullptr},
    };
    int32_t alike = 0;
    for (size_t s = 0; s < sizeof(scenes) / sizeof(scenes[0]); ++s)
    {
        alike += Scene(baker, cache, &scenes[s]);
    }
    // Indexed meshes.
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavTileIndex* index = nullptr;
    CHECK(mnavCreateTileIndex(&def, &world, 1, &index, nullptr) == mnav_success, "an index");
    const mnavBakeInput indexed = {&world, 1, &terrain, 1, volumes, 3, index};
    alike += Scene(baker, cache, &indexed);
    mnavDestroyTileIndex(index);
    printf("rebuilds: %d alike a full bake\n", alike);
    // The obstacles shape the tiles: a crate leaves another tile.
    const mnavBakeVolume crate = Volume(&s_crate, -1, 2, mnav_volumeExclude, 0);
    size_t plain = 0;
    size_t carved = 0;
    CHECK(mnavBakeTileCached(baker, cache, &scenes[1], 0, 0, nullptr) == mnav_success &&
              mnavRebuildTile(baker, cache, 0, 0, nullptr, 0, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, s_expected, TILE_CAPACITY, &plain) == mnav_success &&
              mnavRebuildTile(baker, cache, 0, 0, &crate, 1, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &carved) == mnav_success &&
              (plain != carved || memcmp(s_expected, s_rebuilt, plain) != 0),
          "a crate changes the tile");
}

static mnavBakeReport Rebuilt(mnavBaker* baker, mnavTileCache* cache, int32_t tileX,
                              const mnavBakeVolume* obstacles, int32_t count, mnavResult result)
{
    mnavBakeReport report;
    size_t size = 0;
    CHECK(mnavRebuildTile(baker, cache, tileX, 0, obstacles, count, &report) == result &&
              report.result == result,
          "the rebuild's result");
    CHECK((result == mnav_success) ==
              (mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &size) == mnav_success),
          "a tile only after a rebuild that succeeded");
    return report;
}

static void TestRefusals(mnavBaker* baker, mnavTileCache* cache)
{
    mnavTriangleMesh world = World();
    const mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeReport cached;
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, &cached) == mnav_success, "cached");
    CHECK(mnavBakeTileCached(baker, nullptr, &input, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavBakeTileCached(baker, cache, nullptr, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavBakeTileCached(nullptr, cache, &input, 0, 0, nullptr) == mnav_errorInvalid,
          "a cached bake of NULL arguments refused");
    Rebuilt(baker, cache, 5, nullptr, 0, mnav_errorNotLoaded);
    CHECK(mnavRebuildTile(nullptr, cache, 0, 0, nullptr, 0, nullptr) == mnav_errorInvalid,
          "no baker");
    Rebuilt(baker, nullptr, 0, nullptr, 0, mnav_errorInvalid);
    Rebuilt(baker, cache, 0, nullptr, -1, mnav_errorInvalid);
    Rebuilt(baker, cache, 0, nullptr, 1, mnav_errorInvalid);
    Square s = MakeSquare(4, 4, 8, 8);
    mnavVec2 points[4] = {{4, 4}, {8, 4}, {(float)NAN, 8}, {4, 8}};
    mnavBakeVolume obstacles[2] = {Volume(&s, -1, 2, mnav_volumeExclude, 0),
                                   Volume(&s, -1, 2, mnav_volumeExclude, 0)};
    obstacles[1].pointCount = 2;
    mnavBakeReport r = Rebuilt(baker, cache, 0, obstacles, 2, mnav_errorInvalid);
    CHECK(r.mesh == 1 && r.input.element == mnav_elementNone, "an obstacle named by its index");
    obstacles[1] = (mnavBakeVolume){points, 4, -1, 2, mnav_volumeExclude, 0};
    r = Rebuilt(baker, cache, 0, obstacles, 2, mnav_errorInvalid);
    CHECK(r.mesh == 1 && r.input.element == mnav_elementPoint && r.input.index == 2,
          "an obstacle's NaN point");
    Rebuilt(baker, cache, 0, obstacles, 1, mnav_success);
    // A baker of other settings is refused; one of other limits is not.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius *= 2.0f;
    mnavBaker* other = nullptr;
    CHECK(mnavCreateBaker(&def, &other).result == mnav_success, "another baker");
    Rebuilt(other, cache, 0, nullptr, 0, mnav_errorInvalid);
    mnavDestroyBaker(other);
    def = mnavDefaultBakeDef();
    def.limits.inputTriangles = cached.triangles + 4;
    CHECK(mnavCreateBaker(&def, &other).result == mnav_success, "a baker of other limits");
    Rebuilt(other, cache, 0, obstacles, 1, mnav_success);
    // The tile's triangles and the obstacles' points count as input.
    Rebuilt(other, cache, 0, obstacles, 0, mnav_success);
    obstacles[1] = obstacles[0];
    Rebuilt(other, cache, 0, obstacles, 2, mnav_errorLimit);
    mnavDestroyBaker(other);
    // A cached bake that fails leaves nothing cached for the tile.
    const mnavBakeInput missing = {&world, 1, nullptr, 0, nullptr, 1, nullptr};
    CHECK(mnavBakeTileCached(baker, cache, &missing, 0, 0, nullptr) == mnav_errorInvalid,
          "refused input");
    Rebuilt(baker, cache, 0, nullptr, 0, mnav_errorNotLoaded);
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_success, "cached again");
    mnavDropCachedTile(cache, 0, 0);
    mnavDropCachedTile(cache, 0, 0);
    mnavDropCachedTile(nullptr, 0, 0);
    Rebuilt(baker, cache, 0, nullptr, 0, mnav_errorNotLoaded);
}

static void TestCreate(void)
{
    mnavTileCache* cache = nullptr;
    mnavTileCacheDef def = mnavDefaultTileCacheDef();
    CHECK(mnavCreateTileCache(&def, nullptr) == mnav_errorInvalid &&
              mnavCreateTileCache(nullptr, &cache) == mnav_errorInvalid && cache == nullptr,
          "NULL arguments");
    def.cookie += 1;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorInvalid, "a def not from the default");
    def = mnavDefaultTileCacheDef();
    def.allocator.alloc = CountingAlloc;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorInvalid, "an allocator without free");
    def = mnavDefaultTileCacheDef();
    def.limits.tiles = 0;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorRange, "no tiles");
    def.limits.tiles = MNAV_MAX_CACHED_TILES + 1;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorRange, "too many tiles");
    def.limits.tiles = 1;
    def.limits.memoryBytes = 0;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorRange, "no bytes");
    def.limits.memoryBytes = 16;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_errorLimit && cache == nullptr,
          "too few bytes for the cache itself");
    CHECK(mnavGetTileCacheBytes(nullptr) == 0, "a NULL cache holds nothing");
    mnavDestroyTileCache(nullptr);
}

// A cache of one tile, then of too few bytes.
static void TestLimits(mnavBaker* baker)
{
    mnavTriangleMesh world = World();
    const mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavTileCacheDef def = mnavDefaultTileCacheDef();
    def.limits.tiles = 1;
    mnavTileCache* cache = nullptr;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_success, "a cache of one tile");
    uint64_t empty = mnavGetTileCacheBytes(cache);
    mnavBakeReport report;
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_success &&
              mnavBakeTileCached(baker, cache, &input, 0, 0, nullptr) == mnav_success,
          "a tile cached again replaces itself");
    uint64_t one = mnavGetTileCacheBytes(cache);
    printf("cache: %llu bytes empty, %llu bytes for a 32 m tile of the world\n",
           (unsigned long long)empty, (unsigned long long)(one - empty));
    CHECK(one > empty, "a tile takes bytes");
    size_t size = 0;
    CHECK(mnavBakeTileCached(baker, cache, &input, 1, 0, &report) == mnav_errorLimit &&
              report.result == mnav_errorLimit &&
              mnavCopyBakedTile(baker, s_rebuilt, TILE_CAPACITY, &size) == mnav_errorInvalid,
          "a second tile over the limit, not baked");
    Rebuilt(baker, cache, 1, nullptr, 0, mnav_errorNotLoaded);
    Rebuilt(baker, cache, 0, nullptr, 0, mnav_success);
    mnavDropCachedTile(cache, 0, 0);
    CHECK(mnavGetTileCacheBytes(cache) == empty, "a dropped tile's bytes given back");
    mnavDestroyTileCache(cache);
    def.limits.memoryBytes = empty + (one - empty) / 2;
    CHECK(mnavCreateTileCache(&def, &cache) == mnav_success, "a small cache");
    uint64_t small = mnavGetTileCacheBytes(cache);
    CHECK(mnavBakeTileCached(baker, cache, &input, 0, 0, &report) == mnav_errorLimit &&
              mnavGetTileCacheBytes(cache) == small,
          "a tile over the bytes, nothing kept");
    Rebuilt(baker, cache, 0, nullptr, 0, mnav_errorNotLoaded);
    mnavDestroyTileCache(cache);
}

int main(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    mnavTileCache* cache = nullptr;
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    cacheDef.allocator = CountingAllocator();
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavCreateTileCache(&cacheDef, &cache) == mnav_success,
          "made");
    TestAlike(baker, cache);
    TestRefusals(baker, cache);
    TestCreate();
    TestLimits(baker);
    mnavDestroyTileCache(cache);
    mnavDestroyBaker(baker);
    CHECK(s_held == 0, "the cache gives every byte back");
    return s_failures == 0 ? 0 : 1;
}
