// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flight volume tiles (mnav-0015): every voxel of a baked tile against a
// voxel grid made the plain way from the same heightfield, on the test
// world's floor and boxes and on boxes floating in the air; the settings
// refused; the bytes the same on every platform; the memory given back.

#include "flight.h"
#include "heightfield.h"
#include "raster.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The hash of the test world's four tiles, the same on every platform.
#define FLIGHT_HASH 0xe727168fb46af93full

enum
{
    MOST_SIDE = 96,
    MOST_LAYERS = 160
};

// A voxel grid the plain way: solid voxels of the tile and its border.
static uint8_t s_raw[MOST_SIDE][MOST_SIDE][MOST_LAYERS];

// The def the bake builds its heightfield with, as the plain grid does.
static mnavBakeDef VoxelDef(const mnavBakeDef* def, const mnavFlightSettings* f)
{
    mnavBakeDef v = *def;
    v.cellSize = f->voxel;
    v.cellHeight = f->voxel;
    v.tileCells = f->tileVoxels;
    v.agent.radius = f->radius;
    v.agent.height = f->voxel;
    v.agent.stepHeight = 0.0f;
    v.detailSampleDistance = 0.0f;
    v.detailMaxError = 0.0f;
    v.maxEdgeLength = 0.0f;
    return v;
}

// Marks the heightfield's spans in the plain grid, offset columns in, and
// below each column's lowest span when asked.
static void MarkPlain(const mnavHeightfield* hf, int32_t offset, int32_t width, int32_t floorVoxel,
                      int32_t layers, bool groundBelow)
{
    memset(s_raw, 0, sizeof(s_raw));
    for (int32_t z = 0; z < width; ++z)
    {
        for (int32_t x = 0; x < width; ++x)
        {
            int32_t at = (x + offset) + (z + offset) * hf->frame.width;
            for (uint32_t s = hf->columns[at]; s < hf->columns[at + 1]; ++s)
            {
                int32_t lo = hf->spans[s].bottom - MNAV_HEIGHT_OFFSET - floorVoxel;
                int32_t hi = hf->spans[s].top - MNAV_HEIGHT_OFFSET - floorVoxel;
                lo = groundBelow && s == hf->columns[at] ? 0 : lo;
                for (int32_t y = lo < 0 ? 0 : lo; y < hi && y < layers; ++y)
                {
                    s_raw[x][z][y] = 1;
                }
            }
        }
    }
}

// The plain grid of a tile from its heightfield at the voxel size: spans
// marked, and below each column's lowest span when asked.
static int32_t Plain(const mnavBakeDef* def, const mnavFlightSettings* f,
                     const mnavBakeInput* input, int32_t tileX, int32_t tileZ, int32_t* rOut,
                     int32_t* floorOut, int32_t* layersOut)
{
    mnavBakeDef v = VoxelDef(def, f);
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&v, &cells).result == mnav_success, "voxel def");
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavHeightfield hf = {0};
    CHECK(mnavBuildHeightfieldInput(&memory, &v, &cells, input, tileX, tileZ, &hf) == mnav_success,
          "heightfield");
    int32_t floorVoxel = (int32_t)floorf(f->floor / f->voxel);
    float height = f->ceiling - (float)floorVoxel * f->voxel;
    int32_t cubes = (int32_t)ceilf(height / ((float)f->tileVoxels * f->voxel));
    int32_t layers = (cubes < 1 ? 1 : cubes) * f->tileVoxels;
    int32_t r = cells.agentRadius;
    int32_t width = f->tileVoxels + 2 * r;
    CHECK(width <= MOST_SIDE && layers <= MOST_LAYERS, "the plain grid holds the tile");
    MarkPlain(&hf, cells.border - r, width, floorVoxel, layers, f->groundBelow);
    mnavReleaseHeightfield(&memory, &hf);
    *rOut = r;
    *floorOut = floorVoxel;
    *layersOut = layers;
    return width;
}

// Whether a plain voxel has a solid one within the ball of r.
static bool Blocked(int32_t width, int32_t layers, int32_t r, int32_t x, int32_t y, int32_t z)
{
    for (int32_t dz = -r; dz <= r; ++dz)
    {
        for (int32_t dy = -r; dy <= r; ++dy)
        {
            for (int32_t dx = -r; dx <= r; ++dx)
            {
                int32_t X = x + dx;
                int32_t Y = y + dy;
                int32_t Z = z + dz;
                bool inside = X >= 0 && Z >= 0 && Y >= 0 && X < width && Z < width && Y < layers;
                if (dx * dx + dy * dy + dz * dz <= r * r && inside && s_raw[X][Z][Y] != 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

// The voxels of a tile the plain grid has otherwise; solidOut receives
// the tile's solid ones.
static int32_t Wrong(const mnavFlightTile* tile, int32_t width, int32_t layers, int32_t r,
                     int32_t* solidOut)
{
    int32_t wrong = 0;
    *solidOut = 0;
    for (int32_t z = 0; z < tile->side; ++z)
    {
        for (int32_t y = 0; y < layers; ++y)
        {
            for (int32_t x = 0; x < tile->side; ++x)
            {
                bool got = mnavFlightSolid(tile, x, y, z);
                wrong += Blocked(width, layers, r, x + r, y, z + r) != got ? 1 : 0;
                *solidOut += got ? 1 : 0;
            }
        }
    }
    return wrong;
}

// Bakes a tile and checks every voxel of it against the plain grid;
// returns the tile's hash.
static uint64_t Compare(const mnavBakeDef* def, const mnavFlightSettings* f,
                        const mnavBakeInput* input, int32_t tileX, int32_t tileZ)
{
    int32_t r = 0;
    int32_t floorVoxel = 0;
    int32_t layers = 0;
    int32_t width = Plain(def, f, input, tileX, tileZ, &r, &floorVoxel, &layers);
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavFlightTile tile;
    CHECK(mnavBakeFlightTile(&memory, def, f, input, tileX, tileZ, &tile) == mnav_success, "baked");
    CHECK(tile.floorVoxel == floorVoxel && tile.cubeCount * tile.side == layers, "its shape");
    int32_t solid = 0;
    int32_t wrong = Wrong(&tile, width, layers, r, &solid);
    printf("tile (%d, %d): %d solid of %d voxels, %d nodes, %d leaves, %d wrong\n", tileX, tileZ,
           solid, tile.side * tile.side * layers, tile.nodeCount, tile.leafCount, wrong);
    CHECK(wrong == 0, "every voxel as the plain grid has it");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, tile.roots, tile.cubeCount);
    hash = mnavHash64(hash, tile.rootNodes, tile.cubeCount * (int32_t)sizeof(int32_t));
    for (int32_t i = 0; i < tile.nodeCount; ++i)
    {
        hash = mnavHash64(hash, &tile.nodes[i].firstChild, (int32_t)sizeof(uint32_t));
        hash = mnavHash64(hash, &tile.nodes[i].mask, (int32_t)sizeof(uint16_t));
    }
    hash = mnavHash64(hash, tile.leaves, tile.leafCount * (int32_t)sizeof(uint64_t));
    mnavReleaseFlightTile(&memory, &tile);
    CHECK(memory.used == 0, "the memory given back");
    return hash;
}

static void TestTheWorldsFloorAndBoxes(void)
{
    // 0.5 m voxels, 32 m tiles, a flier of 0.5 m: one voxel of clearance.
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavTriangleMesh world = World();
    mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavFlightSettings f = {0.5f, 64, -2.0f, 20.0f, 0.5f, true};
    CHECK(mnavCheckFlightSettings(&f), "settings");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 4; ++t)
    {
        uint64_t one = Compare(&def, &f, &input, t % 2, t / 2);
        hash = mnavHash64(hash, &one, (int32_t)sizeof(one));
    }
    printf("FLIGHT_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == FLIGHT_HASH, "the pinned hash");
}

// Boxes in the air, of sides 1 to 6 m, from 2 to 40 m up.
static mnavVec3 s_air[40 * 8];
static int32_t s_airIndices[40 * 36];

static mnavTriangleMesh Air(void)
{
    static const int32_t faces[36] = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                                      2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    uint32_t state = 3;
    for (int32_t b = 0; b < 40; ++b)
    {
        float p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (float)(state >> 8 & 0xFFFFu) / 65536.0f;
        }
        float s = 1.0f + 5.0f * p[3];
        float x0 = p[0] * (32.0f - s);
        float y0 = 2.0f + p[1] * 36.0f;
        float z0 = p[2] * (32.0f - s);
        for (int32_t c = 0; c < 8; ++c)
        {
            s_air[b * 8 + c] =
                (mnavVec3){x0 + ((c & 1) != 0 ? s : 0.0f), y0 + ((c & 2) != 0 ? s : 0.0f),
                           z0 + ((c & 4) != 0 ? s : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_airIndices[b * 36 + k] = b * 8 + faces[k];
        }
    }
    return (mnavTriangleMesh){s_air, 40 * 8, s_airIndices, 40 * 12, nullptr};
}

static void TestBoxesInTheAir(void)
{
    // 1 m voxels, 16 m tiles: three cubes from 0 to 48 m; a flier of
    // 1.2 m, two voxels of clearance; then none, with no dilation; the
    // ground off, so nothing fills below the boxes.
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavTriangleMesh air = Air();
    mnavBakeInput input = {&air, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavFlightSettings f = {1.0f, 16, 0.0f, 40.0f, 1.2f, false};
    for (int32_t t = 0; t < 4; ++t)
    {
        (void)Compare(&def, &f, &input, t % 2, t / 2);
    }
    f.radius = 0.0f;
    (void)Compare(&def, &f, &input, 0, 0);
}

static void TestSettingsAreChecked(void)
{
    const mnavFlightSettings good = {1.0f, 32, 0.0f, 10.0f, 0.5f, true};
    CHECK(mnavCheckFlightSettings(&good), "good settings");
    mnavFlightSettings f = good;
    const int32_t sides[5] = {8, 48, 1024, 0, -16};
    for (int32_t k = 0; k < 5; ++k)
    {
        f.tileVoxels = sides[k];
        CHECK(!mnavCheckFlightSettings(&f), "a side not 4 times a power of 2 from 16 to 512");
    }
    f = good;
    f.ceiling = f.floor;
    CHECK(!mnavCheckFlightSettings(&f), "a ceiling not above the floor");
    f = good;
    f.radius = -1.0f;
    CHECK(!mnavCheckFlightSettings(&f), "a negative radius");
    f.radius = (float)NAN;
    CHECK(!mnavCheckFlightSettings(&f), "a radius not a number");
    f = good;
    f.voxel = 0.0f;
    CHECK(!mnavCheckFlightSettings(&f), "a voxel of 0");
    f.voxel = 20.0f;
    CHECK(!mnavCheckFlightSettings(&f), "a voxel past the largest cell");
}

int main(void)
{
    TestSettingsAreChecked();
    TestTheWorldsFloorAndBoxes();
    TestBoxesInTheAir();
    return s_failures == 0 ? 0 : 1;
}
