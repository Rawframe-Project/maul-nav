// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The flight volume's octree as its tiles hold it: every voxel's openness
// against the octree decoded from the tile's bytes; the walk along a
// segment against the plain one, voxel by voxel; every broken structure
// of the payload refused in its section.
//
// White-box: the API answers openness point by point (mnavFlightVolume_IsOpen)
// but shows no tile's octree to check it against, no walk to compare, and
// no payload to break past its hash without the format's writer.

#include "bytes.h"
#include "flight.h"
#include "flight_def.h"
#include "flight_scene.h"
#include "flight_space.h"
#include "flight_tile.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Bakes the test world's four tiles into s_bytes.
static void BakeAll(void)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    mnavBakeInput input = {&world, 1, nullptr, 0, nullptr, 0, nullptr};
    for (int32_t t = 0; t < 4; ++t)
    {
        (void)BakeOne(baker, &input, t);
    }
    mnavDestroyFlightBaker(baker);
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
                wrong += mnavFlightVolume_IsOpen(volume, p, &got) != mnav_success ? 1 : 0;
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

static void TestVoxelsAsTheOctree(void)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    int32_t staged = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        staged += mnavFlightVolume_StageTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
    }
    CHECK(staged == 4 && mnavFlightVolume_Commit(volume) == mnav_success, "committed");
    for (int32_t t = 0; t < 4; ++t)
    {
        CheckVoxels(volume, t);
    }
    mnavDestroyFlightVolume(volume);
}

// What holds a voxel, looked up alone.
static int32_t HeldAlone(const mnavFlightVolume* volume, const int32_t p[3])
{
    mnavFlightCursor cursor = mnavMakeFlightCursor(volume);
    return mnavFlightHolder(&cursor, p[0], p[1], p[2], nullptr);
}

// The walk the plain way: voxel by voxel, every voxel touched looked up
// alone, and where the segment passes an edge or a corner, each voxel a
// part of the steps reaches before the one all of them reach.
static int32_t PlainWalk(const mnavFlightVolume* volume, const double a[3], const double b[3],
                         double* tOut)
{
    int32_t at[3];
    int32_t step[3];
    int32_t end[3];
    double next[3];
    double delta[3];
    for (int32_t k = 0; k < 3; ++k)
    {
        double d = b[k] - a[k];
        at[k] = (int32_t)floor(a[k]);
        end[k] = (int32_t)floor(b[k]);
        step[k] = d > 0.0 ? 1 : (d < 0.0 ? -1 : 0);
        double edge = step[k] > 0 ? (double)at[k] + 1.0 : (double)at[k];
        next[k] = step[k] != 0 ? (edge - a[k]) / d : (double)INFINITY;
        delta[k] = step[k] != 0 ? fabs(1.0 / d) : (double)INFINITY;
    }
    double t = 0.0;
    int32_t held = HeldAlone(volume, at);
    while (held == MNAV_SPACE_OPEN && (at[0] != end[0] || at[1] != end[1] || at[2] != end[2]))
    {
        t = fmin(next[0], fmin(next[1], next[2]));
        if (t > 1.0)
        {
            return MNAV_SPACE_OPEN;
        }
        uint32_t tied = 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            tied |= next[k] - t < 1e-9 ? 1u << k : 0u;
        }
        // The parts of the steps in order, the whole of them last.
        for (uint32_t m = 1; m < 8 && held == MNAV_SPACE_OPEN; ++m)
        {
            if ((m & ~tied) != 0)
            {
                continue;
            }
            int32_t p[3];
            for (int32_t k = 0; k < 3; ++k)
            {
                p[k] = at[k] + ((m >> k & 1u) != 0 ? step[k] : 0);
            }
            held = HeldAlone(volume, p);
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            at[k] += (tied >> k & 1u) != 0 ? step[k] : 0;
            next[k] += (tied >> k & 1u) != 0 ? delta[k] : 0.0;
        }
    }
    if (held != MNAV_SPACE_OPEN)
    {
        *tOut = t;
    }
    return held;
}

static uint32_t s_walkRandom = 7;

// A coordinate from lo to hi: a real one, or on a voxel's center or face.
static double WalkCoordinate(double lo, double hi)
{
    s_walkRandom = s_walkRandom * 1664525u + 1013904223u;
    double x = lo + (hi - lo) * (double)(s_walkRandom >> 8) / 16777216.0;
    switch (s_walkRandom & 3u)
    {
    case 0:
        return floor(x) + 0.5;
    case 1:
        return floor(x);
    default:
        return x;
    }
}

// A segment of kind k: anywhere, along an axis, along a diagonal of two
// or three axes, or within a few voxels; through the test world's four
// tiles in voxels, and past them, the floor and the ceiling.
static void WalkSegment(int32_t k, int32_t layers, double a[3], double b[3])
{
    const double side = 128.0;
    for (int32_t i = 0; i < 3; ++i)
    {
        double hi = i == 1 ? (double)layers + 2.0 : side + 4.0;
        a[i] = WalkCoordinate(-4.0, hi);
        b[i] = WalkCoordinate(-4.0, hi);
    }
    double d = WalkCoordinate(-40.0, 40.0);
    switch (k % 5)
    {
    case 1:
        b[(k / 5) % 3] = a[(k / 5) % 3] + d;
        b[(k / 5 + 1) % 3] = a[(k / 5 + 1) % 3];
        b[(k / 5 + 2) % 3] = a[(k / 5 + 2) % 3];
        break;
    case 2:
        b[0] = a[0] + d;
        b[1] = a[1] + ((k & 8) != 0 ? d : -d);
        b[2] = a[2] + ((k & 16) != 0 ? d : -d);
        break;
    case 3:
        b[(k / 5) % 3] = a[(k / 5) % 3];
        b[(k / 5 + 1) % 3] = a[(k / 5 + 1) % 3] + d;
        b[(k / 5 + 2) % 3] = a[(k / 5 + 2) % 3] - d;
        break;
    case 4:
        for (int32_t i = 0; i < 3; ++i)
        {
            b[i] = a[i] + d / 16.0 * (double)(i + 1);
        }
        break;
    default:
        break;
    }
}

// The walk, with the blocks it keeps and steps over, against the plain
// one: what stops it, and where, to the bit, on every segment.
static void TestWalk(void)
{
    mnavFlightDef def = Def();
    mnavFlightVolume* volume = Volume(&def);
    int32_t staged = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        staged += mnavFlightVolume_StageTile(volume, s_bytes[t], s_sizes[t]).result == mnav_success;
    }
    CHECK(staged == 4 && mnavFlightVolume_Commit(volume) == mnav_success, "committed");
    mnavFlightCursor cursor = mnavMakeFlightCursor(volume);
    int32_t wrong = 0;
    int32_t held[3] = {0, 0, 0};
    for (int32_t k = 0; k < 40000; ++k)
    {
        double a[3];
        double b[3];
        WalkSegment(k, cursor.layers, a, b);
        double t = -1.0;
        double plainT = -1.0;
        int32_t walk = mnavFlightWalk(&cursor, a, b, &t);
        int32_t plain = PlainWalk(volume, a, b, &plainT);
        wrong += walk == plain && memcmp(&t, &plainT, sizeof(t)) == 0 ? 0 : 1;
        held[walk] += 1;
    }
    printf("walks: %d clear, %d solid, %d unloaded, %d wrong\n", held[0], held[1], held[2], wrong);
    CHECK(wrong == 0 && held[MNAV_SPACE_OPEN] > 0 && held[MNAV_SPACE_SOLID] > 0 &&
              held[MNAV_SPACE_UNLOADED] > 0,
          "every walk as the plain one");
    mnavDestroyFlightVolume(volume);
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

// The tile's first root, node and leaf, and where they start.
static void TestPayloadStructure(void)
{
    size_t size = s_sizes[1];
    size_t length = 0;
    size_t roots = FlightFieldAt(s_bytes[1], "ROOT", &length);
    size_t nodes = FlightFieldAt(s_bytes[1], "NODE", &length);
    size_t leaves = FlightFieldAt(s_bytes[1], "LEAF", &length);
    CHECK(roots > 0 && nodes > 0 && leaves > 0, "tile 1 has every field");
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
        bool right = got.result == mnav_errorFormat && got.section == cases[k].section &&
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
    // A spare leaf: one more stored than the octree uses, at the end of
    // LEAF, whose length grows by it.
    size_t end = leaves + length;
    memcpy(s_bad, s_bytes[1], end);
    memcpy(s_bad + end + 8, s_bytes[1] + end, size - end);
    mnavByteWriter w = {s_bad + end};
    mnavPutU64(&w, 1);
    w.at = s_bad + leaves - 4;
    mnavPutU32(&w, length + 8);
    w.at = s_bad + AT_PAYLOAD;
    mnavPutU32(&w, size + 8 - MNAV_FLIGHT_HEADER_BYTES);
    Rehash(size + 8);
    got = StageBad(size + 8);
    wrong += got.section == mnav_tileFlightLeaves && got.index == -1 ? 0 : 1;
    CHECK(wrong == 0, "every broken structure refused in its section");
}

int main(void)
{
    BakeAll();
    TestVoxelsAsTheOctree();
    TestWalk();
    TestPayloadStructure();
    return s_failures == 0 ? 0 : 1;
}
