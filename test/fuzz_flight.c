// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes flight volumes (mnav-0015). The first byte picks the target:
//
// - Even: the tile loader. The rest is tile bytes, the payload's size and
//   hash set to match so that mutations reach the octree's checks. A tile
//   that loads must encode back to the same bytes but its generator's,
//   its form being canonical; every load must give back all it took.
// - Odd: the queries. The rest places up to 12 boxes in a volume of 2 by
//   2 tiles of 16 voxels, one cube high, and then asks for a path, a
//   raycast and a nearest point. Each is checked against every voxel's
//   openness: a path is found exactly where a breadth-first search over
//   the voxels finds one, its steps clear when sampled; a raycast stops
//   no later than sampling finds a blocked point, and only after open
//   ones; a nearest point is open and no farther than any open voxel.

#include "allocator.h"
#include "bytes.h"
#include "flight.h"
#include "flight_def.h"
#include "flight_tile.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    SIDE = 32,
    LAYERS = 16,
    BOXES = 12,
    TILE_BYTES = 1 << 16
};

// Stops the run at a failed check, naming its line.
static void ExpectAt(bool condition, int line)
{
    if (!condition)
    {
        fprintf(stderr, "fuzz_flight.c:%d: check failed\n", line);
        abort();
    }
}

#define Expect(condition) ExpectAt((condition), __LINE__)

// ---- the loader ----

// The def the loader's tiles are read with: 0.5 m voxels, tiles of 32,
// two cubes.
static mnavFlightDef LoaderDef(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.voxelSize = 0.5f;
    def.floor = -1.0f;
    def.ceiling = 30.0f;
    return def;
}

static void Load(const uint8_t* data, size_t size)
{
    uint8_t* bytes = malloc(size > 0 ? size : 1);
    Expect(bytes != nullptr);
    if (size > 0)
    {
        memcpy(bytes, data, size);
    }
    if (size >= MNAV_FLIGHT_HEADER_BYTES)
    {
        size_t payload = size - MNAV_FLIGHT_HEADER_BYTES;
        mnavByteWriter w = {bytes + 32};
        mnavPutU32(&w, payload);
        w.at = bytes + 24;
        mnavPutU64(&w,
                   mnavHash64(MNAV_HASH_INIT, bytes + MNAV_FLIGHT_HEADER_BYTES, (int32_t)payload));
    }
    mnavFlightDef def = LoaderDef();
    mnavFlightShape shape;
    Expect(mnavCheckFlightDef(&def, &shape).result == mnav_success);
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, (uint64_t)1 << 28);
    mnavFlightTile tile;
    uint64_t fingerprint = 0;
    mnavTileResult result =
        mnavDecodeFlightTile(&memory, &def, &shape, bytes, size, &tile, &fingerprint);
    if (result.result == mnav_success)
    {
        mnavVersion generator = {0, 0, 0};
        mnavByteReader r = {bytes + 8, 6, false};
        generator.major = (uint16_t)mnavGetU16(&r);
        generator.minor = (uint16_t)mnavGetU16(&r);
        generator.patch = (uint16_t)mnavGetU16(&r);
        uint8_t* again = nullptr;
        size_t againSize = 0;
        Expect(mnavEncodeFlightTile(&memory, &def, generator, fingerprint, &tile, &again,
                                    &againSize) == mnav_success);
        Expect(againSize == size && memcmp(again, bytes, size) == 0);
        mnavReleaseFlightTileBytes(&memory, again, againSize);
        mnavReleaseFlightTile(&memory, &tile);
    }
    else
    {
        Expect(result.result == mnav_errorInvalid || result.result == mnav_errorVersion ||
               result.result == mnav_errorLimit);
    }
    Expect(memory.used == 0);
    free(bytes);
}

// ---- the queries ----

// The bytes left to read; 0 past their end.
typedef struct Input
{
    const uint8_t* at;
    size_t left;
} Input;

static uint8_t Next(Input* in)
{
    if (in->left == 0)
    {
        return 0;
    }
    in->left -= 1;
    return *in->at++;
}

static mnavVec3 s_vertices[4 + BOXES * 8];
static int32_t s_indices[6 + BOXES * 36];
static bool s_open[SIDE][LAYERS][SIDE];

// A floor and up to 12 boxes of 1 to 8 voxels a side.
static mnavTriangleMesh Scene(Input* in)
{
    const mnavVec3 floor[4] = {{-4, 0, -4}, {-4, 0, 36}, {36, 0, 36}, {36, 0, -4}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    static const int32_t faces[36] = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                                      2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    int32_t boxes = Next(in) % (BOXES + 1);
    for (int32_t b = 0; b < boxes; ++b)
    {
        float x0 = (float)(Next(in) % SIDE);
        float y0 = (float)(Next(in) % LAYERS);
        float z0 = (float)(Next(in) % SIDE);
        float w = (float)(1 + Next(in) % 8);
        float h = (float)(1 + Next(in) % 8);
        float d = (float)(1 + Next(in) % 8);
        for (int32_t c = 0; c < 8; ++c)
        {
            s_vertices[4 + b * 8 + c] =
                (mnavVec3){x0 + ((c & 1) != 0 ? w : 0.0f), y0 + ((c & 2) != 0 ? h : 0.0f),
                           z0 + ((c & 4) != 0 ? d : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + b * 36 + k] = 4 + b * 8 + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, 4 + boxes * 8, s_indices, 2 + boxes * 12, nullptr};
}

static mnavFlightVolume* Bake(Input* in)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.tileVoxels = 16;
    def.floor = 0.0f;
    def.ceiling = 16.0f;
    uint8_t flags = Next(in);
    def.radius = (float)(flags % 3) * 0.75f;
    def.groundBelow = (flags & 4) != 0;
    mnavTriangleMesh scene = Scene(in);
    mnavBakeInput input = {&scene, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavFlightBaker* baker = nullptr;
    mnavFlightVolume* volume = nullptr;
    Expect(mnavCreateFlightBaker(&def, &baker).result == mnav_success);
    Expect(mnavCreateFlightVolume(&def, &volume).result == mnav_success);
    static uint8_t bytes[TILE_BYTES];
    for (int32_t t = 0; t < 4; ++t)
    {
        size_t size = 0;
        Expect(mnavBakeFlightTile(baker, &input, t % 2, t / 2, nullptr) == mnav_success);
        Expect(mnavCopyFlightTile(baker, bytes, TILE_BYTES, &size) == mnav_success);
        Expect(mnavStageFlightTile(volume, bytes, size).result == mnav_success);
    }
    Expect(mnavCommitFlight(volume) == mnav_success);
    mnavDestroyFlightBaker(baker);
    for (int32_t x = 0; x < SIDE; ++x)
    {
        for (int32_t y = 0; y < LAYERS; ++y)
        {
            for (int32_t z = 0; z < SIDE; ++z)
            {
                bool open = false;
                Expect(mnavIsFlightOpen(volume, (mnavPos3){x + 0.5, y + 0.5, z + 0.5}, &open) ==
                       mnav_success);
                s_open[x][y][z] = open;
            }
        }
    }
    return volume;
}

static bool Open(int32_t x, int32_t y, int32_t z)
{
    return x >= 0 && y >= 0 && z >= 0 && x < SIDE && y < LAYERS && z < SIDE && s_open[x][y][z];
}

// What a point holds: 0 open, 1 blocked (or past the floor or ceiling),
// 2 no tile.
static int32_t Held(double x, double y, double z)
{
    if (x < 0.0 || z < 0.0 || x >= SIDE || z >= SIDE)
    {
        return 2;
    }
    return Open((int32_t)floor(x), (int32_t)floor(y), (int32_t)floor(z)) ? 0 : 1;
}

static double Length(mnavPos3 a, mnavPos3 b)
{
    return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
}

// The first fraction of a segment, sampled every 1/64 voxel, that is not
// open, and what holds it; 2 when every sample is open.
static double FirstStop(mnavPos3 a, mnavPos3 b, double* stepOut, int32_t* heldOut)
{
    int32_t steps = (int32_t)(Length(a, b) * 64.0) + 1;
    *stepOut = 1.0 / steps;
    for (int32_t i = 0; i <= steps; ++i)
    {
        double t = (double)i / steps;
        int32_t held = Held(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
        if (held != 0)
        {
            *heldOut = held;
            return t;
        }
    }
    *heldOut = 0;
    return 2.0;
}

// Whether voxel b is reached from voxel a through open voxels sharing
// faces.
static bool Reachable(const int32_t a[3], const int32_t b[3])
{
    static bool seen[SIDE][LAYERS][SIDE];
    static int32_t queue[SIDE * LAYERS * SIDE][3];
    memset(seen, 0, sizeof(seen));
    int32_t head = 0;
    int32_t tail = 0;
    memcpy(queue[tail++], a, sizeof(queue[0]));
    seen[a[0]][a[1]][a[2]] = true;
    static const int32_t steps[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                        {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
    while (head < tail)
    {
        const int32_t* v = queue[head++];
        if (v[0] == b[0] && v[1] == b[1] && v[2] == b[2])
        {
            return true;
        }
        for (int32_t k = 0; k < 6; ++k)
        {
            int32_t x = v[0] + steps[k][0];
            int32_t y = v[1] + steps[k][1];
            int32_t z = v[2] + steps[k][2];
            if (Open(x, y, z) && !seen[x][y][z])
            {
                seen[x][y][z] = true;
                queue[tail][0] = x;
                queue[tail][1] = y;
                queue[tail][2] = z;
                ++tail;
            }
        }
    }
    return false;
}

static mnavPos3 Point(Input* in)
{
    double x = (Next(in) % 128) / 4.0 + 0.125;
    double y = (Next(in) % 64) / 4.0 + 0.125;
    double z = (Next(in) % 128) / 4.0 + 0.125;
    return (mnavPos3){x, y, z};
}

static void Voxel(mnavPos3 p, int32_t v[3])
{
    v[0] = (int32_t)floor(p.x);
    v[1] = (int32_t)floor(p.y);
    v[2] = (int32_t)floor(p.z);
}

static void CheckPath(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 a, mnavPos3 b)
{
    mnavFlightPath path;
    Expect(mnavFindFlightPath(query, volume, a, b, &path) == mnav_success);
    int32_t va[3];
    int32_t vb[3];
    Voxel(a, va);
    Voxel(b, vb);
    if (!Open(va[0], va[1], va[2]) || !Open(vb[0], vb[1], vb[2]))
    {
        Expect(path.end == mnav_pathNone && path.pointCount == 1);
        return;
    }
    if (!Reachable(va, vb))
    {
        Expect(path.end == mnav_pathNone || path.end == mnav_pathNotLoaded);
        return;
    }
    int32_t last = path.pointCount - 1;
    Expect(path.end == mnav_pathFound && path.pointCount >= 2);
    Expect(path.points[0].x == a.x && path.points[0].y == a.y && path.points[0].z == a.z);
    Expect(path.points[last].x == b.x && path.points[last].y == b.y && path.points[last].z == b.z);
    double sum = 0.0;
    for (int32_t i = 0; i < last; ++i)
    {
        double step = 0.0;
        int32_t held = 0;
        (void)FirstStop(path.points[i], path.points[i + 1], &step, &held);
        Expect(held == 0);
        sum += Length(path.points[i], path.points[i + 1]);
    }
    Expect(fabs(sum - path.length) < 1e-6);
}

static void CheckRaycast(const mnavFlightVolume* volume, mnavPos3 a, mnavPos3 b)
{
    mnavFlightHit hit;
    Expect(mnavFlightRaycast(volume, a, b, &hit) == mnav_success);
    double step = 0.0;
    int32_t held = 0;
    double first = FirstStop(a, b, &step, &held);
    if (hit.stop == mnav_flightClear)
    {
        Expect(held == 0 && hit.t == 1.0);
        return;
    }
    // Never past the first sample not open; and every sample before the
    // stop open, so that only an edge or a corner stops it sooner.
    Expect(hit.t <= first + 1e-9 && hit.t >= 0.0);
    double before = 0.0;
    int32_t heldBefore = 0;
    mnavPos3 stop = {a.x + (b.x - a.x) * hit.t, a.y + (b.y - a.y) * hit.t,
                     a.z + (b.z - a.z) * hit.t};
    if (hit.t > 0.0)
    {
        double reach = FirstStop(a, stop, &before, &heldBefore);
        Expect(heldBefore == 0 || reach >= 1.0 - before - 1e-9);
    }
}

static void CheckNearest(const mnavFlightVolume* volume, mnavPos3 p, float radius)
{
    mnavPos3 nearest;
    bool found = false;
    Expect(mnavFindNearestFlightPoint(volume, p, radius, &nearest, &found) == mnav_success);
    double best = (double)INFINITY;
    for (int32_t x = 0; x < SIDE; ++x)
    {
        for (int32_t y = 0; y < LAYERS; ++y)
        {
            for (int32_t z = 0; z < SIDE; ++z)
            {
                if (!s_open[x][y][z])
                {
                    continue;
                }
                double qx = fmin(fmax(p.x, x), x + 1.0);
                double qy = fmin(fmax(p.y, y), y + 1.0);
                double qz = fmin(fmax(p.z, z), z + 1.0);
                best = fmin(best, Length(p, (mnavPos3){qx, qy, qz}));
            }
        }
    }
    if (found)
    {
        bool open = false;
        Expect(mnavIsFlightOpen(volume, nearest, &open) == mnav_success && open);
        Expect(Length(p, nearest) <= (double)radius + 2e-3 && Length(p, nearest) <= best + 2e-3);
    }
    else
    {
        Expect(best > (double)radius - 2e-3);
    }
}

static void Query(const uint8_t* data, size_t size)
{
    Input in = {data, size};
    mnavFlightVolume* volume = Bake(&in);
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = 32768;
    mnavQuery* query = nullptr;
    Expect(mnavCreateQuery(&def, &query) == mnav_success);
    for (int32_t k = 0; k < 4 && in.left > 0; ++k)
    {
        mnavPos3 a = Point(&in);
        mnavPos3 b = Point(&in);
        CheckPath(query, volume, a, b);
        CheckRaycast(volume, a, b);
        CheckNearest(volume, a, (float)(Next(&in) % 16) * 0.5f);
    }
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size == 0 || size > ((size_t)1 << 24))
    {
        return 0;
    }
    if ((data[0] & 1) == 0)
    {
        Load(data + 1, size - 1);
    }
    else
    {
        Query(data + 1, size - 1);
    }
    return 0;
}
