// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The flier's queries (mnav-0015) against plain searches over a grid of
// every voxel's openness, read through mnavIsFlightOpen: paths found
// exactly where the grid's 26-connected search finds one, every step in
// sight, never much longer than the grid's shortest; raycasts against
// fine sampling; nearest open points against every voxel; and the ends,
// limits and refusals.

#include "counting_allocator.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    SIDE = 64,
    LAYERS = 32,
    BOXES = 60,
    TILE_BYTES = 1 << 18
};

// The scene: a floor 64 m square and boxes floating over it.
static mnavVec3 s_vertices[4 + BOXES * 8];
static int32_t s_indices[6 + BOXES * 36];
static bool s_open[SIDE][LAYERS][SIDE];

static uint32_t s_random = 12345;

static float Random(void)
{
    s_random = s_random * 1664525u + 1013904223u;
    return (float)(s_random >> 8 & 0xFFFFu) / 65536.0f;
}

static mnavTriangleMesh Scene(void)
{
    const mnavVec3 floor[4] = {{-2, 0, -2}, {-2, 0, 66}, {66, 0, 66}, {66, 0, -2}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    static const int32_t faces[36] = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                                      2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        float w = 2.0f + 8.0f * Random();
        float h = 1.0f + 6.0f * Random();
        float x0 = Random() * (64.0f - w);
        float y0 = 2.0f + Random() * 22.0f;
        float z0 = Random() * (64.0f - w);
        for (int32_t c = 0; c < 8; ++c)
        {
            s_vertices[4 + b * 8 + c] =
                (mnavVec3){x0 + ((c & 1) != 0 ? w : 0.0f), y0 + ((c & 2) != 0 ? h : 0.0f),
                           z0 + ((c & 4) != 0 ? w : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + b * 36 + k] = 4 + b * 8 + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, 4 + BOXES * 8, s_indices, 2 + BOXES * 12, nullptr};
}

// 1 m voxels, tiles of 16, from 1 m below the floor up 32 m.
static mnavFlightDef Def(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.allocator = CountingAllocator();
    def.tileVoxels = 16;
    def.floor = -1.0f;
    def.ceiling = 31.0f;
    def.radius = 0.5f;
    return def;
}

// Bakes the scene's 16 tiles, all but those in skip, into a volume.
static mnavFlightVolume* Volume(int32_t skipX, int32_t skipZ)
{
    mnavFlightDef def = Def();
    mnavFlightBaker* baker = nullptr;
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success &&
              mnavCreateFlightVolume(&def, &volume).result == mnav_success,
          "baker and volume");
    mnavTriangleMesh scene = Scene();
    mnavBakeInput input = {&scene, 1, nullptr, 0, nullptr, 0, nullptr};
    static uint8_t bytes[TILE_BYTES];
    int32_t wrong = 0;
    for (int32_t t = 0; t < 16; ++t)
    {
        if (t % 4 == skipX && t / 4 == skipZ)
        {
            continue;
        }
        size_t size = 0;
        wrong += mnavBakeFlightTile(baker, &input, t % 4, t / 4, nullptr) != mnav_success;
        wrong += mnavCopyFlightTile(baker, bytes, TILE_BYTES, &size) != mnav_success;
        wrong += mnavStageFlightTile(volume, bytes, size).result != mnav_success;
    }
    CHECK(wrong == 0 && mnavCommitFlight(volume) == mnav_success, "the scene's tiles");
    mnavDestroyFlightBaker(baker);
    return volume;
}

// The world point at voxel (x, y, z) plus a fraction of it.
static mnavPos3 At(double x, double y, double z)
{
    return (mnavPos3){x, y - 1.0, z};
}

static void ReadOpen(const mnavFlightVolume* volume)
{
    int32_t open = 0;
    for (int32_t x = 0; x < SIDE; ++x)
    {
        for (int32_t y = 0; y < LAYERS; ++y)
        {
            for (int32_t z = 0; z < SIDE; ++z)
            {
                bool o = false;
                CHECK(mnavIsFlightOpen(volume, At(x + 0.5, y + 0.5, z + 0.5), &o) == mnav_success,
                      "read");
                s_open[x][y][z] = o;
                open += o ? 1 : 0;
            }
        }
    }
    printf("%d of %d voxels open\n", open, SIDE * LAYERS * SIDE);
}

static bool Open(int32_t x, int32_t y, int32_t z)
{
    return x >= 0 && y >= 0 && z >= 0 && x < SIDE && y < LAYERS && z < SIDE && s_open[x][y][z];
}

// ---- the grid's shortest path, 26-connected, no corner cutting ----
static double s_cost[SIDE][LAYERS][SIDE];
// An indexed heap of voxels by their cost, and each voxel's place in it,
// -1 when not in it.
static int32_t s_heap[SIDE * LAYERS * SIDE];
static int32_t s_place[SIDE * LAYERS * SIDE];
static int32_t s_heapCount;

static double CostOf(int32_t v)
{
    return (&s_cost[0][0][0])[v];
}

static void HeapSet(int32_t at, int32_t v)
{
    s_heap[at] = v;
    s_place[v] = at;
}

static void HeapUp(int32_t at)
{
    int32_t v = s_heap[at];
    while (at > 0 && CostOf(s_heap[(at - 1) / 2]) > CostOf(v))
    {
        HeapSet(at, s_heap[(at - 1) / 2]);
        at = (at - 1) / 2;
    }
    HeapSet(at, v);
}

// Puts a voxel in the heap, or moves it up after its cost fell.
static void HeapPush(int32_t v)
{
    if (s_place[v] < 0)
    {
        HeapSet(s_heapCount++, v);
    }
    HeapUp(s_place[v]);
}

static int32_t HeapPop(void)
{
    int32_t top = s_heap[0];
    s_place[top] = -1;
    int32_t v = s_heap[--s_heapCount];
    if (s_heapCount == 0)
    {
        return top;
    }
    int32_t at = 0;
    for (;;)
    {
        int32_t c = 2 * at + 1;
        if (c >= s_heapCount)
        {
            break;
        }
        c += c + 1 < s_heapCount && CostOf(s_heap[c + 1]) < CostOf(s_heap[c]) ? 1 : 0;
        if (CostOf(s_heap[c]) >= CostOf(v))
        {
            break;
        }
        HeapSet(at, s_heap[c]);
        at = c;
    }
    HeapSet(at, v);
    return top;
}

// Whether a step of (dx, dy, dz) from a voxel keeps to open voxels: every
// voxel in its box.
static bool CanStep(int32_t x, int32_t y, int32_t z, int32_t dx, int32_t dy, int32_t dz)
{
    for (int32_t i = 0; i <= abs(dx); ++i)
    {
        for (int32_t j = 0; j <= abs(dy); ++j)
        {
            for (int32_t k = 0; k <= abs(dz); ++k)
            {
                if (!Open(x + i * dx, y + j * dy, z + k * dz))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

static void Relax(int32_t x, int32_t y, int32_t z, double cost)
{
    for (int32_t d = 0; d < 27; ++d)
    {
        int32_t dx = d % 3 - 1;
        int32_t dy = d / 3 % 3 - 1;
        int32_t dz = d / 9 - 1;
        if ((dx | dy | dz) == 0 || !CanStep(x, y, z, dx, dy, dz))
        {
            continue;
        }
        double step = sqrt((double)(dx * dx + dy * dy + dz * dz));
        if (cost + step < s_cost[x + dx][y + dy][z + dz])
        {
            s_cost[x + dx][y + dy][z + dz] = cost + step;
            HeapPush(((x + dx) * LAYERS + y + dy) * SIDE + z + dz);
        }
    }
}

// The grid's shortest way between voxel centers, Dijkstra's; -1 for none.
static double GridShortest(const int32_t a[3], const int32_t b[3])
{
    for (int32_t i = 0; i < SIDE * LAYERS * SIDE; ++i)
    {
        (&s_cost[0][0][0])[i] = (double)INFINITY;
        s_place[i] = -1;
    }
    s_heapCount = 0;
    s_cost[a[0]][a[1]][a[2]] = 0.0;
    HeapPush((a[0] * LAYERS + a[1]) * SIDE + a[2]);
    while (s_heapCount > 0)
    {
        int32_t v = HeapPop();
        int32_t x = v / (LAYERS * SIDE);
        int32_t y = v / SIDE % LAYERS;
        int32_t z = v % SIDE;
        if (x == b[0] && y == b[1] && z == b[2])
        {
            return s_cost[x][y][z];
        }
        Relax(x, y, z, s_cost[x][y][z]);
    }
    return -1.0;
}

static void RandomOpen(int32_t v[3])
{
    do
    {
        v[0] = (int32_t)(Random() * SIDE);
        v[1] = (int32_t)(Random() * LAYERS);
        v[2] = (int32_t)(Random() * SIDE);
    } while (!Open(v[0], v[1], v[2]));
}

// Whether a segment keeps to open voxels, sampled every 1/64 voxel.
static bool Clear(mnavPos3 a, mnavPos3 b)
{
    double length =
        sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
    int32_t steps = (int32_t)(length * 64.0) + 1;
    for (int32_t i = 0; i <= steps; ++i)
    {
        double t = (double)i / steps;
        double x = a.x + (b.x - a.x) * t;
        double y = a.y + (b.y - a.y) * t + 1.0;
        double z = a.z + (b.z - a.z) * t;
        if (!Open((int32_t)floor(x), (int32_t)floor(y), (int32_t)floor(z)))
        {
            return false;
        }
    }
    return true;
}

static mnavQuery* Query(int32_t nodes, float pathLength)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.allocator = CountingAllocator();
    def.limits.nodes = nodes;
    def.limits.pathLength = pathLength;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

// Checks one path: its ends, every step in sight, its length the sum of
// its steps; returns its length over the grid's shortest, or 0 when the
// grid finds none, as the search must not.
static double CheckPath(mnavQuery* query, const mnavFlightVolume* volume, const int32_t a[3],
                        const int32_t b[3], int32_t* wrongOut)
{
    mnavPos3 start = At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5);
    mnavPos3 end = At(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5);
    mnavFlightPath path;
    CHECK(mnavFindFlightPath(query, volume, start, end, &path) == mnav_success, "searched");
    double grid = GridShortest(a, b);
    if (grid < 0.0)
    {
        // The scene's sides border no tile, so a search that runs out of
        // ways may report them; one that proves there is no way may also
        // spend its budget first.
        bool none = path.end == mnav_pathNone || path.end == mnav_pathNotLoaded ||
                    path.end == mnav_pathOutOfNodes;
        if (!none)
        {
            printf("no grid path, search end %d\n", path.end);
        }
        *wrongOut += none ? 0 : 1;
        return 0.0;
    }
    int32_t last = path.pointCount - 1;
    bool ends = path.end == mnav_pathFound && path.pointCount >= 2 && path.points[0].x == start.x &&
                path.points[0].y == start.y && path.points[0].z == start.z &&
                path.points[last].x == end.x && path.points[last].y == end.y &&
                path.points[last].z == end.z;
    double sum = 0.0;
    bool clear = true;
    for (int32_t i = 0; i < last; ++i)
    {
        mnavPos3 p = path.points[i];
        mnavPos3 q = path.points[i + 1];
        sum +=
            sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
        if (!Clear(p, q))
        {
            mnavFlightHit hit;
            (void)mnavFlightRaycast(volume, p, q, &hit);
            printf("  segment %d of %d: (%.4f %.4f %.4f) to (%.4f %.4f %.4f), raycast stop %d\n", i,
                   last, p.x, p.y + 1.0, p.z, q.x, q.y + 1.0, q.z, hit.stop);
            clear = false;
        }
    }
    if (!(ends && clear && fabs(sum - path.length) < 1e-6))
    {
        printf("path (%d %d %d) to (%d %d %d): end %d, %d points, ends %d, clear %d, length %.6f "
               "against %.6f\n",
               a[0], a[1], a[2], b[0], b[1], b[2], path.end, path.pointCount, ends, clear,
               path.length, sum);
        *wrongOut += 1;
    }
    return grid > 0.0 ? path.length / grid : 1.0;
}

static void TestPaths(const mnavFlightVolume* volume)
{
    mnavQuery* query = Query(16384, 1000.0f);
    int32_t wrong = 0;
    int32_t none = 0;
    double worst = 0.0;
    double sum = 0.0;
    int32_t found = 0;
    for (int32_t k = 0; k < 120; ++k)
    {
        int32_t a[3];
        int32_t b[3];
        RandomOpen(a);
        RandomOpen(b);
        double ratio = CheckPath(query, volume, a, b, &wrong);
        none += ratio == 0.0 ? 1 : 0;
        found += ratio > 0.0 ? 1 : 0;
        sum += ratio;
        worst = ratio > worst ? ratio : worst;
    }
    printf("paths: %d found, %d none, length over the grid's mean %.4f worst %.4f, %d wrong\n",
           found, none, sum / found, worst, wrong);
    CHECK(wrong == 0, "every path as the grid finds it, in sight");
    CHECK(worst < 1.15, "never much longer than the grid's shortest");
    mnavDestroyQuery(query);
}

static void FarEnds(int32_t a[3], int32_t b[3]);

// A path run in slices of a number of nodes: begun, continued until it
// ends, finished.
static mnavResult Sliced(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 a, mnavPos3 b,
                         int32_t nodes, mnavFlightPath* path)
{
    mnavResult result = mnavBeginFlightPath(query, volume, a, b);
    bool ended = false;
    while (result == mnav_success && !ended)
    {
        result = mnavContinueFlightPath(query, volume, nodes, &ended);
    }
    return result == mnav_success ? mnavFinishFlightPath(query, volume, path) : result;
}

// Paths run in slices of any size are the paths found in one call, to
// the bit; a search finished early ends unfinished; a search on a volume
// committed to since, or on another, is stale.
static void TestSlices(const mnavFlightVolume* volume)
{
    mnavQuery* query = Query(16384, 1000.0f);
    static mnavPos3 whole[2 * 16384 + 2];
    const int32_t sizes[4] = {1, 3, 17, 100000};
    int32_t wrong = 0;
    for (int32_t k = 0; k < 40; ++k)
    {
        int32_t a[3];
        int32_t b[3];
        RandomOpen(a);
        RandomOpen(b);
        mnavPos3 from = At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5);
        mnavPos3 to = At(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5);
        mnavFlightPath once;
        CHECK(mnavFindFlightPath(query, volume, from, to, &once) == mnav_success, "a path");
        memcpy(whole, once.points, (size_t)once.pointCount * sizeof(mnavPos3));
        for (int32_t z = 0; z < 4; ++z)
        {
            mnavFlightPath sliced;
            bool same =
                Sliced(query, volume, from, to, sizes[z], &sliced) == mnav_success &&
                sliced.end == once.end && sliced.length == once.length &&
                sliced.pointCount == once.pointCount &&
                memcmp(sliced.points, whole, (size_t)once.pointCount * sizeof(mnavPos3)) == 0;
            wrong += same ? 0 : 1;
        }
    }
    CHECK(wrong == 0, "the same path whatever the slices");
    // Finished after one node: toward the point searched nearest.
    int32_t a[3];
    int32_t b[3];
    FarEnds(a, b);
    mnavPos3 from = At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5);
    mnavPos3 to = At(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5);
    bool ended = true;
    mnavFlightPath early;
    CHECK(mnavBeginFlightPath(query, volume, from, to) == mnav_success &&
              mnavContinueFlightPath(query, volume, 1, &ended) == mnav_success && !ended &&
              mnavFinishFlightPath(query, volume, &early) == mnav_success &&
              early.end == mnav_pathUnfinished && early.pointCount >= 1 &&
              early.points[0].x == from.x && early.points[0].y == from.y &&
              early.points[0].z == from.z,
          "unfinished, from the start");
    CHECK(mnavContinueFlightPath(query, volume, 1, &ended) == mnav_errorInvalid &&
              mnavFinishFlightPath(query, volume, &early) == mnav_errorInvalid,
          "finished: no search left");
    CHECK(mnavBeginFlightPath(query, volume, from, to) == mnav_success &&
              mnavContinueFlightPath(query, volume, 0, &ended) == mnav_errorInvalid &&
              mnavContinueFlightPath(query, volume, 1, nullptr) == mnav_errorInvalid &&
              mnavContinueFlightPath(nullptr, volume, 1, &ended) == mnav_errorInvalid &&
              mnavFinishFlightPath(query, volume, nullptr) == mnav_errorInvalid &&
              mnavBeginFlightPath(query, nullptr, from, to) == mnav_errorInvalid,
          "bad arguments");
    // A search of another kind uses the context's nodes: the flight
    // search is over.
    static const mnavAreaType areas[4] = {mnav_areaWalkable, mnav_areaWalkable, mnav_areaWalkable,
                                          mnav_areaWalkable};
    const mnavGrid grid = {areas, 2, 2, 1.0f};
    mnavGridPath gridPath;
    CHECK(mnavBeginFlightPath(query, volume, from, to) == mnav_success &&
              mnavFindGridPath(query, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){1, 1},
                               &gridPath) == mnav_success &&
              mnavContinueFlightPath(query, volume, 1, &ended) == mnav_errorInvalid,
          "another search ends it");
    // Another volume, then the same one committed to.
    mnavFlightVolume* other = Volume(0, 0);
    CHECK(mnavBeginFlightPath(query, volume, from, to) == mnav_success &&
              mnavContinueFlightPath(query, other, 1, &ended) == mnav_errorStale &&
              mnavFinishFlightPath(query, other, &early) == mnav_errorStale,
          "another volume: stale");
    CHECK(mnavBeginFlightPath(query, other, from, to) == mnav_success &&
              mnavStageFlightTileRemoval(other, 3, 3) == mnav_success &&
              mnavCommitFlight(other) == mnav_success &&
              mnavContinueFlightPath(query, other, 1, &ended) == mnav_errorStale,
          "committed to since: stale");
    mnavDestroyFlightVolume(other);
    mnavDestroyQuery(query);
}

// The path check stops where casting each step's ray in turn first
// stops, with the same hit; found paths check clear; after a tile under a
// path is removed, the check stops at the step into the hole.
static void TestCheck(const mnavFlightVolume* volume)
{
    int32_t wrong = 0;
    int32_t stopped = 0;
    for (int32_t k = 0; k < 200; ++k)
    {
        mnavPos3 points[6];
        for (int32_t i = 0; i < 6; ++i)
        {
            int32_t v[3];
            RandomOpen(v);
            // One draw a statement: their order is the same everywhere.
            double dx = 0.25 + 0.5 * (double)Random();
            double dz = 0.25 + 0.5 * (double)Random();
            points[i] = At(v[0] + dx, v[1] + 0.5, v[2] + dz);
        }
        int32_t expected = -1;
        mnavFlightHit first = {0};
        for (int32_t i = 0; i < 5 && expected < 0; ++i)
        {
            mnavFlightHit hit;
            CHECK(mnavFlightRaycast(volume, points[i], points[i + 1], &hit) == mnav_success, "ray");
            if (hit.stop != mnav_flightClear)
            {
                expected = i;
                first = hit;
            }
        }
        int32_t step = 7;
        mnavFlightHit hit = {0};
        CHECK(mnavCheckFlightPath(volume, points, 6, &step, &hit) == mnav_success, "checked");
        bool same = step == expected &&
                    (step < 0 || (hit.stop == first.stop && hit.t == first.t &&
                                  memcmp(&hit.point, &first.point, sizeof(mnavPos3)) == 0));
        wrong += same ? 0 : 1;
        stopped += step >= 0 ? 1 : 0;
    }
    CHECK(wrong == 0 && stopped > 20, "where the rays stop, with their hits");
    // Paths found are clear.
    mnavQuery* query = Query(16384, 1000.0f);
    int32_t blocked = 0;
    for (int32_t k = 0; k < 30; ++k)
    {
        int32_t a[3];
        int32_t b[3];
        RandomOpen(a);
        RandomOpen(b);
        mnavFlightPath found;
        int32_t step = 7;
        CHECK(mnavFindFlightPath(query, volume, At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5),
                                 At(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5), &found) == mnav_success &&
                  mnavCheckFlightPath(volume, found.points, found.pointCount, &step, nullptr) ==
                      mnav_success,
              "found and checked");
        blocked += step == -1 ? 0 : 1;
    }
    CHECK(blocked == 0, "paths found are clear");
    // A path found clears; with its middle tile gone, it stops there.
    mnavFlightVolume* changed = Volume(-1, -1);
    int32_t ya = 1;
    int32_t yb = 1;
    while (ya < LAYERS - 1 && !Open(4, ya, 20))
    {
        ++ya;
    }
    while (yb < LAYERS - 1 && !Open(59, yb, 20))
    {
        ++yb;
    }
    mnavPos3 from = At(4.5, ya + 0.5, 20.5);
    mnavPos3 to = At(59.5, yb + 0.5, 20.5);
    mnavFlightPath path;
    int32_t step = 7;
    CHECK(mnavFindFlightPath(query, changed, from, to, &path) == mnav_success &&
              path.end == mnav_pathFound &&
              mnavCheckFlightPath(changed, path.points, path.pointCount, &step, nullptr) ==
                  mnav_success &&
              step == -1,
          "a path found is clear");
    static mnavPos3 kept[2 * 16384 + 2];
    int32_t count = path.pointCount;
    memcpy(kept, path.points, (size_t)count * sizeof(mnavPos3));
    CHECK(mnavStageFlightTileRemoval(changed, 2, 1) == mnav_success &&
              mnavCommitFlight(changed) == mnav_success,
          "a tile removed");
    mnavFlightHit hit;
    CHECK(mnavCheckFlightPath(changed, kept, count, &step, &hit) == mnav_success && step >= 0 &&
              hit.stop == mnav_flightNotLoaded && hit.point.x >= 32.0 && hit.point.x <= 48.0,
          "stopped where the tile is gone");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(changed);
    // Refusals change nothing; no steps is clear.
    mnavPos3 bad[2] = {from, {(double)NAN, 0.0, 0.0}};
    step = 7;
    CHECK(mnavCheckFlightPath(volume, bad, 2, &step, nullptr) == mnav_errorRange && step == 7 &&
              mnavCheckFlightPath(nullptr, bad, 1, &step, nullptr) == mnav_errorInvalid &&
              mnavCheckFlightPath(volume, nullptr, 2, &step, nullptr) == mnav_errorInvalid &&
              mnavCheckFlightPath(volume, bad, -1, &step, nullptr) == mnav_errorInvalid &&
              mnavCheckFlightPath(volume, bad, 1, nullptr, nullptr) == mnav_errorInvalid,
          "refusals");
    CHECK(mnavCheckFlightPath(volume, nullptr, 0, &step, nullptr) == mnav_success && step == -1 &&
              mnavCheckFlightPath(volume, &from, 1, &step, nullptr) == mnav_success && step == -1,
          "no steps: clear");
}

// What a sample point holds: 0 open, 1 blocked (a solid voxel, or past
// the floor or ceiling), 2 no tile.
static int32_t Sample(double x, double y, double z)
{
    if (x < 0.0 || z < 0.0 || x >= SIDE || z >= SIDE)
    {
        return 2;
    }
    return Open((int32_t)floor(x), (int32_t)floor(y), (int32_t)floor(z)) ? 0 : 1;
}

// The fraction of a ray, in voxels, where sampling every 1/256 voxel
// first meets a point not open, and what holds it; 2 for none.
static double s_sampleStep;

static double FirstStop(const double a[3], const double b[3], int32_t* stopOut)
{
    double length = sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]) +
                         (b[2] - a[2]) * (b[2] - a[2]));
    int32_t steps = (int32_t)(length * 256.0) + 1;
    s_sampleStep = 1.0 / steps;
    for (int32_t i = 0; i <= steps; ++i)
    {
        double t = (double)i / steps;
        int32_t held =
            Sample(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t);
        if (held != 0)
        {
            *stopOut = held;
            return t;
        }
    }
    *stopOut = 0;
    return 2.0;
}

static void TestRaycasts(const mnavFlightVolume* volume)
{
    int32_t wrong = 0;
    int32_t early = 0;
    int32_t stops[3] = {0, 0, 0};
    for (int32_t k = 0; k < 400; ++k)
    {
        // From an open voxel to anywhere around the scene, past its sides,
        // floor and ceiling too.
        int32_t v[3];
        RandomOpen(v);
        // One draw a statement: an initializer's order is unspecified.
        double a[3];
        double b[3];
        for (int32_t i = 0; i < 3; ++i)
        {
            a[i] = v[i] + (double)Random();
        }
        b[0] = (double)Random() * 80.0 - 8.0;
        b[1] = (double)Random() * 40.0 - 4.0;
        b[2] = (double)Random() * 80.0 - 8.0;
        mnavFlightHit hit;
        CHECK(mnavFlightRaycast(volume, At(a[0], a[1], a[2]), At(b[0], b[1], b[2]), &hit) ==
                  mnav_success,
              "cast");
        int32_t stop = 0;
        double first = FirstStop(a, b, &stop);
        stops[hit.stop] += 1;
        // Never past the first point sampled not open; when earlier, only
        // by grazing an edge or a corner, so by less than a sample's
        // reach of a voxel.
        bool right = hit.stop == mnav_flightClear ? stop == 0 : hit.t <= first + 1e-9;
        bool grazed = stop == 0 || hit.t < first - s_sampleStep - 1e-9;
        right &= hit.stop == mnav_flightClear || grazed ||
                 hit.stop == (stop == 1 ? mnav_flightBlocked : mnav_flightNotLoaded);
        early += hit.stop != mnav_flightClear && grazed ? 1 : 0;
        wrong += right ? 0 : 1;
    }
    printf("raycasts: %d clear, %d blocked, %d not loaded; %d stopped before the first sample, "
           "at an edge or corner; %d wrong\n",
           stops[0], stops[1], stops[2], early, wrong);
    CHECK(wrong == 0, "every raycast as sampling has it");
    CHECK(early < 40, "few grazes");
}

// The distance from p to the nearest open voxel within r, inset as the
// query insets, by every voxel; -1 for none.
static double NearestByVoxels(const double p[3], double r)
{
    double best = -1.0;
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
                const double lo[3] = {x, y, z};
                double sum = 0.0;
                for (int32_t k = 0; k < 3; ++k)
                {
                    double q = fmin(fmax(p[k], lo[k]), lo[k] + 1.0);
                    sum += (q - p[k]) * (q - p[k]);
                }
                double d = sqrt(sum);
                best = d <= r && (best < 0.0 || d < best) ? d : best;
            }
        }
    }
    return best;
}

static void TestNearest(const mnavFlightVolume* volume)
{
    int32_t wrong = 0;
    int32_t found = 0;
    for (int32_t k = 0; k < 150; ++k)
    {
        double p[3];
        p[0] = (double)Random() * SIDE;
        p[1] = (double)Random() * LAYERS;
        p[2] = (double)Random() * SIDE;
        mnavPos3 nearest;
        bool any = false;
        CHECK(mnavFindNearestFlightPoint(volume, At(p[0], p[1], p[2]), 2.5f, &nearest, &any) ==
                  mnav_success,
              "nearest");
        double expect = NearestByVoxels(p, 2.5);
        bool open = false;
        double d = sqrt((nearest.x - p[0]) * (nearest.x - p[0]) +
                        (nearest.y + 1.0 - p[1]) * (nearest.y + 1.0 - p[1]) +
                        (nearest.z - p[2]) * (nearest.z - p[2]));
        bool right = any == (expect >= 0.0);
        if (any)
        {
            right &= mnavIsFlightOpen(volume, nearest, &open) == mnav_success && open &&
                     fabs(d - expect) < 2e-3;
            found += 1;
        }
        wrong += right ? 0 : 1;
    }
    printf("nearest: %d of 150 found, %d wrong\n", found, wrong);
    CHECK(wrong == 0, "every nearest point as every voxel has it");
}

// A solid voxel of the grid.
static void RandomSolid(int32_t v[3])
{
    do
    {
        v[0] = (int32_t)(Random() * SIDE);
        v[1] = (int32_t)(Random() * LAYERS);
        v[2] = (int32_t)(Random() * SIDE);
    } while (Open(v[0], v[1], v[2]));
}

// Path searches refused: points not finite, below or above the volume,
// and NULL arguments.
static void Refusals(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 open)
{
    mnavFlightPath path;
    const mnavPos3 bad[3] = {{(double)NAN, 5.0, 5.0}, {5.0, -5.0, 5.0}, {5.0, 40.0, 5.0}};
    int32_t wrong = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        wrong += mnavFindFlightPath(query, volume, bad[k], open, &path) == mnav_errorRange ? 0 : 1;
        wrong += mnavFindFlightPath(query, volume, open, bad[k], &path) == mnav_errorRange ? 0 : 1;
    }
    CHECK(wrong == 0, "points not finite, below or above the volume");
    CHECK(mnavFindFlightPath(nullptr, volume, open, open, &path) == mnav_errorInvalid &&
              mnavFindFlightPath(query, nullptr, open, open, &path) == mnav_errorInvalid &&
              mnavFindFlightPath(query, volume, open, open, nullptr) == mnav_errorInvalid,
          "NULL arguments");
}

// The ends a path search reports before it begins.
static void TestEnds(const mnavFlightVolume* volume)
{
    mnavQuery* query = Query(16384, 1000.0f);
    mnavFlightPath path;
    int32_t a[3];
    int32_t s[3];
    RandomOpen(a);
    RandomSolid(s);
    mnavPos3 open = At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5);
    mnavPos3 solid = At(s[0] + 0.5, s[1] + 0.5, s[2] + 0.5);
    CHECK(mnavFindFlightPath(query, volume, open, open, &path) == mnav_success &&
              path.end == mnav_pathFound && path.pointCount == 2 && path.length == 0.0,
          "the same point");
    CHECK(mnavFindFlightPath(query, volume, solid, open, &path) == mnav_success &&
              path.end == mnav_pathNone && path.pointCount == 1,
          "a blocked start");
    CHECK(mnavFindFlightPath(query, volume, open, solid, &path) == mnav_success &&
              path.end == mnav_pathNone && path.pointCount == 1 && path.points[0].x == open.x,
          "a blocked end");
    CHECK(mnavFindFlightPath(query, volume, open, At(100.5, 10.5, 10.5), &path) == mnav_success &&
              path.end == mnav_pathNotLoaded,
          "an end with no tile");
    Refusals(query, volume, open);
    mnavDestroyQuery(query);
}

// A path between far corners of the scene: the start and end voxels.
static void FarEnds(int32_t a[3], int32_t b[3])
{
    int32_t tries = 0;
    do
    {
        RandomOpen(a);
        RandomOpen(b);
        ++tries;
    } while ((abs(a[0] - b[0]) + abs(a[2] - b[2]) < 80 || GridShortest(a, b) < 0.0) &&
             tries < 1000);
}

static void TestLimits(const mnavFlightVolume* volume)
{
    int32_t a[3];
    int32_t b[3];
    FarEnds(a, b);
    mnavPos3 start = At(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5);
    mnavPos3 end = At(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5);
    mnavFlightPath path;
    mnavQuery* query = Query(16, 1000.0f);
    CHECK(mnavFindFlightPath(query, volume, start, end, &path) == mnav_success &&
              path.end == mnav_pathOutOfNodes && path.pointCount >= 1 &&
              path.points[0].x == start.x && path.points[0].z == start.z,
          "out of nodes, toward the end");
    mnavDestroyQuery(query);
    query = Query(16384, 20.0f);
    CHECK(mnavFindFlightPath(query, volume, start, end, &path) == mnav_success &&
              path.end == mnav_pathTooLong,
          "past the path length limit");
    mnavDestroyQuery(query);
}

// With tile (1, 1) not loaded: a search beside it still finds its way, a
// start there is refused, and a raycast through it stops.
static void TestHole(void)
{
    mnavFlightVolume* volume = Volume(1, 1);
    mnavQuery* query = Query(16384, 1000.0f);
    mnavFlightPath path;
    CHECK(mnavFindFlightPath(query, volume, At(20.5, 28.5, 20.5), At(4.5, 28.5, 4.5), &path) ==
                  mnav_success &&
              path.end == mnav_pathNotLoaded && path.pointCount == 1,
          "a start with no tile");
    mnavFlightHit hit;
    CHECK(mnavFlightRaycast(volume, At(4.5, 30.5, 24.5), At(40.5, 30.5, 24.5), &hit) ==
                  mnav_success &&
              hit.stop != mnav_flightClear && hit.point.x <= 16.0 + 1e-9,
          "a raycast into the hole");
    mnavPos3 nearest;
    bool found = true;
    CHECK(mnavFindNearestFlightPoint(volume, At(24.5, 28.5, 24.5), 2.0f, &nearest, &found) ==
                  mnav_success &&
              !found,
          "nothing near in the hole");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

static void TestRefusals(const mnavFlightVolume* volume)
{
    mnavFlightHit hit;
    mnavPos3 p = At(5.5, 20.5, 5.5);
    mnavPos3 nan = {(double)NAN, 0.0, 0.0};
    CHECK(mnavFlightRaycast(volume, nan, p, &hit) == mnav_errorRange &&
              mnavFlightRaycast(volume, p, (mnavPos3){1e12, 0.0, 0.0}, &hit) == mnav_errorRange &&
              mnavFlightRaycast(nullptr, p, p, &hit) == mnav_errorInvalid &&
              mnavFlightRaycast(volume, p, p, nullptr) == mnav_errorInvalid,
          "raycast refusals");
    mnavPos3 out;
    bool found = false;
    CHECK(mnavFindNearestFlightPoint(volume, p, -1.0f, &out, &found) == mnav_errorRange &&
              mnavFindNearestFlightPoint(volume, p, (float)NAN, &out, &found) == mnav_errorRange &&
              mnavFindNearestFlightPoint(volume, nan, 1.0f, &out, &found) == mnav_errorRange &&
              mnavFindNearestFlightPoint(nullptr, p, 1.0f, &out, &found) == mnav_errorInvalid &&
              mnavFindNearestFlightPoint(volume, p, 1.0f, nullptr, &found) == mnav_errorInvalid &&
              mnavFindNearestFlightPoint(volume, p, 1.0f, &out, nullptr) == mnav_errorInvalid,
          "nearest refusals");
}

// A volume of 2 by 2 tiles of 16 voxels, one cube high, over a floor,
// with up to 8 boxes given as their lowest corner and sizes.
static mnavFlightVolume* Small(const float boxes[][6], int32_t count, float radius)
{
    static mnavVec3 vertices[4 + 8 * 8];
    static int32_t indices[6 + 8 * 36];
    const mnavVec3 floor[4] = {{-4, 0, -4}, {-4, 0, 36}, {36, 0, 36}, {36, 0, -4}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    static const int32_t faces[36] = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                                      2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    memcpy(vertices, floor, sizeof(floor));
    memcpy(indices, quad, sizeof(quad));
    for (int32_t b = 0; b < count; ++b)
    {
        for (int32_t c = 0; c < 8; ++c)
        {
            vertices[4 + b * 8 + c] = (mnavVec3){boxes[b][0] + ((c & 1) != 0 ? boxes[b][3] : 0.0f),
                                                 boxes[b][1] + ((c & 2) != 0 ? boxes[b][4] : 0.0f),
                                                 boxes[b][2] + ((c & 4) != 0 ? boxes[b][5] : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            indices[6 + b * 36 + k] = 4 + b * 8 + faces[k];
        }
    }
    mnavTriangleMesh mesh = {vertices, 4 + count * 8, indices, 2 + count * 12, nullptr};
    mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavFlightDef def = mnavDefaultFlightDef();
    def.allocator = CountingAllocator();
    def.tileVoxels = 16;
    def.floor = 0.0f;
    def.ceiling = 16.0f;
    def.radius = radius;
    def.groundBelow = false;
    mnavFlightBaker* baker = nullptr;
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success &&
              mnavCreateFlightVolume(&def, &volume).result == mnav_success,
          "a small volume");
    static uint8_t bytes[TILE_BYTES];
    int32_t wrong = 0;
    for (int32_t t = 0; t < 4; ++t)
    {
        size_t size = 0;
        wrong += mnavBakeFlightTile(baker, &input, t % 2, t / 2, nullptr) != mnav_success;
        wrong += mnavCopyFlightTile(baker, bytes, TILE_BYTES, &size) != mnav_success;
        wrong += mnavStageFlightTile(volume, bytes, size).result != mnav_success;
    }
    CHECK(wrong == 0 && mnavCommitFlight(volume) == mnav_success, "its tiles");
    mnavDestroyFlightBaker(baker);
    return volume;
}

static bool OpenAt(const mnavFlightVolume* volume, double x, double y, double z)
{
    bool open = false;
    return mnavIsFlightOpen(volume, (mnavPos3){x, y, z}, &open) == mnav_success && open;
}

// Whether every step of a path is clear by the raycast.
static bool StepsClear(const mnavFlightVolume* volume, const mnavFlightPath* path)
{
    for (int32_t i = 0; i + 1 < path->pointCount; ++i)
    {
        mnavFlightHit hit;
        if (mnavFlightRaycast(volume, path->points[i], path->points[i + 1], &hit) != mnav_success ||
            hit.stop != mnav_flightClear)
        {
            return false;
        }
    }
    return true;
}

// Two boxes meeting at an edge: no ray and no path slips through it.
static void TestEdge(void)
{
    // Kept off the voxels' faces: a triangle on a face marks both voxels.
    const float boxes[2][6] = {{4.25f, 1.25f, 4.25f, 3.5f, 7.5f, 3.5f},
                               {8.25f, 1.25f, 8.25f, 3.5f, 7.5f, 3.5f}};
    mnavFlightVolume* volume = Small(boxes, 2, 0.0f);
    // The voxels on either side of the edge at (8, 8) are open, those it
    // joins solid.
    bool shape = OpenAt(volume, 7.5, 4.5, 8.5) && OpenAt(volume, 8.5, 4.5, 7.5) &&
                 !OpenAt(volume, 7.5, 4.5, 7.5) && !OpenAt(volume, 8.5, 4.5, 8.5);
    CHECK(shape, "two boxes meeting at an edge");
    mnavFlightHit hit;
    CHECK(mnavFlightRaycast(volume, (mnavPos3){7.5, 4.5, 8.5}, (mnavPos3){8.5, 4.5, 7.5}, &hit) ==
                  mnav_success &&
              hit.stop == mnav_flightBlocked,
          "a ray through the edge is blocked");
    mnavQuery* query = Query(16384, 1000.0f);
    mnavFlightPath path;
    CHECK(mnavFindFlightPath(query, volume, (mnavPos3){7.5, 4.5, 8.5}, (mnavPos3){8.5, 4.5, 7.5},
                             &path) == mnav_success &&
              path.end == mnav_pathFound && path.length > 2.0 && StepsClear(volume, &path),
          "a path goes around the edge");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

// A path whose end, deep in a large block, is out of sight of the block
// that reached it: its way bends through that block's center and then
// the face between them (found by fuzz_flight).
static void TestBendToTheEnd(void)
{
    const float boxes[2][6] = {{4.0f, 4.0f, 1.0f, 4.0f, 1.0f, 1.0f},
                               {0.0f, 5.0f, 4.0f, 5.0f, 5.0f, 1.0f}};
    mnavFlightVolume* volume = Small(boxes, 2, 1.5f);
    mnavQuery* query = Query(32768, 1000.0f);
    mnavFlightPath path;
    CHECK(mnavFindFlightPath(query, volume, (mnavPos3){2.125, 4.125, 12.125},
                             (mnavPos3){8.125, 12.125, 0.625}, &path) == mnav_success &&
              path.end == mnav_pathFound && StepsClear(volume, &path),
          "every step clear to the end");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

// Paths whose points would sit on the far edge of a face but for its
// inset, where a point counts as in the voxel beyond (found by
// fuzz_flight).
static void TestFaceEdges(void)
{
    const float boxes[7][6] = {
        {31.0f, 1.0f, 0.0f, 1.0f, 1.0f, 3.0f}, {12.0f, 1.0f, 21.0f, 5.0f, 2.0f, 3.0f},
        {3.0f, 0.0f, 3.0f, 8.0f, 5.0f, 5.0f},  {1.0f, 2.0f, 30.0f, 8.0f, 2.0f, 1.0f},
        {0.0f, 4.0f, 16.0f, 5.0f, 3.0f, 7.0f}, {14.0f, 14.0f, 14.0f, 2.0f, 1.0f, 1.0f},
        {10.0f, 12.0f, 1.0f, 5.0f, 5.0f, 5.0f}};
    mnavFlightVolume* volume = Small(boxes, 7, 1.5f);
    mnavQuery* query = Query(32768, 1000.0f);
    const mnavPos3 ends[2][2] = {{{24.375, 11.125, 27.125}, {27.125, 11.625, 15.375}},
                                 {{15.375, 11.625, 12.125}, {8.125, 12.125, 0.125}}};
    int32_t wrong = 0;
    for (int32_t k = 0; k < 2; ++k)
    {
        mnavFlightPath path;
        bool right =
            mnavFindFlightPath(query, volume, ends[k][0], ends[k][1], &path) == mnav_success;
        right &= path.end != mnav_pathFound || StepsClear(volume, &path);
        wrong += right ? 0 : 1;
    }
    CHECK(wrong == 0, "every step clear by the far edges of faces");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
}

int main(void)
{
    mnavFlightVolume* volume = Volume(-1, -1);
    ReadOpen(volume);
    TestPaths(volume);
    TestSlices(volume);
    TestCheck(volume);
    TestRaycasts(volume);
    TestNearest(volume);
    TestEnds(volume);
    TestLimits(volume);
    TestRefusals(volume);
    mnavDestroyFlightVolume(volume);
    TestHole();
    TestEdge();
    TestBendToTheEnd();
    TestFaceEdges();
    CHECK(s_held == 0, "every byte given back");
    return s_failures == 0 ? 0 : 1;
}
