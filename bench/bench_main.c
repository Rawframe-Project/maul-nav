// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Timings over a terrain of 8 by 8 tiles of 32 m: hills on a 2 m grid
// (32,768 triangles) and 256 boxes. Bakes every tile, then through a
// tile cache, and rebuilds each with a crate on it; streams a window
// of 5 by 5 tiles as a camera crosses the terrain and back, staging the
// tiles that enter and leave and committing each step, for ten laps;
// then finds paths
// on the whole terrain; then steers 1,000 agents through a doorway 4 m wide
// with avoidance, 1,000 agents across the terrain along their corridors,
// steered and avoiding each other, and 1,000 fliers through a gap in a
// wall of spheres;
// then builds flow fields over a grid of 512 by 512 cells;
// then, on a flat world of 24 by 24 tiles with pillars and walls baked
// from outlines, builds a hierarchy and finds long paths with and without
// it; then bakes the terrain and 300 floating boxes into a flight volume
// and finds fliers' paths and casts their rays through it.
// Prints each section's counts and bytes, which do not depend on the
// machine, as comment lines, then its named results, the best of five
// runs. Given a baseline file, such as bench/baseline.txt, it prints each
// result's ratio to the recorded one as well (mnav-0012).

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum
{
    TILES = 8,
    GRID = 128,
    BOXES = 256,
    WINDOW = 2,
    RUNS = 5,
    PATHS = 1000,
    LAPS = 10,
    LINKS = 512,
    TILE_BYTES = 1 << 20
};

// The width of a result's name, as printed and as read back.
#define NAME_WIDTH 32

// The baseline's lines, read whole.
static char s_baseline[16384];

// The value the baseline records for a name, or 0.
static double Recorded(const char* name)
{
    for (const char* line = s_baseline; *line != '\0';)
    {
        const char* end = strchr(line, '\n');
        size_t length = end != NULL ? (size_t)(end - line) : strlen(line);
        size_t named = strlen(name);
        if (line[0] != '#' && length > named && strncmp(line, name, named) == 0 &&
            line[named] == ' ')
        {
            // After the name, which may run past NAME_WIDTH.
            return strtod(line + named, NULL);
        }
        line += length + (end != NULL ? 1 : 0);
    }
    return 0.0;
}

// Prints a result, and its ratio to the baseline's where it has one.
static void Report(const char* name, double value, const char* unit)
{
    double recorded = Recorded(name);
    if (recorded > 0.0)
    {
        printf("%-*s %12.1f %-6s (%.2fx the baseline)\n", NAME_WIDTH, name, value, unit,
               value / recorded);
    }
    else
    {
        printf("%-*s %12.1f %s\n", NAME_WIDTH, name, value, unit);
    }
}

static double Seconds(void)
{
    struct timespec now;
    timespec_get(&now, TIME_UTC);
    return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

static void Check(mnavResult status, const char* what)
{
    if (status != mnav_success)
    {
        fprintf(stderr, "%s failed: %s\n", what, mnavResultName(status));
        exit(1);
    }
}

// An allocator that counts the bytes held and the most held at once.
static size_t s_held;
static size_t s_peak;

static void* Alloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    unsigned char* base = malloc(size + alignment + sizeof(void*));
    if (base == NULL)
    {
        return NULL;
    }
    uintptr_t start = (uintptr_t)(base + sizeof(void*));
    unsigned char* block = base + sizeof(void*) + ((alignment - start % alignment) % alignment);
    ((void**)block)[-1] = base;
    s_held += size;
    s_peak = s_held > s_peak ? s_held : s_peak;
    return block;
}

static void Free(void* block, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held -= size;
    free(((void**)block)[-1]);
}

// A hill height in [0, 3] m: a product of two triangle waves, slopes
// below 15 degrees.
static float Height(float x, float z)
{
    float u = x / 50.0f - (float)(int32_t)(x / 50.0f);
    float v = z / 37.0f - (float)(int32_t)(z / 37.0f);
    float a = u < 0.5f ? 2.0f * u : 2.0f - 2.0f * u;
    float b = v < 0.5f ? 2.0f * v : 2.0f - 2.0f * v;
    return 3.0f * a * b;
}

static mnavVec3 s_vertices[(GRID + 1) * (GRID + 1) + 8 * BOXES];
static int32_t s_indices[(GRID * GRID * 2 + 12 * BOXES) * 3];

static mnavTriangleMesh Terrain(void)
{
    const float step = (float)TILES * 32.0f / (float)GRID;
    int32_t v = 0;
    int32_t i = 0;
    for (int32_t z = 0; z <= GRID; ++z)
    {
        for (int32_t x = 0; x <= GRID; ++x)
        {
            float px = (float)x * step;
            float pz = (float)z * step;
            s_vertices[v++] = (mnavVec3){px, Height(px, pz), pz};
        }
    }
    for (int32_t z = 0; z < GRID; ++z)
    {
        for (int32_t x = 0; x < GRID; ++x)
        {
            int32_t a = z * (GRID + 1) + x;
            const int32_t quad[6] = {a, a + GRID + 1, a + GRID + 2, a, a + GRID + 2, a + 1};
            for (int32_t k = 0; k < 6; ++k)
            {
                s_indices[i++] = quad[k];
            }
        }
    }
    uint32_t state = 7;
    const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                               2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        state = state * 1664525u + 1013904223u;
        float x0 = (float)(state >> 8 & 0xFFFFu) / 65536.0f * ((float)TILES * 32.0f - 4.0f);
        state = state * 1664525u + 1013904223u;
        float z0 = (float)(state >> 8 & 0xFFFFu) / 65536.0f * ((float)TILES * 32.0f - 4.0f);
        float y0 = Height(x0, z0) - 1.0f;
        int32_t first = v;
        for (int32_t k = 0; k < 8; ++k)
        {
            s_vertices[v++] = (mnavVec3){x0 + ((k & 1) ? 2.0f : 0.0f), y0 + ((k & 4) ? 4.0f : 0.0f),
                                         z0 + ((k & 2) ? 2.0f : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[i++] = first + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, v, s_indices, i / 3, NULL};
}

static uint8_t* s_tiles[TILES * TILES];
static size_t s_sizes[TILES * TILES];

static mnavBakeDef Def(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    return def;
}

static void Bake(void)
{
    mnavBakeDef def = Def();
    mnavTriangleMesh terrain = Terrain();
    static uint8_t buffer[TILE_BYTES];
    double best = 1e30;
    size_t bytes = 0;
    int32_t polygons = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        mnavBaker* baker = NULL;
        Check(mnavCreateBaker(&def, &baker).result, "baker");
        double start = Seconds();
        bytes = 0;
        polygons = 0;
        for (int32_t t = 0; t < TILES * TILES; ++t)
        {
            mnavBakeReport report;
            Check(mnavBakeTile(baker, &terrain, 1, t % TILES, t / TILES, &report), "bake");
            Check(mnavCopyBakedTile(baker, buffer, TILE_BYTES, &s_sizes[t]), "copy");
            if (run == 0)
            {
                s_tiles[t] = malloc(s_sizes[t]);
                for (size_t k = 0; k < s_sizes[t]; ++k)
                {
                    s_tiles[t][k] = buffer[k];
                }
            }
            bytes += s_sizes[t];
            polygons += report.polygons;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
        mnavDestroyBaker(baker);
    }
    printf("# bake: %d tiles, %d triangles, %d polygons, %zu bytes, %.0f us per tile\n",
           TILES * TILES, terrain.triangleCount, polygons, bytes, best * 1e6 / (TILES * TILES));
    Report("bake, a tile", best * 1e6 / (TILES * TILES), "us");
}

// Caches every tile of the terrain, then rebuilds each with a crate of
// 2 by 2 m on it (mnav-0016), the crate moving a meter each run.
static void Rebuild(void)
{
    mnavBakeDef def = Def();
    mnavTriangleMesh terrain = Terrain();
    const mnavBakeInput input = {&terrain, 1, NULL, 0, NULL, 0, NULL};
    mnavBaker* baker = NULL;
    mnavTileCache* cache = NULL;
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    Check(mnavCreateBaker(&def, &baker).result, "baker");
    Check(mnavCreateTileCache(&cacheDef, &cache), "cache");
    uint64_t empty = mnavGetTileCacheBytes(cache);
    double start = Seconds();
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        Check(mnavBakeTileCached(baker, cache, &input, t % TILES, t / TILES, NULL), "cached bake");
    }
    double cached = (Seconds() - start) / (TILES * TILES);
    uint64_t bytes = mnavGetTileCacheBytes(cache) - empty;
    double best = 1e30;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        start = Seconds();
        for (int32_t t = 0; t < TILES * TILES; ++t)
        {
            float x = (float)(t % TILES) * 32.0f + 10.0f + (float)run;
            float z = (float)(t / TILES) * 32.0f + 12.0f;
            const mnavVec2 ring[4] = {{x, z}, {x + 2.0f, z}, {x + 2.0f, z + 2.0f}, {x, z + 2.0f}};
            const mnavBakeVolume crate = {ring, 4, -2.0f, 6.0f, mnav_volumeExclude, 0};
            Check(mnavRebuildTile(baker, cache, t % TILES, t / TILES, &crate, 1, NULL), "rebuild");
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    printf("# rebuild: %d tiles cached in %llu bytes, %llu per tile; a cached bake %.0f us\n",
           TILES * TILES, (unsigned long long)bytes, (unsigned long long)(bytes / (TILES * TILES)),
           cached * 1e6);
    Report("rebuild, a tile", best * 1e6 / (TILES * TILES), "us");
    mnavDestroyTileCache(cache);
    mnavDestroyBaker(baker);
}

// Whether tile (x, z) is within the window round the camera's tile.
static bool Seen(int32_t x, int32_t z, int32_t cx, int32_t cz)
{
    return x >= cx - WINDOW && x <= cx + WINDOW && z >= cz - WINDOW && z <= cz + WINDOW && x >= 0 &&
           z >= 0 && x < TILES && z < TILES;
}

static void Stream(int32_t links)
{
    mnavBakeDef def = Def();
    def.limits.tiles = 64;
    double bestStep = 1e30;
    double worstStep = 0.0;
    double total = 0.0;
    int32_t steps = 0;
    int32_t staged = 0;
    size_t peak = 0;
    size_t firstLap = 0;
    size_t lastLap = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        size_t before = s_held;
        s_peak = s_held;
        mnavNavmesh* navmesh = NULL;
        Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
        // Short jumps scattered over the terrain, snapped at each commit.
        uint32_t state = 3;
        for (int32_t l = 0; l < links; ++l)
        {
            state = state * 1664525u + 1013904223u;
            double x = (double)(state >> 8 & 0xFFFFu) / 65536.0 * (TILES * 32.0 - 4.0);
            state = state * 1664525u + 1013904223u;
            double z = (double)(state >> 8 & 0xFFFFu) / 65536.0 * (TILES * 32.0 - 4.0);
            mnavLinkDef jump = {{x, (double)Height((float)x, (float)z), z},
                                {x + 3.0, (double)Height((float)x + 3.0f, (float)z), z},
                                1.0f,
                                5.0f,
                                mnav_linkJump,
                                true,
                                0.0f};
            mnavLinkId id;
            Check(mnavStageLink(navmesh, &jump, &id), "link");
        }
        // The camera crosses row 3 and comes back on row 4, a tile a step.
        int32_t path[2 * TILES][2];
        int32_t count = 0;
        for (int32_t x = 0; x < TILES; ++x)
        {
            path[count][0] = x;
            path[count++][1] = 3;
        }
        for (int32_t x = TILES - 1; x >= 0; --x)
        {
            path[count][0] = x;
            path[count++][1] = 4;
        }
        int32_t cx = -100;
        int32_t cz = -100;
        steps = 0;
        staged = 0;
        double runTotal = 0.0;
        for (int32_t q = 0; q < count * LAPS; ++q)
        {
            int32_t p = q % count;
            int32_t nx = path[p][0];
            int32_t nz = path[p][1];
            double start = Seconds();
            for (int32_t t = 0; t < TILES * TILES; ++t)
            {
                int32_t x = t % TILES;
                int32_t z = t / TILES;
                bool was = Seen(x, z, cx, cz);
                bool now = Seen(x, z, nx, nz);
                if (was && !now)
                {
                    Check(mnavStageTileRemoval(navmesh, x, z), "removal");
                    staged += 1;
                }
                else if (now && !was)
                {
                    Check(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result, "stage");
                    staged += 1;
                }
            }
            Check(mnavCommit(navmesh), "commit");
            double took = Seconds() - start;
            cx = nx;
            cz = nz;
            firstLap = q == count - 1 ? s_held - before : firstLap;
            if (q > 0)
            {
                bestStep = took < bestStep ? took : bestStep;
                worstStep = took > worstStep ? took : worstStep;
                runTotal += took;
                steps += 1;
            }
        }
        total = run == 0 || runTotal < total ? runTotal : total;
        lastLap = s_held - before;
        peak = s_peak - before;
        mnavDestroyNavmesh(navmesh);
    }
    printf("# stream with %d links: %d steps, %d tiles staged, %.0f us per step (best %.0f, worst "
           "%.0f), "
           "%zu bytes held after the first lap, %zu after the last, %zu at most\n",
           links, steps, staged, total * 1e6 / steps, bestStep * 1e6, worstStep * 1e6, firstLap,
           lastLap, peak);
    Report(links == 0 ? "stream step, no links" : "stream step, 512 links", total * 1e6 / steps,
           "us");
}

// The same searches in slices of 64 nodes each, as a server spreading
// many agents' searches over its ticks runs them.
static void Sliced(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest (*ends)[2])
{
    double best = 1e30;
    int32_t slices = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        slices = 0;
        double start = Seconds();
        for (int32_t i = 0; i < PATHS; ++i)
        {
            if (ends[i][0].polygon.slot == 0 || ends[i][1].polygon.slot == 0)
            {
                continue;
            }
            Check(mnavBeginPath(query, navmesh, NULL, ends[i][0].polygon, ends[i][0].point,
                                ends[i][1].polygon, ends[i][1].point),
                  "begin");
            bool ended = false;
            while (!ended)
            {
                Check(mnavContinuePath(query, navmesh, 64, &ended), "continue");
                slices += 1;
            }
            mnavPath path;
            Check(mnavFinishPath(query, navmesh, &path), "finish");
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    printf("# sliced paths: %d slices of 64 nodes for %d searches, %.1f us per path\n", slices,
           PATHS, best * 1e6 / PATHS);
    Report("sliced path query", best * 1e6 / PATHS, "us");
    Report("sliced path, a slice", best * 1e6 / slices, "us");
}

// The length of a path's legs on the ground.
static double Ground(const mnavPath* path)
{
    double length = 0.0;
    for (int32_t i = 1; i < path->pointCount; ++i)
    {
        double dx = path->points[i].x - path->points[i - 1].x;
        double dz = path->points[i].z - path->points[i - 1].z;
        length += sqrt(dx * dx + dz * dz);
    }
    return length;
}

// The same queries by the exact search (mnav-0005), and the ground the
// two searches' paths cover.
static void Shortest(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest (*ends)[2])
{
    double plain = 0.0;
    double exact = 0.0;
    for (int32_t i = 0; i < PATHS; ++i)
    {
        if (ends[i][0].polygon.slot == 0 || ends[i][1].polygon.slot == 0)
        {
            continue;
        }
        mnavPath path;
        Check(mnavFindPath(query, navmesh, NULL, ends[i][0].polygon, ends[i][0].point,
                           ends[i][1].polygon, ends[i][1].point, &path),
              "path");
        plain += path.end == mnav_pathFound ? Ground(&path) : 0.0;
        Check(mnavFindShortestPath(query, navmesh, NULL, ends[i][0].polygon, ends[i][0].point,
                                   ends[i][1].polygon, ends[i][1].point, &path),
              "shortest path");
        exact += path.end == mnav_pathFound ? Ground(&path) : 0.0;
    }
    double best = 1e30;
    int32_t found = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        found = 0;
        double start = Seconds();
        for (int32_t i = 0; i < PATHS; ++i)
        {
            if (ends[i][0].polygon.slot == 0 || ends[i][1].polygon.slot == 0)
            {
                continue;
            }
            mnavPath path;
            Check(mnavFindShortestPath(query, navmesh, NULL, ends[i][0].polygon, ends[i][0].point,
                                       ends[i][1].polygon, ends[i][1].point, &path),
                  "shortest path");
            found += path.end == mnav_pathFound ? 1 : 0;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    printf("# shortest paths: %d of %d found, %.1f m on the ground to the A* paths' %.1f m, "
           "%.1f us per path\n",
           found, PATHS, exact, plain, best * 1e6 / PATHS);
    Report("shortest path query", best * 1e6 / PATHS, "us");
}

static void Paths(void)
{
    mnavBakeDef def = Def();
    mnavNavmesh* navmesh = NULL;
    Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        Check(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result, "stage");
    }
    Check(mnavCommit(navmesh), "commit");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    static mnavNearest ends[PATHS][2];
    uint32_t state = 11;
    for (int32_t i = 0; i < PATHS; ++i)
    {
        for (int32_t k = 0; k < 2; ++k)
        {
            state = state * 1664525u + 1013904223u;
            double x = (double)(state >> 8 & 0xFFFFu) / 65536.0 * TILES * 32.0;
            state = state * 1664525u + 1013904223u;
            double z = (double)(state >> 8 & 0xFFFFu) / 65536.0 * TILES * 32.0;
            mnavPos3 p = {x, (double)Height((float)x, (float)z), z};
            Check(mnavFindNearest(navmesh, NULL, p, (mnavVec3){2.0f, 4.0f, 2.0f}, &ends[i][k]),
                  "nearest");
        }
    }
    double best = 1e30;
    int32_t found = 0;
    int32_t corners = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        found = 0;
        corners = 0;
        double start = Seconds();
        for (int32_t i = 0; i < PATHS; ++i)
        {
            if (ends[i][0].polygon.slot == 0 || ends[i][1].polygon.slot == 0)
            {
                continue;
            }
            mnavPath path;
            Check(mnavFindPath(query, navmesh, NULL, ends[i][0].polygon, ends[i][0].point,
                               ends[i][1].polygon, ends[i][1].point, &path),
                  "path");
            found += path.end == mnav_pathFound ? 1 : 0;
            corners += path.pointCount;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    printf("# paths: %d of %d found, %d points, %.1f us per path\n", found, PATHS, corners,
           best * 1e6 / PATHS);
    Report("path query", best * 1e6 / PATHS, "us");
    Report("path queries per second", PATHS / best, "/s");
    Sliced(query, navmesh, ends);
    Shortest(query, navmesh, ends);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// 1,000 agents in a block 40 m by 25 m cross a wall through a doorway 4 m
// wide to a block on the other side, in steps of 0.1 s.
static void Doorway(void)
{
    enum
    {
        AGENTS = 1000,
        STEPS = 600
    };
    static mnavAgent agents[AGENTS];
    static mnavPos2 goals[AGENTS];
    static mnavPos2 velocities[AGENTS];
    const mnavPos2 below[4] = {{-1.0, -60.0}, {1.0, -60.0}, {1.0, -2.0}, {-1.0, -2.0}};
    const mnavPos2 above[4] = {{-1.0, 2.0}, {1.0, 2.0}, {1.0, 60.0}, {-1.0, 60.0}};
    const mnavObstacle wall[2] = {{below, 4, 0.0, {0.0, 0.0}, 1, 0},
                                  {above, 4, 0.0, {0.0, 0.0}, 2, 0}};
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.limits.agents = AGENTS;
    def.neighborDistance = 3.0;
    def.timeHorizon = 2.0;
    def.obstacleTimeHorizon = 1.0;
    mnavAvoidance* avoidance = NULL;
    Check(mnavCreateAvoidance(&def, &avoidance), "avoidance");
    double best = 1e30;
    int32_t arrived = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        for (int32_t i = 0; i < AGENTS; ++i)
        {
            double x = -45.0 + (double)(i % 25) * 1.6;
            double y = -20.0 + (double)(i / 25) * 1.0 + (double)(i % 3) * 0.1;
            agents[i] =
                (mnavAgent){{x, y}, {0.0, 0.0}, {0.0, 0.0}, 0.3, 1.5, 1.0, (uint64_t)i, 0, 0};
            goals[i] = (mnavPos2){-x, y};
        }
        double start = Seconds();
        for (int32_t s = 0; s < STEPS; ++s)
        {
            for (int32_t i = 0; i < AGENTS; ++i)
            {
                // Through the doorway's middle while still before the wall.
                mnavPos2 aim = agents[i].position.x < -1.5 ? (mnavPos2){0.0, 0.0} : goals[i];
                double dx = aim.x - agents[i].position.x;
                double dy = aim.y - agents[i].position.y;
                double length = sqrt(dx * dx + dy * dy);
                double scale = length > 1.0 ? 1.0 / length : 1.0;
                agents[i].preferred = (mnavPos2){dx * scale, dy * scale};
            }
            Check(mnavAvoid(avoidance, agents, AGENTS, wall, 2, 0.1, velocities), "avoid");
            for (int32_t i = 0; i < AGENTS; ++i)
            {
                agents[i].velocity = velocities[i];
                agents[i].position.x += velocities[i].x * 0.1;
                agents[i].position.y += velocities[i].y * 0.1;
            }
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
        arrived = 0;
        for (int32_t i = 0; i < AGENTS; ++i)
        {
            arrived += agents[i].position.x > 1.5 ? 1 : 0;
        }
    }
    printf(
        "# doorway: %d agents, %d through after %d steps, %.0f us per step, %.0f agents per ms\n",
        AGENTS, arrived, STEPS, best * 1e6 / STEPS, AGENTS * STEPS / (best * 1e3));
    Report("avoidance step, 1000 agents", best * 1e6 / STEPS, "us");
    Report("avoidance, agents per ms", AGENTS * STEPS / (best * 1e3), "/ms");
    mnavDestroyAvoidance(avoidance);
}

// A crowd on the terrain's navmesh: 1,000 agents in two groups of 500 cross
// a square of 60 m from opposite sides, each step every corridor giving
// its corners, mnavSteer its velocity, mnavAvoid one clear of the others
// and mnavMoveCorridor the move along the surface, in steps of 0.1 s; then
// again with the walls near each agent, found by mnavFindWalls within
// 2 m when it has moved half a meter, each once, as avoidance's segments.
enum
{
    CROWD = 1000,
    CROWD_STEPS = 600,
    CROWD_CORRIDOR = 128,
    CROWD_WALLS = 8,
    // A power of two over twice the walls the agents may hold.
    CROWD_WALL_TABLE = 1 << 14
};

static const double CROWD_WALL_RANGE = 2.0;

static mnavPolygonId s_crowdBuffers[CROWD][CROWD_CORRIDOR];
static mnavCorridor s_crowd[CROWD];
static mnavAgent s_crowdAgents[CROWD];
static mnavWallSegment s_crowdWalls[CROWD][CROWD_WALLS];
static int32_t s_crowdWallCounts[CROWD];
static mnavPos3 s_crowdWallsAt[CROWD];
static mnavObstacle s_crowdObstacles[CROWD * CROWD_WALLS];
static mnavPos2 s_crowdEnds[CROWD * CROWD_WALLS][2];
static int32_t s_crowdWallTable[CROWD_WALL_TABLE];

// Plans every agent's corridor across the square; returns how many found
// their way.
static int32_t PlanCrowd(mnavQuery* query, const mnavNavmesh* navmesh)
{
    int32_t planned = 0;
    const mnavVec3 box = {2.0f, 4.0f, 2.0f};
    for (int32_t i = 0; i < CROWD; ++i)
    {
        double z = 64.0 + 1.2 * (double)(i % 500 / 10) + 0.05 * (double)(i % 7);
        double x = 66.0 + 1.2 * (double)(i % 10);
        double from = i < 500 ? x : 192.0 - x;
        double to = 192.0 - from;
        mnavNearest a;
        mnavNearest b;
        mnavPath path;
        Check(mnavFindNearest(navmesh, NULL,
                              (mnavPos3){from, (double)Height((float)from, (float)z), z}, box, &a),
              "crowd start");
        Check(mnavFindNearest(navmesh, NULL, (mnavPos3){to, (double)Height((float)to, (float)z), z},
                              box, &b),
              "crowd end");
        Check(mnavFindPath(query, navmesh, NULL, a.polygon, a.point, b.polygon, b.point, &path),
              "crowd path");
        Check(mnavResetCorridor(&s_crowd[i], s_crowdBuffers[i], CROWD_CORRIDOR, a.polygon, a.point),
              "crowd corridor");
        planned += mnavSetCorridor(&s_crowd[i], &path) == mnav_success ? 1 : 0;
        s_crowdWallCounts[i] = -1;
        s_crowdAgents[i] = (mnavAgent){
            {a.point.x, a.point.z}, {0.0, 0.0}, {0.0, 0.0}, 0.3, 1.5, 1.0, (uint64_t)i, 0, 0};
    }
    return planned;
}

// Finds the walls again of the agents that have moved a quarter of the
// range, then gathers every agent's walls, each once, moved out by the
// agent's radius; returns how many.
static int32_t CrowdWalls(mnavQuery* query, const mnavNavmesh* navmesh)
{
    for (int32_t i = 0; i < CROWD; ++i)
    {
        double dx = s_crowd[i].position.x - s_crowdWallsAt[i].x;
        double dz = s_crowd[i].position.z - s_crowdWallsAt[i].z;
        double quarter = CROWD_WALL_RANGE * 0.25;
        if (s_crowdWallCounts[i] >= 0 && dx * dx + dz * dz < quarter * quarter)
        {
            continue;
        }
        mnavWallsFound found;
        mnavResult result =
            mnavFindWalls(query, navmesh, NULL, s_crowd[i].polygons[0], s_crowd[i].position,
                          CROWD_WALL_RANGE, s_crowdWalls[i], CROWD_WALLS, &found);
        Check(result == mnav_errorCapacity ? mnav_success : result, "crowd walls");
        s_crowdWallCounts[i] = found.count < CROWD_WALLS ? found.count : CROWD_WALLS;
        s_crowdWallsAt[i] = s_crowd[i].position;
    }
    memset(s_crowdWallTable, 0xFF, sizeof(s_crowdWallTable));
    int32_t count = 0;
    for (int32_t i = 0; i < CROWD; ++i)
    {
        double radius = s_crowdAgents[i].radius;
        for (int32_t k = 0; k < s_crowdWallCounts[i]; ++k)
        {
            const mnavWallSegment* wall = &s_crowdWalls[i][k];
            if (wall->normal.x == 0.0 && wall->normal.z == 0.0)
            {
                continue;
            }
            mnavPos2 ends[2] = {
                {wall->start.x - wall->normal.x * radius, wall->start.z - wall->normal.z * radius},
                {wall->end.x - wall->normal.x * radius, wall->end.z - wall->normal.z * radius}};
            uint32_t cell = (uint32_t)mnavHash64(MNAV_HASH_INIT, ends, (int32_t)sizeof(ends)) &
                            (CROWD_WALL_TABLE - 1);
            while (s_crowdWallTable[cell] >= 0 &&
                   memcmp(s_crowdEnds[s_crowdWallTable[cell]], ends, sizeof(ends)) != 0)
            {
                cell = (cell + 1) & (CROWD_WALL_TABLE - 1);
            }
            if (s_crowdWallTable[cell] >= 0)
            {
                continue;
            }
            s_crowdWallTable[cell] = count;
            memcpy(s_crowdEnds[count], ends, sizeof(ends));
            s_crowdObstacles[count] =
                (mnavObstacle){s_crowdEnds[count], 2, 0.0, {0.0, 0.0}, (uint64_t)count + 1, 0};
            count += 1;
        }
    }
    return count;
}

// Runs the crowd's steps, with the walls or without, adding each part's
// seconds; returns how many arrived.
static int32_t RunCrowd(mnavQuery* query, const mnavNavmesh* navmesh, mnavAvoidance* avoidance,
                        bool walls, double parts[4])
{
    static mnavPos2 velocities[CROWD];
    mnavSteerDef steer = mnavDefaultSteerDef();
    steer.maxSpeed = 1.5f;
    int32_t arrived = 0;
    for (int32_t s = 0; s < CROWD_STEPS; ++s)
    {
        double t0 = Seconds();
        arrived = 0;
        for (int32_t i = 0; i < CROWD; ++i)
        {
            mnavCorners corners;
            mnavSteering steering;
            Check(mnavCorridorCorners(query, navmesh, &s_crowd[i], &corners), "crowd corners");
            Check(mnavSteer(&corners, &steer, &steering), "crowd steer");
            arrived += steering.state == mnav_steerArrived ? 1 : 0;
            s_crowdAgents[i].preferred = (mnavPos2){steering.velocity.x, steering.velocity.z};
        }
        double t1 = Seconds();
        int32_t wallCount = walls ? CrowdWalls(query, navmesh) : 0;
        double tw = Seconds();
        Check(mnavAvoid(avoidance, s_crowdAgents, CROWD, s_crowdObstacles, wallCount, 0.1,
                        velocities),
              "crowd avoid");
        double t2 = Seconds();
        for (int32_t i = 0; i < CROWD; ++i)
        {
            mnavPos3 at = s_crowd[i].position;
            mnavPos3 wanted = {at.x + velocities[i].x * 0.1, at.y, at.z + velocities[i].y * 0.1};
            Check(mnavMoveCorridor(query, navmesh, NULL, &s_crowd[i], wanted, NULL), "crowd move");
            s_crowdAgents[i].position = (mnavPos2){s_crowd[i].position.x, s_crowd[i].position.z};
            s_crowdAgents[i].velocity = velocities[i];
        }
        double t3 = Seconds();
        parts[0] += t1 - t0;
        parts[1] += t2 - tw;
        parts[2] += t3 - t2;
        parts[3] += tw - t1;
    }
    return arrived;
}

static void Crowd(void)
{
    mnavBakeDef def = Def();
    mnavNavmesh* navmesh = NULL;
    Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        Check(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result, "stage");
    }
    Check(mnavCommit(navmesh), "commit");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    mnavAvoidanceDef avoidanceDef = mnavDefaultAvoidanceDef();
    avoidanceDef.allocator = (mnavAllocator){Alloc, Free, NULL};
    avoidanceDef.limits.agents = CROWD;
    avoidanceDef.limits.obstacleVertices = CROWD * CROWD_WALLS * 2;
    mnavAvoidance* avoidance = NULL;
    Check(mnavCreateAvoidance(&avoidanceDef, &avoidance), "avoidance");
    for (int32_t walls = 0; walls < 2; ++walls)
    {
        double best = 1e30;
        double bestParts[4] = {0.0, 0.0, 0.0, 0.0};
        int32_t planned = 0;
        int32_t arrived = 0;
        for (int32_t run = 0; run < RUNS; ++run)
        {
            planned = PlanCrowd(query, navmesh);
            double parts[4] = {0.0, 0.0, 0.0, 0.0};
            arrived = RunCrowd(query, navmesh, avoidance, walls == 1, parts);
            double took = parts[0] + parts[1] + parts[2] + parts[3];
            if (took < best)
            {
                best = took;
                memcpy(bestParts, parts, sizeof(parts));
            }
        }
        printf("# crowd%s: %d agents, %d planned, %d arrived after %d steps, %.0f us per step: "
               "corners and steering %.0f%%, walls %.0f%%, avoidance %.0f%%, moves %.0f%%\n",
               walls == 1 ? " with walls" : "", CROWD, planned, arrived, CROWD_STEPS,
               best * 1e6 / CROWD_STEPS, 100.0 * bestParts[0] / best, 100.0 * bestParts[3] / best,
               100.0 * bestParts[1] / best, 100.0 * bestParts[2] / best);
        Report(walls == 1 ? "crowd step with walls, 1000" : "crowd step, 1000 agents",
               best * 1e6 / CROWD_STEPS, "us");
    }
    mnavDestroyAvoidance(avoidance);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// 1,000 fliers in a cube 16 m wide cross a wall of spheres through a gap
// 6 m wide in its middle to a cube on the other side, in steps of 0.1 s.
static void Gap(void)
{
    enum
    {
        FLIERS = 1000,
        STEPS = 600,
        SIDE = 13
    };
    static mnavAgent3D fliers[FLIERS];
    static mnavPos3 goals[FLIERS];
    static mnavPos3 velocities[FLIERS];
    static mnavSphere wall[SIDE * SIDE];
    int32_t spheres = 0;
    for (int32_t a = 0; a < SIDE; ++a)
    {
        for (int32_t b = 0; b < SIDE; ++b)
        {
            double y = (double)(a - SIDE / 2) * 2.5;
            double z = (double)(b - SIDE / 2) * 2.5;
            if (fabs(y) < 4.0 && fabs(z) < 4.0)
            {
                continue;
            }
            wall[spheres] = (mnavSphere){{0.0, y, z}, 1.5, {0.0, 0.0, 0.0}, (uint64_t)spheres, 0};
            spheres += 1;
        }
    }
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.limits.agents = FLIERS;
    def.neighborDistance = 3.0;
    def.timeHorizon = 2.0;
    def.obstacleTimeHorizon = 1.0;
    mnavAvoidance* avoidance = NULL;
    Check(mnavCreateAvoidance(&def, &avoidance), "avoidance");
    double best = 1e30;
    int32_t arrived = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        for (int32_t i = 0; i < FLIERS; ++i)
        {
            double x = -30.0 + (double)(i % 10) * 1.6;
            double y = -7.2 + (double)(i / 10 % 10) * 1.6;
            double z = -7.2 + (double)(i / 100) * 1.6;
            fliers[i] = (mnavAgent3D){
                {x, y, z}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.3, 1.5, 1.0, (uint64_t)i, 0, 0};
            goals[i] = (mnavPos3){-x, y, z};
        }
        double start = Seconds();
        for (int32_t s = 0; s < STEPS; ++s)
        {
            for (int32_t i = 0; i < FLIERS; ++i)
            {
                // Through the gap's middle while still before the wall.
                mnavPos3 at = fliers[i].position;
                mnavPos3 aim = at.x < -1.5 ? (mnavPos3){0.0, 0.0, 0.0} : goals[i];
                double dx = aim.x - at.x;
                double dy = aim.y - at.y;
                double dz = aim.z - at.z;
                double length = sqrt(dx * dx + dy * dy + dz * dz);
                double scale = length > 1.0 ? 1.0 / length : 1.0;
                fliers[i].preferred = (mnavPos3){dx * scale, dy * scale, dz * scale};
            }
            Check(mnavAvoid3D(avoidance, fliers, FLIERS, wall, spheres, 0.1, velocities),
                  "avoid in space");
            for (int32_t i = 0; i < FLIERS; ++i)
            {
                fliers[i].velocity = velocities[i];
                fliers[i].position.x += velocities[i].x * 0.1;
                fliers[i].position.y += velocities[i].y * 0.1;
                fliers[i].position.z += velocities[i].z * 0.1;
            }
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
        arrived = 0;
        for (int32_t i = 0; i < FLIERS; ++i)
        {
            arrived += fliers[i].position.x > 1.5 ? 1 : 0;
        }
    }
    printf("# gap: %d fliers, %d spheres, %d through after %d steps, %.0f us per step, %.0f "
           "fliers per ms\n",
           FLIERS, spheres, arrived, STEPS, best * 1e6 / STEPS, FLIERS * STEPS / (best * 1e3));
    Report("avoidance in space step, 1000 fliers", best * 1e6 / STEPS, "us");
    Report("avoidance in space, fliers per ms", FLIERS * STEPS / (best * 1e3), "/ms");
    mnavDestroyAvoidance(avoidance);
}

// Times one repair of a field.
static double Repair(mnavFlowField* field, const mnavGrid* grid, const mnavCell* goals,
                     const mnavCell* changed, int32_t changedCount)
{
    double start = Seconds();
    Check(mnavUpdateFlowField(field, grid, goals, 4, changed, changedCount), "repair");
    Check(mnavContinueFlowField(field, grid, INT32_MAX, NULL), "repaired");
    return Seconds() - start;
}

// Repairs of the four-goal field: a cell in the middle blocked and opened
// again, a corner goal moved a cell and back, and a wall's gap closed and
// opened again.
static void FlowRepairs(mnavFlowField* field, const mnavGrid* grid, const mnavCell* goals)
{
    mnavAreaType* areas = (mnavAreaType*)grid->areas;
    mnavCell moved[4] = {goals[0], goals[1], goals[2], goals[3]};
    mnavCell gap[24];
    int32_t row = 252;
    int32_t first = (512 - (row * 5) % 512) % 512;
    for (int32_t i = 0; i < 24; ++i)
    {
        gap[i] = (mnavCell){(first + i) % 512, row};
    }
    const mnavCell middle = {256, 250};
    double cell = 1e30;
    double goal = 1e30;
    double wall = 1e30;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        for (int32_t k = 0; k < 2; ++k)
        {
            areas[middle.y * 512 + middle.x] = k == 0 ? mnav_areaNone : 1u;
            double took = Repair(field, grid, moved, &middle, 1);
            cell = took < cell ? took : cell;
            moved[0] = (mnavCell){1 - k, 0};
            took = Repair(field, grid, moved, NULL, 0);
            goal = took < goal ? took : goal;
            for (int32_t i = 0; i < 24; ++i)
            {
                areas[gap[i].y * 512 + gap[i].x] = k == 0 ? mnav_areaNone : 1u;
            }
            took = Repair(field, grid, moved, gap, 24);
            wall = took < wall ? took : wall;
        }
    }
    printf("# flow repairs: a cell %.0f us, a corner goal moved %.0f us, a gap closed or opened "
           "%.0f us\n",
           cell * 1e6, goal * 1e6, wall * 1e6);
    Report("flow repair, a cell", cell * 1e6, "us");
    Report("flow repair, a goal moved", goal * 1e6, "us");
    Report("flow repair, a gap", wall * 1e6, "us");
}

// A flow field over 512 by 512 cells of 0.5 m: a wall in every eighth
// row with a gap that moves along, and 1 in 16 other cells dearer; one
// goal in a corner, then four goals.
static void Flow(void)
{
    enum
    {
        SIDE = 512
    };
    static mnavAreaType areas[SIDE * SIDE];
    for (int32_t y = 0; y < SIDE; ++y)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            bool wall = y % 8 == 4 && (x + y * 5) % SIDE >= 24;
            areas[y * SIDE + x] = wall ? mnav_areaNone : ((x * 7 + y * 3) % 16 == 0 ? 2u : 1u);
        }
    }
    mnavGrid grid = {areas, SIDE, SIDE, 0.5f};
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.costs[2] = 3.0f;
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.cells = SIDE * SIDE;
    mnavFlowField* field = NULL;
    Check(mnavCreateFlowField(&def, &field), "flow field");
    const mnavCell goals[4] = {{0, 0}, {511, 511}, {0, 511}, {511, 0}};
    for (int32_t count = 1; count <= 4; count += 3)
    {
        double best = 1e30;
        for (int32_t run = 0; run < RUNS; ++run)
        {
            double start = Seconds();
            Check(mnavBuildFlowField(field, &grid, &filter, goals, count), "build");
            double took = Seconds() - start;
            best = took < best ? took : best;
        }
        mnavFlow far;
        Check(mnavFlowAt(field, (mnavCell){256, 256}, &far), "read");
        printf("# flow: %d by %d cells, %d goal(s), %.0f us per build, middle costs %.1f\n", SIDE,
               SIDE, count, best * 1e6, far.cost);
        Report(count == 1 ? "flow build, 1 goal" : "flow build, 4 goals", best * 1e6, "us");
    }
    FlowRepairs(field, &grid, goals);
    mnavDestroyFlowField(field);
}

enum
{
    WORLD_TILES = 24,
    WORLD_OUTLINES = 9300
};

static mnavVec2 s_worldPoints[WORLD_OUTLINES][4];
static mnavOutline s_worldOutlines[WORLD_OUTLINES];

static int32_t WorldBox(int32_t n, float x0, float z0, float x1, float z1, mnavAreaType area)
{
    mnavVec2* p = s_worldPoints[n];
    p[0] = (mnavVec2){x0, z0};
    p[1] = (mnavVec2){x1, z0};
    p[2] = (mnavVec2){x1, z1};
    p[3] = (mnavVec2){x0, z1};
    s_worldOutlines[n] = (mnavOutline){p, 4, area};
    return n + 1;
}

// The flat world: a floor, a pillar of 2 m every 8 m and, every 64 m, a
// wall across with one gap of 6 m.
static mnavNavmesh* FlatWorld(float* sideOut)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tiles = WORLD_TILES * WORLD_TILES;
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    // Areas may change, for the hierarchy update.
    def.tier = mnav_tierModifiers;
    float side = def.cellSize * (float)def.tileCells * (float)WORLD_TILES;
    int32_t n = WorldBox(0, 0.0f, 0.0f, side, side, mnav_areaWalkable);
    for (float z = 4.0f; z < side; z += 8.0f)
    {
        for (float x = 4.0f; x < side; x += 8.0f)
        {
            n = WorldBox(n, x, z, x + 2.0f, z + 2.0f, mnav_areaNone);
        }
    }
    int32_t k = 0;
    for (float z = 60.0f; z < side - 10.0f; z += 64.0f)
    {
        float gap = (float)((k++ * 37) % ((int32_t)side - 40) + 10);
        n = WorldBox(n, 1.0f, z, gap, z + 1.0f, mnav_areaNone);
        n = WorldBox(n, gap + 6.0f, z, side - 1.0f, z + 1.0f, mnav_areaNone);
    }
    mnavBaker* baker = NULL;
    mnavNavmesh* navmesh = NULL;
    Check(mnavCreateBaker(&def, &baker).result, "baker");
    Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
    size_t capacity = (size_t)1 << 20;
    uint8_t* buffer = malloc(capacity);
    for (int32_t z = 0; z < WORLD_TILES; ++z)
    {
        for (int32_t x = 0; x < WORLD_TILES; ++x)
        {
            size_t size = 0;
            Check(mnavBakeTile2D(baker, s_worldOutlines, n, x, z, NULL), "bake 2D");
            Check(mnavCopyBakedTile(baker, buffer, capacity, &size), "copy");
            Check(mnavStageTile(navmesh, buffer, size).result, "stage");
        }
    }
    Check(mnavCommit(navmesh), "commit");
    free(buffer);
    mnavDestroyBaker(baker);
    *sideOut = side;
    return navmesh;
}

static double Best(mnavResult (*find)(void*), void* context)
{
    double best = 1e30;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        double start = Seconds();
        Check(find(context), "find");
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    return best;
}

typedef struct LongPath
{
    mnavQuery* query;
    mnavHierarchy* hierarchy;
    const mnavNavmesh* navmesh;
    mnavNearest a;
    mnavNearest b;
    mnavPath path;
} LongPath;

static mnavResult FindPlain(void* context)
{
    LongPath* p = context;
    return mnavFindPath(p->query, p->navmesh, NULL, p->a.polygon, p->a.point, p->b.polygon,
                        p->b.point, &p->path);
}

static mnavResult FindThrough(void* context)
{
    LongPath* p = context;
    return mnavFindHierarchicalPath(p->query, p->hierarchy, p->navmesh, p->a.polygon, p->a.point,
                                    p->b.polygon, p->b.point, &p->path);
}

// A navmesh flow field over the whole flat world toward one goal in a
// corner, whole and in steps of 1,024 polygons.
static void NavFlow(const mnavNavmesh* navmesh, float side)
{
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.limits.polygons = 1 << 20;
    mnavNavFlow* field = NULL;
    Check(mnavCreateNavFlow(&def, &field), "navmesh flow field");
    mnavNearest goal;
    mnavNearest far;
    mnavVec3 box = {3.0f, 2.0f, 3.0f};
    Check(mnavFindNearest(navmesh, NULL, (mnavPos3){2.0, 0.0, 2.0}, box, &goal), "nearest");
    Check(mnavFindNearest(navmesh, NULL, (mnavPos3){(double)side - 2.0, 0.0, (double)side - 2.0},
                          box, &far),
          "nearest");
    const mnavNavFlowGoal goals[1] = {{goal.polygon, goal.point}};
    double best = 1e30;
    double stepped = 1e30;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        double start = Seconds();
        Check(mnavBuildNavFlow(field, navmesh, NULL, goals, 1), "build");
        double took = Seconds() - start;
        best = took < best ? took : best;
        start = Seconds();
        Check(mnavBeginNavFlow(field, navmesh, NULL, NULL, goals, 1), "begin");
        bool ended = false;
        while (!ended)
        {
            Check(mnavContinueNavFlow(field, navmesh, 1024, &ended), "continue");
        }
        took = Seconds() - start;
        stepped = took < stepped ? took : stepped;
    }
    mnavPolygonFlow way;
    Check(mnavNavFlowAt(field, navmesh, far.polygon, &way), "read");
    printf("# navmesh flow field: far corner at cost %.0f, built in %.0f us, %.0f us in steps\n",
           way.cost, best * 1e6, stepped * 1e6);
    Report("navmesh flow field", best * 1e6, "us");
    Report("navmesh flow field, stepped", stepped * 1e6, "us");
    mnavDestroyNavFlow(field);
}

// Long paths on the flat world, plainly with 131,072 nodes and through a
// hierarchy of 4 by 4 tiles with the default 8,192.
static void Hierarchy(void)
{
    float side = 0.0f;
    mnavNavmesh* navmesh = FlatWorld(&side);
    mnavQueryDef def = mnavDefaultQueryDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.limits.pathLength = 1.0e5f;
    mnavQuery* small = NULL;
    Check(mnavCreateQuery(&def, &small), "query");
    def.limits.nodes = 131072;
    mnavQuery* big = NULL;
    Check(mnavCreateQuery(&def, &big), "query");
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.allocator = (mnavAllocator){Alloc, Free, NULL};
    mnavHierarchy* hierarchy = NULL;
    Check(mnavCreateHierarchy(&hierarchyDef, &hierarchy), "hierarchy");
    mnavHierarchyReport report;
    double start = Seconds();
    Check(mnavBuildHierarchy(hierarchy, small, navmesh, NULL, &report), "build");
    double built = Seconds() - start;
    printf("# hierarchy: %d clusters, %d transitions, %d edges, built in %.0f us\n",
           report.clusters, report.transitions, report.edges, built * 1e6);
    Report("hierarchy build", built * 1e6, "us");
    double plainSum = 0.0;
    double throughSum = 0.0;
    double w = (double)side;
    const double ends[3][4] = {
        {2, 2, w - 2, w - 2}, {2, w - 2, w - 2, 2}, {w / 2, 2, w / 2, w - 2}};
    for (int32_t i = 0; i < 3; ++i)
    {
        LongPath p = {.query = big, .hierarchy = hierarchy, .navmesh = navmesh};
        mnavVec3 box = {3.0f, 2.0f, 3.0f};
        Check(mnavFindNearest(navmesh, NULL, (mnavPos3){ends[i][0], 0.0, ends[i][1]}, box, &p.a),
              "nearest");
        Check(mnavFindNearest(navmesh, NULL, (mnavPos3){ends[i][2], 0.0, ends[i][3]}, box, &p.b),
              "nearest");
        double plain = Best(FindPlain, &p);
        double plainLength = p.path.length;
        p.query = small;
        double through = Best(FindThrough, &p);
        printf("# long path %d: plain %.0f m in %.0f us, hierarchical %.0f m in %.0f us\n", i,
               plainLength, plain * 1e6, p.path.length, through * 1e6);
        plainSum += plain;
        throughSum += through;
    }
    Report("long path, plain", plainSum * 1e6 / 3.0, "us");
    Report("long path, hierarchical", throughSum * 1e6 / 3.0, "us");
    // One polygon's area changed in a tile in the middle, back and forth:
    // the update searches the clusters the change reaches, not all.
    mnavNearest middle;
    Check(mnavFindNearest(navmesh, NULL, (mnavPos3){w / 2 + 3, 0.0, w / 2 + 3},
                          (mnavVec3){3.0f, 2.0f, 3.0f}, &middle),
          "nearest");
    double update = 1e30;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        Check(mnavStageArea(navmesh, middle.polygon, run % 2 == 0 ? 3 : mnav_areaWalkable), "area");
        Check(mnavCommit(navmesh), "commit");
        double begin = Seconds();
        Check(mnavUpdateHierarchy(hierarchy, small, navmesh, &report), "update");
        double took = Seconds() - begin;
        update = took < update ? took : update;
    }
    printf("# hierarchy update: %d of %d transitions searched\n", report.searches,
           report.transitions);
    Report("hierarchy update, one tile", update * 1e6, "us");
    NavFlow(navmesh, side);
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(big);
    mnavDestroyQuery(small);
    mnavDestroyNavmesh(navmesh);
}

// The flight section: boxes 2 to 10 m across floating 4 to 40 m over the
// terrain, which, with the terrain, a flight volume of 1 m voxels holds
// from 2 m below the ground to 62 m up.
enum
{
    DEBRIS = 300,
    FLIGHT_PATHS = 200,
    FLIGHT_RAYS = 2000
};

static mnavVec3 s_debrisVertices[DEBRIS * 8];
static int32_t s_debrisIndices[DEBRIS * 36];

static float Unit(uint32_t* state)
{
    *state = *state * 1664525u + 1013904223u;
    return (float)(*state >> 8 & 0xFFFFu) / 65536.0f;
}

static mnavTriangleMesh Debris(void)
{
    const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                               2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
    uint32_t state = 11;
    for (int32_t b = 0; b < DEBRIS; ++b)
    {
        float side = 2.0f + 8.0f * Unit(&state);
        float x0 = Unit(&state) * ((float)TILES * 32.0f - side);
        float y0 = 4.0f + 36.0f * Unit(&state);
        float z0 = Unit(&state) * ((float)TILES * 32.0f - side);
        for (int32_t k = 0; k < 8; ++k)
        {
            s_debrisVertices[b * 8 + k] =
                (mnavVec3){x0 + ((k & 1) ? side : 0.0f), y0 + ((k & 4) ? side : 0.0f),
                           z0 + ((k & 2) ? side : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_debrisIndices[b * 36 + k] = b * 8 + faces[k];
        }
    }
    return (mnavTriangleMesh){s_debrisVertices, DEBRIS * 8, s_debrisIndices, DEBRIS * 12, NULL};
}

static mnavFlightDef FlightDef(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.allocator = (mnavAllocator){Alloc, Free, NULL};
    def.floor = -2.0f;
    def.ceiling = 62.0f;
    return def;
}

// Bakes the flight tiles, the best of the runs, into a committed volume.
static mnavFlightVolume* FlightBake(void)
{
    mnavFlightDef def = FlightDef();
    mnavTriangleMesh meshes[2] = {Terrain(), Debris()};
    mnavBakeInput input = {meshes, 2, NULL, 0, NULL, 0, NULL};
    static uint8_t buffer[TILE_BYTES];
    mnavFlightVolume* volume = NULL;
    Check(mnavCreateFlightVolume(&def, &volume).result, "flight volume");
    double best = 1e30;
    size_t bytes = 0;
    int64_t nodes = 0;
    int64_t leaves = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        mnavFlightBaker* baker = NULL;
        Check(mnavCreateFlightBaker(&def, &baker).result, "flight baker");
        double start = Seconds();
        bytes = 0;
        nodes = 0;
        leaves = 0;
        for (int32_t t = 0; t < TILES * TILES; ++t)
        {
            mnavFlightBakeReport report;
            size_t size = 0;
            Check(mnavBakeFlightTile(baker, &input, t % TILES, t / TILES, &report), "flight bake");
            Check(mnavCopyFlightTile(baker, buffer, TILE_BYTES, &size), "flight copy");
            if (run == 0)
            {
                Check(mnavStageFlightTile(volume, buffer, size).result, "flight stage");
            }
            bytes += size;
            nodes += report.nodes;
            leaves += report.leaves;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
        mnavDestroyFlightBaker(baker);
    }
    Check(mnavCommitFlight(volume), "flight commit");
    printf("# flight bake: %d tiles, %lld nodes, %lld leaves, %zu bytes, %.0f us per tile\n",
           TILES * TILES, (long long)nodes, (long long)leaves, bytes, best * 1e6 / (TILES * TILES));
    Report("flight bake, a tile", best * 1e6 / (TILES * TILES), "us");
    return volume;
}

// A point open to the flier, snapped from one drawn over the volume.
static mnavPos3 FlightPoint(const mnavFlightVolume* volume, uint32_t* state)
{
    for (;;)
    {
        // One draw a statement: an initializer's order is unspecified.
        double x = (double)Unit(state) * TILES * 32.0;
        double y = 4.0 + 50.0 * (double)Unit(state);
        double z = (double)Unit(state) * TILES * 32.0;
        mnavPos3 p = {x, y, z};
        mnavPos3 open;
        bool found = false;
        Check(mnavFindNearestFlightPoint(volume, p, 4.0f, &open, &found), "flight nearest");
        if (found)
        {
            return open;
        }
    }
}

static void FlightPaths(const mnavFlightVolume* volume)
{
    static mnavPos3 ends[FLIGHT_PATHS][2];
    uint32_t state = 5;
    for (int32_t k = 0; k < FLIGHT_PATHS; ++k)
    {
        ends[k][0] = FlightPoint(volume, &state);
        ends[k][1] = FlightPoint(volume, &state);
    }
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = 65536;
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&def, &query), "query");
    double best = 1e30;
    int32_t found = 0;
    int64_t points = 0;
    double length = 0.0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        found = 0;
        points = 0;
        length = 0.0;
        double start = Seconds();
        for (int32_t k = 0; k < FLIGHT_PATHS; ++k)
        {
            mnavFlightPath path;
            Check(mnavFindFlightPath(query, volume, ends[k][0], ends[k][1], &path), "flight path");
            found += path.end == mnav_pathFound ? 1 : 0;
            points += path.pointCount;
            length += path.length;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    // The same paths in slices of 64 nodes, as a host spreading them
    // over frames runs them; the paths are the same.
    double slicedBest = 1e30;
    int64_t slices = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        slices = 0;
        double start = Seconds();
        for (int32_t k = 0; k < FLIGHT_PATHS; ++k)
        {
            bool ended = false;
            mnavFlightPath path;
            Check(mnavBeginFlightPath(query, volume, ends[k][0], ends[k][1]), "begin");
            while (!ended)
            {
                Check(mnavContinueFlightPath(query, volume, 64, &ended), "continue");
                slices += 1;
            }
            Check(mnavFinishFlightPath(query, volume, &path), "finish");
        }
        double took = Seconds() - start;
        slicedBest = took < slicedBest ? took : slicedBest;
    }
    mnavDestroyQuery(query);
    printf("# flight paths: %d of %d found, %.1f m in all, %lld points, %.1f us per path; in "
           "slices of 64 nodes, %lld slices, %.1f us per path\n",
           found, FLIGHT_PATHS, length, (long long)points, best * 1e6 / FLIGHT_PATHS,
           (long long)slices, slicedBest * 1e6 / FLIGHT_PATHS);
    Report("flight path query", best * 1e6 / FLIGHT_PATHS, "us");
    Report("flight path in slices of 64 nodes", slicedBest * 1e6 / FLIGHT_PATHS, "us");
}

static void FlightRays(const mnavFlightVolume* volume)
{
    static mnavPos3 ends[FLIGHT_RAYS][2];
    uint32_t state = 9;
    for (int32_t k = 0; k < FLIGHT_RAYS; ++k)
    {
        ends[k][0] = FlightPoint(volume, &state);
        double dx = 60.0 * (double)Unit(&state) - 30.0;
        double dy = 20.0 * (double)Unit(&state) - 10.0;
        double dz = 60.0 * (double)Unit(&state) - 30.0;
        ends[k][1] = (mnavPos3){ends[k][0].x + dx, ends[k][0].y + dy, ends[k][0].z + dz};
    }
    double best = 1e30;
    int32_t clear = 0;
    for (int32_t run = 0; run < RUNS; ++run)
    {
        clear = 0;
        double start = Seconds();
        for (int32_t k = 0; k < FLIGHT_RAYS; ++k)
        {
            mnavFlightHit hit;
            Check(mnavFlightRaycast(volume, ends[k][0], ends[k][1], &hit), "flight ray");
            clear += hit.stop == mnav_flightClear ? 1 : 0;
        }
        double took = Seconds() - start;
        best = took < best ? took : best;
    }
    printf("# flight rays: %d of %d clear, %.2f us per ray\n", clear, FLIGHT_RAYS,
           best * 1e6 / FLIGHT_RAYS);
    Report("flight raycast", best * 1e6 / FLIGHT_RAYS, "us");
}

static void Flight(void)
{
    mnavFlightVolume* volume = FlightBake();
    FlightPaths(volume);
    FlightRays(volume);
    mnavDestroyFlightVolume(volume);
}

int main(int argc, char** argv)
{
    if (argc > 1)
    {
        FILE* file = fopen(argv[1], "rb");
        size_t read = file != NULL ? fread(s_baseline, 1, sizeof(s_baseline) - 1, file) : 0;
        s_baseline[read] = '\0';
        if (file != NULL)
        {
            fclose(file);
        }
    }
    Bake();
    Rebuild();
    Stream(0);
    Stream(LINKS);
    Paths();
    Doorway();
    Crowd();
    Gap();
    Flow();
    Hierarchy();
    Flight();
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        free(s_tiles[t]);
    }
    return 0;
}
