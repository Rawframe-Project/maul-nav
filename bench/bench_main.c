// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Timings over a terrain of 8 by 8 tiles of 32 m: hills on a 2 m grid
// (32,768 triangles) and 256 boxes. Bakes every tile; streams a window
// of 5 by 5 tiles as a camera crosses the terrain and back, staging the
// tiles that enter and leave and committing each step, for ten laps;
// then finds paths
// on the whole terrain; then steers 1,000 agents through a doorway 4 m wide
// with avoidance; then builds flow fields over a grid of 512 by 512 cells.
// Prints the best of five runs in microseconds,
// with counts and bytes, which do not depend on the machine.

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
    printf("bake: %d tiles, %d triangles, %d polygons, %zu bytes, %.0f us per tile\n",
           TILES * TILES, terrain.triangleCount, polygons, bytes, best * 1e6 / (TILES * TILES));
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
                                true};
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
    printf("stream with %d links: %d steps, %d tiles staged, %.0f us per step (best %.0f, worst "
           "%.0f), "
           "%zu bytes held after the first lap, %zu after the last, %zu at most\n",
           links, steps, staged, total * 1e6 / steps, bestStep * 1e6, worstStep * 1e6, firstLap,
           lastLap, peak);
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
    printf("paths: %d of %d found, %d points, %.1f us per path\n", found, PATHS, corners,
           best * 1e6 / PATHS);
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
    const mnavObstacle wall[2] = {{below, 4, 0.0, {0.0, 0.0}, 1}, {above, 4, 0.0, {0.0, 0.0}, 2}};
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
            agents[i] = (mnavAgent){{x, y}, {0.0, 0.0}, {0.0, 0.0}, 0.3, 1.5, 1.0, (uint64_t)i};
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
    printf("doorway: %d agents, %d through after %d steps, %.0f us per step, %.0f agents per ms\n",
           AGENTS, arrived, STEPS, best * 1e6 / STEPS, AGENTS * STEPS / (best * 1e3));
    mnavDestroyAvoidance(avoidance);
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
        printf("flow: %d by %d cells, %d goal(s), %.0f us per build, middle costs %.1f\n", SIDE,
               SIDE, count, best * 1e6, far.cost);
    }
    mnavDestroyFlowField(field);
}

int main(void)
{
    Bake();
    Stream(0);
    Stream(LINKS);
    Paths();
    Doorway();
    Flow();
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        free(s_tiles[t]);
    }
    return 0;
}
