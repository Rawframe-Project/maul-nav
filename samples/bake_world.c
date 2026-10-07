// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking a world from one large mesh: hills of 128 m by 128 m on a 1 m
// grid, with walls, are one mesh of about 33,000 triangles over 4 by 4
// tiles. A tile index of the mesh is made once; four workers, each with
// its own baker, share it and bake the tiles, each reading only the
// triangles near its tile. The tiles are checked against a bake on one
// thread without the index, to the byte, then loaded into a navmesh and
// crossed by a path. Returns 0 when the tiles agree and the path is
// found.

#include "maul-nav/bake.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

enum
{
    TILES = 4,
    WORKERS = 4,
    SIDE = 129,
    WALLS = 6,
    TILE_ROOM = 1 << 20
};

// One worker: its first tile (it bakes every WORKERS-th from there), the
// def and input it shares with the others, and how it ended.
typedef struct Worker
{
    int32_t first;
    const mnavBakeDef* def;
    const mnavBakeInput* input;
    mnavResult result;
} Worker;

// The workers' tiles go here, one buffer per tile.
static uint8_t* s_tiles[TILES * TILES];
static size_t s_sizes[TILES * TILES];

static mnavVec3 s_vertices[SIDE * SIDE + WALLS * 4];
static int32_t s_indices[(SIDE - 1) * (SIDE - 1) * 6 + WALLS * 6];

// Hills on a 1 m grid, two triangles a cell wound up, and walls 2 m tall
// standing across them.
static mnavTriangleMesh World(void)
{
    int32_t v = 0;
    int32_t i = 0;
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 0; x < SIDE; ++x)
        {
            float y = 1.5f * sinf((float)x * 0.05f) * cosf((float)z * 0.04f);
            s_vertices[v++] = (mnavVec3){(float)x, y, (float)z};
        }
    }
    for (int32_t z = 0; z + 1 < SIDE; ++z)
    {
        for (int32_t x = 0; x + 1 < SIDE; ++x)
        {
            int32_t a = z * SIDE + x;
            const int32_t cell[6] = {a, a + SIDE, a + SIDE + 1, a, a + SIDE + 1, a + 1};
            memcpy(&s_indices[i], cell, sizeof(cell));
            i += 6;
        }
    }
    for (int32_t w = 0; w < WALLS; ++w)
    {
        // A wall along X at z = 18 + 18 w, from x = 10 to 70 or 58 to 118.
        float z = 18.0f + 18.0f * (float)w;
        float x0 = w % 2 == 0 ? 10.0f : 58.0f;
        int32_t b = v;
        s_vertices[v++] = (mnavVec3){x0, -2.0f, z};
        s_vertices[v++] = (mnavVec3){x0 + 60.0f, -2.0f, z};
        s_vertices[v++] = (mnavVec3){x0 + 60.0f, 4.0f, z};
        s_vertices[v++] = (mnavVec3){x0, 4.0f, z};
        const int32_t quad[6] = {b, b + 1, b + 2, b, b + 2, b + 3};
        memcpy(&s_indices[i], quad, sizeof(quad));
        i += 6;
    }
    return (mnavTriangleMesh){s_vertices, v, s_indices, i / 3, NULL};
}

// Bakes a worker's tiles with its own baker.
static void BakeTiles(Worker* w)
{
    mnavBaker* baker = NULL;
    w->result = mnavCreateBaker(w->def, &baker).result;
    for (int32_t t = w->first; t < TILES * TILES && w->result == mnav_success; t += WORKERS)
    {
        w->result = mnavBakeTileInput(baker, w->input, t % TILES, t / TILES, NULL);
        if (w->result == mnav_success)
        {
            s_tiles[t] = malloc(TILE_ROOM);
            w->result = s_tiles[t] == NULL
                            ? mnav_errorCapacity
                            : mnavCopyBakedTile(baker, s_tiles[t], TILE_ROOM, &s_sizes[t]);
        }
    }
    mnavDestroyBaker(baker);
}

// The host's own threads: four workers run BakeTiles.
#ifdef _WIN32
static DWORD WINAPI Run(LPVOID w)
{
    BakeTiles(w);
    return 0;
}

static void RunWorkers(Worker workers[WORKERS])
{
    HANDLE threads[WORKERS];
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        threads[k] = CreateThread(NULL, 0, Run, &workers[k], 0, NULL);
    }
    WaitForMultipleObjects(WORKERS, threads, TRUE, INFINITE);
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        CloseHandle(threads[k]);
    }
}
#else
static void* Run(void* w)
{
    BakeTiles(w);
    return NULL;
}

static void RunWorkers(Worker workers[WORKERS])
{
    pthread_t threads[WORKERS];
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        pthread_create(&threads[k], NULL, Run, &workers[k]);
    }
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        pthread_join(threads[k], NULL);
    }
}
#endif

// Whether one thread's bake without the index gives the same bytes.
static int32_t SameAsOneThread(const mnavBakeDef* def, const mnavTriangleMesh* mesh)
{
    mnavBaker* baker = NULL;
    uint8_t* bytes = malloc(TILE_ROOM);
    int32_t same = 0;
    if (bytes != NULL && mnavCreateBaker(def, &baker).result == mnav_success)
    {
        for (int32_t t = 0; t < TILES * TILES; ++t)
        {
            size_t size = 0;
            same += mnavBakeTile(baker, mesh, 1, t % TILES, t / TILES, NULL) == mnav_success &&
                            mnavCopyBakedTile(baker, bytes, TILE_ROOM, &size) == mnav_success &&
                            size == s_sizes[t] && memcmp(bytes, s_tiles[t], size) == 0
                        ? 1
                        : 0;
        }
    }
    mnavDestroyBaker(baker);
    free(bytes);
    return same;
}

// Loads the tiles and finds a path from one corner to the far one.
static int32_t Cross(const mnavBakeDef* def)
{
    mnavNavmesh* navmesh = NULL;
    mnavQuery* query = NULL;
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    if (mnavCreateNavmesh(def, &navmesh).result != mnav_success ||
        mnavCreateQuery(&queryDef, &query) != mnav_success)
    {
        mnavDestroyNavmesh(navmesh);
        return -1;
    }
    mnavResult loaded = mnav_success;
    for (int32_t t = 0; t < TILES * TILES && loaded == mnav_success; ++t)
    {
        loaded = mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result;
    }
    mnavNearest from = {0};
    mnavNearest to = {0};
    mnavPath path = {0};
    const mnavVec3 box = {2.0f, 4.0f, 2.0f};
    int32_t points = -1;
    if (loaded == mnav_success && mnavCommit(navmesh) == mnav_success &&
        mnavFindNearest(navmesh, NULL, (mnavPos3){4.0, 0.0, 4.0}, box, &from) == mnav_success &&
        mnavFindNearest(navmesh, NULL, (mnavPos3){124.0, 0.0, 124.0}, box, &to) == mnav_success &&
        mnavFindPath(query, navmesh, NULL, from.polygon, from.point, to.polygon, to.point, &path) ==
            mnav_success &&
        path.end == mnav_pathFound)
    {
        printf("path: %d points, %.1f m\n", path.pointCount, path.length);
        points = path.pointCount;
    }
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return points;
}

int main(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    const mnavTriangleMesh world = World();

    // The index is made once; every worker reads it.
    mnavTileIndex* index = NULL;
    mnavResult made = mnavCreateTileIndex(&def, &world, 1, &index, NULL);
    if (made != mnav_success)
    {
        printf("the index: %d\n", made);
        return 1;
    }
    const mnavBakeInput input = {&world, 1, NULL, 0, NULL, 0, index};
    Worker workers[WORKERS];
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        workers[k] = (Worker){.first = k, .def = &def, .input = &input};
    }
    RunWorkers(workers);
    mnavDestroyTileIndex(index);
    bool baked = true;
    for (int32_t k = 0; k < WORKERS; ++k)
    {
        baked = baked && workers[k].result == mnav_success;
    }
    size_t bytes = 0;
    for (int32_t t = 0; t < TILES * TILES && baked; ++t)
    {
        bytes += s_sizes[t];
    }
    printf("baked: %d triangles into %d tiles of %zu bytes on %d workers\n", world.triangleCount,
           TILES * TILES, bytes, WORKERS);
    int32_t same = baked ? SameAsOneThread(&def, &world) : 0;
    printf("the same as one thread without the index: %d of %d tiles\n", same, TILES * TILES);
    int32_t points = same == TILES * TILES ? Cross(&def) : -1;
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        free(s_tiles[t]);
    }
    return points > 0 ? 0 : 1;
}
