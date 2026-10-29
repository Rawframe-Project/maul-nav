// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Maul Nav's side of the differential harness.

#include "diff.h"

// The query context's insides, for the nodes a search made: the harness
// is test only and reads them as the white-box suites do.
#include "query.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

enum
{
    TILE_ROOM = 1 << 20
};

struct DiffMaul
{
    mnavNavmesh* navmesh;
    mnavQuery* query;
};

static int32_t TileOf(float v)
{
    return (int32_t)floorf(v / 32.0f);
}

// Bakes every tile of the scene's bounds and, with a navmesh, stages it;
// bytes, which may be NULL, receives the tiles' bytes in all.
static bool Load(mnavNavmesh* navmesh, const DiffScene* scene, int64_t* bytes)
{
    float lo[2] = {1e30f, 1e30f};
    float hi[2] = {-1e30f, -1e30f};
    for (int32_t v = 0; v < scene->vertexCount; ++v)
    {
        for (int32_t k = 0; k < 2; ++k)
        {
            float c = scene->vertices[v * 3 + k * 2];
            lo[k] = c < lo[k] ? c : lo[k];
            hi[k] = c > hi[k] ? c : hi[k];
        }
    }
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    if (mnavCreateBaker(&def, &baker).result != mnav_success)
    {
        return false;
    }
    const mnavTriangleMesh mesh = {(const mnavVec3*)scene->vertices, scene->vertexCount,
                                   scene->indices, scene->triangleCount, nullptr};
    static uint8_t tile[TILE_ROOM];
    bool good = true;
    int64_t total = 0;
    for (int32_t z = TileOf(lo[1]); z <= TileOf(hi[1]) && good; ++z)
    {
        for (int32_t x = TileOf(lo[0]); x <= TileOf(hi[0]) && good; ++x)
        {
            size_t size = 0;
            good = mnavBaker_BakeTile(baker,
                                      &(mnavBakeInput){&mesh, 1, nullptr, 0, nullptr, 0, nullptr},
                                      x, z, nullptr) == mnav_success &&
                   mnavBaker_CopyTile(baker, tile, TILE_ROOM, &size) == mnav_success &&
                   (navmesh == nullptr ||
                    mnavNavmesh_StageTile(navmesh, tile, size).result == mnav_success);
            total += (int64_t)size;
        }
    }
    mnavDestroyBaker(baker);
    if (bytes != nullptr)
    {
        *bytes = total;
    }
    return good && (navmesh == nullptr || mnavNavmesh_Commit(navmesh) == mnav_success);
}

int64_t DiffMaulBake(const DiffScene* scene)
{
    int64_t bytes = 0;
    return Load(nullptr, scene, &bytes) ? bytes : -1;
}

DiffMaul* DiffMaulBuild(const DiffScene* scene)
{
    DiffMaul* maul = calloc(1, sizeof(DiffMaul));
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    if (maul == nullptr || mnavCreateNavmesh(&def, &maul->navmesh).result != mnav_success ||
        mnavCreateQuery(&queryDef, &maul->query) != mnav_success ||
        !Load(maul->navmesh, scene, nullptr))
    {
        DiffMaulDestroy(maul);
        return nullptr;
    }
    return maul;
}

void DiffMaulDestroy(DiffMaul* maul)
{
    if (maul != nullptr)
    {
        mnavDestroyQuery(maul->query);
        mnavDestroyNavmesh(maul->navmesh);
        free(maul);
    }
}

bool DiffMaulPoint(DiffMaul* maul, uint64_t seed, float out[3])
{
    mnavRandomPoint p;
    if (mnavNavmesh_FindRandomPoint(maul->navmesh, seed, nullptr, &p) != mnav_success)
    {
        return false;
    }
    out[0] = (float)p.point.x;
    out[1] = (float)p.point.y;
    out[2] = (float)p.point.z;
    return true;
}

static bool Nearest(const DiffMaul* maul, const float at[3], mnavNearest* n)
{
    mnavPos3 p = {at[0], at[1], at[2]};
    return mnavNavmesh_FindNearest(maul->navmesh, p, (mnavVec3){0.5f, 2.0f, 0.5f}, nullptr, n) ==
               mnav_success &&
           n->polygon.slot != 0;
}

int32_t DiffMaulPath(DiffMaul* maul, const float a[3], const float b[3], double* length,
                     int32_t* nodes)
{
    mnavNearest s;
    mnavNearest e;
    if (!Nearest(maul, a, &s) || !Nearest(maul, b, &e))
    {
        return -1;
    }
    mnavPath path;
    mnavResult result = mnavQuery_FindPath(maul->query, maul->navmesh, s.polygon, s.point,
                                           e.polygon, e.point, nullptr, &path);
    if (nodes != nullptr)
    {
        *nodes = maul->query->nodeCount;
    }
    if (result != mnav_success || path.end != mnav_pathFound)
    {
        return 0;
    }
    double sum = 0.0;
    for (int32_t i = 0; i + 1 < path.pointCount; ++i)
    {
        mnavPos3 p = path.points[i];
        mnavPos3 q = path.points[i + 1];
        sum +=
            sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z));
    }
    *length = sum;
    return 1;
}
