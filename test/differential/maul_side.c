// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Maul Nav's side of the differential harness.

#include "defs_for.h"
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
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    if (maul == nullptr || mnavCreateNavmesh(&meshDef, &maul->navmesh).result != mnav_success ||
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

int32_t DiffMaulRaycast(DiffMaul* maul, const float a[3], const float b[3], float start[3],
                        double* t, float normal[3])
{
    mnavNearest s;
    if (!Nearest(maul, a, &s))
    {
        return -1;
    }
    start[0] = (float)s.point.x;
    start[1] = (float)s.point.y;
    start[2] = (float)s.point.z;
    mnavRay ray;
    if (mnavQuery_Raycast(maul->query, maul->navmesh, s.polygon, s.point,
                          (mnavPos3){(double)b[0], (double)b[1], (double)b[2]}, nullptr,
                          &ray) != mnav_success)
    {
        return 2;
    }
    *t = ray.t;
    normal[0] = ray.normal.x;
    normal[1] = ray.normal.y;
    normal[2] = ray.normal.z;
    return ray.end == mnav_rayWall ? 1 : (ray.end == mnav_rayReached ? 0 : 2);
}

int32_t DiffMaulRayWalls(DiffMaul* maul, const float a[3], const float b[3], const double at[2],
                         double radius, float* segments, int32_t capacity)
{
    mnavNearest s;
    if (!Nearest(maul, a, &s))
    {
        return -1;
    }
    mnavRay ray;
    if (mnavQuery_Raycast(maul->query, maul->navmesh, s.polygon, s.point,
                          (mnavPos3){(double)b[0], (double)b[1], (double)b[2]}, nullptr,
                          &ray) != mnav_success)
    {
        return -1;
    }
    // The first polygon the ray crossed that lies under at, before the
    // walls' search ends the ray's query; a point on an edge, as a ray's
    // stop is, may round outside each, so failing that the point 0.01 m
    // back toward the start is the center.
    double ux = s.point.x - at[0];
    double uz = s.point.z - at[1];
    double length = sqrt(ux * ux + uz * uz);
    double back = length > 0.01 ? 0.01 / length : 0.0;
    mnavPos3 center = {at[0], 0.0, at[1]};
    mnavPolygonId polygon = {0};
    for (int32_t k = 0; k < 2 && polygon.slot == 0; ++k)
    {
        center.x = at[0] + (double)k * back * ux;
        center.z = at[1] + (double)k * back * uz;
        for (int32_t i = 0; i < ray.polygonCount && polygon.slot == 0; ++i)
        {
            if (mnavPolygon_GetHeight(maul->navmesh, ray.polygons[i], center.x, center.z,
                                      &center.y) == mnav_success)
            {
                polygon = ray.polygons[i];
            }
        }
    }
    if (polygon.slot == 0)
    {
        return -1;
    }
    enum
    {
        WALL_ROOM = 64
    };
    static mnavWall walls[WALL_ROOM];
    mnavWallsFound found;
    if (mnavQuery_FindWalls(maul->query, maul->navmesh, polygon, center, radius + 0.01, nullptr,
                            walls, WALL_ROOM, &found) != mnav_success ||
        found.limited || found.wallCount > capacity)
    {
        return -2;
    }
    for (int32_t i = 0; i < found.wallCount; ++i)
    {
        float* w = &segments[i * 6];
        w[0] = (float)walls[i].start.x;
        w[1] = (float)walls[i].start.y;
        w[2] = (float)walls[i].start.z;
        w[3] = (float)walls[i].end.x;
        w[4] = (float)walls[i].end.y;
        w[5] = (float)walls[i].end.z;
    }
    return found.wallCount;
}

int32_t DiffMaulWallDistance(DiffMaul* maul, const float at[3], double radius, float center[3],
                             double* distance, float wall[3])
{
    mnavNearest s;
    if (!Nearest(maul, at, &s))
    {
        return -1;
    }
    center[0] = (float)s.point.x;
    center[1] = (float)s.point.y;
    center[2] = (float)s.point.z;
    mnavNearestWall nearest;
    if (mnavQuery_FindWallDistance(maul->query, maul->navmesh, s.polygon, s.point, radius, nullptr,
                                   &nearest) != mnav_success ||
        nearest.limited)
    {
        return 2;
    }
    if (!nearest.found)
    {
        return 0;
    }
    *distance = nearest.wall.distance;
    wall[0] = (float)nearest.wall.point.x;
    wall[1] = (float)nearest.wall.point.y;
    wall[2] = (float)nearest.wall.point.z;
    return 1;
}

int32_t DiffMaulMove(DiffMaul* maul, const float a[3], const float to[3], float start[3],
                     float end[3])
{
    mnavNearest s;
    if (!Nearest(maul, a, &s))
    {
        return -1;
    }
    start[0] = (float)s.point.x;
    start[1] = (float)s.point.y;
    start[2] = (float)s.point.z;
    mnavMove move;
    if (mnavQuery_MoveAlongSurface(maul->query, maul->navmesh, s.polygon, s.point,
                                   (mnavPos3){to[0], to[1], to[2]}, nullptr, &move) != mnav_success)
    {
        return 2;
    }
    end[0] = (float)move.point.x;
    end[1] = (float)move.point.y;
    end[2] = (float)move.point.z;
    return move.end == mnav_moveReached ? 0 : (move.end == mnav_moveWall ? 1 : 2);
}
