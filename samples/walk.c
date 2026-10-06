// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// From geometry to a walking agent: a floor of 64 m by 64 m with a wall
// across it is baked into four tiles, the tiles are loaded into a
// navmesh, a path is found round the wall, and an agent follows it
// through a path corridor, a meter a step, toward the corridor's next
// corner. Prints the path and the walk; returns 0 when the agent
// arrives.

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

enum
{
    TILE_ROOM = 1 << 18,
    CORRIDOR = 256
};

// A floor and a wall 2 m tall from x = 0 to x = 48 at z = 30, so that the
// way from one side to the other goes round its east end.
static const mnavVec3 s_vertices[12] = {
    {0.0f, 0.0f, 0.0f},  {0.0f, 0.0f, 64.0f},  {64.0f, 0.0f, 64.0f}, {64.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 30.0f}, {48.0f, 0.0f, 30.0f}, {48.0f, 0.0f, 31.0f}, {0.0f, 0.0f, 31.0f},
    {0.0f, 2.0f, 30.0f}, {48.0f, 2.0f, 30.0f}, {48.0f, 2.0f, 31.0f}, {0.0f, 2.0f, 31.0f}};
static const int32_t s_indices[] = {0, 1,  2,  0, 2,  3,                      // the floor
                                    8, 11, 10, 8, 10, 9,                      // the wall's top
                                    4, 8,  9,  4, 9,  5, 5, 9,  10, 5, 10, 6, // its sides
                                    6, 10, 11, 6, 11, 7, 7, 11, 8,  7, 8,  4};

static void Check(mnavResult result, const char* what)
{
    if (result != mnav_success)
    {
        fprintf(stderr, "%s: %s\n", what, mnavResultName(result));
        exit(1);
    }
}

static mnavNavmesh* BakeAndLoad(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = NULL;
    mnavNavmesh* navmesh = NULL;
    Check(mnavCreateBaker(&def, &baker).result, "baker");
    Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
    const mnavTriangleMesh mesh = {s_vertices, 12, s_indices,
                                   (int32_t)(sizeof(s_indices) / sizeof(s_indices[0]) / 3), NULL};
    uint8_t* bytes = malloc(TILE_ROOM);
    // The default tiles are 32 m: four cover the floor. Each bake is
    // independent, so a host may run them on its own workers.
    for (int32_t z = 0; z < 2; ++z)
    {
        for (int32_t x = 0; x < 2; ++x)
        {
            size_t size = 0;
            Check(mnavBakeTile(baker, &mesh, 1, x, z, NULL), "bake");
            Check(mnavCopyBakedTile(baker, bytes, TILE_ROOM, &size), "copy");
            Check(mnavStageTile(navmesh, bytes, size).result, "stage");
        }
    }
    Check(mnavCommit(navmesh), "commit");
    free(bytes);
    mnavDestroyBaker(baker);
    return navmesh;
}

int main(void)
{
    mnavNavmesh* navmesh = BakeAndLoad();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    mnavNearest start;
    mnavNearest end;
    Check(mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 8.0}, box, &start), "start");
    Check(mnavFindNearest(navmesh, NULL, (mnavPos3){8.0, 0.0, 56.0}, box, &end), "end");
    mnavPath path;
    Check(mnavFindPath(query, navmesh, NULL, start.polygon, start.point, end.polygon, end.point,
                       &path),
          "path");
    if (path.end != mnav_pathFound)
    {
        fprintf(stderr, "no path\n");
        return 1;
    }
    printf("path of %.1f m through %d polygons:\n", path.length, path.polygonCount);
    for (int32_t i = 0; i < path.pointCount; ++i)
    {
        printf("  (%.2f, %.2f, %.2f)\n", path.points[i].x, path.points[i].y, path.points[i].z);
    }
    // Follow it: each step, a meter toward the corridor's next corner.
    static mnavPolygonId buffer[CORRIDOR];
    mnavCorridor corridor;
    Check(mnavResetCorridor(&corridor, buffer, CORRIDOR, start.polygon, start.point), "reset");
    Check(mnavSetCorridor(&corridor, &path), "corridor");
    int32_t steps = 0;
    for (; steps < 200; ++steps)
    {
        mnavCorners corners;
        Check(mnavCorridorCorners(query, navmesh, &corridor, &corners), "corners");
        mnavPos3 at = corridor.position;
        mnavPos3 next = corners.points[corners.pointCount > 1 ? 1 : 0];
        double dx = next.x - at.x;
        double dz = next.z - at.z;
        double left = sqrt(dx * dx + dz * dz);
        if (corners.pointCount <= 2 && left < 0.01)
        {
            break;
        }
        double t = left > 1.0 ? 1.0 / left : 1.0;
        mnavPos3 wanted = {at.x + dx * t, at.y, at.z + dz * t};
        Check(mnavMoveCorridor(query, navmesh, NULL, &corridor, wanted, NULL), "move");
    }
    double dx = corridor.position.x - end.point.x;
    double dz = corridor.position.z - end.point.z;
    bool arrived = sqrt(dx * dx + dz * dz) < 0.01;
    printf("walked %d steps to (%.2f, %.2f): %s\n", steps, corridor.position.x, corridor.position.z,
           arrived ? "arrived" : "stopped short");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return arrived ? 0 : 1;
}
