// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Uses the installed library as a consumer would, in C17: it checks that
// the linked library is the version it was compiled against, bakes one
// flat tile, loads it and finds a path across it.

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    mnavVersion version = mnavGetVersion();
    if (version.major != MNAV_VERSION_MAJOR || version.minor != MNAV_VERSION_MINOR)
    {
        printf("FAIL: linked %u.%u, compiled against %d.%d\n", version.major, version.minor,
               MNAV_VERSION_MAJOR, MNAV_VERSION_MINOR);
        return 1;
    }
    // A floor over the first 32 m tile.
    const mnavVec3 vertices[4] = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 32.0f}, {32.0f, 0.0f, 32.0f}, {32.0f, 0.0f, 0.0f}};
    const int32_t indices[6] = {0, 1, 2, 0, 2, 3};
    const mnavTriangleMesh floor = {vertices, 4, indices, 2, NULL};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = NULL;
    mnavNavmesh* navmesh = NULL;
    mnavQuery* query = NULL;
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    size_t room = (size_t)1 << 16;
    size_t size = 0;
    unsigned char* bytes = malloc(room);
    int ok = bytes != NULL && mnavCreateBaker(&def, &baker).result == mnav_success &&
             mnavBakeTile(baker, &floor, 1, 0, 0, NULL) == mnav_success &&
             mnavCopyBakedTile(baker, bytes, room, &size) == mnav_success &&
             mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
             mnavStageTile(navmesh, bytes, size).result == mnav_success &&
             mnavCommit(navmesh) == mnav_success &&
             mnavCreateQuery(&queryDef, &query) == mnav_success;
    mnavNearest a;
    mnavNearest b;
    mnavPath path;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    ok = ok && mnavFindNearest(navmesh, NULL, (mnavPos3){2.0, 0.0, 2.0}, box, &a) == mnav_success &&
         mnavFindNearest(navmesh, NULL, (mnavPos3){30.0, 0.0, 30.0}, box, &b) == mnav_success &&
         mnavFindPath(query, navmesh, NULL, a.polygon, a.point, b.polygon, b.point, &path) ==
             mnav_success &&
         path.end == mnav_pathFound;
    printf("maul-nav %u.%u.%u: %s, %.1f m\n", version.major, version.minor, version.patch,
           ok ? "a path across the tile" : "FAIL", ok ? path.length : 0.0);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    mnavDestroyBaker(baker);
    free(bytes);
    return ok ? 0 : 1;
}
