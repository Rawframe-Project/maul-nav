// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The nearest point on the navmesh (mnav-0005).

#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of a grid of queries' results, the same on every platform.
#define GRID_HASH 0xd0083298e4d1be5eull

static mnavNavmesh* Load(int32_t tiles)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < tiles; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavNearest Near(const mnavNavmesh* navmesh, double x, double y, double z, float extent)
{
    mnavNearest nearest;
    CHECK(mnavFindNearest(navmesh, (mnavPos3){x, y, z}, (mnavVec3){extent, extent, extent},
                          &nearest) == mnav_success,
          "queried");
    return nearest;
}

static void TestPointOnTheFloor(void)
{
    mnavNavmesh* navmesh = Load(4);
    mnavNearest n = Near(navmesh, 10.0, 0.5, 10.0, 2.0f);
    CHECK(n.polygon.slot != 0 && n.over, "over a polygon");
    CHECK(n.point.x == 10.0 && n.point.z == 10.0, "straight below");
    CHECK(n.point.y >= 0.0 && n.point.y <= 0.25, "on the floor, within a cell height");
    CHECK(!n.incomplete, "every tile in the box loaded");
    // From well above, the same polygon and point.
    mnavNearest above = Near(navmesh, 10.0, 1.8, 10.0, 2.0f);
    CHECK(above.over && above.polygon.polygon == n.polygon.polygon && above.point.y == n.point.y,
          "the floor below");
    mnavDestroyNavmesh(navmesh);
}

static void TestPointInsideAnObstacle(void)
{
    // The box from (20, 20) to (24, 24), 1.5 m tall: its inside and a
    // margin of the agent's radius round it are not walkable, and the box
    // height of 1 m keeps its top out of reach.
    mnavNavmesh* navmesh = Load(4);
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, (mnavPos3){22.0, 0.3, 22.0}, (mnavVec3){4.0f, 0.5f, 4.0f}, &n) ==
              mnav_success,
          "queried");
    CHECK(n.polygon.slot != 0 && !n.over, "beside a polygon, not over one");
    double dx = n.point.x - 22.0;
    double dz = n.point.z - 22.0;
    double d = sqrt(dx * dx + dz * dz);
    CHECK(d >= 2.0 && d <= 3.0, "at the walkable edge round the box");
    mnavDestroyNavmesh(navmesh);
}

static void TestNothingInTheBox(void)
{
    mnavNavmesh* navmesh = Load(4);
    mnavNearest n = Near(navmesh, 22.0, 0.3, 22.0, 0.5f);
    CHECK(n.polygon.slot == 0 && !n.incomplete, "nothing near, and all of it loaded");
    n = Near(navmesh, 10.0, 30.0, 10.0, 2.0f);
    CHECK(n.polygon.slot == 0, "nothing within the box's height");
    mnavDestroyNavmesh(navmesh);
    // Only the first tile loaded: a box reaching into the second is
    // incomplete, and a box wholly on it finds nothing, incompletely.
    navmesh = Load(1);
    n = Near(navmesh, 31.0, 0.5, 10.0, 2.0f);
    CHECK(n.polygon.slot != 0 && n.incomplete, "found, but part of the box is not loaded");
    n = Near(navmesh, 40.0, 0.5, 10.0, 2.0f);
    CHECK(n.polygon.slot == 0 && n.incomplete, "nothing, because nothing is loaded there");
    mnavDestroyNavmesh(navmesh);
}

static void TestBadArgumentsAreRefused(void)
{
    mnavNavmesh* navmesh = Load(1);
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, (mnavPos3){(double)NAN, 0, 0}, (mnavVec3){1, 1, 1}, &n) ==
              mnav_errorInvalid,
          "a NaN point");
    CHECK(mnavFindNearest(navmesh, (mnavPos3){0, (double)INFINITY, 0}, (mnavVec3){1, 1, 1}, &n) ==
              mnav_errorInvalid,
          "an infinite point");
    CHECK(mnavFindNearest(navmesh, (mnavPos3){0, 0, 0}, (mnavVec3){-1, 1, 1}, &n) ==
              mnav_errorInvalid,
          "a negative extent");
    CHECK(mnavFindNearest(navmesh, (mnavPos3){0, 0, 0}, (mnavVec3){1, NAN, 1}, &n) ==
              mnav_errorInvalid,
          "a NaN extent");
    CHECK(mnavFindNearest(nullptr, (mnavPos3){0, 0, 0}, (mnavVec3){1, 1, 1}, &n) ==
              mnav_errorInvalid,
          "no navmesh");
    CHECK(mnavFindNearest(navmesh, (mnavPos3){0, 0, 0}, (mnavVec3){1, 1, 1}, nullptr) ==
              mnav_errorInvalid,
          "nowhere to put it");
    CHECK(mnavFindNearest(navmesh, (mnavPos3){1.0e300, 0, 0}, (mnavVec3){1.0e30f, 1, 1}, &n) ==
              mnav_success,
          "a huge box past the extent is clamped to it");
    mnavDestroyNavmesh(navmesh);
}

static void TestGridIsPinned(void)
{
    mnavNavmesh* navmesh = Load(4);
    uint64_t hash = MNAV_HASH_INIT;
    int32_t found = 0;
    for (int32_t i = 0; i < 40; ++i)
    {
        for (int32_t j = 0; j < 40; ++j)
        {
            double x = -2.0 + i * 1.7;
            double z = -2.0 + j * 1.7;
            double y = 0.1 * ((i + j) % 9);
            mnavNearest n = Near(navmesh, x, y, z, 1.5f);
            uint64_t words[6];
            memcpy(&words[0], &n.point.x, 8);
            memcpy(&words[1], &n.point.y, 8);
            memcpy(&words[2], &n.point.z, 8);
            words[3] = n.polygon.polygon;
            words[4] = n.polygon.slot;
            words[5] = (uint64_t)n.over * 2 + (uint64_t)n.incomplete;
            hash = mnavHash64(hash, words, (int32_t)sizeof(words));
            found += n.polygon.slot != 0 ? 1 : 0;
        }
    }
    printf("GRID_HASH=%016llx found=%d of 1600\n", (unsigned long long)hash, found);
    CHECK(hash == GRID_HASH, "the pinned hash");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeWorld();
    TestPointOnTheFloor();
    TestPointInsideAnObstacle();
    TestNothingInTheBox();
    TestBadArgumentsAreRefused();
    TestGridIsPinned();
    return s_failures == 0 ? 0 : 1;
}
