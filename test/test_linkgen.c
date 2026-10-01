// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Link generation (mnav-0009): drops off a platform, jumps across a gap,
// none past a pillar one walks round, the clearance test, the buffer, and
// a path through the links staged.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the links generated, the same on every platform.
#define LINKS_HASH 0x80a7236cbc019e3dull

enum
{
    BOXES = 4,
    VERTICES = 4 + 8 * BOXES,
    TRIANGLES = 2 + 12 * BOXES,
    CAPACITY = 1 << 17,
    LINKS = 512
};

static mnavVec3 s_vertices[VERTICES];
static int32_t s_indices[TRIANGLES * 3];
static uint8_t s_tiles[2][CAPACITY];
static size_t s_sizes[2];
static mnavLinkDef s_links[LINKS];

// A trench floor 6 m down; slab A from x = -1 to 30 and slab B from 31.2
// on, a gap of 1.2 m between; a platform 1.5 m high on A, a pillar on B.
// The boxes' tops face up, so that they are walkable.
static mnavTriangleMesh World(void)
{
    const mnavVec3 floor[4] = {
        {-1.0f, -6.0f, -1.0f}, {-1.0f, -6.0f, 33.0f}, {65.0f, -6.0f, 33.0f}, {65.0f, -6.0f, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    // x0, z0, x1, z1, y0, y1.
    const float boxes[BOXES][6] = {{-1, -1, 30, 33, -1, 0},
                                   {31.2f, -1, 65, 33, -1, 0},
                                   {10, 10, 20, 22, 0, 1.5f},
                                   {45, 15, 46, 16, 0, 3}};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        mnavVec3* v = &s_vertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] = (mnavVec3){boxes[b][(k & 1) ? 2 : 0], boxes[b][(k & 4) ? 5 : 4],
                              boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 7, 5, 4, 6, 7, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, VERTICES, s_indices, TRIANGLES, nullptr};
}

static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 2; ++t)
    {
        CHECK(mnavBakeTile(baker, &world, 1, t, 0, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_tiles[t], CAPACITY, &s_sizes[t]) == mnav_success &&
                  mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "baked and staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavDestroyBaker(baker);
    return navmesh;
}

static mnavQuery* Query(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static mnavLinkGenDef Def(void)
{
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    def.jumpMax = 3.0f;
    return def;
}

static bool Refuse(void* context, mnavPos3 from, mnavPos3 to)
{
    (void)from;
    (void)to;
    *(int32_t*)context += 1;
    return false;
}

static void TestKinds(const mnavNavmesh* navmesh)
{
    mnavQuery* query = Query();
    mnavLinkGenDef def = Def();
    int32_t count = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
              mnav_success,
          "generated");
    int32_t drops = 0;
    int32_t across = 0;
    int32_t back = 0;
    int32_t wrong = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        const mnavLinkDef* l = &s_links[i];
        bool onPlatform = l->start.y > 1.2 && l->start.x > 9.0 && l->start.x < 21.0;
        if (l->kind == mnav_linkDrop && onPlatform && fabs(l->end.y) < 0.3 && !l->twoWay)
        {
            drops += 1;
        }
        else if (l->kind == mnav_linkJump && l->start.x < 30.0 && l->end.x > 31.2)
        {
            across += 1;
        }
        else if (l->kind == mnav_linkJump && l->start.x > 31.2 && l->end.x < 30.0)
        {
            back += 1;
        }
        else
        {
            wrong += 1;
            printf("unexpected link (%.2f %.2f %.2f) -> (%.2f %.2f %.2f) kind %d\n", l->start.x,
                   l->start.y, l->start.z, l->end.x, l->end.y, l->end.z, l->kind);
        }
    }
    printf("links %d: drops %d, jumps across %d and back %d\n", count, drops, across, back);
    CHECK(drops > 20 && across > 10 && back > 10 && wrong == 0,
          "drops off the platform and jumps both ways across the gap, nothing else");
    uint64_t hash =
        mnavHash64(MNAV_HASH_INIT, s_links, (int32_t)((size_t)count * sizeof(mnavLinkDef)));
    printf("LINKS_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == LINKS_HASH, "the pinned hash");
    // Without the walking rule, links past the pillar appear.
    def.detour = 0.0f;
    int32_t loose = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &loose) ==
                  mnav_success &&
              loose > count,
          "the walking rule drops links");
    mnavDestroyQuery(query);
}

static void TestChecksAndBuffer(const mnavNavmesh* navmesh)
{
    mnavQuery* query = Query();
    mnavLinkGenDef def = Def();
    int32_t all = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &all) ==
              mnav_success,
          "generated");
    int32_t count = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, 3, &count) ==
                  mnav_errorCapacity &&
              count >= all,
          "a short buffer, counted");
    // Tile 1 holds the pillar and none of the edges that make links.
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 1, 0, 1, 0, s_links, LINKS, &count) ==
                  mnav_success &&
              count == 0,
          "only the tiles asked for");
    int32_t asked = 0;
    def.clear = Refuse;
    def.context = &asked;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
                  mnav_success &&
              count == 0 && asked > 0,
          "the host refuses every way");
    def = Def();
    def.dropMax = 0.0f;
    def.jumpMax = 0.0f;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
                  mnav_success &&
              count == 0,
          "both kinds off");
    def = Def();
    def.spacing = 0.0f;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
              mnav_errorRange,
          "no spacing");
    def = Def();
    def.detour = 0.5f;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
              mnav_errorRange,
          "a detour under 1");
    def = Def();
    def.cookie = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
              mnav_errorInvalid,
          "not a def");
    def = Def();
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 1, 0, 0, 0, s_links, LINKS, &count) ==
                  mnav_errorInvalid &&
              mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, nullptr, 4, &count) ==
                  mnav_errorInvalid,
          "bad arguments");
    mnavDestroyQuery(query);
}

static void TestPathThroughLinks(mnavNavmesh* navmesh)
{
    // From the platform to the far slab: down a drop, then across a jump.
    mnavQuery* query = Query();
    mnavLinkGenDef def = Def();
    int32_t count = 0;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
              mnav_success,
          "generated");
    for (int32_t i = 0; i < count; ++i)
    {
        mnavLinkId id;
        CHECK(mnavStageLink(navmesh, &s_links[i], &id) == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavNearest a;
    mnavNearest b;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){15.0, 1.5, 16.0},
                          (mnavVec3){1.0f, 1.0f, 1.0f}, &a) == mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){55.0, 0.0, 16.0},
                              (mnavVec3){1.0f, 1.0f, 1.0f}, &b) == mnav_success &&
              a.polygon.slot != 0 && b.polygon.slot != 0,
          "ends");
    mnavPath path;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.linkCount == 2,
          "a drop and a jump");
    mnavDestroyQuery(query);
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    TestKinds(navmesh);
    TestChecksAndBuffer(navmesh);
    TestPathThroughLinks(navmesh);
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
