// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The navmesh: staged tiles, atomic commits, generation ids and links
// across tile sides (N23).

#include "allocator.h"
#include "navmesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the 2 by 2 world's links, the same on every platform.
#define WORLD_LINKS_HASH 0x267cf7327d41f6a2ull

enum
{
    BOXES = 6,
    WORLD_VERTICES = 4 + 8 * BOXES,
    WORLD_TRIANGLES = 2 + 12 * BOXES,
    TILE_CAPACITY = 1 << 16
};

static mnavVec3 s_vertices[WORLD_VERTICES];
static int32_t s_indices[WORLD_TRIANGLES * 3];

// A floor over four 32 m tiles, with boxes standing across their sides.
static mnavTriangleMesh World(void)
{
    const mnavVec3 floor[4] = {
        {-1.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 65.0f}, {65.0f, 0.0f, 65.0f}, {65.0f, 0.0f, -1.0f}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, floor, sizeof(floor));
    memcpy(s_indices, quad, sizeof(quad));
    const float boxes[BOXES][4] = {{30, 10, 34, 12}, {30, 40, 33, 44}, {8, 30, 12, 34},
                                   {50, 31, 52, 35}, {20, 20, 24, 24}, {31, 31, 33, 33}};
    for (int32_t b = 0; b < BOXES; ++b)
    {
        mnavVec3* v = &s_vertices[4 + 8 * b];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] = (mnavVec3){boxes[b][(k & 1) ? 2 : 0], (k & 4) ? 2.5f : 0.0f,
                              boxes[b][(k & 2) ? 3 : 1]};
        }
        const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * b + k] = 4 + 8 * b + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, WORLD_VERTICES, s_indices, WORLD_TRIANGLES, nullptr};
}

// The world's four tiles, baked once.
static uint8_t s_tiles[4][TILE_CAPACITY];
static size_t s_sizes[4];

static void BakeWorld(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh world = World();
    for (int32_t t = 0; t < 4; ++t)
    {
        mnavBakeReport report;
        CHECK(mnavBakeTile(baker, &world, 1, t & 1, t >> 1, &report) == mnav_success, "baked");
        CHECK(mnavCopyBakedTile(baker, s_tiles[t], TILE_CAPACITY, &s_sizes[t]) == mnav_success,
              "copied");
    }
    mnavDestroyBaker(baker);
}

static mnavNavmesh* Make(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    return navmesh;
}

static mnavResult StageAt(mnavNavmesh* navmesh, int32_t t)
{
    return mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result;
}

// Every link answered by one back over the same part of the side, every
// target id current.
static bool Symmetric(const mnavNavmesh* navmesh)
{
    for (int32_t s = 0; s < navmesh->slotCount; ++s)
    {
        const mnavTile* tile = navmesh->slots[s].tile;
        for (int32_t l = 0; tile != nullptr && l < tile->linkCount; ++l)
        {
            const mnavLink* link = &tile->links[l];
            const mnavTile* other = nullptr;
            if (mnavPolygonOf(navmesh, link->target, &other) == nullptr)
            {
                return false;
            }
            bool back = false;
            int32_t q = (int32_t)link->target.polygon;
            for (int32_t m = other->firstLink[q]; m < other->firstLink[q + 1]; ++m)
            {
                const mnavLink* reply = &other->links[m];
                back = back || (reply->target.slot == (uint32_t)s + 1 &&
                                reply->target.polygon == link->polygon && reply->low == link->low &&
                                reply->high == link->high);
            }
            if (!back)
            {
                return false;
            }
        }
    }
    return true;
}

static int32_t Links(const mnavNavmesh* navmesh, int32_t x, int32_t z)
{
    int32_t slot = -1;
    const mnavTile* tile = mnavTileAt(navmesh, x, z, &slot);
    return tile != nullptr ? tile->linkCount : -1;
}

static uint64_t HashLinks(const mnavNavmesh* navmesh)
{
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t p = 0; p < navmesh->placeCount; ++p)
    {
        const mnavTile* tile = navmesh->slots[navmesh->places[p].slot].tile;
        for (int32_t l = 0; l < tile->linkCount; ++l)
        {
            const mnavLink* link = &tile->links[l];
            // The target by its tile's place: slots depend on the order of
            // commits, places do not.
            const mnavSlot* target = &navmesh->slots[link->target.slot - 1];
            int32_t words[9] = {navmesh->places[p].x,
                                navmesh->places[p].z,
                                link->polygon,
                                link->edge * 8 + link->side,
                                link->low,
                                link->high,
                                target->x,
                                target->z,
                                (int32_t)link->target.polygon};
            hash = mnavHash64(hash, words, (int32_t)sizeof(words));
        }
    }
    return hash;
}

static void TestStagedTilesAppearAtCommit(void)
{
    mnavNavmesh* navmesh = Make();
    mnavTileId id = {9, 9};
    CHECK(mnavGetTile(navmesh, 0, 0, &id) == mnav_errorNotLoaded && id.slot == 0, "empty");
    CHECK(StageAt(navmesh, 0) == mnav_success, "staged");
    CHECK(mnavGetTile(navmesh, 0, 0, &id) == mnav_errorNotLoaded, "not before the commit");
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    CHECK(mnavGetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 1 && id.generation == 1,
          "slot 1, generation 1");
    CHECK(Links(navmesh, 0, 0) == 0, "no neighbor, no links");
    CHECK(mnavCommit(navmesh) == mnav_success, "nothing staged");
    mnavDestroyNavmesh(navmesh);
}

static void TestNeighborsLinkBothWays(void)
{
    mnavNavmesh* navmesh = Make();
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(StageAt(navmesh, t) == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    CHECK(Links(navmesh, 0, 0) > 0 && Links(navmesh, 1, 1) > 0, "links on every tile");
    CHECK(Symmetric(navmesh), "every link answered");
    uint64_t hash = HashLinks(navmesh);
    printf("WORLD_LINKS_HASH=%016llx links=%d %d %d %d\n", (unsigned long long)hash,
           Links(navmesh, 0, 0), Links(navmesh, 1, 0), Links(navmesh, 0, 1), Links(navmesh, 1, 1));
    CHECK(hash == WORLD_LINKS_HASH, "the pinned hash");
    // Staged in another order, the same navmesh.
    mnavNavmesh* other = Make();
    for (int32_t t = 3; t >= 0; --t)
    {
        CHECK(StageAt(other, t) == mnav_success, "staged");
    }
    CHECK(mnavCommit(other) == mnav_success && HashLinks(other) == hash, "order does not matter");
    mnavDestroyNavmesh(other);
    // One tile at a time, the same links.
    other = Make();
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(StageAt(other, t) == mnav_success && mnavCommit(other) == mnav_success, "one");
    }
    CHECK(HashLinks(other) == hash && Symmetric(other), "one by one, the same links");
    mnavDestroyNavmesh(other);
    mnavDestroyNavmesh(navmesh);
}

static void TestReplacingAndRemovingMakeIdsStale(void)
{
    mnavNavmesh* navmesh = Make();
    CHECK(StageAt(navmesh, 0) == mnav_success && StageAt(navmesh, 1) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "two tiles");
    mnavPolygonId old = {1, 1, 0};
    const mnavTile* tile = nullptr;
    CHECK(mnavPolygonOf(navmesh, old, &tile) != nullptr, "a current id");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "replaced");
    CHECK(mnavPolygonOf(navmesh, old, &tile) == nullptr, "the old id is stale");
    mnavTileId id;
    CHECK(mnavGetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 1 && id.generation == 2,
          "the same slot, the next generation");
    CHECK(Symmetric(navmesh), "the neighbor's links follow");
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "removed");
    CHECK(mnavGetTile(navmesh, 1, 0, &id) == mnav_errorNotLoaded, "gone");
    CHECK(Links(navmesh, 0, 0) == 0, "its neighbor's links went with it");
    CHECK(StageAt(navmesh, 3) == mnav_success && mnavCommit(navmesh) == mnav_success, "another");
    CHECK(mnavGetTile(navmesh, 1, 1, &id) == mnav_success && id.slot == 2 && id.generation == 3,
          "the freed slot, reused with its next generation");
    CHECK(mnavStageTileRemoval(navmesh, 7, 7) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "removing nothing");
    mnavDestroyNavmesh(navmesh);
}

static void TestFailedCommitChangesNothing(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tileLinks = 1;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "one tile");
    uint64_t before = navmesh->memory.used;
    CHECK(StageAt(navmesh, 1) == mnav_success, "staged");
    uint64_t staged = navmesh->memory.used;
    CHECK(mnavCommit(navmesh) == mnav_errorLimit, "past the links limit");
    mnavTileId id;
    CHECK(mnavGetTile(navmesh, 1, 0, &id) == mnav_errorNotLoaded && Links(navmesh, 0, 0) == 0,
          "nothing applied");
    CHECK(navmesh->stagedCount == 1 && navmesh->memory.used == staged, "the change stays staged");
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "unstaged by a removal");
    // The commit's new slot and place arrays have room for one more entry;
    // the tile itself, kilobytes, is gone.
    CHECK(navmesh->memory.used < before + 64 && staged > before + 1024, "the staged tile freed");
    mnavDestroyNavmesh(navmesh);
    def = mnavDefaultBakeDef();
    def.limits.tiles = 1;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(StageAt(navmesh, 0) == mnav_success && StageAt(navmesh, 1) == mnav_success &&
              mnavCommit(navmesh) == mnav_errorLimit,
          "past the tiles limit");
    mnavDestroyNavmesh(navmesh);
}

static void TestTilesMustMatchTheDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.stepHeight = 0.5f;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    mnavTileResult result = mnavStageTile(navmesh, s_tiles[0], s_sizes[0]);
    CHECK(result.result == mnav_errorInvalid && result.section == mnav_tileHeader,
          "baked for another agent");
    result = mnavStageTile(navmesh, s_tiles[0], s_sizes[0] - 1);
    CHECK(result.result == mnav_errorInvalid, "damaged bytes");
    CHECK(navmesh->stagedCount == 0, "nothing staged");
    mnavDestroyNavmesh(navmesh);
    CHECK(mnavStageTile(nullptr, s_tiles[0], s_sizes[0]).result == mnav_errorInvalid &&
              mnavCommit(nullptr) == mnav_errorInvalid &&
              mnavStageTileRemoval(nullptr, 0, 0) == mnav_errorInvalid,
          "no navmesh");
    mnavDestroyNavmesh(nullptr);
}

// Counts the bytes held through the def's allocator.
static size_t s_held;

static void* Alloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    // Some C libraries refuse alignments below a pointer's.
    alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    void* block = aligned_alloc(alignment, (size + alignment - 1) / alignment * alignment);
    s_held += block != nullptr ? size : 0;
    return block;
}

static void Free(void* block, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held -= size;
    free(block);
}

static void TestDestroyGivesEverythingBack(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = (mnavAllocator){Alloc, Free, nullptr};
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(StageAt(navmesh, t) == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    CHECK(StageAt(navmesh, 2) == mnav_success, "staged, not committed");
    CHECK(s_held > 0, "held");
    mnavDestroyNavmesh(navmesh);
    CHECK(s_held == 0, "all given back");
}

int main(void)
{
    BakeWorld();
    TestStagedTilesAppearAtCommit();
    TestNeighborsLinkBothWays();
    TestReplacingAndRemovingMakeIdsStale();
    TestFailedCommitChangesNothing();
    TestTilesMustMatchTheDef();
    TestDestroyGivesEverythingBack();
    return s_failures == 0 ? 0 : 1;
}
