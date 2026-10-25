// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The navmesh's tables, which no public call shows: the links between
// neighboring tiles, by polygon, edge, side, the part of the side they
// cover and their target; tile slots at their last generations; and
// mnavPolygonOf's refusal of a stale id. White-box: the API joins
// polygons across a tile side (mnavCheckCorridor) but names no link's
// span, reaches a slot's last generation only after 2^32 commits to it,
// and checks every id before it looks a polygon up.

#include "navmesh.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>

// The hash of the 2 by 2 world's links, the same on every platform.
#define WORLD_LINKS_HASH 0xb252e67c3385bcbbull

static mnavNavmesh* Make(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Tests here replace loaded tiles.
    def.tier = mnav_tierDynamic;
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

static void TestLinksFollowTheirNeighbors(void)
{
    mnavNavmesh* navmesh = Make();
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              Links(navmesh, 0, 0) == 0,
          "no neighbor, no links");
    CHECK(StageAt(navmesh, 1) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              Links(navmesh, 0, 0) > 0,
          "a neighbor, links");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              Symmetric(navmesh),
          "replaced, the neighbor's links follow");
    CHECK(mnavStageTileRemoval(navmesh, 1, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && Links(navmesh, 0, 0) == 0,
          "removed, its neighbor's links went with it");
    mnavDestroyNavmesh(navmesh);
}

static void TestFailedCommitLinksNothing(void)
{
    // Past the links limit, the commit adds tile (1, 0) to nothing, and
    // tile (0, 0) keeps its links, none.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.tileLinks = 1;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "one tile");
    CHECK(StageAt(navmesh, 1) == mnav_success && mnavCommit(navmesh) == mnav_errorLimit,
          "past the links limit");
    CHECK(Links(navmesh, 0, 0) == 0, "nothing linked");
    mnavDestroyNavmesh(navmesh);
}

static void TestStaleIdsFindNoPolygon(void)
{
    mnavNavmesh* navmesh = Make();
    mnavPolygonId old = {1, 1, 0};
    const mnavTile* tile = nullptr;
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              mnavPolygonOf(navmesh, old, &tile) != nullptr && tile != nullptr,
          "a current id");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              mnavPolygonOf(navmesh, old, &tile) == nullptr && tile == nullptr,
          "replaced, the old id finds nothing");
    mnavDestroyNavmesh(navmesh);
}

static void TestSlotAboutToWrapIsRetired(void)
{
    mnavNavmesh* navmesh = Make();
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "one");
    navmesh->slots[0].generation = UINT32_MAX;
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "again");
    mnavTileId id;
    CHECK(mnavGetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 2 && id.generation == 1,
          "a new slot");
    CHECK(navmesh->slots[0].retired && navmesh->slots[0].tile == nullptr, "the old one retired");
    CHECK(mnavStageTileRemoval(navmesh, 0, 0) == mnav_success &&
              StageAt(navmesh, 1) == mnav_success && mnavCommit(navmesh) == mnav_success,
          "slot 2 freed, another tile");
    // Slot 2 freed by the removal goes to the new tile two generations on,
    // so the removed tile's ids stay stale; slot 1 stays retired.
    CHECK(mnavGetTile(navmesh, 1, 0, &id) == mnav_success && id.slot == 2 && id.generation == 3,
          "the lowest free slot, never the retired one");
    mnavDestroyNavmesh(navmesh);
}

static void TestSlotFreedAtTheLastGenerationIsRetired(void)
{
    mnavNavmesh* navmesh = Make();
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success, "one");
    navmesh->slots[0].generation = UINT32_MAX - 2;
    mnavTileId id;
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              mnavGetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 1 &&
              id.generation == UINT32_MAX - 1,
          "replaced in its slot");
    CHECK(mnavStageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "removed");
    // Freed at the last generation, the slot has no next one to be taken
    // at: it is retired, not reused at a generation that wrapped to 0.
    CHECK(navmesh->slots[0].retired && navmesh->slots[0].generation == UINT32_MAX,
          "the slot retired");
    CHECK(StageAt(navmesh, 0) == mnav_success && mnavCommit(navmesh) == mnav_success &&
              mnavGetTile(navmesh, 0, 0, &id) == mnav_success && id.slot == 2 && id.generation == 1,
          "the tile again, in a new slot");
    CHECK(mnavCheckPolygon(navmesh, (mnavPolygonId){id.slot, id.generation, 0}) == mnav_success,
          "its ids current");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    LoadWorldTiles();
    TestNeighborsLinkBothWays();
    TestLinksFollowTheirNeighbors();
    TestFailedCommitLinksNothing();
    TestStaleIdsFindNoPolygon();
    TestSlotAboutToWrapIsRetired();
    TestSlotFreedAtTheLastGenerationIsRetired();
    return s_failures == 0 ? 0 : 1;
}
