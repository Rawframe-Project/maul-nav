// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The navmesh: staged tiles, atomic commits, generation ids and links
// across tile sides (N23).

#include "allocator.h"
#include "navmesh.h"
#include "test_harness.h"
#include "tile.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the 2 by 2 world's links, the same on every platform.
#define WORLD_LINKS_HASH 0xb252e67c3385bcbbull

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
    // the tile itself, close to a kilobyte, is gone.
    CHECK(navmesh->memory.used < before + 64 && staged > before + 512, "the staged tile freed");
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

// A tile of one square polygon from cell (x0, 0) to (x1, 10) at its
// place, its four corners at the heights given, -X side first: (x0, 0),
// (x0, 10), (x1, 10), (x1, 0). Edges on the tile's sides carry them.
static size_t HandTile(uint8_t* out, int32_t place, int32_t x0, int32_t x1, const int32_t* y)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    mnavMeshVertex vertices[4] = {{(uint16_t)x0, (uint16_t)(32768 + y[0]), 0},
                                  {(uint16_t)x0, (uint16_t)(32768 + y[1]), 10},
                                  {(uint16_t)x1, (uint16_t)(32768 + y[2]), 10},
                                  {(uint16_t)x1, (uint16_t)(32768 + y[3]), 0}};
    mnavPolygon polygon = {
        {0, 1, 2, 3, 0xFFFF, 0xFFFF}, {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}, {0}, 4, 1, 0};
    polygon.sides[0] = x0 == 0 ? 1 : 0;
    polygon.sides[2] = x1 == def.tileCells ? 3 : 0;
    polygon.sides[3] = 4;
    uint8_t removable[4] = {0};
    mnavPolyMesh mesh = {vertices, removable, 4, 4, &polygon, 1, 1, def.tileCells, 0};
    mnavDetailVertex detailVertices[4];
    for (int32_t k = 0; k < 4; ++k)
    {
        detailVertices[k] =
            (mnavDetailVertex){vertices[k].x * 16, vertices[k].y, vertices[k].z * 16};
    }
    mnavDetailPart part = {0, 0, 4, 2};
    mnavDetailTriangle triangles[2] = {{{0, 1, 2}, 0}, {{0, 2, 3}, 0}};
    mnavDetailMesh detail = {&part, 1, detailVertices, 4, 4, triangles, 2, 2, 0, 0, 0};
    mnavTileInfo info = {mnavGetVersion(),
                         0,
                         place,
                         0,
                         def.tileCells,
                         def.cellSize,
                         def.cellHeight,
                         cells.agentHeight,
                         cells.agentRadius,
                         cells.agentStep,
                         def.origin};
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    uint8_t* bytes = nullptr;
    size_t size = 0;
    CHECK(mnavEncodeTile(&memory, &info, &mesh, &detail, &bytes, &size) == mnav_success, "encoded");
    memcpy(out, bytes, size);
    mnavReleaseTileBytes(&memory, bytes, size);
    return size;
}

// The links between hand tiles (0, 0) and (1, 0) facing each other, the
// left one's +X edge at heights left, the right one's -X edge at right.
static int32_t HandLinks(const int32_t* left, const int32_t* right)
{
    static uint8_t a[1024];
    static uint8_t b[1024];
    const int32_t flatLeft[4] = {0, 0, left[1], left[0]};
    const int32_t flatRight[4] = {right[0], right[1], 0, 0};
    size_t sizeA = HandTile(a, 0, 100, 128, flatLeft);
    size_t sizeB = HandTile(b, 1, 0, 20, flatRight);
    mnavNavmesh* navmesh = Make();
    CHECK(mnavStageTile(navmesh, a, sizeA).result == mnav_success &&
              mnavStageTile(navmesh, b, sizeB).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "hand tiles committed");
    int32_t links = Links(navmesh, 0, 0);
    CHECK(links == Links(navmesh, 1, 0) && Symmetric(navmesh), "both ways");
    mnavDestroyNavmesh(navmesh);
    return links;
}

static void TestLinksNeedHeightsWithinAStepAtBothEnds(void)
{
    // The agent's step is 6 cell heights. Heights are (z = 0, z = 10).
    const int32_t flat[2] = {0, 0};
    const int32_t step[2] = {6, 6};
    const int32_t past[2] = {7, 7};
    const int32_t rising[2] = {0, 7};
    CHECK(HandLinks(flat, flat) == 1, "level edges link");
    CHECK(HandLinks(flat, step) == 1, "exactly a step apart links");
    CHECK(HandLinks(flat, past) == 0, "past a step does not");
    CHECK(HandLinks(flat, rising) == 0, "close at one end only does not");
    CHECK(HandLinks(rising, rising) == 1, "the same slope links");
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

int main(void)
{
    BakeWorld();
    TestLinksNeedHeightsWithinAStepAtBothEnds();
    TestSlotAboutToWrapIsRetired();
    TestStagedTilesAppearAtCommit();
    TestNeighborsLinkBothWays();
    TestReplacingAndRemovingMakeIdsStale();
    TestFailedCommitChangesNothing();
    TestTilesMustMatchTheDef();
    TestDestroyGivesEverythingBack();
    return s_failures == 0 ? 0 : 1;
}
