// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bakes and queries on several threads at once, as a host's workers run
// them: the test world's 100 tiles of 8 m baked by 1, 2, 4 and 8 workers,
// each with its own baker and taking tiles as they come, give the same
// bytes as one thread; navmeshes staged in another order answer the same
// paths; and 8 threads querying one navmesh, each with its own query
// context, get what one thread gets. Run under ThreadSanitizer, the test
// also shows that bakers and queries share nothing they write.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

// The hash of the 100 tiles' bytes, in tile order, the same on every
// platform.
#define TILES_HASH 0x26e94dbc10d0097bull

enum
{
    SIDE = 10,
    TILES = SIDE * SIDE,
    ROOM = 1 << 16,
    MAX_WORKERS = 8,
    PATHS = 64
};

static uint8_t s_one[TILES][ROOM];
static size_t s_oneSizes[TILES];
static uint8_t s_many[TILES][ROOM];
static size_t s_manySizes[TILES];

static mnavBakeDef SmallDef(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    return def;
}

// The tiles still to bake, taken in turn by the workers, and whether any
// bake failed.
static atomic_int s_next;
static atomic_bool s_failed;
static mnavTriangleMesh s_world;

static bool BakeOne(mnavBaker* baker, int32_t t, uint8_t* bytes, size_t* size)
{
    return mnavBakeTile(baker, &s_world, 1, t % SIDE - 1, t / SIDE - 1, nullptr) == mnav_success &&
           mnavCopyBakedTile(baker, bytes, ROOM, size) == mnav_success;
}

static void* Worker(void* unused)
{
    (void)unused;
    mnavBakeDef def = SmallDef();
    mnavBaker* baker = nullptr;
    if (mnavCreateBaker(&def, &baker).result != mnav_success)
    {
        atomic_store(&s_failed, true);
        return nullptr;
    }
    for (int32_t t = atomic_fetch_add(&s_next, 1); t < TILES; t = atomic_fetch_add(&s_next, 1))
    {
        if (!BakeOne(baker, t, s_many[t], &s_manySizes[t]))
        {
            atomic_store(&s_failed, true);
        }
    }
    mnavDestroyBaker(baker);
    return nullptr;
}

static void TestBakesAtAnyWorkerCount(void)
{
    s_world = World();
    mnavBakeDef def = SmallDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "a baker");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < TILES; ++t)
    {
        CHECK(BakeOne(baker, t, s_one[t], &s_oneSizes[t]), "baked on one thread");
        hash = mnavHash64(hash, s_one[t], (int32_t)s_oneSizes[t]);
    }
    mnavDestroyBaker(baker);
    printf("workers: TILES_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == TILES_HASH, "the pinned hash");
    const int32_t counts[3] = {2, 4, 8};
    for (int32_t c = 0; c < 3; ++c)
    {
        memset(s_manySizes, 0, sizeof(s_manySizes));
        atomic_store(&s_next, 0);
        atomic_store(&s_failed, false);
        pthread_t threads[MAX_WORKERS];
        for (int32_t w = 0; w < counts[c]; ++w)
        {
            CHECK(pthread_create(&threads[w], nullptr, Worker, nullptr) == 0, "a worker");
        }
        for (int32_t w = 0; w < counts[c]; ++w)
        {
            pthread_join(threads[w], nullptr);
        }
        bool same = !atomic_load(&s_failed);
        for (int32_t t = 0; t < TILES; ++t)
        {
            same = same && s_manySizes[t] == s_oneSizes[t] &&
                   memcmp(s_many[t], s_one[t], s_oneSizes[t]) == 0;
        }
        CHECK(same, "every tile's bytes the same at any worker count");
    }
}

static mnavNavmesh* Load(bool backward)
{
    mnavBakeDef def = SmallDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    for (int32_t i = 0; i < TILES; ++i)
    {
        int32_t t = backward ? TILES - 1 - i : i;
        CHECK(mnavStageTile(navmesh, s_one[t], s_oneSizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

// A path's ends, from a seed, on the world's floor.
static void Ends(int32_t i, mnavPos3* a, mnavPos3* b)
{
    uint32_t state = 7u + (uint32_t)i * 2654435761u;
    double v[4];
    for (int32_t k = 0; k < 4; ++k)
    {
        state = state * 1664525u + 1013904223u;
        v[k] = 1.0 + (double)(state >> 8 & 0xFFFFu) / 65536.0 * 62.0;
    }
    *a = (mnavPos3){v[0], 0.0, v[1]};
    *b = (mnavPos3){v[2], 0.0, v[3]};
}

// The hash of path i's points, cost and end: what does not name a slot.
static uint64_t PathHash(mnavQuery* query, const mnavNavmesh* navmesh, int32_t i)
{
    mnavPos3 a;
    mnavPos3 b;
    Ends(i, &a, &b);
    mnavNearest na = {0};
    mnavNearest nb = {0};
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    if (mnavFindNearest(navmesh, nullptr, a, box, &na) != mnav_success ||
        mnavFindNearest(navmesh, nullptr, b, box, &nb) != mnav_success || na.polygon.slot == 0 ||
        nb.polygon.slot == 0)
    {
        return 1;
    }
    mnavPath path;
    if (mnavFindPath(query, navmesh, nullptr, na.polygon, na.point, nb.polygon, nb.point, &path) !=
        mnav_success)
    {
        return 2;
    }
    uint64_t hash = MNAV_HASH_INIT;
    hash = mnavHash64(hash, &path.end, (int32_t)sizeof(path.end));
    hash = mnavHash64(hash, &path.cost, (int32_t)sizeof(path.cost));
    return mnavHash64(hash, path.points, path.pointCount * (int32_t)sizeof(mnavPos3));
}

typedef struct Querier
{
    const mnavNavmesh* navmesh;
    int32_t first;
    uint64_t hashes[PATHS];
    bool failed;
} Querier;

static void* Query(void* context)
{
    Querier* q = context;
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    q->failed = mnavCreateQuery(&def, &query) != mnav_success;
    // Each thread starts elsewhere in the list, so the threads run
    // different paths at any moment.
    for (int32_t k = 0; !q->failed && k < PATHS; ++k)
    {
        int32_t i = (q->first + k) % PATHS;
        q->hashes[i] = PathHash(query, q->navmesh, i);
    }
    mnavDestroyQuery(query);
    return nullptr;
}

static void TestQueriesOnManyThreads(void)
{
    mnavNavmesh* forward = Load(false);
    mnavNavmesh* backward = Load(true);
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a query");
    uint64_t one[PATHS];
    int32_t found = 0;
    bool sameOrder = true;
    for (int32_t i = 0; i < PATHS; ++i)
    {
        one[i] = PathHash(query, forward, i);
        found += one[i] > 2 ? 1 : 0;
        sameOrder = sameOrder && PathHash(query, backward, i) == one[i];
    }
    mnavDestroyQuery(query);
    // Some ends fall inside the boxes, where nothing is walkable.
    printf("workers: %d of %d paths searched\n", found, PATHS);
    CHECK(found > PATHS * 3 / 4, "most paths searched");
    CHECK(sameOrder, "a navmesh staged in another order answers the same paths");
    static Querier queriers[MAX_WORKERS];
    pthread_t threads[MAX_WORKERS];
    for (int32_t w = 0; w < MAX_WORKERS; ++w)
    {
        queriers[w] = (Querier){forward, w * (PATHS / MAX_WORKERS), {0}, false};
        CHECK(pthread_create(&threads[w], nullptr, Query, &queriers[w]) == 0, "a querier");
    }
    bool same = true;
    for (int32_t w = 0; w < MAX_WORKERS; ++w)
    {
        pthread_join(threads[w], nullptr);
        same = same && !queriers[w].failed && memcmp(queriers[w].hashes, one, sizeof(one)) == 0;
    }
    CHECK(same, "8 threads querying one navmesh get what one thread gets");
    mnavDestroyNavmesh(backward);
    mnavDestroyNavmesh(forward);
}

int main(void)
{
    TestBakesAtAnyWorkerCount();
    TestQueriesOnManyThreads();
    return s_failures == 0 ? 0 : 1;
}
