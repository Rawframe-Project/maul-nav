// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A second thread changing the navmesh while a link generation's
// clearance test runs over it: staging a tile and its removal succeed
// with no misuse, since they change nothing the generation reads, and a
// commit is refused with mnav_errorBusy and counted as misuse. The links
// are those of a generation with no second thread.

#include "stored_tiles.h"
#include "test_harness.h"

#include "maul-nav/base.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    LINKS = 512
};

static mnavLinkDef s_links[LINKS];
static mnavLinkDef s_again[LINKS];

static mnavNavmesh* s_navmesh;
// Set by the clearance test to let the second thread run, and by the
// second thread when it is done.
static atomic_bool s_go;
static atomic_bool s_done;
static bool s_waited;
static int32_t s_tests;
// What the second thread's calls returned.
static mnavResult s_staged;
static mnavResult s_removed;
static mnavResult s_committed;

static void* Changer(void* unused)
{
    (void)unused;
    while (!atomic_load(&s_go))
    {
        sched_yield();
    }
    s_staged = mnavStageTile(s_navmesh, s_trench0, sizeof(s_trench0)).result;
    s_removed = mnavStageTileRemoval(s_navmesh, 1, 0);
    s_committed = mnavCommit(s_navmesh);
    atomic_store(&s_done, true);
    return nullptr;
}

// Passes every other pair; at the first test, while it runs, lets the
// second thread make its calls and waits for them.
static bool Clearance(void* context, mnavPos3 from, mnavPos3 to)
{
    (void)context;
    (void)from;
    (void)to;
    if (!s_waited)
    {
        s_waited = true;
        atomic_store(&s_go, true);
        while (!atomic_load(&s_done))
        {
            sched_yield();
        }
    }
    return (s_tests++ & 1) == 0;
}

// The trench (test/trench.h) at the dynamic tier.
static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavStageTile(navmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the trench");
    return navmesh;
}

static int32_t Generate(mnavQuery* query, mnavLinkDef* links)
{
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    def.jumpMax = 3.0f;
    def.clear = Clearance;
    int32_t count = 0;
    s_tests = 0;
    CHECK(mnavGenerateLinks(query, s_navmesh, nullptr, &def, &(mnavTileRange){0, 0, 1, 0}, links,
                            LINKS, &count) == mnav_success &&
              count > 0,
          "links with a clearance test");
    return count;
}

static void TestStagingBeside(void)
{
    s_navmesh = Load();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "a context");
    // With no second thread first, for the links to compare with.
    s_waited = true;
    int32_t expected = Generate(query, s_again);
    s_waited = false;
    atomic_store(&s_go, false);
    atomic_store(&s_done, false);
    pthread_t thread;
    CHECK(pthread_create(&thread, nullptr, Changer, nullptr) == 0, "a second thread");
    int32_t count = Generate(query, s_links);
    pthread_join(thread, nullptr);
    CHECK(count == expected && memcmp(s_links, s_again, (size_t)count * sizeof(s_links[0])) == 0,
          "the same links");
    CHECK(s_staged == mnav_success && s_removed == mnav_success, "staging allowed beside");
    CHECK(s_committed == mnav_errorBusy && mnavGetNavmeshMisuse(s_navmesh) == 1,
          "a commit refused, one misuse");
    CHECK(mnavGetQueryMisuse(query) == 0, "no misuse on the context");
    // What the second thread staged waits for the next commit.
    mnavTileId tile;
    CHECK(mnavCommit(s_navmesh) == mnav_success &&
              mnavGetTile(s_navmesh, 0, 0, &tile) == mnav_success &&
              mnavGetTile(s_navmesh, 1, 0, &tile) == mnav_errorNotLoaded &&
              mnavGetNavmeshMisuse(s_navmesh) == 1,
          "committed after");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(s_navmesh);
}

int main(void)
{
    TestStagingBeside();
    return s_failures == 0 ? 0 : 1;
}
