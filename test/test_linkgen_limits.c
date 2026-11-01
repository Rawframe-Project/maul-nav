// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Where link generation stops: at a link past its total, and at a
// landing's search box meeting more polygons than it may, each with
// mnav_errorLimit and the links found so far; and a call measuring with a
// short buffer, whose count past the buffer is room rather than links,
// which stops at the total with mnav_errorCapacity instead.
//
// White-box: the limits are MNAV_MAX_LINKS and MNAV_MAX_LINK_LANDINGS,
// which no scene of a test's size reaches, so the generation runs here
// with small ones.

#include "defs_for.h"
#include "linkgen.h"
#include "stored_tiles.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

enum
{
    LINKS = 512
};

static mnavLinkDef s_links[LINKS];

// The trench of test_linkgen, from its stored tiles.
static mnavNavmesh* Load(void)
{
    const uint8_t* const tiles[2] = {s_trench0, s_trench1};
    const size_t sizes[2] = {sizeof(s_trench0), sizeof(s_trench1)};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 2; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, tiles[t], sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static void TestLimits(void)
{
    mnavNavmesh* navmesh = Load();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    def.jumpMax = 3.0f;
    const mnavTileRange range = {0, 0, 1, 0};
    int32_t all = 0;
    CHECK(mnavQuery_GenerateLinks(query, navmesh, &def, &(mnavTileRange){0, 0, 1, 0}, nullptr,
                                  s_links, LINKS, &all) == mnav_success &&
              all > 2,
          "more than two links in all");
    int32_t count = 0;
    CHECK(mnavGenerateLinksWithin(query, navmesh, nullptr, &def, range, s_links, LINKS, &count,
                                  (mnavLinkGenLimits){all, MNAV_MAX_LINK_LANDINGS}) ==
                  mnav_success &&
              count == all,
          "a total of exactly the links found is enough");
    CHECK(mnavGenerateLinksWithin(query, navmesh, nullptr, &def, range, s_links, LINKS, &count,
                                  (mnavLinkGenLimits){2, MNAV_MAX_LINK_LANDINGS}) ==
                  mnav_errorLimit &&
              count == 2,
          "a third link stops generation, two counted");
    CHECK(mnavGenerateLinksWithin(query, navmesh, nullptr, &def, range, s_links, LINKS, &count,
                                  (mnavLinkGenLimits){MNAV_MAX_LINKS, 0}) == mnav_errorLimit &&
              count == 0,
          "a landing box meeting a polygon past no room stops generation");
    // A total of exactly the links found: a short buffer counts near
    // duplicates past it and reaches the total, which is room enough.
    const int32_t shortCapacities[2] = {0, 3};
    for (int32_t k = 0; k < 2; ++k)
    {
        count = -1;
        CHECK(mnavGenerateLinksWithin(
                  query, navmesh, nullptr, &def, range, shortCapacities[k] > 0 ? s_links : nullptr,
                  shortCapacities[k], &count,
                  (mnavLinkGenLimits){all, MNAV_MAX_LINK_LANDINGS}) == mnav_errorCapacity &&
                  count == all,
              "a measuring call stops at the total, room for the call that holds them");
    }
    // Past a buffer of one, the total of two is reached: room, not a
    // refusal; a buffer of two then holds two links and meets a third.
    CHECK(mnavGenerateLinksWithin(query, navmesh, nullptr, &def, range, s_links, 1, &count,
                                  (mnavLinkGenLimits){2, MNAV_MAX_LINK_LANDINGS}) ==
                  mnav_errorCapacity &&
              count == 2 &&
              mnavGenerateLinksWithin(query, navmesh, nullptr, &def, range, s_links, 2, &count,
                                      (mnavLinkGenLimits){2, MNAV_MAX_LINK_LANDINGS}) ==
                  mnav_errorLimit &&
              count == 2,
          "measured as two, refused at a third in a buffer of two");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestLimits();
    return s_failures == 0 ? 0 : 1;
}
