// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ABI stamp. Every creation that takes no other library object, and
// mnavSteer, checks its def's cookie and then its version: a def stamped
// for another version is refused with mnav_errorVersion and nothing is
// made, and a def with a wrong cookie is invalid whatever its version
// says, so the cookie is read first. The query filter and the link
// generation def carry only a cookie.

#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// Stamps of other headers: the next patch, the next minor, a zeroed def,
// and the stamp's form without its top bit.
static const uint32_t s_others[4] = {
    MNAV_ABI_VERSION + 1u,
    MNAV_ABI_VERSION + (1u << 8),
    0u,
    MNAV_ABI_VERSION & 0x7FFFFFFFu,
};

static void TestStamp(void)
{
    mnavVersion linked = mnavGetVersion();
    uint32_t expected =
        0x80000000u | ((uint32_t)linked.major << 16) |
        (linked.major == 0 ? ((uint32_t)linked.minor << 8) | linked.patch : (uint32_t)linked.minor);
    CHECK(MNAV_ABI_VERSION == expected, "the headers' stamp is the linked library's version");
    CHECK((MNAV_ABI_VERSION & 0x80000000u) != 0, "the stamp's top bit is set");
    CHECK(mnavDefaultBakeDef().version == MNAV_ABI_VERSION &&
              mnavDefaultFlightDef().version == MNAV_ABI_VERSION &&
              mnavDefaultQueryDef().version == MNAV_ABI_VERSION &&
              mnavDefaultSteerDef().version == MNAV_ABI_VERSION &&
              mnavDefaultAvoidanceDef().version == MNAV_ABI_VERSION &&
              mnavDefaultFlowFieldDef().version == MNAV_ABI_VERSION &&
              mnavDefaultNavFlowDef().version == MNAV_ABI_VERSION &&
              mnavDefaultHierarchyDef().version == MNAV_ABI_VERSION &&
              mnavDefaultTileCacheDef().version == MNAV_ABI_VERSION,
          "every root default carries the headers' stamp");
}

// The results a creation gives a def of another version, with the right
// cookie and with a wrong one.
typedef struct Refusals
{
    mnavResult version;
    mnavResult cookie;
} Refusals;

static bool Refused(Refusals r)
{
    return r.version == mnav_errorVersion && r.cookie == mnav_errorInvalid;
}

static void TestBake(uint32_t other)
{
    mnavBakeDef stale = mnavDefaultBakeDef();
    stale.version = other;
    mnavBakeDef wrong = stale;
    wrong.cookie ^= 1u;

    mnavBakeDefResult checked = mnavValidateBakeDef(&stale, nullptr);
    mnavBakeDefResult cookie = mnavValidateBakeDef(&wrong, nullptr);
    CHECK(checked.result == mnav_errorVersion && checked.setting == mnav_settingNone &&
              cookie.result == mnav_errorInvalid && cookie.setting == mnav_settingCookie,
          "a bake def checked");

    mnavBaker* baker = nullptr;
    Refusals r = {mnavCreateBaker(&stale, &baker).result, mnavCreateBaker(&wrong, &baker).result};
    CHECK(Refused(r) && baker == nullptr, "a baker");

    mnavNavmesh* navmesh = nullptr;
    r = (Refusals){mnavCreateNavmesh(&stale, &navmesh).result,
                   mnavCreateNavmesh(&wrong, &navmesh).result};
    CHECK(Refused(r) && navmesh == nullptr, "a navmesh");

    mnavTileIndex* index = nullptr;
    r = (Refusals){mnavCreateTileIndex(&stale, nullptr, 0, &index, nullptr),
                   mnavCreateTileIndex(&wrong, nullptr, 0, &index, nullptr)};
    CHECK(Refused(r) && index == nullptr, "a tile index");
    r = (Refusals){mnavCreateTileIndex2D(&stale, nullptr, 0, &index, nullptr),
                   mnavCreateTileIndex2D(&wrong, nullptr, 0, &index, nullptr)};
    CHECK(Refused(r) && index == nullptr, "a 2D tile index");

    mnavVec3 v[3] = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    int32_t indices[3] = {0, 1, 2};
    mnavTriangleMesh mesh = {v, 3, indices, 1, nullptr};
    r = (Refusals){mnavValidateTriangleMesh(&stale, &mesh).result,
                   mnavValidateTriangleMesh(&wrong, &mesh).result};
    CHECK(Refused(r), "a mesh checked");
    const mnavVec2 square[4] = {{1, 1}, {31, 1}, {31, 31}, {1, 31}};
    const mnavOutline outline = {square, 4, mnav_areaWalkable};
    r = (Refusals){mnavValidateOutline(&stale, &outline).result,
                   mnavValidateOutline(&wrong, &outline).result};
    CHECK(Refused(r), "an outline checked");

    mnavTileCacheDef cacheStale = mnavDefaultTileCacheDef();
    cacheStale.version = other;
    mnavTileCacheDef cacheWrong = cacheStale;
    cacheWrong.cookie ^= 1u;
    mnavTileCache* cache = nullptr;
    r = (Refusals){mnavCreateTileCache(&cacheStale, &cache),
                   mnavCreateTileCache(&cacheWrong, &cache)};
    CHECK(Refused(r) && cache == nullptr, "a tile cache");
}

static void TestFlight(uint32_t other)
{
    mnavFlightDef stale = mnavDefaultFlightDef();
    stale.version = other;
    mnavFlightDef wrong = stale;
    wrong.cookie ^= 1u;
    mnavFlightDefResult checked = mnavValidateFlightDef(&stale);
    mnavFlightDefResult cookie = mnavValidateFlightDef(&wrong);
    CHECK(checked.result == mnav_errorVersion && checked.setting == mnav_flightSettingNone &&
              cookie.result == mnav_errorInvalid && cookie.setting == mnav_flightSettingCookie,
          "a flight def checked");
    mnavFlightBaker* baker = nullptr;
    Refusals r = {mnavCreateFlightBaker(&stale, &baker).result,
                  mnavCreateFlightBaker(&wrong, &baker).result};
    CHECK(Refused(r) && baker == nullptr, "a flight baker");
    mnavFlightVolume* volume = nullptr;
    r = (Refusals){mnavCreateFlightVolume(&stale, &volume).result,
                   mnavCreateFlightVolume(&wrong, &volume).result};
    CHECK(Refused(r) && volume == nullptr, "a flight volume");
}

static void TestQuery(uint32_t other)
{
    mnavQueryDef stale = mnavDefaultQueryDef();
    stale.version = other;
    mnavQueryDef wrong = stale;
    wrong.cookie ^= 1u;
    mnavQuery* query = nullptr;
    Refusals r = {mnavCreateQuery(&stale, &query), mnavCreateQuery(&wrong, &query)};
    CHECK(Refused(r) && query == nullptr, "a query context");

    mnavSteerDef steerStale = mnavDefaultSteerDef();
    steerStale.version = other;
    mnavSteerDef steerWrong = steerStale;
    steerWrong.cookie ^= 1u;
    const mnavPos3 point = {0, 0, 0};
    const mnavCorners corners = {&point, 1, nullptr, 0};
    mnavSteering steering;
    r = (Refusals){mnavSteer(&corners, &steerStale, &steering),
                   mnavSteer(&corners, &steerWrong, &steering)};
    CHECK(Refused(r), "a steering def");

    mnavHierarchyDef hierarchyStale = mnavDefaultHierarchyDef();
    hierarchyStale.version = other;
    mnavHierarchyDef hierarchyWrong = hierarchyStale;
    hierarchyWrong.cookie ^= 1u;
    mnavHierarchy* hierarchy = nullptr;
    r = (Refusals){mnavCreateHierarchy(&hierarchyStale, &hierarchy),
                   mnavCreateHierarchy(&hierarchyWrong, &hierarchy)};
    CHECK(Refused(r) && hierarchy == nullptr, "a hierarchy");

    mnavNavFlowDef navFlowStale = mnavDefaultNavFlowDef();
    navFlowStale.version = other;
    mnavNavFlowDef navFlowWrong = navFlowStale;
    navFlowWrong.cookie ^= 1u;
    mnavNavFlow* navFlow = nullptr;
    r = (Refusals){mnavCreateNavFlow(&navFlowStale, &navFlow),
                   mnavCreateNavFlow(&navFlowWrong, &navFlow)};
    CHECK(Refused(r) && navFlow == nullptr, "a navmesh flow field");
}

static void TestFieldsAndAvoidance(uint32_t other)
{
    mnavFlowFieldDef flowStale = mnavDefaultFlowFieldDef();
    flowStale.version = other;
    mnavFlowFieldDef flowWrong = flowStale;
    flowWrong.cookie ^= 1u;
    mnavFlowField* flow = nullptr;
    Refusals r = {mnavCreateFlowField(&flowStale, &flow), mnavCreateFlowField(&flowWrong, &flow)};
    CHECK(Refused(r) && flow == nullptr, "a flow field");

    mnavAvoidanceDef avoidanceStale = mnavDefaultAvoidanceDef();
    avoidanceStale.version = other;
    mnavAvoidanceDef avoidanceWrong = avoidanceStale;
    avoidanceWrong.cookie ^= 1u;
    mnavAvoidance* avoidance = nullptr;
    r = (Refusals){mnavCreateAvoidance(&avoidanceStale, &avoidance),
                   mnavCreateAvoidance(&avoidanceWrong, &avoidance)};
    CHECK(Refused(r) && avoidance == nullptr, "an avoidance set");
}

// The filter and the link generation def carry only a cookie: a wrong one
// is invalid, and the right one is read with the objects' versions.
static void TestCookieOnly(void)
{
    mnavBakeDef bakeDef = mnavDefaultBakeDef();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavNavmesh* navmesh = nullptr;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateNavmesh(&bakeDef, &navmesh).result == mnav_success &&
              mnavCreateQuery(&queryDef, &query) == mnav_success,
          "an empty navmesh and a query");
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    mnavNearest nearest;
    const mnavPos3 at = {1, 0, 1};
    const mnavVec3 box = {1, 1, 1};
    CHECK(mnavFindNearest(navmesh, &filter, at, box, &nearest) == mnav_success,
          "a default filter is read");
    filter.cookie ^= 1u;
    CHECK(mnavFindNearest(navmesh, &filter, at, box, &nearest) == mnav_errorInvalid,
          "a filter with a wrong cookie");
    mnavLinkGenDef linkDef = mnavDefaultLinkGenDef();
    int32_t count = -1;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &linkDef, 0, 0, 0, 0, nullptr, 0, &count) ==
                  mnav_success &&
              count == 0,
          "a default link generation def is read");
    linkDef.cookie ^= 1u;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &linkDef, 0, 0, 0, 0, nullptr, 0, &count) ==
              mnav_errorInvalid,
          "a link generation def with a wrong cookie");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestStamp();
    for (int32_t i = 0; i < 4; ++i)
    {
        TestBake(s_others[i]);
        TestFlight(s_others[i]);
        TestQuery(s_others[i]);
        TestFieldsAndAvoidance(s_others[i]);
    }
    TestCookieOnly();
    return s_failures == 0 ? 0 : 1;
}
