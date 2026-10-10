// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The version every def and the query filter carry (mnav-0001): one
// stamped for another minor or major version is refused with
// mnav_errorVersion and nothing is made; a wrong cookie is still invalid.

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

// The versions of headers one minor and one major past these.
static const uint32_t s_others[2] = {MNAV_ABI_VERSION + 1, MNAV_ABI_VERSION + (1u << 16)};

static void TestStamp(void)
{
    mnavVersion linked = mnavGetVersion();
    CHECK(MNAV_ABI_VERSION == (((uint32_t)linked.major << 16) | linked.minor),
          "the headers' version is the library's");
    CHECK(mnavDefaultBakeDef().version == MNAV_ABI_VERSION &&
              mnavDefaultFlightDef().version == MNAV_ABI_VERSION &&
              mnavDefaultQueryDef().version == MNAV_ABI_VERSION &&
              mnavDefaultQueryFilter().version == MNAV_ABI_VERSION &&
              mnavDefaultSteerDef().version == MNAV_ABI_VERSION &&
              mnavDefaultAvoidanceDef().version == MNAV_ABI_VERSION &&
              mnavDefaultFlowFieldDef().version == MNAV_ABI_VERSION &&
              mnavDefaultNavFlowDef().version == MNAV_ABI_VERSION &&
              mnavDefaultHierarchyDef().version == MNAV_ABI_VERSION &&
              mnavDefaultTileCacheDef().version == MNAV_ABI_VERSION &&
              mnavDefaultLinkGenDef().version == MNAV_ABI_VERSION,
          "every default carries the headers' version");
}

static void TestBake(uint32_t other)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.version = other;
    mnavBakeDefResult checked = mnavValidateBakeDef(&def, nullptr);
    CHECK(checked.result == mnav_errorVersion && checked.setting == mnav_settingNone,
          "a bake def checked");
    mnavBaker* baker = nullptr;
    mnavNavmesh* navmesh = nullptr;
    mnavTileIndex* index = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_errorVersion && baker == nullptr, "a baker");
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_errorVersion && navmesh == nullptr,
          "a navmesh");
    mnavBakeReport report;
    CHECK(mnavCreateTileIndex(&def, nullptr, 0, &index, &report) == mnav_errorVersion &&
              report.result == mnav_errorVersion && index == nullptr,
          "a tile index");
    CHECK(mnavCreateTileIndex2D(&def, nullptr, 0, &index, nullptr) == mnav_errorVersion &&
              index == nullptr,
          "a 2D tile index");
    mnavVec3 v[3] = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    int32_t indices[3] = {0, 1, 2};
    mnavTriangleMesh mesh = {v, 3, indices, 1, nullptr};
    CHECK(mnavValidateTriangleMesh(&def, &mesh).result == mnav_errorVersion, "a mesh checked");
    const mnavVec2 square[4] = {{1, 1}, {31, 1}, {31, 31}, {1, 31}};
    const mnavOutline outline = {square, 4, mnav_areaWalkable};
    CHECK(mnavValidateOutline(&def, &outline).result == mnav_errorVersion, "an outline checked");
}

static void TestFlight(uint32_t other)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.version = other;
    mnavFlightDefResult checked = mnavValidateFlightDef(&def);
    CHECK(checked.result == mnav_errorVersion && checked.setting == mnav_flightSettingNone,
          "a flight def checked");
    mnavFlightBaker* baker = nullptr;
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_errorVersion && baker == nullptr,
          "a flight baker");
    CHECK(mnavCreateFlightVolume(&def, &volume).result == mnav_errorVersion && volume == nullptr,
          "a flight volume");
}

static void TestObjects(uint32_t other)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavAvoidanceDef avoidanceDef = mnavDefaultAvoidanceDef();
    mnavFlowFieldDef flowDef = mnavDefaultFlowFieldDef();
    mnavNavFlowDef navFlowDef = mnavDefaultNavFlowDef();
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    queryDef.version = other;
    avoidanceDef.version = other;
    flowDef.version = other;
    navFlowDef.version = other;
    hierarchyDef.version = other;
    cacheDef.version = other;
    mnavQuery* query = nullptr;
    mnavAvoidance* avoidance = nullptr;
    mnavFlowField* flow = nullptr;
    mnavNavFlow* navFlow = nullptr;
    mnavHierarchy* hierarchy = nullptr;
    mnavTileCache* cache = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_errorVersion && query == nullptr, "a query");
    CHECK(mnavCreateAvoidance(&avoidanceDef, &avoidance) == mnav_errorVersion &&
              avoidance == nullptr,
          "an avoidance set");
    CHECK(mnavCreateFlowField(&flowDef, &flow) == mnav_errorVersion && flow == nullptr,
          "a flow field");
    CHECK(mnavCreateNavFlow(&navFlowDef, &navFlow) == mnav_errorVersion && navFlow == nullptr,
          "a nav flow");
    CHECK(mnavCreateHierarchy(&hierarchyDef, &hierarchy) == mnav_errorVersion &&
              hierarchy == nullptr,
          "a hierarchy");
    CHECK(mnavCreateTileCache(&cacheDef, &cache) == mnav_errorVersion && cache == nullptr,
          "a tile cache");
}

// The defs and the filter read without a creation: through a query, a
// navmesh, or mnavSteer alone.
static void TestCalls(uint32_t other)
{
    mnavBakeDef bakeDef = mnavDefaultBakeDef();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavNavmesh* navmesh = nullptr;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateNavmesh(&bakeDef, &navmesh).result == mnav_success &&
              mnavCreateQuery(&queryDef, &query) == mnav_success,
          "an empty navmesh and a query");
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.version = other;
    mnavNearest nearest;
    CHECK(mnavFindNearest(navmesh, &filter, (mnavPos3){1, 0, 1}, (mnavVec3){1, 1, 1}, &nearest) ==
              mnav_errorVersion,
          "a filter");
    mnavLinkGenDef linkDef = mnavDefaultLinkGenDef();
    linkDef.version = other;
    int32_t count = -1;
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &linkDef, 0, 0, 0, 0, nullptr, 0, &count) ==
              mnav_errorVersion,
          "a link generation def");
    linkDef = mnavDefaultLinkGenDef();
    CHECK(mnavGenerateLinks(query, navmesh, &filter, &linkDef, 0, 0, 0, 0, nullptr, 0, &count) ==
              mnav_errorVersion,
          "a filter for link generation");
    mnavSteerDef steerDef = mnavDefaultSteerDef();
    steerDef.version = other;
    const mnavPos3 point = {0, 0, 0};
    const mnavCorners corners = {&point, 1, nullptr, 0};
    mnavSteering steering;
    CHECK(mnavSteer(&corners, &steerDef, &steering) == mnav_errorVersion, "a steer def");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A wrong cookie is refused as invalid whatever the version says.
static void TestCookie(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.cookie ^= 1u;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorInvalid, "a wrong cookie, the right version");
    def.version = s_others[0];
    CHECK(mnavCreateQuery(&def, &query) == mnav_errorInvalid, "a wrong cookie, another version");
    mnavBakeDef bake = mnavDefaultBakeDef();
    bake.cookie ^= 1u;
    bake.version = s_others[1];
    mnavBakeDefResult checked = mnavValidateBakeDef(&bake, nullptr);
    CHECK(checked.result == mnav_errorInvalid && checked.setting == mnav_settingCookie,
          "a bake def with a wrong cookie");
}

int main(void)
{
    TestStamp();
    for (int32_t i = 0; i < 2; ++i)
    {
        TestBake(s_others[i]);
        TestFlight(s_others[i]);
        TestObjects(s_others[i]);
        TestCalls(s_others[i]);
    }
    TestCookie();
    return s_failures == 0 ? 0 : 1;
}
