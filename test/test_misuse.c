// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Misuse counts: an argument a correct program never passes, refused with
// mnav_errorInvalid, counts one misuse on the first owner object the call
// is given, and nothing else. A call that runs other searches inside
// counts once. A stale id, bad tile bytes, refused input geometry, a place
// outside the object and a value past a published bound count none, and
// neither does a link generation or a hierarchy build with good input.

#include "stored_tiles.h"
#include "test_harness.h"
#include "trench.h"

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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    LINKS = 512
};

static const mnavPolygonId s_never = {999, 1, 0};
static mnavLinkDef s_links[LINKS];

static mnavNavmesh* Trench2(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavNavmesh_StageTile(navmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "the trench");
    return navmesh;
}

static mnavQuery* Query(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a context");
    return query;
}

static mnavNearest Nearest(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest nearest;
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){x, 0.0, z}, (mnavVec3){2.0f, 4.0f, 2.0f},
                                  nullptr, &nearest) == mnav_success &&
              nearest.polygon.slot != 0,
          "a polygon");
    return nearest;
}

// The navmesh and the context: each refusal on the first object given,
// the navmesh when the context is NULL; a search made of others counts
// once.
static void TestNavmeshAndQuery(void)
{
    mnavNavmesh* navmesh = Trench2();
    mnavQuery* query = Query();
    const mnavNearest a = Nearest(navmesh, 5.0, 5.0);
    const mnavPos3 p = a.point;
    mnavTileId tile;
    mnavPath path;
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavPolygon_StageArea(navmesh, s_never, mnav_areaWalkable) == mnav_errorInvalid &&
              mnavNavmesh_GetMisuse(navmesh) == 2,
          "two on the navmesh");
    CHECK(mnavQuery_FindPath(query, navmesh, s_never, p, a.polygon, p, nullptr, &path) ==
                  mnav_errorInvalid &&
              mnavQuery_GetMisuse(query) == 1 && mnavNavmesh_GetMisuse(navmesh) == 2,
          "a path search counts once, on the context");
    CHECK(mnavQuery_FindPath(nullptr, navmesh, a.polygon, p, a.polygon, p, nullptr, &path) ==
                  mnav_errorInvalid &&
              mnavNavmesh_GetMisuse(navmesh) == 3,
          "with no context, on the navmesh");
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &tile) == mnav_success &&
              mnavNavmesh_GetMisuse(navmesh) == 3,
          "a good call counts none");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// Refusals a correct program meets count none.
static void TestNotMisuse(void)
{
    mnavNavmesh* navmesh = Trench2();
    const mnavNearest a = Nearest(navmesh, 5.0, 5.0);
    double height = 0.0;
    mnavNearest nearest;
    static uint8_t bad[sizeof(s_trench0)];
    memcpy(bad, s_trench0, sizeof(bad));
    bad[0] ^= 0xFF;
    mnavQueryFilter costly = mnavDefaultQueryFilter();
    costly.costs[mnav_areaWalkable] = 2.0f * MNAV_MAX_AREA_COST;
    CHECK(mnavPolygon_GetHeight(navmesh, a.polygon, 500.0, 5.0, &height) == mnav_errorOutside &&
              mnavNavmesh_FindNearest(navmesh, a.point, (mnavVec3){1.0f, 1.0f, 1.0f}, &costly,
                                      &nearest) == mnav_errorLimit &&
              mnavNavmesh_StageTile(navmesh, bad, sizeof(bad)).result != mnav_success,
          "outside, past a published bound, bad bytes");
    CHECK(mnavNavmesh_StageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavPolygon_GetHeight(navmesh, a.polygon, a.point.x, a.point.z, &height) ==
                  mnav_errorStale,
          "a stale id");
    CHECK(mnavNavmesh_GetMisuse(navmesh) == 0, "none counted");
    mnavDestroyNavmesh(navmesh);
}

// A baker counts a refused argument, but a mesh its check refuses is
// input, which the report names, and counts none.
static void TestBakers(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "a baker");
    mnavTriangleMesh mesh = Trench();
    mnavVec3 vertices[TRENCH_VERTICES];
    memcpy(vertices, mesh.vertices, sizeof(vertices));
    vertices[0].x = NAN;
    mesh.vertices = vertices;
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeReport report;
    CHECK(mnavBaker_BakeTile(baker, &input, 0, 0, &report) == mnav_errorInvalid &&
              report.input.result != mnav_success && mnavBaker_GetMisuse(baker) == 0,
          "refused geometry counts none");
    CHECK(mnavBaker_BakeTile(baker, nullptr, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavBaker_GetMisuse(baker) == 1,
          "no input counts one");
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    cacheDef.limits.tiles = 4;
    mnavTileCache* cache = nullptr;
    CHECK(mnavCreateTileCache(&cacheDef, &cache) == mnav_success &&
              mnavBaker_BakeTileCached(nullptr, cache, &input, 0, 0, nullptr) ==
                  mnav_errorInvalid &&
              mnavTileCache_GetMisuse(cache) == 1 && mnavBaker_GetMisuse(baker) == 1,
          "no baker counts on the cache");
    mnavDestroyTileCache(cache);
    mnavDestroyBaker(baker);
    mnavFlightDef flight = mnavDefaultFlightDef();
    mnavFlightBaker* flightBaker = nullptr;
    mnavFlightVolume* volume = nullptr;
    bool open = false;
    CHECK(mnavCreateFlightBaker(&flight, &flightBaker).result == mnav_success &&
              mnavFlightBaker_BakeTile(flightBaker, &input, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavFlightBaker_GetMisuse(flightBaker) == 0 &&
              mnavFlightBaker_BakeTile(flightBaker, nullptr, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavFlightBaker_GetMisuse(flightBaker) == 1,
          "a flight baker: geometry none, no input one");
    CHECK(mnavCreateFlightVolume(&flight, &volume).result == mnav_success &&
              mnavFlightVolume_IsOpen(volume, (mnavPos3){(double)NAN, 0.0, 0.0}, &open) ==
                  mnav_errorInvalid &&
              mnavFlightVolume_GetMisuse(volume) == 1,
          "a flight volume");
    mnavDestroyFlightVolume(volume);
    mnavDestroyFlightBaker(flightBaker);
}

// The fields, the hierarchy and the avoidance set.
static void TestOthers(void)
{
    mnavFlowFieldDef flowDef = mnavDefaultFlowFieldDef();
    flowDef.cells = 64;
    mnavFlowField* flow = nullptr;
    mnavAreaType areas[16];
    memset(areas, mnav_areaWalkable, sizeof(areas));
    const mnavGrid grid = {areas, 4, 4, 1.0f};
    const mnavCell goal = {0, 0};
    mnavFlow way;
    CHECK(mnavCreateFlowField(&flowDef, &flow) == mnav_success &&
              mnavFlowField_Build(flow, nullptr, &goal, 1, nullptr) == mnav_errorInvalid &&
              mnavFlowField_GetMisuse(flow) == 1,
          "a flow build counts once");
    CHECK(mnavFlowField_Build(flow, &grid, &goal, 1, nullptr) == mnav_success &&
              mnavFlowField_GetFlow(flow, (mnavCell){9, 9}, &way) == mnav_errorOutside &&
              mnavFlowField_GetMisuse(flow) == 1,
          "outside the region counts none");
    mnavDestroyFlowField(flow);
    mnavNavmesh* navmesh = Trench2();
    mnavNavFlowDef navDef = mnavDefaultNavFlowDef();
    navDef.limits.polygons = 256;
    navDef.limits.tiles = 16;
    navDef.limits.links = 16;
    mnavNavFlow* field = nullptr;
    CHECK(mnavCreateNavFlow(&navDef, &field) == mnav_success &&
              mnavNavFlow_Continue(field, navmesh, 0, nullptr) == mnav_errorInvalid &&
              mnavNavFlow_GetMisuse(field) == 1 && mnavNavmesh_GetMisuse(navmesh) == 0,
          "a nav flow, on the field");
    mnavDestroyNavFlow(field);
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.limits.tiles = 16;
    hierarchyDef.limits.transitions = 64;
    hierarchyDef.limits.edges = 256;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&hierarchyDef, &hierarchy) == mnav_success &&
              mnavHierarchy_Build(hierarchy, nullptr, navmesh, nullptr, nullptr) ==
                  mnav_errorInvalid &&
              mnavHierarchy_GetMisuse(hierarchy) == 1,
          "a hierarchy");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyNavmesh(navmesh);
    mnavAvoidanceDef avoidDef = mnavDefaultAvoidanceDef();
    avoidDef.limits.agents = 4;
    avoidDef.limits.obstacleVertices = 4;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&avoidDef, &avoidance) == mnav_success &&
              mnavAvoidance_Avoid(avoidance, nullptr, 1, nullptr, 0, 1.0, nullptr) ==
                  mnav_errorInvalid &&
              mnavAvoidance_GetMisuse(avoidance) == 1,
          "an avoidance set");
    mnavDestroyAvoidance(avoidance);
}

// Link generation and a hierarchy build search the navmesh many times,
// asking heights off polygons on the way: with good input, no misuse.
static void TestGoodInput(void)
{
    mnavNavmesh* navmesh = Trench2();
    mnavQuery* query = Query();
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    def.jumpMax = 3.0f;
    int32_t count = 0;
    CHECK(mnavQuery_GenerateLinks(query, navmesh, &def, &(mnavTileRange){0, 0, 1, 0}, nullptr,
                                  s_links, LINKS, &count) == mnav_success &&
              count > 0,
          "links");
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.limits.tiles = 16;
    hierarchyDef.limits.transitions = 64;
    hierarchyDef.limits.edges = 256;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&hierarchyDef, &hierarchy) == mnav_success &&
              mnavHierarchy_Build(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavHierarchy_Update(hierarchy, query, navmesh, nullptr) == mnav_success,
          "a hierarchy");
    CHECK(mnavNavmesh_GetMisuse(navmesh) == 0 && mnavQuery_GetMisuse(query) == 0 &&
              mnavHierarchy_GetMisuse(hierarchy) == 0,
          "no misuse");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestNavmeshAndQuery();
    TestNotMisuse();
    TestBakers();
    TestOthers();
    TestGoodInput();
    return s_failures == 0 ? 0 : 1;
}
