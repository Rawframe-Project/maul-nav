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
              mnavStageTile(navmesh, s_trench0, sizeof(s_trench0)).result == mnav_success &&
              mnavStageTile(navmesh, s_trench1, sizeof(s_trench1)).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
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
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, z}, (mnavVec3){2.0f, 4.0f, 2.0f},
                          &nearest) == mnav_success &&
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
    CHECK(mnavGetTile(navmesh, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavStageArea(navmesh, s_never, mnav_areaWalkable) == mnav_errorInvalid &&
              mnavGetNavmeshMisuse(navmesh) == 2,
          "two on the navmesh");
    CHECK(mnavFindPath(query, navmesh, nullptr, s_never, p, a.polygon, p, &path) ==
                  mnav_errorInvalid &&
              mnavGetQueryMisuse(query) == 1 && mnavGetNavmeshMisuse(navmesh) == 2,
          "a path search counts once, on the context");
    CHECK(mnavFindPath(nullptr, navmesh, nullptr, a.polygon, p, a.polygon, p, &path) ==
                  mnav_errorInvalid &&
              mnavGetNavmeshMisuse(navmesh) == 3,
          "with no context, on the navmesh");
    CHECK(mnavGetTile(navmesh, 0, 0, &tile) == mnav_success && mnavGetNavmeshMisuse(navmesh) == 3,
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
    CHECK(mnavGetHeight(navmesh, a.polygon, 500.0, 5.0, &height) == mnav_errorOutside &&
              mnavFindNearest(navmesh, &costly, a.point, (mnavVec3){1.0f, 1.0f, 1.0f}, &nearest) ==
                  mnav_errorLimit &&
              mnavStageTile(navmesh, bad, sizeof(bad)).result != mnav_success,
          "outside, past a published bound, bad bytes");
    CHECK(mnavStageTileRemoval(navmesh, 0, 0) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetHeight(navmesh, a.polygon, a.point.x, a.point.z, &height) == mnav_errorStale,
          "a stale id");
    CHECK(mnavGetNavmeshMisuse(navmesh) == 0, "none counted");
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
    CHECK(mnavBakeTile(baker, &input, 0, 0, &report) == mnav_errorInvalid &&
              report.input.result != mnav_success && mnavGetBakerMisuse(baker) == 0,
          "refused geometry counts none");
    CHECK(mnavBakeTile(baker, nullptr, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavGetBakerMisuse(baker) == 1,
          "no input counts one");
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    cacheDef.limits.tiles = 4;
    mnavTileCache* cache = nullptr;
    CHECK(mnavCreateTileCache(&cacheDef, &cache) == mnav_success &&
              mnavBakeTileCached(nullptr, cache, &input, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavGetTileCacheMisuse(cache) == 1 && mnavGetBakerMisuse(baker) == 1,
          "no baker counts on the cache");
    mnavDestroyTileCache(cache);
    mnavDestroyBaker(baker);
    mnavFlightDef flight = mnavDefaultFlightDef();
    mnavFlightBaker* flightBaker = nullptr;
    mnavFlightVolume* volume = nullptr;
    bool open = false;
    CHECK(mnavCreateFlightBaker(&flight, &flightBaker).result == mnav_success &&
              mnavBakeFlightTile(flightBaker, &input, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavGetFlightBakerMisuse(flightBaker) == 0 &&
              mnavBakeFlightTile(flightBaker, nullptr, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavGetFlightBakerMisuse(flightBaker) == 1,
          "a flight baker: geometry none, no input one");
    CHECK(mnavCreateFlightVolume(&flight, &volume).result == mnav_success &&
              mnavIsFlightOpen(volume, (mnavPos3){(double)NAN, 0.0, 0.0}, &open) ==
                  mnav_errorInvalid &&
              mnavGetFlightVolumeMisuse(volume) == 1,
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
              mnavBuildFlowField(flow, nullptr, nullptr, &goal, 1) == mnav_errorInvalid &&
              mnavGetFlowFieldMisuse(flow) == 1,
          "a flow build counts once");
    CHECK(mnavBuildFlowField(flow, &grid, nullptr, &goal, 1) == mnav_success &&
              mnavFlowAt(flow, (mnavCell){9, 9}, &way) == mnav_errorOutside &&
              mnavGetFlowFieldMisuse(flow) == 1,
          "outside the region counts none");
    mnavDestroyFlowField(flow);
    mnavNavmesh* navmesh = Trench2();
    mnavNavFlowDef navDef = mnavDefaultNavFlowDef();
    navDef.limits.polygons = 256;
    navDef.limits.tiles = 16;
    navDef.limits.links = 16;
    mnavNavFlow* field = nullptr;
    CHECK(mnavCreateNavFlow(&navDef, &field) == mnav_success &&
              mnavContinueNavFlow(field, navmesh, 0, nullptr) == mnav_errorInvalid &&
              mnavGetNavFlowMisuse(field) == 1 && mnavGetNavmeshMisuse(navmesh) == 0,
          "a nav flow, on the field");
    mnavDestroyNavFlow(field);
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.limits.tiles = 16;
    hierarchyDef.limits.transitions = 64;
    hierarchyDef.limits.edges = 256;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&hierarchyDef, &hierarchy) == mnav_success &&
              mnavBuildHierarchy(hierarchy, nullptr, navmesh, nullptr, nullptr) ==
                  mnav_errorInvalid &&
              mnavGetHierarchyMisuse(hierarchy) == 1,
          "a hierarchy");
    mnavDestroyHierarchy(hierarchy);
    mnavDestroyNavmesh(navmesh);
    mnavAvoidanceDef avoidDef = mnavDefaultAvoidanceDef();
    avoidDef.limits.agents = 4;
    avoidDef.limits.obstacleVertices = 4;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&avoidDef, &avoidance) == mnav_success &&
              mnavAvoid(avoidance, nullptr, 1, nullptr, 0, 1.0, nullptr) == mnav_errorInvalid &&
              mnavGetAvoidanceMisuse(avoidance) == 1,
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
    CHECK(mnavGenerateLinks(query, navmesh, nullptr, &def, 0, 0, 1, 0, s_links, LINKS, &count) ==
                  mnav_success &&
              count > 0,
          "links");
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.limits.tiles = 16;
    hierarchyDef.limits.transitions = 64;
    hierarchyDef.limits.edges = 256;
    mnavHierarchy* hierarchy = nullptr;
    CHECK(mnavCreateHierarchy(&hierarchyDef, &hierarchy) == mnav_success &&
              mnavBuildHierarchy(hierarchy, query, navmesh, nullptr, nullptr) == mnav_success &&
              mnavUpdateHierarchy(hierarchy, query, navmesh, nullptr) == mnav_success,
          "a hierarchy");
    CHECK(mnavGetNavmeshMisuse(navmesh) == 0 && mnavGetQueryMisuse(query) == 0 &&
              mnavGetHierarchyMisuse(hierarchy) == 0,
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
