// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The refusals coverage found no test making: NULL outputs and arguments
// of the public functions, ids out of range, a filter not made by its
// default, an agent not finite, a removed link's slot skipped when drawn,
// and two settings at the edge of their conversion to cells.

#include "defs_for.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/draw.h"
#include "maul-nav/flow.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

static mnavDebugVertex s_vertices2[4096];
static uint32_t s_lines[4096];

static mnavNavmesh* Load(void)
{
    LoadWorldTiles();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    meshDef.tier = mnav_tierModifiers;
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static void TestMaking(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavFlowDef navFlowDef = mnavDefaultNavFlowDef();
    mnavFlowFieldDef flowDef = mnavDefaultFlowFieldDef();
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, nullptr).result == mnav_errorInvalid &&
              mnavCreateNavFlow(&navFlowDef, nullptr) == mnav_errorInvalid &&
              mnavCreateFlowField(&flowDef, nullptr) == mnav_errorInvalid,
          "no place for the object");
    // An agent radius of 128.3 cells rounds up past the tile's 128.
    def.agent.radius = 128.3f * def.cellSize;
    CHECK(mnavValidateBakeDef(&def, nullptr).result == mnav_errorInvalid, "a radius past a tile");
    // A height so small it rounds to no cells still keeps one cell clear
    // above.
    def = mnavDefaultBakeDef();
    def.agent.height = def.cellHeight * 0.0005f;
    def.agent.stepHeight = 0.0f;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "a height of a quarter cell");
    mnavTriangleMesh world = World();
    CHECK(mnavBaker_BakeTile(baker, &(mnavBakeInput){&world, 1, nullptr, 0, nullptr, 0, nullptr}, 0,
                             0, nullptr) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    // An include volume over a tile with no ground: nothing to keep.
    def = mnavDefaultBakeDef();
    const mnavVec2 square[4] = {{300, 300}, {330, 300}, {330, 330}, {300, 330}};
    const mnavBakeVolume include = {square, 4, -1.0f, 1.0f, mnav_volumeInclude, 0};
    const mnavBakeInput input = {&world, 1, nullptr, 0, &include, 1, nullptr};
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBaker_BakeTile(baker, &input, 9, 9, &report) == mnav_success &&
              report.polygons == 0,
          "an include volume over empty ground");
    mnavDestroyBaker(baker);
}

// A mesh checked on its own: the first vertex not finite or out of range,
// the first triangle naming a vertex past the last; a 2D tile past the
// extent.
static void TestInput(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavVec3 v[3] = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    int32_t indices[3] = {0, 1, 2};
    mnavTriangleMesh mesh = {v, 3, indices, 1, nullptr};
    CHECK(mnavValidateTriangleMesh(&def, &mesh).result == mnav_success, "a good mesh");
    v[2].y = (float)NAN;
    mnavInputResult r = mnavValidateTriangleMesh(&def, &mesh);
    CHECK(r.result == mnav_errorInvalid && r.element == mnav_elementVertex && r.index == 2,
          "a vertex not finite");
    v[2].y = 1.0e9f;
    r = mnavValidateTriangleMesh(&def, &mesh);
    CHECK(r.result == mnav_errorLimit && r.element == mnav_elementVertex && r.index == 2,
          "a vertex out of range");
    v[2].y = 0.0f;
    indices[1] = 3;
    r = mnavValidateTriangleMesh(&def, &mesh);
    CHECK(r.result == mnav_errorInvalid && r.element == mnav_elementTriangle && r.index == 0,
          "a triangle past the vertices");
    const mnavVec2 square[4] = {{1, 1}, {31, 1}, {31, 31}, {1, 31}};
    const mnavOutline outline = {square, 4, mnav_areaWalkable};
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBaker_BakeTile2D(baker, &(mnavBake2DInput){&outline, 1, nullptr}, 1 << 20, 0,
                                   nullptr) == mnav_errorLimit,
          "a 2D tile past the extent");
    mnavDestroyBaker(baker);
}

static void TestNavmeshReads(mnavNavmesh* navmesh)
{
    mnavTileId tile;
    mnavAreaType area = 0;
    const mnavPolygonId none = {0, 0, 0};
    CHECK(mnavNavmesh_GetTile(nullptr, 0, 0, &tile) == mnav_errorInvalid &&
              mnavNavmesh_GetTile(navmesh, 0, 0, nullptr) == mnav_errorInvalid &&
              mnavPolygon_GetArea(navmesh, none, &area) == mnav_errorInvalid,
          "navmesh reads");
}

static void TestQueries(mnavNavmesh* navmesh)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "a query");
    mnavNearest a;
    mnavNearest b;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){4, 0, 4}, box, nullptr, &a) == mnav_success &&
              mnavNavmesh_FindNearest(navmesh, (mnavPos3){60, 0, 60}, box, nullptr, &b) ==
                  mnav_success,
          "ends");
    CHECK(mnavQuery_BeginPath(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr) ==
                  mnav_success &&
              mnavQuery_FinishPath(query, navmesh, nullptr) == mnav_errorInvalid,
          "finishing into nothing");
    CHECK(mnavQuery_CheckReachable(query, navmesh, a.polygon, a.point, b.polygon, b.point, nullptr,
                                   nullptr) == mnav_errorInvalid,
          "reachability into nothing");
    // A corridor naming a slot past the navmesh's: not current.
    mnavPolygonId stray[2] = {a.polygon, a.polygon};
    stray[1].slot = 999;
    mnavCorridor strayCorridor = {a.point, a.point, stray, 2, 2};
    int32_t valid = -1;
    CHECK(mnavNavmesh_CheckCorridor(navmesh, &strayCorridor, nullptr, &valid) == mnav_success &&
              valid == 1,
          "a corridor polygon past the slots");
    mnavCorridor empty = {a.point, a.point, nullptr, 0, 0};
    mnavCorners corners;
    CHECK(mnavQuery_FindCorridorCorners(query, &empty, navmesh, &corners) == mnav_errorInvalid,
          "a corridor of no polygons");
    mnavQueryFilter bad = mnavDefaultQueryFilter();
    bad.cookie = 0;
    mnavLinkGenDef genDef = mnavDefaultLinkGenDef();
    mnavLinkDef links[4];
    int32_t count = 0;
    CHECK(mnavQuery_GenerateLinks(query, navmesh, &genDef, &(mnavTileRange){0, 0, 1, 1}, &bad,
                                  links, 4, &count) == mnav_errorInvalid,
          "links generated with a filter not made by its default");
    // A navmesh flow field read for a polygon never handed out.
    mnavNavFlowDef def = mnavDefaultNavFlowDef();
    mnavNavFlow* field = nullptr;
    const mnavNavFlowGoal goal = {a.polygon, a.point};
    mnavPolygonId never = a.polygon;
    never.slot = 999;
    mnavPolygonFlow flow;
    CHECK(mnavCreateNavFlow(&def, &field) == mnav_success &&
              mnavNavFlow_Build(field, navmesh, &goal, 1, nullptr) == mnav_success &&
              mnavNavFlow_GetFlow(field, navmesh, never, &flow) == mnav_errorInvalid,
          "a field read for a polygon never handed out");
    mnavDestroyNavFlow(field);
    mnavDestroyQuery(query);
}

static void TestDebug(mnavNavmesh* navmesh)
{
    mnavDebugBuffer buffer = {{0, 0, 0}, s_vertices2, 4096, 0, nullptr, 0, 0, s_lines, 4096, 0};
    CHECK(mnavNavmesh_DebugLinks(nullptr, &buffer) == mnav_errorInvalid &&
              mnavDebugPath(nullptr, &buffer) == mnav_errorInvalid &&
              mnavNavmesh_DebugCorridor(nullptr, nullptr, 0, &buffer) == mnav_errorInvalid,
          "drawing nothing");
    // Of two links, one removed: only the other is drawn.
    const mnavLinkDef def = {{4, 0, 4}, {12, 0, 4}, 1.0f, 1.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId first;
    mnavLinkId second;
    CHECK(mnavStageLink(navmesh, &def, &first) == mnav_success &&
              mnavStageLink(navmesh, &def, &second) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success &&
              mnavStageLinkRemoval(navmesh, first) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "a link removed");
    CHECK(mnavNavmesh_DebugLinks(navmesh, &buffer) == mnav_success && buffer.lineCount == 8 * 2,
          "the other link's arc alone");
}

static void TestAvoidance(void)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "a set");
    const mnavAgent agent = {{(double)NAN, 0.0}, {0, 0}, {0, 0}, 0.5, 1.0, 1.0, 1, 0, 0};
    mnavVec2 velocity;
    mnavDebugBuffer buffer = {{0, 0, 0}, s_vertices2, 4096, 0, nullptr, 0, 0, s_lines, 4096, 0};
    CHECK(mnavAvoidance_Avoid(avoidance, &agent, 1, nullptr, 0, 0.1, &velocity) ==
                  mnav_errorInvalid &&
              mnavAvoidance_Debug(avoidance, &agent, 1, 0.0, &buffer) == mnav_errorInvalid,
          "an agent not finite, stepped or drawn");
    mnavDestroyAvoidance(avoidance);
}

int main(void)
{
    TestMaking();
    TestInput();
    mnavNavmesh* navmesh = Load();
    TestNavmeshReads(navmesh);
    TestQueries(navmesh);
    TestDebug(navmesh);
    TestAvoidance();
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
