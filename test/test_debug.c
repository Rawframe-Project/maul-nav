// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug geometry (mnav-0010): navmeshes, links, paths, corridors, flow
// arrows and avoidance neighbours as vertices and indices; counting past
// a full buffer; positions relative to the origin.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/draw.h"
#include "maul-nav/flow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the navmesh's debug data, the same on every platform.
#define NAVMESH_HASH 0x8ca5b587db0a3c29ull

enum
{
    VERTEX_ROOM = 1 << 16,
    INDEX_ROOM = 1 << 17
};

static mnavDebugVertex s_debugVertices[VERTEX_ROOM];
static uint32_t s_debugTriangles[INDEX_ROOM];
static uint32_t s_debugLines[INDEX_ROOM];

static mnavDebugBuffer Buffer(mnavPos3 origin)
{
    return (mnavDebugBuffer){origin, s_debugVertices, VERTEX_ROOM, 0, s_debugTriangles, INDEX_ROOM,
                             0,      s_debugLines,    INDEX_ROOM,  0};
}

static mnavNavmesh* Load(void)
{
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

// How many vertices of a kind, and of a kind and value.
static int32_t Count(const mnavDebugBuffer* b, mnavDebugKind kind, int32_t value)
{
    int32_t n = 0;
    for (int32_t i = 0; i < b->vertexCount; ++i)
    {
        n += b->vertices[i].kind == kind && (value < 0 || b->vertices[i].value == value) ? 1 : 0;
    }
    return n;
}

static void TestNavmesh(const mnavNavmesh* navmesh)
{
    mnavDebugBuffer b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &b) == mnav_success, "drawn");
    printf("navmesh: %d vertices, %d triangle and %d line indices\n", b.vertexCount,
           b.triangleCount, b.lineCount);
    CHECK(b.triangleCount > 0 && b.triangleCount % 3 == 0 && b.lineCount % 2 == 0 &&
              b.vertexCount == b.triangleCount + b.lineCount,
          "triangles and lines, each with vertices of its own");
    CHECK(Count(&b, mnav_debugPolygon, mnav_areaWalkable) == b.triangleCount &&
              Count(&b, mnav_debugInnerEdge, -1) > 0 && Count(&b, mnav_debugWall, -1) > 0 &&
              Count(&b, mnav_debugTileSide, 1) > 0 && Count(&b, mnav_debugTileSide, 0) > 0 &&
              Count(&b, mnav_debugTileBounds, -1) == 4 * 4 * 2,
          "every kind: fills, inner edges, walls, joined and open sides, four bounds");
    int32_t outside = 0;
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        const mnavDebugVertex* v = &b.vertices[i];
        outside += v->x < -1.5f || v->x > 65.5f || v->z < -1.5f || v->z > 65.5f ? 1 : 0;
    }
    CHECK(outside == 0, "within the world");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, b.vertices,
                               (int32_t)((size_t)b.vertexCount * sizeof(mnavDebugVertex)));
    hash = mnavHash64(hash, b.triangles, (int32_t)((size_t)b.triangleCount * sizeof(uint32_t)));
    hash = mnavHash64(hash, b.lines, (int32_t)((size_t)b.lineCount * sizeof(uint32_t)));
    printf("NAVMESH_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == NAVMESH_HASH, "the pinned hash");
    // An empty buffer counts what the whole needs.
    mnavDebugBuffer none = {{0.0, 0.0, 0.0}, nullptr, 0, 0, nullptr, 0, 0, nullptr, 0, 0};
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &none) == mnav_errorCapacity &&
              none.vertexCount == b.vertexCount && none.triangleCount == b.triangleCount &&
              none.lineCount == b.lineCount,
          "counted without room");
    // Relative to another origin, every position moves by it.
    mnavDebugBuffer moved = Buffer((mnavPos3){100.0, 2.0, -50.0});
    mnavDebugVertex first = b.vertices[0];
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &moved) == mnav_success &&
              fabsf(moved.vertices[0].x - (first.x - 100.0f)) < 1e-3f &&
              fabsf(moved.vertices[0].y - (first.y - 2.0f)) < 1e-3f &&
              fabsf(moved.vertices[0].z - (first.z + 50.0f)) < 1e-3f,
          "relative to the origin");
    // One tile only, at either end of the range.
    mnavDebugBuffer one = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugNavmesh(navmesh, 1, 1, 1, 1, &one) == mnav_success &&
              Count(&one, mnav_debugTileBounds, -1) == 8 && one.vertexCount < b.vertexCount,
          "one tile");
    one = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 0, 0, &one) == mnav_success &&
              Count(&one, mnav_debugTileBounds, -1) == 8,
          "the first tile");
    // Room for every index but not every vertex: still too small.
    mnavDebugBuffer tight = Buffer((mnavPos3){0.0, 0.0, 0.0});
    tight.vertexCapacity = b.vertexCount - 1;
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &tight) == mnav_errorCapacity &&
              tight.vertexCount == b.vertexCount,
          "one vertex short");
    mnavDebugBuffer bad = Buffer((mnavPos3){0.0, 0.0, 0.0});
    bad.lineCount = -1;
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &bad) == mnav_errorInvalid, "a bad count");
    bad = Buffer((mnavPos3){(double)NAN, 0.0, 0.0});
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &bad) == mnav_errorInvalid, "a bad origin");
    bad = Buffer((mnavPos3){0.0, 0.0, 0.0});
    bad.vertices = nullptr;
    CHECK(mnavDebugNavmesh(navmesh, 0, 0, 1, 1, &bad) == mnav_errorInvalid &&
              mnavDebugNavmesh(navmesh, 1, 0, 0, 1, &one) == mnav_errorInvalid,
          "no vertex array; a range backward");
}

static void TestLinksPathsCorridors(mnavNavmesh* navmesh)
{
    // Its start half a meter above the floor, which it snaps down to.
    mnavLinkDef both = {{5.0, 0.5, 5.0}, {9.0, 0.0, 5.0}, 1.0f, 1.0f, mnav_linkJump, true};
    mnavLinkDef away = {{5.0, 0.0, 9.0}, {500.0, 0.0, 9.0}, 1.0f, 1.0f, mnav_linkDrop, false};
    mnavLinkId a;
    mnavLinkId c;
    CHECK(mnavStageLink(navmesh, &both, &a) == mnav_success &&
              mnavStageLink(navmesh, &away, &c) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "linked");
    mnavDebugBuffer b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugLinks(navmesh, &b) == mnav_success && b.lineCount == 2 * 2 * 8 &&
              Count(&b, mnav_debugLinkBothWays, mnav_linkJump) == 16 &&
              Count(&b, mnav_debugLink, mnav_linkDrop + 256) == 16,
          "an arc each, the detached one marked");
    // The arc rises a quarter of the link's length at its middle.
    float top = -1e9f;
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        top = b.vertices[i].kind == mnav_debugLinkBothWays && b.vertices[i].y > top
                  ? b.vertices[i].y
                  : top;
    }
    CHECK(top > 0.9f && top < 1.2f, "the arc's rise");
    mnavLinkState state;
    CHECK(mnavGetLink(navmesh, a, &state) == mnav_success && state.attached &&
              b.vertices[0].y == (float)state.start.y && state.start.y < 0.4,
          "from the snapped start");
    // A removal staged and not committed: still drawn.
    CHECK(mnavStageLinkRemoval(navmesh, c) == mnav_success, "removal staged");
    b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugLinks(navmesh, &b) == mnav_success && b.lineCount == 2 * 2 * 8,
          "a link going at the next commit");
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest s;
    mnavNearest e;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){2.0, 0.0, 2.0}, (mnavVec3){1, 1, 1}, &s) ==
                  mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){60.0, 0.0, 60.0}, (mnavVec3){1, 1, 1},
                              &e) == mnav_success,
          "ends");
    mnavPath path;
    CHECK(mnavFindPath(query, navmesh, nullptr, s.polygon, s.point, e.polygon, e.point, &path) ==
                  mnav_success &&
              path.pointCount >= 2,
          "a path");
    b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugPath(&path, &b) == mnav_success && b.lineCount == 2 * (path.pointCount - 1) &&
              Count(&b, mnav_debugPath, -1) == b.vertexCount,
          "the path's lines");
    b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugCorridor(navmesh, path.polygons, path.polygonCount, &b) == mnav_success &&
              b.triangleCount >= 3 * path.polygonCount &&
              Count(&b, mnav_debugCorridor, path.polygonCount - 1) > 0,
          "the corridor's fills, tagged by place");
    // With a tile of the corridor removed, its polygons are skipped.
    int32_t whole = b.triangleCount;
    CHECK(mnavStageTileRemoval(navmesh, 1, 1) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a tile removed");
    b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugCorridor(navmesh, path.polygons, path.polygonCount, &b) == mnav_success &&
              b.triangleCount > 0 && b.triangleCount < whole,
          "the gone tile's polygons skipped");
    mnavPolygonId never = {9999, 1, 0};
    CHECK(mnavDebugCorridor(navmesh, &never, 1, &b) == mnav_errorInvalid, "an id never handed out");
    mnavDestroyQuery(query);
}

static void TestFlowAndAvoidance(void)
{
    // A 4 by 3 open grid with its goal in a corner: every other cell gets
    // an arrow of three lines.
    mnavAreaType areas[12];
    memset(areas, 1, sizeof(areas));
    mnavGrid grid = {areas, 4, 3, 2.0f};
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.cells = 12;
    mnavFlowField* field = nullptr;
    const mnavCell goal = {0, 0};
    CHECK(mnavCreateFlowField(&def, &field) == mnav_success &&
              mnavBuildFlowField(field, &grid, nullptr, &goal, 1) == mnav_success,
          "a field");
    mnavDebugBuffer b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugFlowField(field, 0.5, &b) == mnav_success && b.lineCount == 11 * 3 * 2 &&
              Count(&b, mnav_debugFlow, -1) == b.vertexCount,
          "eleven arrows");
    // The arrow of cell (1, 0) points along -X at its center, height 0.5.
    CHECK(b.vertices[0].x == 3.8f && b.vertices[1].x == 2.2f && b.vertices[0].y == 0.5f &&
              b.vertices[0].z == 1.0f,
          "from (1, 0) toward the goal");
    CHECK(mnavDebugFlowField(field, (double)NAN, &b) == mnav_errorInvalid, "no height");
    mnavDestroyFlowField(field);
    // Three agents, two near: the near pair sees each other.
    mnavAvoidanceDef avoidDef = mnavDefaultAvoidanceDef();
    avoidDef.limits.agents = 8;
    avoidDef.neighborDistance = 3.0;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&avoidDef, &avoidance) == mnav_success, "a set");
    mnavAgent agents[3] = {{{0.0, 0.0}, {0, 0}, {0, 0}, 0.5, 1.0, 1.0, 1},
                           {{2.0, 0.0}, {0, 0}, {0, 0}, 0.5, 1.0, 1.0, 2},
                           {{20.0, 0.0}, {0, 0}, {0, 0}, 0.5, 1.0, 1.0, 3}};
    b = Buffer((mnavPos3){0.0, 0.0, 0.0});
    CHECK(mnavDebugAvoidance(avoidance, agents, 3, 0.1, &b) == mnav_success &&
              Count(&b, mnav_debugAgent, -1) == 3 * 16 * 2 &&
              Count(&b, mnav_debugNeighbor, -1) == 2 * 2,
          "outlines and the near pair's lines");
    CHECK(mnavDebugAvoidance(avoidance, agents, 9, 0.1, &b) == mnav_errorLimit &&
              mnavDebugAvoidance(avoidance, agents, 3, (double)INFINITY, &b) == mnav_errorInvalid,
          "the limit; a bad height");
    mnavDestroyAvoidance(avoidance);
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    TestNavmesh(navmesh);
    TestLinksPathsCorridors(navmesh);
    TestFlowAndAvoidance();
    mnavDestroyNavmesh(navmesh);
    return s_failures == 0 ? 0 : 1;
}
