// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Every read the documentation lets threads share, run on 8 threads at
// once: one navmesh with links, one navmesh flow field and one grid, read
// by threads that each own their query context, corridor, debug buffers,
// fields and hierarchy. Each thread runs the same rounds of reads,
// starting at a different round so that different reads meet, and must
// get what one thread gets, round by round. Run under ThreadSanitizer,
// the test also shows that these reads write nothing they share.

#include "test_harness.h"
#include "world.h"

#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum
{
    THREADS = 8,
    ROUNDS = 16,
    GRID_SIDE = 64,
    DRAW_VERTICES = 1 << 16,
    DRAW_INDICES = 1 << 17,
    CORRIDOR = 64,
    GENERATED = 64
};

// What every thread shares, set up before they start and only read
// while they run.
static mnavNavmesh* s_navmesh;
static mnavNavFlow* s_field;
static mnavAreaType s_areas[GRID_SIDE * GRID_SIDE];
static mnavGrid s_grid;

static uint64_t Mix(uint64_t hash, const void* data, size_t size)
{
    return mnavHash64(hash, data, (int32_t)size);
}

static uint64_t MixResult(uint64_t hash, mnavResult result)
{
    int32_t value = (int32_t)result;
    return Mix(hash, &value, sizeof(value));
}

static uint64_t MixDouble(uint64_t hash, double value)
{
    return Mix(hash, &value, sizeof(value));
}

static uint64_t MixInt(uint64_t hash, int64_t value)
{
    return Mix(hash, &value, sizeof(value));
}

// A point of round r on the world's floor, from a seed.
static mnavPos3 PointOf(uint32_t* state)
{
    double v[2];
    for (int32_t k = 0; k < 2; ++k)
    {
        *state = *state * 1664525u + 1013904223u;
        v[k] = 1.0 + (double)(*state >> 8 & 0xFFFFu) / 65536.0 * 62.0;
    }
    return (mnavPos3){v[0], 0.0, v[1]};
}

// What one thread owns.
typedef struct Reader
{
    mnavQuery* query;
    mnavNavFlow* field;
    mnavFlowField* flow;
    mnavHierarchy* hierarchy;
    mnavHierarchyReport report;
    mnavDebugVertex* vertices;
    uint32_t* triangles;
    uint32_t* lines;
    mnavLinkDef links[GENERATED];
    mnavPolygonId corridor[CORRIDOR];
    int32_t first;
    uint64_t hashes[ROUNDS];
    bool failed;
} Reader;

static uint64_t Drawn(Reader* reader, mnavResult result, const mnavDebugBuffer* buffer,
                      uint64_t hash)
{
    hash = MixResult(hash, result);
    hash = MixInt(hash, buffer->vertexCount);
    hash = MixInt(hash, buffer->triangleCount);
    hash = MixInt(hash, buffer->lineCount);
    for (int32_t i = 0; result == mnav_success && i < buffer->vertexCount; ++i)
    {
        const mnavDebugVertex* v = &reader->vertices[i];
        hash = Mix(hash, &v->x, sizeof(float) * 3);
        hash = MixInt(hash, (int64_t)v->kind * 65536 + v->value);
    }
    if (result == mnav_success)
    {
        hash = Mix(hash, reader->triangles, (size_t)buffer->triangleCount * sizeof(uint32_t));
        hash = Mix(hash, reader->lines, (size_t)buffer->lineCount * sizeof(uint32_t));
    }
    return hash;
}

static mnavDebugBuffer Buffer(Reader* reader)
{
    return (mnavDebugBuffer){{0.0, 0.0, 0.0},
                             reader->vertices,
                             DRAW_VERTICES,
                             0,
                             reader->triangles,
                             DRAW_INDICES,
                             0,
                             reader->lines,
                             DRAW_INDICES,
                             0};
}

static uint64_t Path(uint64_t hash, mnavResult result, const mnavPath* path)
{
    hash = MixResult(hash, result);
    if (result == mnav_success)
    {
        hash = MixInt(hash, path->end);
        hash = MixDouble(hash, path->cost);
        hash = Mix(hash, path->polygons, (size_t)path->polygonCount * sizeof(mnavPolygonId));
        hash = Mix(hash, path->points, (size_t)path->pointCount * sizeof(mnavPos3));
    }
    return hash;
}

// The queries at points: nearest, polygons in a box, heights, areas,
// tiles, links, rays, moves, walls, random points, reachability, paths
// whole and sliced.
static uint64_t Points(Reader* reader, uint32_t* state, uint64_t hash)
{
    mnavQuery* query = reader->query;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    mnavNearest a = {0};
    mnavNearest b = {0};
    hash = MixResult(hash, mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &a));
    hash = MixResult(hash, mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &b));
    hash = Mix(hash, &a.polygon, sizeof(a.polygon));
    hash = Mix(hash, &a.point, sizeof(a.point));
    hash = Mix(hash, &b.polygon, sizeof(b.polygon));
    mnavPolygonId found[32];
    mnavFound count = {0};
    hash = MixResult(hash, mnavFindPolygons(s_navmesh, nullptr, a.point,
                                            (mnavVec3){4.0f, 2.0f, 4.0f}, found, 32, &count));
    hash = MixInt(hash, count.count);
    hash = Mix(hash, found, (size_t)(count.count < 32 ? count.count : 32) * sizeof(mnavPolygonId));
    double height = 0.0;
    hash = MixResult(hash, mnavGetHeight(s_navmesh, a.polygon, a.point.x, a.point.z, &height));
    hash = MixDouble(hash, height);
    mnavAreaType area = 0;
    hash = MixResult(hash, mnavGetArea(s_navmesh, a.polygon, &area));
    hash = MixInt(hash, area);
    mnavTileId tile = {0};
    hash = MixResult(hash, mnavGetTile(s_navmesh, (int32_t)(a.point.x / 32.0),
                                       (int32_t)(a.point.z / 32.0), &tile));
    hash = Mix(hash, &tile, sizeof(tile));
    hash = MixInt(hash, mnavGetTier(s_navmesh));
    mnavLinkState link = {0};
    hash = MixResult(hash, mnavGetLink(s_navmesh, (mnavLinkId){1, 1}, &link));
    hash = MixInt(hash, link.crossings);
    mnavRay ray = {0};
    hash =
        MixResult(hash, mnavRaycast(query, s_navmesh, nullptr, a.polygon, a.point, b.point, &ray));
    hash = MixDouble(hash, ray.t);
    hash = MixInt(hash, ray.polygonCount);
    mnavMove move = {0};
    hash = MixResult(
        hash, mnavMoveAlongSurface(query, s_navmesh, nullptr, a.polygon, a.point, b.point, &move));
    hash = Mix(hash, &move.point, sizeof(move.point));
    mnavWall wall = {0};
    hash = MixResult(
        hash, mnavFindWallDistance(query, s_navmesh, nullptr, a.polygon, a.point, 4.0, &wall));
    hash = MixDouble(hash, wall.distance);
    mnavRandomPoint random = {0};
    hash = MixResult(hash, mnavFindRandomPoint(s_navmesh, nullptr, *state, &random));
    hash = Mix(hash, &random.point, sizeof(random.point));
    hash = MixResult(hash, mnavFindRandomPointAround(query, s_navmesh, nullptr, a.polygon, a.point,
                                                     6.0, *state, &random));
    hash = Mix(hash, &random.point, sizeof(random.point));
    mnavPathEnd end = 0;
    hash = MixResult(hash, mnavCheckReachable(query, s_navmesh, nullptr, a.polygon, a.point,
                                              b.polygon, b.point, &end));
    hash = MixInt(hash, end);
    mnavPath path = {0};
    hash =
        Path(hash,
             mnavFindPath(query, s_navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path),
             &path);
    mnavResult sliced =
        mnavBeginPath(query, s_navmesh, nullptr, a.polygon, a.point, b.polygon, b.point);
    bool ended = sliced != mnav_success;
    for (int32_t k = 0; !ended && k < 10000; ++k)
    {
        sliced = mnavContinuePath(query, s_navmesh, 16, &ended);
        ended = ended || sliced != mnav_success;
    }
    if (sliced == mnav_success)
    {
        sliced = mnavFinishPath(query, s_navmesh, &path);
    }
    return Path(hash, sliced, &path);
}

// A corridor along a path: corners, moves, checks, a shortcut, a replan,
// then its drawing.
static uint64_t Corridor(Reader* reader, uint32_t* state, uint64_t hash)
{
    mnavQuery* query = reader->query;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    mnavNearest a = {0};
    mnavNearest b = {0};
    if (mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &a) != mnav_success ||
        mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &b) != mnav_success ||
        a.polygon.slot == 0 || b.polygon.slot == 0)
    {
        return MixInt(hash, -1);
    }
    mnavCorridor corridor;
    hash = MixResult(hash,
                     mnavResetCorridor(&corridor, reader->corridor, CORRIDOR, a.polygon, a.point));
    mnavPath path = {0};
    if (mnavFindPath(query, s_navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
        mnav_success)
    {
        hash = MixResult(hash, mnavSetCorridor(&corridor, &path));
    }
    mnavCorners corners = {0};
    hash = MixResult(hash, mnavCorridorCorners(query, s_navmesh, &corridor, &corners));
    hash = Mix(hash, corners.points, (size_t)corners.pointCount * sizeof(mnavPos3));
    mnavMove move = {0};
    mnavPos3 toward = corners.pointCount > 1 ? corners.points[1] : b.point;
    hash = MixResult(hash, mnavMoveCorridor(query, s_navmesh, nullptr, &corridor, toward, &move));
    hash = MixResult(hash,
                     mnavMoveCorridorTarget(query, s_navmesh, nullptr, &corridor, b.point, &move));
    int32_t valid = 0;
    hash = MixResult(hash, mnavCheckCorridor(s_navmesh, nullptr, &corridor, &valid));
    hash = MixInt(hash, valid);
    bool shortened = false;
    hash = MixResult(
        hash, mnavShortcutCorridor(query, s_navmesh, nullptr, &corridor, b.point, &shortened));
    hash = MixInt(hash, shortened);
    hash = Path(hash, mnavReplanCorridor(query, s_navmesh, nullptr, &corridor, box, &path), &path);
    hash = Mix(hash, &corridor.position, sizeof(corridor.position));
    hash = Mix(hash, corridor.polygons, (size_t)corridor.count * sizeof(mnavPolygonId));
    mnavDebugBuffer buffer = Buffer(reader);
    mnavResult drawn = mnavDebugCorridor(s_navmesh, corridor.polygons, corridor.count, &buffer);
    return Drawn(reader, drawn, &buffer, hash);
}

static uint64_t Flow(uint64_t hash, mnavResult result, const mnavPolygonFlow* flow)
{
    hash = MixResult(hash, result);
    if (result == mnav_success)
    {
        hash = MixDouble(hash, flow->cost);
        hash = Mix(hash, &flow->next, sizeof(flow->next));
        hash = Mix(hash, &flow->left, sizeof(flow->left));
        hash = Mix(hash, &flow->right, sizeof(flow->right));
        hash = Mix(hash, &flow->link, sizeof(flow->link));
    }
    return hash;
}

// The shared field read, a field of the thread's own built on the shared
// navmesh, a grid path and a grid field on the shared grid, links
// generated, a hierarchical path, and the navmesh, its links and the
// shared field drawn.
static uint64_t Wide(Reader* reader, int32_t round, uint32_t* state, uint64_t hash)
{
    mnavQuery* query = reader->query;
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    mnavNearest a = {0};
    mnavNearest b = {0};
    hash = MixResult(hash, mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &a));
    hash = MixResult(hash, mnavFindNearest(s_navmesh, nullptr, PointOf(state), box, &b));
    mnavPolygonFlow flow;
    hash = Flow(hash, mnavNavFlowAt(s_field, s_navmesh, a.polygon, &flow), &flow);
    mnavNavFlowGoal goal = {b.polygon, b.point};
    hash = MixResult(hash, mnavBuildNavFlow(reader->field, s_navmesh, nullptr, &goal, 1));
    hash = Flow(hash, mnavNavFlowAt(reader->field, s_navmesh, a.polygon, &flow), &flow);
    mnavCell start = {(int32_t)(a.point.x) % GRID_SIDE, (int32_t)(a.point.z) % GRID_SIDE};
    mnavCell goalCell = {(int32_t)(b.point.x) % GRID_SIDE, (int32_t)(b.point.z) % GRID_SIDE};
    mnavGridPath gridPath = {0};
    mnavResult walked = mnavFindGridPath(query, &s_grid, nullptr, start, goalCell, &gridPath);
    hash = MixResult(hash, walked);
    if (walked == mnav_success)
    {
        hash = MixDouble(hash, gridPath.cost);
        hash = Mix(hash, gridPath.cells, (size_t)gridPath.cellCount * sizeof(mnavCell));
    }
    hash = MixResult(hash, mnavBuildFlowField(reader->flow, &s_grid, nullptr, &goalCell, 1));
    mnavFlow cell = {0};
    hash = MixResult(hash, mnavFlowAt(reader->flow, start, &cell));
    hash = MixDouble(hash, cell.cost);
    hash = Mix(hash, &cell.next, sizeof(cell.next));
    mnavLinkGenDef def = mnavDefaultLinkGenDef();
    int32_t generated = 0;
    hash = MixResult(hash, mnavGenerateLinks(query, s_navmesh, nullptr, &def, round % 2, 0,
                                             round % 2, 1, reader->links, GENERATED, &generated));
    hash = MixInt(hash, generated);
    for (int32_t i = 0; i < generated && i < GENERATED; ++i)
    {
        hash = Mix(hash, &reader->links[i].start, sizeof(mnavPos3));
        hash = Mix(hash, &reader->links[i].end, sizeof(mnavPos3));
    }
    mnavPath path = {0};
    hash = Path(hash,
                mnavFindHierarchicalPath(query, reader->hierarchy, s_navmesh, a.polygon, a.point,
                                         b.polygon, b.point, &path),
                &path);
    mnavDebugBuffer buffer = Buffer(reader);
    hash = Drawn(reader, mnavDebugNavmesh(s_navmesh, round % 2, 0, 1, 1, &buffer), &buffer, hash);
    buffer = Buffer(reader);
    hash = Drawn(reader, mnavDebugLinks(s_navmesh, &buffer), &buffer, hash);
    buffer = Buffer(reader);
    return Drawn(reader, mnavDebugNavFlow(s_field, s_navmesh, &buffer), &buffer, hash);
}

static uint64_t Round(Reader* reader, int32_t round)
{
    uint32_t state = 11u + (uint32_t)round * 2654435761u;
    uint64_t hash = MNAV_HASH_INIT;
    hash = Points(reader, &state, hash);
    hash = Corridor(reader, &state, hash);
    hash = Wide(reader, round, &state, hash);
    return hash;
}

static bool Open(Reader* reader)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavNavFlowDef fieldDef = mnavDefaultNavFlowDef();
    mnavFlowFieldDef flowDef = mnavDefaultFlowFieldDef();
    flowDef.cells = GRID_SIDE * GRID_SIDE;
    mnavHierarchyDef hierarchyDef = mnavDefaultHierarchyDef();
    hierarchyDef.clusterTiles = 1;
    reader->vertices = malloc(DRAW_VERTICES * sizeof(mnavDebugVertex));
    reader->triangles = malloc(DRAW_INDICES * sizeof(uint32_t));
    reader->lines = malloc(DRAW_INDICES * sizeof(uint32_t));
    return reader->vertices != nullptr && reader->triangles != nullptr &&
           reader->lines != nullptr && mnavCreateQuery(&queryDef, &reader->query) == mnav_success &&
           mnavCreateNavFlow(&fieldDef, &reader->field) == mnav_success &&
           mnavCreateFlowField(&flowDef, &reader->flow) == mnav_success &&
           mnavCreateHierarchy(&hierarchyDef, &reader->hierarchy) == mnav_success &&
           mnavBuildHierarchy(reader->hierarchy, reader->query, s_navmesh, nullptr,
                              &reader->report) == mnav_success;
}

static void Close(Reader* reader)
{
    mnavDestroyHierarchy(reader->hierarchy);
    mnavDestroyFlowField(reader->flow);
    mnavDestroyNavFlow(reader->field);
    mnavDestroyQuery(reader->query);
    free(reader->lines);
    free(reader->triangles);
    free(reader->vertices);
}

static void* Read(void* context)
{
    Reader* reader = context;
    reader->failed = !Open(reader);
    for (int32_t k = 0; !reader->failed && k < ROUNDS; ++k)
    {
        int32_t round = (reader->first + k) % ROUNDS;
        reader->hashes[round] = Round(reader, round);
    }
    Close(reader);
    return nullptr;
}

// The navmesh of the world's four tiles with two links, a field toward
// one goal on it, and a grid of walls and costs.
static void Share(void)
{
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    CHECK(mnavCreateNavmesh(&def, &s_navmesh).result == mnav_success, "a navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(s_navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    const mnavLinkDef links[2] = {
        {{4.0, 0.0, 4.0}, {60.0, 0.0, 60.0}, 1.0f, 2.0f, mnav_linkJump, true, 0.0f},
        {{60.0, 0.0, 4.0}, {4.0, 0.0, 60.0}, 1.0f, 0.0f, mnav_linkDrop, false, 0.0f}};
    for (int32_t k = 0; k < 2; ++k)
    {
        mnavLinkId id;
        CHECK(mnavStageLink(s_navmesh, &links[k], &id) == mnav_success, "a link");
    }
    CHECK(mnavCommit(s_navmesh) == mnav_success, "committed");
    mnavNavFlowDef fieldDef = mnavDefaultNavFlowDef();
    CHECK(mnavCreateNavFlow(&fieldDef, &s_field) == mnav_success, "a field");
    mnavNearest goal = {0};
    CHECK(mnavFindNearest(s_navmesh, nullptr, (mnavPos3){32.0, 0.0, 33.0},
                          (mnavVec3){1.0f, 2.0f, 1.0f}, &goal) == mnav_success &&
              goal.polygon.slot != 0,
          "a goal");
    mnavNavFlowGoal goals[1] = {{goal.polygon, goal.point}};
    CHECK(mnavBuildNavFlow(s_field, s_navmesh, nullptr, goals, 1) == mnav_success, "built");
    uint32_t seed = 5u;
    for (int32_t c = 0; c < GRID_SIDE * GRID_SIDE; ++c)
    {
        seed = seed * 1664525u + 1013904223u;
        uint32_t r = (seed >> 16) % 20u;
        s_areas[c] = r < 4u ? mnav_areaNone : (mnavAreaType)(r < 14u ? 1u : 2u + r % 3u);
    }
    s_grid = (mnavGrid){s_areas, GRID_SIDE, GRID_SIDE, 1.0f};
}

static Reader s_readers[THREADS + 1];

static void TestEveryReadOnManyThreads(void)
{
    Share();
    Reader* one = &s_readers[THREADS];
    *one = (Reader){0};
    Read(one);
    CHECK(!one->failed && one->report.clusters == 4 && one->report.edges > 0,
          "one thread read, over a hierarchy of four clusters");
    pthread_t threads[THREADS];
    for (int32_t w = 0; w < THREADS; ++w)
    {
        s_readers[w] = (Reader){0};
        s_readers[w].first = w * (ROUNDS / THREADS);
        CHECK(pthread_create(&threads[w], nullptr, Read, &s_readers[w]) == 0, "a reader");
    }
    bool same = true;
    for (int32_t w = 0; w < THREADS; ++w)
    {
        pthread_join(threads[w], nullptr);
        same = same && !s_readers[w].failed && s_readers[w].report.edges == one->report.edges &&
               memcmp(s_readers[w].hashes, one->hashes, sizeof(one->hashes)) == 0;
    }
    CHECK(same, "8 threads reading at once get what one thread gets");
    mnavDestroyNavFlow(s_field);
    mnavDestroyNavmesh(s_navmesh);
}

int main(void)
{
    TestEveryReadOnManyThreads();
    return s_failures == 0 ? 0 : 1;
}
