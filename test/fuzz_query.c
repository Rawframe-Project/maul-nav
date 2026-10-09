// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes query inputs (mnav-0005): a run of queries read from the bytes,
// on the test world with one tile left unloaded and a link: paths, A* and
// exact, walls, points, boxes and radii from a range around the world or
// raw bits, polygon ids found or made up, filters with costs out of range,
// seeds, budgets, corridors, and grids with paths and flow fields over
// them. Each query must end in a typed status, give finite results when it
// succeeds, and keep its counts within its limits; the query contexts give
// back all they took.

#include "counting_allocator.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

static double Raw(Reader* r)
{
    uint64_t bits = 0;
    for (int32_t k = 0; k < 8; ++k)
    {
        bits |= (uint64_t)Byte(r) << (8 * k);
    }
    double d;
    memcpy(&d, &bits, sizeof(d));
    return d;
}

// A coordinate: one time in sixteen raw bits, otherwise a quarter meter
// step from -4 to 67.75 m.
static double Coordinate(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? Raw(r) : (double)Byte(r) * 0.28125 - 4.0;
}

static mnavPos3 Point(Reader* r)
{
    return (mnavPos3){Coordinate(r), Coordinate(r) * 0.0625, Coordinate(r)};
}

// A length from 0 to 16 m, or raw bits.
static double Length(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? Raw(r) : (double)(Byte(r) % 64) * 0.25;
}

static bool Finite(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

static bool Typed(mnavResult result)
{
    return result <= mnav_success && result >= mnav_errorTier;
}

static mnavNavmesh* s_navmesh;
static mnavQuery* s_query;
static mnavPolygonId s_ids[8];
// The bytes the query context holds.
static size_t s_baseline;

// The world, tile (1, 1) left out, a point link and an edge link joining
// places on it; a few polygon ids found on it.
static void Load(void)
{
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    Expect(mnavCreateNavmesh(&def, &s_navmesh).result == mnav_success);
    for (int32_t t = 0; t < 3; ++t)
    {
        Expect(mnavStageTile(s_navmesh, s_tiles[t], s_sizes[t]).result == mnav_success);
    }
    const mnavLinkDef link = {{5, 0, 5}, {40, 0, 5}, 1.0f, 2.0f, mnav_linkJump, true, 0.0f};
    const mnavLinkDef edge = {{20, 0, 30}, {20, 0, 50}, 1.0f, 3.0f, mnav_linkClimb, true, 6.0f};
    mnavLinkId id;
    Expect(mnavStageLink(s_navmesh, &edge, &id) == mnav_success);
    Expect(mnavStageLink(s_navmesh, &link, &id) == mnav_success &&
           mnavCommit(s_navmesh) == mnav_success);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.allocator = CountingAllocator();
    queryDef.limits.nodes = 512;
    Expect(mnavCreateQuery(&queryDef, &s_query) == mnav_success);
    for (int32_t i = 0; i < 8; ++i)
    {
        mnavNearest nearest;
        mnavPos3 at = {4.0 + 7.0 * i, 0.0, 3.0 + 3.0 * i};
        Expect(mnavFindNearest(s_navmesh, nullptr, at, (mnavVec3){2, 2, 2}, &nearest) ==
               mnav_success);
        s_ids[i] = nearest.polygon;
    }
    s_baseline = s_held;
}

// A polygon id: one found, or one made up.
static mnavPolygonId Id(Reader* r)
{
    uint8_t pick = Byte(r);
    if (pick < 192)
    {
        return s_ids[pick % 8];
    }
    return (mnavPolygonId){Byte(r) % 6u, Byte(r) % 4u, (uint32_t)Byte(r) * 3u};
}

// A filter: the default, or with some areas left out, some kinds barred
// and some costs changed, now and then out of range.
static mnavQueryFilter Filter(Reader* r)
{
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    uint8_t pick = Byte(r);
    if (pick % 4 == 0)
    {
        filter.areas &= ~((uint64_t)Byte(r) << 1);
        filter.kinds &= ~(uint64_t)Byte(r);
        filter.costs[1 + Byte(r) % 3] = pick % 32 == 0 ? (float)Raw(r) : 0.5f + (float)(pick % 7);
    }
    return filter;
}

static void CheckPath(mnavResult result, const mnavPath* path)
{
    Expect(Typed(result));
    if (result != mnav_success)
    {
        return;
    }
    Expect(path->polygonCount >= 0 && path->polygonCount <= 512 && path->pointCount >= 0 &&
           path->linkCount >= 0 && path->linkCount <= path->polygonCount);
    for (int32_t i = 0; i < path->pointCount; ++i)
    {
        Expect(Finite(path->points[i]));
    }
}

static void FindPath(Reader* r, const mnavQueryFilter* filter, bool sliced)
{
    mnavPolygonId a = Id(r);
    mnavPolygonId b = Id(r);
    mnavPos3 start = Point(r);
    mnavPos3 end = Point(r);
    mnavPath path;
    if (!sliced)
    {
        CheckPath(mnavFindPath(s_query, s_navmesh, filter, a, start, b, end, &path), &path);
        CheckPath(mnavFindShortestPath(s_query, s_navmesh, filter, a, start, b, end, &path), &path);
        return;
    }
    mnavResult result = mnavBeginPath(s_query, s_navmesh, filter, a, start, b, end);
    Expect(Typed(result));
    int32_t budget = 1 + Byte(r) % 64;
    bool ended = false;
    for (int32_t k = 0; k < 64 && result == mnav_success && !ended; ++k)
    {
        result = mnavContinuePath(s_query, s_navmesh, budget, &ended);
    }
    Expect(Typed(result));
    CheckPath(mnavFinishPath(s_query, s_navmesh, &path), &path);
}

static void Spatial(Reader* r, const mnavQueryFilter* filter, uint8_t op)
{
    mnavPolygonId id = Id(r);
    mnavPos3 a = Point(r);
    mnavPos3 b = Point(r);
    mnavResult result = mnav_success;
    if (op == 0)
    {
        mnavRay ray;
        result = mnavRaycast(s_query, s_navmesh, filter, id, a, b, &ray);
        Expect(result != mnav_success || (ray.polygonCount >= 0 && isfinite(ray.normalX)));
    }
    else if (op == 1)
    {
        mnavMove move;
        result = mnavMoveAlongSurface(s_query, s_navmesh, filter, id, a, b, &move);
        Expect(result != mnav_success || Finite(move.point));
    }
    else if (op == 2 && Byte(r) % 2 == 0)
    {
        mnavWall wall;
        result = mnavFindWallDistance(s_query, s_navmesh, filter, id, a, Length(r), &wall);
        Expect(result != mnav_success || !wall.found || isfinite(wall.distance));
    }
    else if (op == 2)
    {
        mnavWallSegment walls[16];
        int32_t capacity = Byte(r) % 17;
        mnavWallsFound found;
        result =
            mnavFindWalls(s_query, s_navmesh, filter, id, a, Length(r), walls, capacity, &found);
        bool listed = result == mnav_success || result == mnav_errorCapacity;
        Expect(!listed ||
               (found.count >= 0 && (result == mnav_errorCapacity) == (found.count > capacity)));
        for (int32_t k = 0; listed && k < found.count && k < capacity; ++k)
        {
            Expect(Finite(walls[k].start) && Finite(walls[k].end) && isfinite(walls[k].distance) &&
                   (k == 0 || walls[k - 1].distance <= walls[k].distance));
        }
    }
    else if (op == 3)
    {
        double height = 0.0;
        result = mnavGetHeight(s_navmesh, id, a.x, a.z, &height);
        Expect(result != mnav_success || isfinite(height));
    }
    else if (op == 4)
    {
        mnavRandomPoint point;
        double bits = Raw(r);
        uint64_t seed;
        memcpy(&seed, &bits, sizeof(seed));
        result = Byte(r) % 2 == 0 ? mnavFindRandomPoint(s_navmesh, filter, seed, &point)
                                  : mnavFindRandomPointAround(s_query, s_navmesh, filter, id, a,
                                                              Length(r), seed, &point);
        Expect(result != mnav_success || Finite(point.point));
    }
    else
    {
        mnavPathEnd end;
        result = mnavCheckReachable(s_query, s_navmesh, filter, id, a, Id(r), b, &end);
    }
    Expect(Typed(result));
}

static void Boxes(Reader* r, const mnavQueryFilter* filter)
{
    mnavPos3 center = Point(r);
    mnavVec3 half = {Length(r), Length(r), Length(r)};
    if (Byte(r) % 2 == 0)
    {
        mnavNearest nearest;
        mnavResult result = mnavFindNearest(s_navmesh, filter, center, half, &nearest);
        Expect(Typed(result) && (result != mnav_success || Finite(nearest.point)));
        return;
    }
    mnavPolygonId polygons[16];
    int32_t capacity = Byte(r) % 17;
    mnavFound found;
    mnavResult result =
        mnavFindPolygons(s_navmesh, filter, center, half, polygons, capacity, &found);
    Expect(Typed(result) && (result != mnav_success || found.count <= capacity));
}

// The ground distance from p to the segment from a to b.
static double ToSegment(mnavPos3 p, mnavPos3 a, mnavPos3 b)
{
    double dx = b.x - a.x;
    double dz = b.z - a.z;
    double length2 = dx * dx + dz * dz;
    double t = length2 > 0.0 ? ((p.x - a.x) * dx + (p.z - a.z) * dz) / length2 : 0.0;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return hypot(a.x + dx * t - p.x, a.z + dz * t - p.z);
}

// The runs of area along a corridor: from its position, in order along
// its corners, each on its segment; the count alike at every capacity.
static mnavResult Areas(Reader* r, const mnavCorridor* corridor)
{
    mnavAreaRun runs[16];
    int32_t capacity = Byte(r) % 17;
    int32_t count = -1;
    mnavCorners corners;
    mnavResult result =
        mnavCorridorAreas(s_query, s_navmesh, corridor, &corners, runs, capacity, &count);
    if (result != mnav_success && result != mnav_errorCapacity)
    {
        return result;
    }
    Expect(count >= 1 && (result == mnav_success) == (count <= capacity));
    int32_t again = -1;
    Expect(mnavCorridorAreas(s_query, s_navmesh, corridor, &corners, runs, 16, &again) ==
               (again <= 16 ? mnav_success : mnav_errorCapacity) &&
           again == count);
    int32_t written = count < 16 ? count : 16;
    Expect(runs[0].corner == 0 && runs[0].start.x == corridor->position.x &&
           runs[0].start.z == corridor->position.z);
    for (int32_t k = 0; k < written; ++k)
    {
        const mnavAreaRun* run = &runs[k];
        Expect(Finite(run->start) && run->area < MNAV_AREA_TYPES && run->corner >= 0 &&
               run->corner < corners.pointCount && (k == 0 || run->corner >= runs[k - 1].corner));
        mnavPos3 a = corners.points[run->corner];
        mnavPos3 b =
            corners.points[run->corner + 1 < corners.pointCount ? run->corner + 1 : run->corner];
        double scale = 1.0 + fabs(a.x) + fabs(a.z) + fabs(b.x) + fabs(b.z);
        Expect(ToSegment(run->start, a, b) <= 1e-6 * scale);
    }
    return result;
}

// Steering along a corridor's corners with a def from the bytes: a
// finite velocity on the ground no faster than the def's speed, stopped
// on arriving or at a link; a def out of range refused.
static mnavResult Steer(Reader* r, const mnavCorridor* corridor)
{
    mnavCorners corners;
    mnavResult result = mnavCorridorCorners(s_query, s_navmesh, corridor, &corners);
    if (result != mnav_success)
    {
        return result;
    }
    mnavSteerDef def = mnavDefaultSteerDef();
    def.maxSpeed = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.25);
    def.slowDistance = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.1);
    def.arriveDistance = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.01);
    def.anticipateTurns = Byte(r) % 2 == 0;
    bool good = def.maxSpeed >= 0.0f && def.maxSpeed <= MNAV_MAX_STEER_SPEED &&
                def.slowDistance >= 0.0f && def.slowDistance <= MNAV_MAX_STEER_DISTANCE &&
                def.arriveDistance >= 0.0f && def.arriveDistance <= MNAV_MAX_STEER_DISTANCE;
    mnavSteering steering;
    result = mnavSteer(&corners, &def, &steering);
    Expect(result == (good ? mnav_success : mnav_errorRange));
    if (result != mnav_success)
    {
        return result;
    }
    double speed = hypot(steering.velocity.x, steering.velocity.z);
    Expect(Finite(steering.velocity) && steering.velocity.y == 0.0 &&
           speed <= (double)def.maxSpeed * (1.0 + 1e-9) + 1e-12);
    Expect(steering.state == mnav_steerMoving || speed == 0.0);
    Expect((steering.state == mnav_steerAtLink) == (steering.link >= 0) &&
           steering.link < (corners.linkCount > 0 ? corners.linkCount : 1));
    return result;
}

static void Corridor(Reader* r, const mnavQueryFilter* filter)
{
    mnavPolygonId buffer[64];
    mnavCorridor corridor;
    mnavPolygonId id = Id(r);
    mnavResult result = mnavResetCorridor(&corridor, buffer, 1 + Byte(r) % 64, id, Point(r));
    Expect(Typed(result));
    if (result != mnav_success)
    {
        return;
    }
    mnavPath path;
    result =
        mnavFindPath(s_query, s_navmesh, filter, id, corridor.position, Id(r), Point(r), &path);
    if (result == mnav_success)
    {
        Expect(Typed(mnavSetCorridor(&corridor, &path)));
    }
    for (int32_t k = 0; k < 4; ++k)
    {
        uint8_t op = Byte(r) % 8;
        mnavMove move;
        mnavCorners corners;
        int32_t valid = 0;
        bool shortened = false;
        result =
            op == 0 ? mnavMoveCorridor(s_query, s_navmesh, filter, &corridor, Point(r), &move)
            : op == 1
                ? mnavMoveCorridorTarget(s_query, s_navmesh, filter, &corridor, Point(r), &move)
            : op == 2 ? mnavCorridorCorners(s_query, s_navmesh, &corridor, &corners)
            : op == 3 ? mnavCheckCorridor(s_navmesh, filter, &corridor, &valid)
            : op == 4
                ? mnavShortcutCorridor(s_query, s_navmesh, filter, &corridor, Point(r), &shortened)
            : op == 5 ? mnavReplanCorridor(s_query, s_navmesh, filter, &corridor,
                                           (mnavVec3){1, 1, 1}, &path)
            : op == 6 ? Areas(r, &corridor)
                      : Steer(r, &corridor);
        Expect(Typed(result) && corridor.count >= 1 && corridor.count <= corridor.capacity);
    }
}

// A grid of up to 12 by 12 cells from the bytes, a path across it and a
// flow field over a region of it, repaired once.
static void Grid(Reader* r, const mnavQueryFilter* filter)
{
    static mnavAreaType areas[144];
    int32_t width = Byte(r) % 13;
    int32_t height = Byte(r) % 13;
    for (int32_t i = 0; i < 144; ++i)
    {
        uint8_t pick = Byte(r);
        areas[i] = (mnavAreaType)(pick % 5 == 0 ? 0 : 1 + pick % 3);
    }
    mnavGrid grid = {areas, width, height, Byte(r) % 8 == 0 ? (float)Raw(r) : 0.5f};
    mnavCell a = {(int32_t)(Byte(r) % 14) - 1, (int32_t)(Byte(r) % 14) - 1};
    mnavCell b = {(int32_t)(Byte(r) % 14) - 1, (int32_t)(Byte(r) % 14) - 1};
    mnavGridPath path;
    mnavResult result = mnavFindGridPath(s_query, &grid, filter, a, b, &path);
    Expect(Typed(result) && (result != mnav_success || path.cellCount >= 0));
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    def.allocator = CountingAllocator();
    def.cells = 64;
    mnavFlowField* field = nullptr;
    Expect(mnavCreateFlowField(&def, &field) == mnav_success);
    mnavFlowRegion region = {Byte(r) % 6, Byte(r) % 6, Byte(r) % 10, Byte(r) % 10};
    mnavCell goals[2] = {a, b};
    result =
        mnavBeginFlowField(field, &grid, filter, Byte(r) % 2 == 0 ? &region : nullptr, goals, 2);
    Expect(Typed(result));
    bool ended = false;
    for (int32_t k = 0; k < 200 && result == mnav_success && !ended; ++k)
    {
        result = mnavContinueFlowField(field, &grid, 1 + Byte(r) % 16, &ended);
    }
    if (ended)
    {
        mnavCell changed = {Byte(r) % 12, Byte(r) % 12};
        areas[changed.y * 12 + changed.x] = (mnavAreaType)(Byte(r) % 3);
        result = mnavUpdateFlowField(field, &grid, &b, 1, &changed, 1);
        Expect(Typed(result));
        for (int32_t k = 0; k < 400 && result == mnav_success && !ended; ++k)
        {
            result = mnavContinueFlowField(field, &grid, 1 + Byte(r) % 16, &ended);
        }
    }
    Expect(Typed(result));
    mnavDestroyFlowField(field);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_navmesh == nullptr)
    {
        Load();
    }
    Reader r = {data, size, 0};
    mnavQueryFilter filter = Filter(&r);
    for (int32_t n = 0; n < 24 && r.at < r.size; ++n)
    {
        uint8_t op = Byte(&r) % 12;
        bool none = Byte(&r) % 8 == 0;
        const mnavQueryFilter* f = none ? nullptr : &filter;
        if (op < 2)
        {
            FindPath(&r, f, op == 1);
        }
        else if (op < 8)
        {
            Spatial(&r, f, (uint8_t)(op - 2));
        }
        else if (op == 8)
        {
            Boxes(&r, f);
        }
        else if (op == 9)
        {
            Corridor(&r, f);
        }
        else
        {
            Grid(&r, f);
        }
    }
    Expect(s_held == s_baseline);
    return 0;
}
