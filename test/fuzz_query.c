// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes query inputs: a run of queries read from the bytes,
// on the test world with one tile left unloaded and a link: paths, A* and
// exact, walls, points, boxes and radii from a range around the world or
// raw bits, polygon ids found or made up, filters with costs out of range,
// seeds, budgets, corridors, and grids with paths and flow fields over
// them. Each query must end in a typed status, give finite results when it
// succeeds, and keep its counts within its limits; the query contexts give
// back all they took.

#include "counting_allocator.h"
#include "defs_for.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <float.h>
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

// A value at the edge of the query range: its bound, the next double past
// it, the largest double, or a power of two from 2^333 to 2^1023, either
// sign.
static double Edge(Reader* r)
{
    uint8_t pick = Byte(r);
    double v = pick % 4 == 0   ? MNAV_MAX_QUERY_DISTANCE
               : pick % 4 == 1 ? nextafter(MNAV_MAX_QUERY_DISTANCE, (double)INFINITY)
               : pick % 4 == 2 ? DBL_MAX
                               : ldexp(1.0, 333 + Byte(r) % 691);
    return (pick & 4) != 0 ? -v : v;
}

// A coordinate: one time in sixteen raw bits, one time in sixteen at the
// edge of the query range, otherwise a quarter meter step from -4 to
// 67.75 m.
static double Coordinate(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? Raw(r) : pick % 16 == 1 ? Edge(r) : (double)Byte(r) * 0.28125 - 4.0;
}

static mnavPos3 Point(Reader* r)
{
    return (mnavPos3){Coordinate(r), Coordinate(r) * 0.0625, Coordinate(r)};
}

// A length from 0 to 16 m, or raw bits, or at the edge of the query range.
static double Length(Reader* r)
{
    uint8_t pick = Byte(r);
    return pick % 16 == 0 ? Raw(r) : pick % 16 == 1 ? fabs(Edge(r)) : (double)(Byte(r) % 64) * 0.25;
}

static bool Finite(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

// Whether a finite value lies past the query range; the world's origin is
// (0, 0, 0).
static bool Beyond(double v)
{
    return isfinite(v) && fabs(v) > MNAV_MAX_QUERY_DISTANCE;
}

// Whether a finite point lies past the query range on an axis, or on the
// ground alone for an end whose height is never read.
static bool Past(mnavPos3 p)
{
    return Finite(p) && (Beyond(p.x) || Beyond(p.y) || Beyond(p.z));
}

static bool PastGround(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.z) && (Beyond(p.x) || Beyond(p.z));
}

// A call given finite input of which some lies past the query range is
// refused with mnav_errorLimit, before anything else it would refuse.
static void ExpectRange(bool finite, bool past, mnavResult result)
{
    Expect(!(finite && past) || result == mnav_errorLimit);
}

static bool Typed(mnavResult result)
{
    // Every status the library defines has a name.
    return strcmp(mnavGetResultName(result), "unknown result") != 0;
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
    LoadWorldTiles();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    Expect(mnavCreateNavmesh(&meshDef, &s_navmesh).result == mnav_success);
    for (int32_t t = 0; t < 3; ++t)
    {
        Expect(mnavNavmesh_StageTile(s_navmesh, s_tiles[t], s_sizes[t]).result == mnav_success);
    }
    const mnavLinkDef link = {{5, 0, 5}, {40, 0, 5}, 1.0f, 2.0f, mnav_linkJump, true, 0.0f};
    const mnavLinkDef edge = {{20, 0, 30}, {20, 0, 50}, 1.0f, 3.0f, mnav_linkClimb, true, 6.0f};
    mnavLinkId id;
    Expect(mnavStageLink(s_navmesh, &edge, &id) == mnav_success);
    Expect(mnavStageLink(s_navmesh, &link, &id) == mnav_success &&
           mnavNavmesh_Commit(s_navmesh) == mnav_success);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.allocator = CountingAllocator();
    queryDef.limits.nodes = 512;
    Expect(mnavCreateQuery(&queryDef, &s_query) == mnav_success);
    for (int32_t i = 0; i < 8; ++i)
    {
        mnavNearest nearest;
        mnavPos3 at = {4.0 + 7.0 * i, 0.0, 3.0 + 3.0 * i};
        Expect(mnavNavmesh_FindNearest(s_navmesh, at, (mnavVec3){2, 2, 2}, nullptr, &nearest) ==
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
    bool finite = Finite(start) && Finite(end);
    bool past = Past(start) || Past(end);
    if (!sliced)
    {
        mnavResult found = mnavQuery_FindPath(s_query, s_navmesh, a, start, b, end, filter, &path);
        ExpectRange(finite, past, found);
        CheckPath(found, &path);
        found = mnavQuery_FindShortestPath(s_query, s_navmesh, a, start, b, end, filter, &path);
        ExpectRange(finite, past, found);
        CheckPath(found, &path);
        return;
    }
    mnavResult result = mnavQuery_BeginPath(s_query, s_navmesh, a, start, b, end, filter);
    ExpectRange(finite, past, result);
    Expect(Typed(result));
    int32_t budget = 1 + Byte(r) % 64;
    bool ended = false;
    for (int32_t k = 0; k < 64 && result == mnav_success && !ended; ++k)
    {
        result = mnavQuery_ContinuePath(s_query, s_navmesh, budget, &ended);
    }
    Expect(Typed(result));
    CheckPath(mnavQuery_FinishPath(s_query, s_navmesh, &path), &path);
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
        result = mnavQuery_Raycast(s_query, s_navmesh, id, a, b, filter, &ray);
        ExpectRange(Finite(a) && isfinite(b.x) && isfinite(b.z), Past(a) || PastGround(b), result);
        Expect(result != mnav_success || (ray.polygonCount >= 0 && isfinite(ray.normal.x)));
    }
    else if (op == 1)
    {
        mnavMove move;
        result = mnavQuery_MoveAlongSurface(s_query, s_navmesh, id, a, b, filter, &move);
        ExpectRange(Finite(a) && isfinite(b.x) && isfinite(b.z), Past(a) || PastGround(b), result);
        Expect(result != mnav_success || Finite(move.point));
    }
    else if (op == 2 && Byte(r) % 2 == 0)
    {
        mnavNearestWall wall;
        double radius = Length(r);
        result = mnavQuery_FindWallDistance(s_query, s_navmesh, id, a, radius, filter, &wall);
        ExpectRange(Finite(a) && isfinite(radius) && radius >= 0.0, Past(a) || Beyond(radius),
                    result);
        Expect(result != mnav_success || !wall.found || isfinite(wall.wall.distance));
    }
    else if (op == 2)
    {
        mnavWall walls[16];
        int32_t capacity = Byte(r) % 17;
        mnavWallsFound found;
        double radius = Length(r);
        result =
            mnavQuery_FindWalls(s_query, s_navmesh, id, a, radius, filter, walls, capacity, &found);
        ExpectRange(Finite(a) && isfinite(radius) && radius >= 0.0, Past(a) || Beyond(radius),
                    result);
        bool listed = result == mnav_success || result == mnav_errorCapacity;
        Expect(!listed || (found.wallCount >= 0 &&
                           (result == mnav_errorCapacity) == (found.wallCount > capacity)));
        for (int32_t k = 0; listed && k < found.wallCount && k < capacity; ++k)
        {
            Expect(Finite(walls[k].start) && Finite(walls[k].end) && isfinite(walls[k].distance) &&
                   (k == 0 || walls[k - 1].distance <= walls[k].distance));
        }
    }
    else if (op == 3)
    {
        double height = 0.0;
        result = mnavPolygon_GetHeight(s_navmesh, id, a.x, a.z, &height);
        ExpectRange(isfinite(a.x) && isfinite(a.z), PastGround(a), result);
        Expect(result != mnav_success || isfinite(height));
    }
    else if (op == 4)
    {
        mnavRandomPoint point;
        double bits = Raw(r);
        uint64_t seed;
        memcpy(&seed, &bits, sizeof(seed));
        if (Byte(r) % 2 == 0)
        {
            result = mnavNavmesh_FindRandomPoint(s_navmesh, seed, filter, &point);
        }
        else
        {
            double radius = Length(r);
            result = mnavQuery_FindRandomPointAround(s_query, s_navmesh, id, a, radius, seed,
                                                     filter, &point);
            ExpectRange(Finite(a) && isfinite(radius) && radius >= 0.0, Past(a) || Beyond(radius),
                        result);
        }
        Expect(result != mnav_success || Finite(point.point));
    }
    else
    {
        mnavPathEnd end;
        result = mnavQuery_CheckReachable(s_query, s_navmesh, id, a, Id(r), b, filter, &end);
        ExpectRange(Finite(a) && Finite(b), Past(a) || Past(b), result);
    }
    Expect(Typed(result));
}

// Whether half extents are finite and at least 0.
static bool Good(mnavVec3 half)
{
    return isfinite(half.x) && isfinite(half.y) && isfinite(half.z) && half.x >= 0.0f &&
           half.y >= 0.0f && half.z >= 0.0f;
}

static void Boxes(Reader* r, const mnavQueryFilter* filter)
{
    mnavPos3 center = Point(r);
    mnavVec3 half = {Length(r), Length(r), Length(r)};
    if (Byte(r) % 2 == 0)
    {
        mnavNearest nearest;
        mnavResult result = mnavNavmesh_FindNearest(s_navmesh, center, half, filter, &nearest);
        ExpectRange(Finite(center) && Good(half), Past(center), result);
        Expect(Typed(result) && (result != mnav_success || Finite(nearest.point)));
        return;
    }
    mnavPolygonId polygons[16];
    int32_t capacity = Byte(r) % 17;
    mnavFound found;
    mnavResult result =
        mnavNavmesh_FindPolygons(s_navmesh, center, half, filter, polygons, capacity, &found);
    ExpectRange(Finite(center) && Good(half), Past(center), result);
    Expect(Typed(result) && (result != mnav_success || found.polygonCount <= capacity));
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
        mnavQuery_FindCorridorAreas(s_query, corridor, s_navmesh, &corners, runs, capacity, &count);
    if (result != mnav_success && result != mnav_errorCapacity)
    {
        return result;
    }
    Expect(count >= 1 && (result == mnav_success) == (count <= capacity));
    int32_t again = -1;
    Expect(mnavQuery_FindCorridorAreas(s_query, corridor, s_navmesh, &corners, runs, 16, &again) ==
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
    mnavResult result = mnavQuery_FindCorridorCorners(s_query, corridor, s_navmesh, &corners);
    if (result != mnav_success)
    {
        return result;
    }
    mnavSteerDef def = mnavDefaultSteerDef();
    def.maxSpeed = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.25);
    def.slowDistance = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.1);
    def.arriveDistance = (float)(Byte(r) % 4 == 0 ? Raw(r) : Byte(r) * 0.01);
    def.enableTurnAnticipation = Byte(r) % 2 == 0;
    // A NaN or a value below 0 is wrong; an infinity passes the bound.
    bool valid = def.maxSpeed >= 0.0f && def.slowDistance >= 0.0f && def.arriveDistance >= 0.0f;
    bool within = def.maxSpeed <= MNAV_MAX_STEER_SPEED &&
                  def.slowDistance <= MNAV_MAX_STEER_DISTANCE &&
                  def.arriveDistance <= MNAV_MAX_STEER_DISTANCE;
    mnavSteering steering;
    result = mnavSteer(&corners, &def, &steering);
    Expect(result == (!valid ? mnav_errorInvalid : within ? mnav_success : mnav_errorLimit));
    if (result != mnav_success)
    {
        return result;
    }
    // The velocity is rounded to binary32, to nearest: its speed may pass
    // the def's by that rounding.
    mnavPos3 velocity = {(double)steering.velocity.x, (double)steering.velocity.y,
                         (double)steering.velocity.z};
    double speed = hypot(velocity.x, velocity.z);
    Expect(Finite(velocity) && velocity.y == 0.0 &&
           speed <= (double)def.maxSpeed * (1.0 + 0x1p-22) + 0x1p-148);
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
    result = mnavQuery_FindPath(s_query, s_navmesh, id, corridor.position, Id(r), Point(r), filter,
                                &path);
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
        // A corridor past the range is refused by every call that takes
        // the navmesh; a wanted or toward point past it on the ground by
        // the moves and the shortcut.
        bool far = Past(corridor.position) || Past(corridor.target);
        mnavPos3 wanted = Point(r);
        bool toward = op == 0 || op == 1 || op == 4;
        result =
            op == 0 ? mnavQuery_MoveCorridor(s_query, &corridor, s_navmesh, wanted, filter, &move)
            : op == 1
                ? mnavQuery_MoveCorridorTarget(s_query, &corridor, s_navmesh, wanted, filter, &move)
            : op == 2 ? mnavQuery_FindCorridorCorners(s_query, &corridor, s_navmesh, &corners)
            : op == 3 ? mnavNavmesh_CheckCorridor(s_navmesh, &corridor, filter, &valid)
            : op == 4 ? mnavQuery_ShortcutCorridor(s_query, &corridor, s_navmesh, wanted, filter,
                                                   &shortened)
            : op == 5 ? mnavQuery_ReplanCorridor(s_query, &corridor, s_navmesh, (mnavVec3){1, 1, 1},
                                                 filter, &path)
            : op == 6 ? Areas(r, &corridor)
                      : Steer(r, &corridor);
        if (op < 7)
        {
            ExpectRange(true, far, result);
        }
        if (toward && !far)
        {
            ExpectRange(isfinite(wanted.x) && isfinite(wanted.z), PastGround(wanted), result);
        }
        Expect(Typed(result) && corridor.polygonCount >= 1 &&
               corridor.polygonCount <= corridor.capacity);
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
    mnavResult result = mnavQuery_FindGridPath(s_query, &grid, a, b, filter, &path);
    Expect(Typed(result) && (result != mnav_success || path.cellCount >= 0));
    mnavGridFlowDef def = mnavDefaultGridFlowDef();
    def.allocator = CountingAllocator();
    def.cells = 64;
    mnavGridFlow* field = nullptr;
    Expect(mnavCreateGridFlow(&def, &field) == mnav_success);
    mnavCellRange region = {Byte(r) % 6, Byte(r) % 6, Byte(r) % 10, Byte(r) % 10};
    mnavCell goals[2] = {a, b};
    result =
        mnavGridFlow_Begin(field, &grid, Byte(r) % 2 == 0 ? &region : nullptr, goals, 2, filter);
    Expect(Typed(result));
    bool ended = false;
    for (int32_t k = 0; k < 200 && result == mnav_success && !ended; ++k)
    {
        result = mnavGridFlow_Continue(field, &grid, 1 + Byte(r) % 16, &ended);
    }
    if (ended)
    {
        mnavCell changed = {Byte(r) % 12, Byte(r) % 12};
        areas[changed.y * 12 + changed.x] = (mnavAreaType)(Byte(r) % 3);
        result = mnavGridFlow_Update(field, &grid, &b, 1, &changed, 1);
        Expect(Typed(result));
        for (int32_t k = 0; k < 400 && result == mnav_success && !ended; ++k)
        {
            result = mnavGridFlow_Continue(field, &grid, 1 + Byte(r) % 16, &ended);
        }
    }
    Expect(Typed(result));
    mnavDestroyGridFlow(field);
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
