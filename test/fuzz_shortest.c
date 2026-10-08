// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the exact shortest-path search (mnav-0005) against the A* search:
// two tiles of 5 m squares on a 4 by 4 grid each, meeting at the tiles'
// border, some of another area, joined by up to two off-mesh links, read
// from the bytes; then pairs of points on them. Where the A* search finds
// a way the exact search must find one, never dearer than the A* path as
// walked; where it finds none, neither may the exact search; and no turn
// of the exact path may be cut short by a raycast from the point before it
// to the point after it. The contexts give back all they took.

#include "counting_allocator.h"
#include "hand_tile.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

enum
{
    BLOCK = 20,
    LINKS = 2,
    PAIRS = 6,
    NODES = 4096
};

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

static uint8_t s_bytes[2][8192];

// A tile's blocks: the first tile's run from x 48 to its eastern side, the
// second's from its western side, both from z 24, clear of other sides.
static int32_t Squares(Reader* r, int32_t place, HandSquare* squares)
{
    uint16_t filled = (uint16_t)(Byte(r) | Byte(r) << 8);
    uint8_t other = Byte(r);
    int32_t count = 0;
    for (int32_t k = 0; k < 16; ++k)
    {
        if ((filled >> k & 1u) == 0)
        {
            continue;
        }
        int32_t x0 = (place == 0 ? 48 : 0) + BLOCK * (k % 4);
        int32_t z0 = 24 + BLOCK * (k / 4);
        mnavAreaType area = (other >> (k % 8) & 1u) != 0 && k >= 8 ? 2 : 0;
        squares[count++] = (HandSquare){x0, z0, x0 + BLOCK, z0 + BLOCK, {0}, area};
    }
    return count;
}

// A point in one of the squares, in world meters.
static mnavPos3 PointIn(Reader* r, const HandSquare* squares, int32_t count, int32_t place)
{
    const HandSquare* q = &squares[Byte(r) % count];
    double x = (q->x0 + 1 + Byte(r) % (BLOCK - 1)) * 0.25 + (Byte(r) % 4) * 0.05;
    double z = (q->z0 + 1 + Byte(r) % (BLOCK - 1)) * 0.25 + (Byte(r) % 4) * 0.05;
    return (mnavPos3){place * 32.0 + x, 0.0, z};
}

typedef struct Layout
{
    HandSquare squares[2][HAND_SQUARES];
    int32_t counts[2];
    float linkCosts[8];
} Layout;

static mnavPos3 AnyPoint(Reader* r, const Layout* layout)
{
    int32_t place = layout->counts[0] == 0 ? 1 : (layout->counts[1] == 0 ? 0 : Byte(r) % 2);
    return PointIn(r, layout->squares[place], layout->counts[place], place);
}

// Stages up to two links between points of the squares, at costs from
// 0.5 to 8, one or two way.
static void Links(Reader* r, mnavNavmesh* navmesh, Layout* layout)
{
    int32_t links = Byte(r) % (LINKS + 1);
    for (int32_t i = 0; i < links; ++i)
    {
        float cost = 0.5f * (float)(1 + Byte(r) % 16);
        mnavLinkDef def = {AnyPoint(r, layout), AnyPoint(r, layout), 0.5f, cost,
                           mnav_linkJump,       Byte(r) % 2 == 0,    0.0f};
        mnavLinkId id = {0, 0};
        if (mnavStageLink(navmesh, &def, &id) == mnav_success && id.slot < 8)
        {
            layout->linkCosts[id.slot] = cost;
        }
    }
}

static double Flat(mnavPos3 a, mnavPos3 b)
{
    return hypot(a.x - b.x, a.z - b.z);
}

// A path's cost as walked: its legs on the ground at a cost of 1, each
// link crossed at its own cost.
static double Walked(const mnavPath* path, const Layout* layout)
{
    double cost = 0.0;
    int32_t next = 0;
    for (int32_t i = 1; i < path->pointCount; ++i)
    {
        bool link = next < path->linkCount && path->links[next].point == i - 1;
        cost += link ? (double)layout->linkCosts[path->links[next].link.slot % 8]
                     : Flat(path->points[i - 1], path->points[i]);
        next += link ? 1 : 0;
    }
    return cost;
}

// Whether point i of a path is a link's takeoff or landing point.
static bool AtLink(const mnavPath* path, int32_t i)
{
    for (int32_t k = 0; k < path->linkCount; ++k)
    {
        if (path->links[k].point == i || path->links[k].point + 1 == i)
        {
            return true;
        }
    }
    return false;
}

// No turn of the path is cut short by a raycast from the point before it
// to the point after it.
static void ExpectTaut(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                       const mnavPath* path)
{
    for (int32_t i = 1; i + 1 < path->pointCount; ++i)
    {
        mnavPos3 a = path->points[i - 1];
        mnavPos3 c = path->points[i + 1];
        double saved = Flat(a, path->points[i]) + Flat(path->points[i], c) - Flat(a, c);
        mnavNearest from;
        if (AtLink(path, i - 1) || AtLink(path, i) || AtLink(path, i + 1) || saved < 1e-6 ||
            mnavFindNearest(navmesh, filter, a, (mnavVec3){0.01f, 1.0f, 0.01f}, &from) !=
                mnav_success ||
            from.polygon.slot == 0)
        {
            continue;
        }
        mnavRay ray;
        Expect(mnavRaycast(query, navmesh, filter, from.polygon, from.point, c, &ray) ==
               mnav_success);
        Expect(ray.end != mnav_rayReached);
    }
}

static void Compare(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                    const Layout* layout, mnavPos3 p, mnavPos3 q)
{
    mnavNearest a;
    mnavNearest b;
    mnavVec3 box = {0.1f, 1.0f, 0.1f};
    if (mnavFindNearest(navmesh, filter, p, box, &a) != mnav_success ||
        mnavFindNearest(navmesh, filter, q, box, &b) != mnav_success || a.polygon.slot == 0 ||
        b.polygon.slot == 0)
    {
        return;
    }
    mnavPath plain;
    Expect(mnavFindPath(query, navmesh, filter, a.polygon, a.point, b.polygon, b.point, &plain) ==
           mnav_success);
    mnavPathEnd plainEnd = plain.end;
    double plainCost = plainEnd == mnav_pathFound ? Walked(&plain, layout) : 0.0;
    mnavPath exact;
    Expect(mnavFindShortestPath(query, navmesh, filter, a.polygon, a.point, b.polygon, b.point,
                                &exact) == mnav_success);
    Expect(exact.end == plainEnd);
    for (int32_t i = 0; i < exact.pointCount; ++i)
    {
        Expect(isfinite(exact.points[i].x) && isfinite(exact.points[i].z));
    }
    if (exact.end != mnav_pathFound)
    {
        return;
    }
    double exactCost = Walked(&exact, layout);
    Expect(fabs(exactCost - exact.cost) <= 1e-9 * (1.0 + exactCost));
    Expect(exactCost <= plainCost + 1e-9 * (1.0 + plainCost));
    ExpectTaut(query, navmesh, filter, &exact);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    Reader r = {data, size, 0};
    Layout layout = {0};
    layout.counts[0] = Squares(&r, 0, layout.squares[0]);
    layout.counts[1] = Squares(&r, 1, layout.squares[1]);
    if (layout.counts[0] + layout.counts[1] == 0)
    {
        return 0;
    }
    size_t held = s_held;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    mnavNavmesh* navmesh = nullptr;
    Expect(mnavCreateNavmesh(&def, &navmesh).result == mnav_success);
    for (int32_t place = 0; place < 2; ++place)
    {
        if (layout.counts[place] > 0)
        {
            size_t bytes =
                HandTileBytes(s_bytes[place], place, layout.squares[place], layout.counts[place]);
            Expect(mnavStageTile(navmesh, s_bytes[place], bytes).result == mnav_success);
        }
    }
    Expect(mnavCommit(navmesh) == mnav_success);
    Links(&r, navmesh, &layout);
    Expect(mnavCommit(navmesh) == mnav_success);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.allocator = CountingAllocator();
    queryDef.limits.nodes = NODES;
    mnavQuery* query = nullptr;
    Expect(mnavCreateQuery(&queryDef, &query) == mnav_success);
    mnavQueryFilter filter = mnavDefaultQueryFilter();
    filter.areas &= Byte(&r) % 4 == 0 ? ~((uint64_t)1 << 2) : ~(uint64_t)0;
    for (int32_t i = 0; i < PAIRS && r.at < r.size; ++i)
    {
        mnavPos3 p = AnyPoint(&r, &layout);
        mnavPos3 q = AnyPoint(&r, &layout);
        Compare(query, navmesh, &filter, &layout, p, q);
    }
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    Expect(s_held == held);
    return 0;
}
