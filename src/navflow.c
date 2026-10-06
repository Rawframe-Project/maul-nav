// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields over the navmesh (mnav-0013): Dijkstra's search backward from
// the goals over polygons, as Detour's findPolysAroundCircle searches
// forward from one. Polygons are numbered slot by slot, so the heap's order
// by cost, then number, is the order by cost, then slot and polygon. The
// off-mesh links are sorted once per build by the polygon they land on, so
// the search can step back across them.

#include "maul-nav/navflow.h"

#include "allocator.h"
#include "navmesh.h"
#include "offmesh.h"
#include "query_filter.h"
#include "sort.h"

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>

// Marks a def built by mnavDefaultNavFlowDef.
#define NAVFLOW_DEF_COOKIE 0x4E41564Eu

// A polygon's place in the heap when it is not in it: never reached, or
// done.
#define NOT_OPEN (-1)
#define DONE     (-2)

// What a polygon holds: its cost, place, next polygon, the portal to it and
// the link slot taken or -1.
typedef struct Way
{
    double cost;
    mnavPos3 place;
    mnavPos3 left;
    mnavPos3 right;
    int32_t next;
    int32_t link;
} Way;

struct mnavNavFlow
{
    mnavMemory memory;
    mnavNavFlowDef def;
    Way* ways;
    int32_t* places;
    int32_t* heap;
    int32_t heapCount;
    // Each slot's first polygon number, slotCount + 1 of them.
    int32_t* firsts;
    int32_t firstCapacity;
    // The attachments by landing polygon: its number above 32 bits, the
    // attachment's index below.
    uint64_t* landings;
    int32_t landingCount;
    int32_t landingCapacity;
    // The navmesh built on and its commits then; NULL with nothing built.
    const mnavNavmesh* navmesh;
    uint64_t commits;
    int32_t polygonCount;
    mnavQueryFilter filter;
};

mnavNavFlowDef mnavDefaultNavFlowDef(void)
{
    return (mnavNavFlowDef){NAVFLOW_DEF_COOKIE, {0}, 65536};
}

mnavResult mnavCreateNavFlow(const mnavNavFlowDef* def, mnavNavFlow** fieldOut)
{
    if (fieldOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *fieldOut = nullptr;
    if (def == nullptr || def->cookie != NAVFLOW_DEF_COOKIE ||
        (def->allocator.alloc == nullptr) != (def->allocator.free == nullptr))
    {
        return mnav_errorInvalid;
    }
    if (def->polygons < 1 || def->polygons > MNAV_MAX_NAVFLOW_POLYGONS)
    {
        return mnav_errorRange;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavNavFlow* f = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavNavFlow), alignof(mnavNavFlow), (void**)&f);
    if (result != mnav_success)
    {
        return result;
    }
    *f = (mnavNavFlow){0};
    f->memory = memory;
    f->def = *def;
    size_t n = (size_t)def->polygons;
    result = mnavAllocate(&f->memory, n, sizeof(Way), alignof(Way), (void**)&f->ways);
    if (result == mnav_success)
    {
        result = mnavAllocate(&f->memory, n, sizeof(int32_t), alignof(int32_t), (void**)&f->places);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&f->memory, n, sizeof(int32_t), alignof(int32_t), (void**)&f->heap);
    }
    if (result != mnav_success)
    {
        mnavDestroyNavFlow(f);
        return result;
    }
    *fieldOut = f;
    return mnav_success;
}

void mnavDestroyNavFlow(mnavNavFlow* field)
{
    if (field == nullptr)
    {
        return;
    }
    mnavMemory memory = field->memory;
    size_t n = (size_t)field->def.polygons;
    mnavRelease(&memory, field->ways, n, sizeof(Way), alignof(Way));
    mnavRelease(&memory, field->places, n, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, field->heap, n, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, field->firsts, (size_t)field->firstCapacity, sizeof(int32_t),
                alignof(int32_t));
    mnavRelease(&memory, field->landings, (size_t)field->landingCapacity, sizeof(uint64_t),
                alignof(uint64_t));
    mnavRelease(&memory, field, 1, sizeof(mnavNavFlow), alignof(mnavNavFlow));
}

// The heap, by cost, then by number.
static bool Sooner(const mnavNavFlow* f, int32_t a, int32_t b)
{
    return f->ways[a].cost != f->ways[b].cost ? f->ways[a].cost < f->ways[b].cost : a < b;
}

static void Place(mnavNavFlow* f, int32_t at, int32_t polygon)
{
    f->heap[at] = polygon;
    f->places[polygon] = at;
}

static void SiftUp(mnavNavFlow* f, int32_t at)
{
    int32_t polygon = f->heap[at];
    while (at > 0)
    {
        int32_t up = (at - 1) / 2;
        if (!Sooner(f, polygon, f->heap[up]))
        {
            break;
        }
        Place(f, at, f->heap[up]);
        at = up;
    }
    Place(f, at, polygon);
}

static int32_t Pop(mnavNavFlow* f)
{
    int32_t top = f->heap[0];
    int32_t last = f->heap[--f->heapCount];
    int32_t at = 0;
    while (f->heapCount > 0)
    {
        int32_t child = 2 * at + 1;
        if (child >= f->heapCount)
        {
            break;
        }
        if (child + 1 < f->heapCount && Sooner(f, f->heap[child + 1], f->heap[child]))
        {
            child += 1;
        }
        if (!Sooner(f, f->heap[child], last))
        {
            break;
        }
        Place(f, at, f->heap[child]);
        at = child;
    }
    if (f->heapCount > 0)
    {
        Place(f, at, last);
    }
    f->places[top] = DONE;
    return top;
}

// Gives polygon p a way through polygon q at a cost, standing at place.
static void Lower(mnavNavFlow* f, int32_t p, double cost, const Way* way)
{
    f->ways[p] = *way;
    f->ways[p].cost = cost;
    if (f->places[p] == NOT_OPEN)
    {
        Place(f, f->heapCount++, p);
    }
    SiftUp(f, f->places[p]);
}

static double Distance(mnavPos3 a, mnavPos3 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = b.z - a.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Offers polygon p, not yet done, a way into polygon q through the portal
// from left to right: it stands at the portal's midpoint.
static void Offer(mnavNavFlow* f, int32_t p, int32_t q, mnavPos3 left, mnavPos3 right,
                  double areaCost)
{
    if (f->places[p] == DONE)
    {
        return;
    }
    mnavPos3 middle = {(left.x + right.x) * 0.5, (left.y + right.y) * 0.5,
                       (left.z + right.z) * 0.5};
    double cost = f->ways[q].cost + Distance(middle, f->ways[q].place) * areaCost;
    if (cost < f->ways[p].cost)
    {
        Way way = {cost, middle, left, right, q, -1};
        Lower(f, p, cost, &way);
    }
}

// The polygon a number names: its slot and index.
static void Polygon(const mnavNavFlow* f, int32_t number, int32_t* slot, int32_t* polygon)
{
    int32_t lo = 0;
    int32_t hi = f->navmesh->slotCount;
    while (hi - lo > 1)
    {
        int32_t middle = lo + (hi - lo) / 2;
        if (f->firsts[middle] <= number)
        {
            lo = middle;
        }
        else
        {
            hi = middle;
        }
    }
    *slot = lo;
    *polygon = number - f->firsts[lo];
}

static bool Usable(const mnavNavFlow* f, const mnavTile* tile, int32_t polygon)
{
    return mnavIncludes(&f->filter, tile->mesh.polygons[polygon].area);
}

// Steps back from polygon q over its inner edges and tile links.
static void ExpandEdges(mnavNavFlow* f, int32_t q, int32_t slot, int32_t polygon)
{
    const mnavNavmesh* navmesh = f->navmesh;
    const mnavSlot* s = &navmesh->slots[slot];
    const mnavTile* tile = s->tile;
    const mnavPolygon* poly = &tile->mesh.polygons[polygon];
    // Every polygon has three corners or more; none has no edges to cross.
    if (poly->count == 0)
    {
        return;
    }
    double areaCost = (double)f->filter.costs[poly->area];
    mnavFrame frame = mnavFrameOf(navmesh, s->x, s->z);
    for (int32_t j = 0; j < poly->count; ++j)
    {
        int32_t next = poly->neighbors[j];
        if (next == MNAV_NO_INDEX || !Usable(f, tile, next))
        {
            continue;
        }
        // The portal as the agent crossing from the neighbour sees it.
        mnavPos3 a = mnavVertexWorld(&frame, &tile->mesh.vertices[poly->vertices[j]]);
        mnavPos3 b =
            mnavVertexWorld(&frame, &tile->mesh.vertices[poly->vertices[(j + 1) % poly->count]]);
        Offer(f, f->firsts[slot] + next, q, b, a, areaCost);
    }
    for (int32_t l = tile->firstLink[polygon]; l < tile->firstLink[polygon + 1]; ++l)
    {
        const mnavLink* link = &tile->links[l];
        const mnavTile* beyond = navmesh->slots[link->target.slot - 1].tile;
        if (!Usable(f, beyond, (int32_t)link->target.polygon))
        {
            continue;
        }
        const mnavMeshVertex* va = &tile->mesh.vertices[poly->vertices[link->edge]];
        const mnavMeshVertex* vb =
            &tile->mesh.vertices[poly->vertices[(link->edge + 1) % poly->count]];
        bool alongZ = link->side == 1 || link->side == 3;
        double au = alongZ ? va->z : va->x;
        double bu = alongZ ? vb->z : vb->x;
        // A tile-side edge runs along the side; one that does not has no
        // portal to give.
        if (bu == au)
        {
            continue;
        }
        mnavPos3 a = mnavVertexWorld(&frame, va);
        mnavPos3 b = mnavVertexWorld(&frame, vb);
        double tLow = ((double)link->low - au) / (bu - au);
        double tHigh = ((double)link->high - au) / (bu - au);
        mnavPos3 first = {a.x + tLow * (b.x - a.x), a.y + tLow * (b.y - a.y),
                          a.z + tLow * (b.z - a.z)};
        mnavPos3 second = {a.x + tHigh * (b.x - a.x), a.y + tHigh * (b.y - a.y),
                           a.z + tHigh * (b.z - a.z)};
        bool rising = bu > au;
        int32_t p = f->firsts[link->target.slot - 1] + (int32_t)link->target.polygon;
        Offer(f, p, q, rising ? second : first, rising ? first : second, areaCost);
    }
}

// Steps back from polygon q over the off-mesh links landing on it.
static void ExpandLinks(mnavNavFlow* f, int32_t q, double areaCost)
{
    const mnavNavmesh* navmesh = f->navmesh;
    // The first landing on q, by a binary search of the sorted keys.
    uint64_t low = (uint64_t)(uint32_t)q << 32;
    int32_t lo = 0;
    int32_t hi = f->landingCount;
    while (lo < hi)
    {
        int32_t middle = lo + (hi - lo) / 2;
        lo = f->landings[middle] < low ? middle + 1 : lo;
        hi = f->landings[middle] < low ? hi : middle;
    }
    for (int32_t i = lo; i < f->landingCount && (f->landings[i] >> 32) == (uint32_t)q; ++i)
    {
        int32_t index = (int32_t)(uint32_t)f->landings[i];
        mnavAttachment a = mnavAttachmentOf(navmesh->attachments[index]);
        const mnavOffLink* link = &navmesh->links[a.link];
        int32_t p = f->firsts[a.slot] + a.polygon;
        if (!mnavCrosses(&f->filter, link->def.kind) ||
            !Usable(f, navmesh->slots[a.slot].tile, a.polygon) || f->places[p] == DONE)
        {
            continue;
        }
        mnavPos3 takeoff = a.reverse ? link->state.end : link->state.start;
        mnavPos3 landing = a.reverse ? link->state.start : link->state.end;
        double cost = f->ways[q].cost + Distance(landing, f->ways[q].place) * areaCost +
                      (double)link->def.cost;
        if (cost < f->ways[p].cost)
        {
            Way way = {cost, takeoff, takeoff, takeoff, q, a.link};
            Lower(f, p, cost, &way);
        }
    }
}

// Numbers the polygons slot by slot and sorts the attachments by the
// polygon they land on.
static mnavResult Index(mnavNavFlow* f, const mnavNavmesh* navmesh)
{
    mnavResult result = mnavReserve(&f->memory, (void**)&f->firsts, &f->firstCapacity, 0,
                                    navmesh->slotCount + 1, sizeof(int32_t), alignof(int32_t));
    int64_t total = 0;
    for (int32_t s = 0; result == mnav_success && s < navmesh->slotCount; ++s)
    {
        f->firsts[s] = (int32_t)total;
        const mnavTile* tile = navmesh->slots[s].tile;
        total += tile != nullptr ? tile->mesh.polygonCount : 0;
        if (total > f->def.polygons)
        {
            result = mnav_errorLimit;
        }
    }
    // Twice the attachments: the second half is the sort's scratch.
    int32_t count = navmesh->attachmentCount;
    if (result == mnav_success)
    {
        f->firsts[navmesh->slotCount] = (int32_t)total;
        f->polygonCount = (int32_t)total;
        result = mnavReserve(&f->memory, (void**)&f->landings, &f->landingCapacity, 0, 2 * count,
                             sizeof(uint64_t), alignof(uint64_t));
    }
    if (result != mnav_success)
    {
        return result;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        mnavAttachment a = mnavAttachmentOf(navmesh->attachments[i]);
        const mnavLinkState* state = &navmesh->links[a.link].state;
        mnavPolygonId landing = a.reverse ? state->startPolygon : state->endPolygon;
        uint32_t p = (uint32_t)(f->firsts[landing.slot - 1] + (int32_t)landing.polygon);
        f->landings[i] = (uint64_t)p << 32 | (uint32_t)i;
    }
    f->landingCount =
        count > 0 ? (int32_t)mnavSortUnique(f->landings, f->landings + count, (size_t)count) : 0;
    return mnav_success;
}

// Opens every goal on a polygon the filter includes.
static mnavResult Seed(mnavNavFlow* f, const mnavNavFlowGoal* goals, int32_t goalCount)
{
    for (int32_t i = 0; i < goalCount; ++i)
    {
        const mnavPos3 p = goals[i].point;
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z))
        {
            return mnav_errorInvalid;
        }
        mnavResult result = mnavCheckPolygon(f->navmesh, goals[i].polygon);
        if (result != mnav_success)
        {
            return result;
        }
        int32_t slot = (int32_t)goals[i].polygon.slot - 1;
        int32_t polygon = (int32_t)goals[i].polygon.polygon;
        int32_t number = f->firsts[slot] + polygon;
        if (Usable(f, f->navmesh->slots[slot].tile, polygon) && f->ways[number].cost != 0.0)
        {
            Way way = {0.0, p, p, p, -1, -1};
            Lower(f, number, 0.0, &way);
        }
    }
    return mnav_success;
}

mnavResult mnavBuildNavFlow(mnavNavFlow* field, const mnavNavmesh* navmesh,
                            const mnavQueryFilter* filter, const mnavNavFlowGoal* goals,
                            int32_t goalCount)
{
    if (field == nullptr)
    {
        return mnav_errorInvalid;
    }
    field->navmesh = nullptr;
    const mnavQueryFilter* usable = nullptr;
    if (navmesh == nullptr || goalCount < 0 || (goalCount > 0 && goals == nullptr))
    {
        return mnav_errorInvalid;
    }
    mnavResult result = mnavCheckFilter(filter, &usable);
    result = result == mnav_success ? Index(field, navmesh) : result;
    if (result != mnav_success)
    {
        return result;
    }
    field->filter = *usable;
    field->navmesh = navmesh;
    for (int32_t p = 0; p < field->polygonCount; ++p)
    {
        field->ways[p] = (Way){(double)INFINITY, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, -1, -1};
        field->places[p] = NOT_OPEN;
    }
    field->heapCount = 0;
    result = Seed(field, goals, goalCount);
    while (result == mnav_success && field->heapCount > 0)
    {
        int32_t q = Pop(field);
        int32_t slot = 0;
        int32_t polygon = 0;
        Polygon(field, q, &slot, &polygon);
        const mnavTile* tile = navmesh->slots[slot].tile;
        ExpandEdges(field, q, slot, polygon);
        ExpandLinks(field, q, (double)field->filter.costs[tile->mesh.polygons[polygon].area]);
    }
    field->navmesh = result == mnav_success ? navmesh : nullptr;
    field->commits = navmesh->commits;
    return result;
}

mnavResult mnavNavFlowAt(const mnavNavFlow* field, const mnavNavmesh* navmesh,
                         mnavPolygonId polygon, mnavPolygonFlow* flowOut)
{
    if (field == nullptr || navmesh == nullptr || flowOut == nullptr || field->navmesh == nullptr)
    {
        return mnav_errorInvalid;
    }
    if (navmesh != field->navmesh || navmesh->commits != field->commits)
    {
        return mnav_errorStale;
    }
    mnavResult result = mnavCheckPolygon(navmesh, polygon);
    if (result != mnav_success)
    {
        return result;
    }
    const Way* way = &field->ways[field->firsts[polygon.slot - 1] + (int32_t)polygon.polygon];
    *flowOut = (mnavPolygonFlow){way->cost, {0, 0, 0}, way->left, way->right, {0, 0}};
    if (way->next >= 0)
    {
        int32_t slot = 0;
        int32_t index = 0;
        Polygon(field, way->next, &slot, &index);
        flowOut->next =
            (mnavPolygonId){(uint32_t)slot + 1, navmesh->slots[slot].generation, (uint32_t)index};
    }
    if (way->link >= 0)
    {
        int32_t parent = navmesh->links[way->link].parent;
        flowOut->link = (mnavLinkId){(uint32_t)parent + 1, navmesh->links[parent].generation};
    }
    return mnav_success;
}
