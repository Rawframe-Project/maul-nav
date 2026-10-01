// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The path search: A* over portals, with named limits (N25).

#include "allocator.h"
#include "navmesh.h"
#include "offmesh.h"
#include "polymesh.h"
#include "query.h"
#include "query_filter.h"
#include "raster.h"

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// Marks a def built by mnavDefaultQueryDef.
#define QUERY_DEF_COOKIE 0x4E415651u

mnavQueryDef mnavDefaultQueryDef(void)
{
    mnavQueryDef def = {0};
    def.cookie = QUERY_DEF_COOKIE;
    def.limits.nodes = 8192;
    def.limits.pathLength = 1000.0f;
    return def;
}

void mnavDestroyQuery(mnavQuery* query)
{
    if (query == nullptr)
    {
        return;
    }
    mnavMemory* memory = &query->memory;
    size_t nodes = (size_t)query->limits.nodes;
    mnavRelease(memory, query->links, nodes, sizeof(mnavPathLink), alignof(mnavPathLink));
    mnavRelease(memory, query->points, 2 * nodes + 1, sizeof(mnavPos3), alignof(mnavPos3));
    mnavRelease(memory, query->portals, 2 * nodes + 1, sizeof(mnavPortal), alignof(mnavPortal));
    mnavRelease(memory, query->corridor, nodes, sizeof(mnavPolygonId), alignof(mnavPolygonId));
    mnavRelease(memory, query->table, (size_t)query->tableMask + 1, sizeof(int32_t),
                alignof(int32_t));
    mnavRelease(memory, query->heap, nodes, sizeof(int32_t), alignof(int32_t));
    mnavRelease(memory, query->nodes, nodes, sizeof(mnavSearchNode), alignof(mnavSearchNode));
    mnavMemory last = *memory;
    mnavRelease(&last, query, 1, sizeof(mnavQuery), alignof(mnavQuery));
}

mnavResult mnavCreateQuery(const mnavQueryDef* def, mnavQuery** queryOut)
{
    if (queryOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *queryOut = nullptr;
    if (def == nullptr || def->cookie != QUERY_DEF_COOKIE)
    {
        return mnav_errorInvalid;
    }
    const mnavQueryLimits* limits = &def->limits;
    if (limits->nodes < 1 || limits->nodes > MNAV_MAX_QUERY_NODES ||
        !(limits->pathLength > 0.0f && limits->pathLength <= MNAV_MAX_PATH_LENGTH))
    {
        return mnav_errorRange;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavQuery* query = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavQuery), alignof(mnavQuery), (void**)&query);
    if (result != mnav_success)
    {
        return result;
    }
    *query = (mnavQuery){0};
    query->memory = memory;
    query->limits = *limits;
    // A table at least twice the nodes keeps probes short.
    uint32_t table = 2;
    while (table < 2u * (uint32_t)limits->nodes)
    {
        table *= 2;
    }
    query->tableMask = table - 1;
    size_t nodes = (size_t)limits->nodes;
    result = mnavAllocate(&query->memory, nodes, sizeof(mnavSearchNode), alignof(mnavSearchNode),
                          (void**)&query->nodes);
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, nodes, sizeof(int32_t), alignof(int32_t),
                              (void**)&query->heap);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, table, sizeof(int32_t), alignof(int32_t),
                              (void**)&query->table);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, nodes, sizeof(mnavPolygonId), alignof(mnavPolygonId),
                              (void**)&query->corridor);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, 2 * nodes + 1, sizeof(mnavPortal),
                              alignof(mnavPortal), (void**)&query->portals);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, 2 * nodes + 1, sizeof(mnavPos3), alignof(mnavPos3),
                              (void**)&query->points);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, nodes, sizeof(mnavPathLink), alignof(mnavPathLink),
                              (void**)&query->links);
    }
    if (result != mnav_success)
    {
        mnavDestroyQuery(query);
        return result;
    }
    *queryOut = query;
    return mnav_success;
}

static uint32_t Hash(int32_t slot, int32_t polygon, int32_t tag, int32_t low)
{
    uint64_t h = (uint64_t)(uint32_t)slot * 0x9E3779B97F4A7C15ull;
    h ^= (uint64_t)(uint32_t)polygon * 0xC2B2AE3D27D4EB4Full;
    h ^= ((uint64_t)(uint32_t)tag << 32 | (uint32_t)low) * 0x165667B19E3779F9ull;
    h ^= h >> 29;
    return (uint32_t)(h ^ (h >> 32));
}

// The table cell holding the node with a key, or the empty cell where it
// would go.
static uint32_t Find(const mnavQuery* query, int32_t slot, int32_t polygon, int32_t tag,
                     int32_t low)
{
    uint32_t cell = Hash(slot, polygon, tag, low) & query->tableMask;
    for (;;)
    {
        int32_t n = query->table[cell];
        if (n == MNAV_NO_NODE)
        {
            return cell;
        }
        const mnavSearchNode* node = &query->nodes[n];
        if (node->slot == slot && node->polygon == polygon && node->tag == tag && node->low == low)
        {
            return cell;
        }
        cell = (cell + 1) & query->tableMask;
    }
}

// Whether node a leaves the open list before node b: the lower total, then
// the one made first.
static bool Sooner(const mnavQuery* query, int32_t a, int32_t b)
{
    const mnavSearchNode* na = &query->nodes[a];
    const mnavSearchNode* nb = &query->nodes[b];
    double ta = na->cost + na->remaining;
    double tb = nb->cost + nb->remaining;
    return ta != tb ? ta < tb : a < b;
}

static void Place(mnavQuery* query, int32_t at, int32_t n)
{
    query->heap[at] = n;
    query->nodes[n].heap = at;
}

static void SiftUp(mnavQuery* query, int32_t at)
{
    int32_t n = query->heap[at];
    while (at > 0 && Sooner(query, n, query->heap[(at - 1) / 2]))
    {
        Place(query, at, query->heap[(at - 1) / 2]);
        at = (at - 1) / 2;
    }
    Place(query, at, n);
}

static int32_t Pop(mnavQuery* query)
{
    int32_t top = query->heap[0];
    query->nodes[top].heap = MNAV_NO_NODE;
    int32_t last = query->heap[--query->heapCount];
    int32_t at = 0;
    while (query->heapCount > 0)
    {
        int32_t child = 2 * at + 1;
        if (child >= query->heapCount)
        {
            break;
        }
        if (child + 1 < query->heapCount &&
            Sooner(query, query->heap[child + 1], query->heap[child]))
        {
            child += 1;
        }
        if (!Sooner(query, query->heap[child], last))
        {
            break;
        }
        Place(query, at, query->heap[child]);
        at = child;
    }
    if (query->heapCount > 0)
    {
        Place(query, at, last);
    }
    return top;
}

// One search: the end it heads for and what stopped ways on.
typedef struct Search
{
    mnavQuery* query;
    const mnavNavmesh* navmesh;
    const mnavQueryFilter* filter;
    // The cheapest included area's cost, scaling the heuristic.
    double cheapest;
    mnavPos3 end;
    int32_t endSlot;
    int32_t endPolygon;
    double limit;
    bool outOfNodes;
    bool tooLong;
    bool notLoaded;
} Search;

static double Distance(mnavPos3 a, mnavPos3 b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static mnavPos3 Midpoint(mnavPos3 a, mnavPos3 b)
{
    return (mnavPos3){(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5};
}

// Opens the node behind a portal from node from, or lowers its cost when
// this way is cheaper; closed nodes are final.
static void Open(Search* s, int32_t from, const mnavSearchNode* key, mnavPos3 a, mnavPos3 b,
                 double linkCost)
{
    mnavQuery* query = s->query;
    const mnavSearchNode* parent = &query->nodes[from];
    bool offMesh = key->tag == MNAV_TAG_OFFMESH;
    // Where the walk in the parent's polygon goes, and where the node
    // stands: a portal's midpoint, the end point, or an off-mesh link's
    // takeoff and landing points.
    mnavPos3 via = key->tag == MNAV_TAG_END ? s->end : (offMesh ? a : Midpoint(a, b));
    mnavPos3 at = offMesh ? b : via;
    // The walk lies in the parent's polygon, which is convex.
    const mnavPolygon* crossed =
        &s->navmesh->slots[parent->slot].tile->mesh.polygons[parent->polygon];
    double step = Distance(parent->at, via);
    double span = offMesh ? Distance(a, b) : 0.0;
    double cost = parent->cost + step * (double)s->filter->costs[crossed->area] + linkCost;
    double length = parent->length + step + span;
    double toEnd = key->tag == MNAV_TAG_END ? 0.0 : Distance(at, s->end);
    if (length + toEnd > s->limit)
    {
        s->tooLong = true;
        return;
    }
    uint32_t cell = Find(query, key->slot, key->polygon, key->tag, key->low);
    int32_t n = query->table[cell];
    if (n != MNAV_NO_NODE)
    {
        mnavSearchNode* node = &query->nodes[n];
        if (node->heap != MNAV_NO_NODE && cost < node->cost)
        {
            node->cost = cost;
            node->length = length;
            node->parent = from;
            SiftUp(query, node->heap);
        }
        return;
    }
    if (query->nodeCount == query->limits.nodes)
    {
        s->outOfNodes = true;
        return;
    }
    n = query->nodeCount++;
    query->nodes[n] = (mnavSearchNode){
        a,        b,        at,   cost,        length, toEnd * s->cheapest, key->slot, key->polygon,
        key->tag, key->low, from, MNAV_NO_NODE};
    query->table[cell] = n;
    query->heap[query->heapCount] = n;
    SiftUp(query, query->heapCount++);
}

// Opens the polygon across an inner edge j of node n's polygon.
static void ExpandInner(Search* s, int32_t n, const mnavTile* tile, int32_t j)
{
    const mnavSearchNode* node = &s->query->nodes[n];
    const mnavPolygon* polygon = &tile->mesh.polygons[node->polygon];
    int32_t next = polygon->neighbors[j];
    uint16_t from = polygon->vertices[j];
    uint16_t to = polygon->vertices[(j + 1) % polygon->count];
    const mnavPolygon* other = &tile->mesh.polygons[next];
    if (!mnavIncludes(s->filter, other->area))
    {
        return;
    }
    for (int32_t i = 0; i < other->count; ++i)
    {
        if (other->vertices[i] == to && other->vertices[(i + 1) % other->count] == from)
        {
            mnavFrame f = mnavFrameOf(s->navmesh, s->navmesh->slots[node->slot].x,
                                      s->navmesh->slots[node->slot].z);
            mnavSearchNode key = {.slot = node->slot, .polygon = next, .tag = i, .low = 0};
            Open(s, n, &key, mnavVertexWorld(&f, &tile->mesh.vertices[from]),
                 mnavVertexWorld(&f, &tile->mesh.vertices[to]), 0.0);
            return;
        }
    }
}

// The point of edge ab whose coordinate along a tile side is u.
static mnavPos3 AlongSide(mnavPos3 a, mnavPos3 b, double au, double bu, double u)
{
    double t = (u - au) / (bu - au);
    return (mnavPos3){a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

// Opens the polygons the tile links of edge j of node n's polygon reach,
// or notes that no tile is loaded across it.
static void ExpandSide(Search* s, int32_t n, const mnavTile* tile, int32_t j)
{
    const mnavSearchNode* node = &s->query->nodes[n];
    const mnavSlot* slot = &s->navmesh->slots[node->slot];
    const mnavPolygon* polygon = &tile->mesh.polygons[node->polygon];
    int32_t side = polygon->sides[j];
    int32_t x = slot->x;
    int32_t z = slot->z;
    int32_t facing = 0;
    int32_t across = -1;
    mnavAcross(side, &x, &z, &facing);
    if (mnavTileAt(s->navmesh, x, z, &across) == nullptr)
    {
        s->notLoaded = true;
        return;
    }
    mnavFrame f = mnavFrameOf(s->navmesh, slot->x, slot->z);
    const mnavMeshVertex* va = &tile->mesh.vertices[polygon->vertices[j]];
    const mnavMeshVertex* vb = &tile->mesh.vertices[polygon->vertices[(j + 1) % polygon->count]];
    bool alongZ = side == 1 || side == 3;
    double au = alongZ ? va->z : va->x;
    double bu = alongZ ? vb->z : vb->x;
    mnavPos3 a = mnavVertexWorld(&f, va);
    mnavPos3 b = mnavVertexWorld(&f, vb);
    for (int32_t l = tile->firstLink[node->polygon]; l < tile->firstLink[node->polygon + 1]; ++l)
    {
        const mnavLink* link = &tile->links[l];
        bool back = node->tag == MNAV_TAG_LINK + side && node->low == link->low;
        const mnavTile* beyond = s->navmesh->slots[link->target.slot - 1].tile;
        mnavAreaType area = beyond->mesh.polygons[link->target.polygon].area;
        if (link->edge != j || back || !mnavIncludes(s->filter, area))
        {
            continue;
        }
        // The overlap's ends in the edge's own direction.
        bool rising = bu > au;
        mnavPos3 first = AlongSide(a, b, au, bu, rising ? link->low : link->high);
        mnavPos3 second = AlongSide(a, b, au, bu, rising ? link->high : link->low);
        mnavSearchNode key = {.slot = (int32_t)link->target.slot - 1,
                              .polygon = (int32_t)link->target.polygon,
                              .tag = MNAV_TAG_LINK + facing,
                              .low = link->low};
        Open(s, n, &key, first, second, 0.0);
    }
}

// Opens the polygons the off-mesh links leaving node n's polygon land on,
// for the kinds and areas the filter includes.
static void ExpandOffMesh(Search* s, int32_t n)
{
    const mnavNavmesh* navmesh = s->navmesh;
    const mnavSearchNode* node = &s->query->nodes[n];
    int32_t first = 0;
    int32_t count = mnavAttachmentsFrom(navmesh, node->slot, node->polygon, &first);
    for (int32_t i = first; i < first + count; ++i)
    {
        mnavAttachment attachment = mnavAttachmentOf(navmesh->attachments[i]);
        const mnavOffLink* link = &navmesh->links[attachment.link];
        const mnavLinkState* state = &link->state;
        mnavPolygonId landing = attachment.reverse ? state->startPolygon : state->endPolygon;
        const mnavTile* tile = navmesh->slots[landing.slot - 1].tile;
        if (!mnavCrosses(s->filter, link->def.kind) ||
            !mnavIncludes(s->filter, tile->mesh.polygons[landing.polygon].area))
        {
            continue;
        }
        mnavSearchNode key = {.slot = (int32_t)landing.slot - 1,
                              .polygon = (int32_t)landing.polygon,
                              .tag = MNAV_TAG_OFFMESH,
                              .low = attachment.link * 2 + (attachment.reverse ? 1 : 0)};
        Open(s, n, &key, attachment.reverse ? state->end : state->start,
             attachment.reverse ? state->start : state->end, (double)link->def.cost);
    }
}

static void Expand(Search* s, int32_t n)
{
    const mnavSearchNode* node = &s->query->nodes[n];
    const mnavTile* tile = s->navmesh->slots[node->slot].tile;
    const mnavPolygon* polygon = &tile->mesh.polygons[node->polygon];
    if (node->slot == s->endSlot && node->polygon == s->endPolygon)
    {
        mnavSearchNode key = {
            .slot = node->slot, .polygon = node->polygon, .tag = MNAV_TAG_END, .low = 0};
        Open(s, n, &key, s->end, s->end, 0.0);
    }
    // mnavSearchNode pointers stay valid as nodes are added: the array never moves.
    // The edge the node came in through leads only back.
    for (int32_t j = 0; j < polygon->count; ++j)
    {
        bool inner = polygon->neighbors[j] != MNAV_NO_INDEX;
        if (inner && j != node->tag)
        {
            ExpandInner(s, n, tile, j);
        }
        if (!inner && polygon->sides[j] != 0)
        {
            ExpandSide(s, n, tile, j);
        }
    }
    ExpandOffMesh(s, n);
}

static bool FinitePoint(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

// Whether node a lies nearer the end than node b: less still to go, then
// the lower cost, then made first.
static bool Nearer(const mnavQuery* query, int32_t a, int32_t b)
{
    const mnavSearchNode* na = &query->nodes[a];
    const mnavSearchNode* nb = &query->nodes[b];
    if (na->remaining != nb->remaining)
    {
        return na->remaining < nb->remaining;
    }
    return na->cost != nb->cost ? na->cost < nb->cost : a < b;
}

// The heuristic's scale: the cheapest included area's cost, or less for
// a kind of link the filter crosses that costs less per meter (N30).
static double Scale(const mnavNavmesh* navmesh, const mnavQueryFilter* filter)
{
    double scale = mnavCheapest(filter);
    for (int32_t k = 0; k < MNAV_LINK_KINDS; ++k)
    {
        double perMeter = navmesh->costPerMeter[k];
        scale = mnavCrosses(filter, (mnavLinkKind)k) && perMeter < scale ? perMeter : scale;
    }
    return scale;
}

// Writes the polygons from the start to node last into the corridor.
static int32_t Corridor(mnavQuery* query, const mnavNavmesh* navmesh, int32_t last)
{
    int32_t count = 0;
    for (int32_t n = last; n != MNAV_NO_NODE; n = query->nodes[n].parent)
    {
        const mnavSearchNode* node = &query->nodes[n];
        mnavPolygonId id = {(uint32_t)node->slot + 1, navmesh->slots[node->slot].generation,
                            (uint32_t)node->polygon};
        const mnavPolygonId* previous = count > 0 ? &query->corridor[count - 1] : nullptr;
        if (previous == nullptr || previous->slot != id.slot || previous->polygon != id.polygon)
        {
            query->corridor[count++] = id;
        }
    }
    for (int32_t i = 0; i < count / 2; ++i)
    {
        mnavPolygonId swap = query->corridor[i];
        query->corridor[i] = query->corridor[count - 1 - i];
        query->corridor[count - 1 - i] = swap;
    }
    return count;
}

// Writes the portals from the start point to node last, and the end point
// or, short of it, the last portal's midpoint as a portal of one point.
static int32_t Portals(mnavQuery* query, int32_t last)
{
    int32_t count = 0;
    for (int32_t n = last; n != MNAV_NO_NODE; n = query->nodes[n].parent)
    {
        // A portal's ends are in the order of the polygon walked out of,
        // whose inside lies to the right of each edge: the first end is on
        // the walker's left. An off-mesh link gives its takeoff point, then
        // its landing point; written backward here.
        const mnavSearchNode* node = &query->nodes[n];
        if (node->tag == MNAV_TAG_OFFMESH)
        {
            query->portals[count++] = (mnavPortal){node->b, node->b, -1};
            query->portals[count++] = (mnavPortal){node->a, node->a, node->low};
            continue;
        }
        query->portals[count++] = (mnavPortal){node->a, node->b, -1};
    }
    for (int32_t i = 0; i < count / 2; ++i)
    {
        mnavPortal swap = query->portals[i];
        query->portals[i] = query->portals[count - 1 - i];
        query->portals[count - 1 - i] = swap;
    }
    const mnavSearchNode* node = &query->nodes[last];
    if (node->tag != MNAV_TAG_END)
    {
        query->portals[count++] = (mnavPortal){node->at, node->at, -1};
    }
    return count;
}

// Twice the signed area of triangle abc on the ground: positive when c lies
// to the left of the way from a to b.
static double Area2(mnavPos3 a, mnavPos3 b, mnavPos3 c)
{
    return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

static bool Same(mnavPos3 a, mnavPos3 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

static void Push(mnavQuery* query, int32_t* count, mnavPos3 p)
{
    if (*count == 0 || !Same(query->points[*count - 1], p))
    {
        query->points[(*count)++] = p;
    }
}

// The funnel: its apex and two sides, each with the portal it came from.
typedef struct Funnel
{
    mnavPos3 apex;
    mnavPos3 left;
    mnavPos3 right;
    int32_t apexAt;
    int32_t leftAt;
    int32_t rightAt;
} Funnel;

// Makes a side's point the funnel's new apex, a corner of the path.
static void Corner(mnavQuery* query, int32_t* count, Funnel* f, mnavPos3 p, int32_t at)
{
    Push(query, count, p);
    *f = (Funnel){p, p, p, at, at, at};
}

// Pulls portals first to last tight into a straight path: a portal end on
// or inside a side of the funnel narrows it; one on or past the other side
// makes that side's point a corner, and the scan goes on from there. The
// first portal is a point, always kept: the start or a landing point.
static int32_t Pull(mnavQuery* query, int32_t first, int32_t last, int32_t count)
{
    const mnavPortal* p = query->portals;
    query->points[count++] = p[first].left;
    Funnel f = {p[first].left, p[first].left, p[first].right, first, first, first};
    for (int32_t i = first + 1; i <= last; ++i)
    {
        if (Area2(f.apex, f.right, p[i].right) >= 0.0)
        {
            if (!Same(f.apex, f.right) && Area2(f.apex, f.left, p[i].right) >= 0.0)
            {
                Corner(query, &count, &f, f.left, f.leftAt);
                i = f.apexAt;
                continue;
            }
            f.right = p[i].right;
            f.rightAt = i;
        }
        if (Area2(f.apex, f.left, p[i].left) <= 0.0)
        {
            if (!Same(f.apex, f.left) && Area2(f.apex, f.right, p[i].left) <= 0.0)
            {
                Corner(query, &count, &f, f.right, f.rightAt);
                i = f.apexAt;
                continue;
            }
            f.left = p[i].left;
            f.leftAt = i;
        }
    }
    Push(query, &count, p[last].left);
    return count;
}

// The straight path, stretch by stretch between off-mesh links: each
// stretch ends at a takeoff point, the next begins at its landing point.
static int32_t Straighten(mnavQuery* query, const mnavNavmesh* navmesh, int32_t portals,
                          int32_t* linkCount)
{
    int32_t count = 0;
    int32_t first = 0;
    *linkCount = 0;
    for (int32_t i = 0; i < portals; ++i)
    {
        int32_t crossed = query->portals[i].link;
        if (crossed < 0 && i + 1 < portals)
        {
            continue;
        }
        count = Pull(query, first, i, count);
        first = i + 1;
        if (crossed >= 0)
        {
            const mnavOffLink* link = &navmesh->links[crossed / 2];
            query->links[(*linkCount)++] = (mnavPathLink){
                {(uint32_t)crossed / 2 + 1, link->generation}, link->def.kind, count - 1};
        }
    }
    return count;
}

static mnavPathEnd EndOf(const Search* s)
{
    if (s->outOfNodes)
    {
        return mnav_pathOutOfNodes;
    }
    if (s->tooLong)
    {
        return mnav_pathTooLong;
    }
    return s->notLoaded ? mnav_pathNotLoaded : mnav_pathNone;
}

mnavResult mnavFindPath(mnavQuery* query, const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                        mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon,
                        mnavPos3 end, mnavPath* pathOut)
{
    if (query == nullptr || navmesh == nullptr || pathOut == nullptr || !FinitePoint(start) ||
        !FinitePoint(end))
    {
        return mnav_errorInvalid;
    }
    mnavResult result = mnavCheckPolygon(navmesh, startPolygon);
    result = result == mnav_success ? mnavCheckPolygon(navmesh, endPolygon) : result;
    const mnavQueryFilter* usable = nullptr;
    result = result == mnav_success ? mnavCheckFilter(filter, &usable) : result;
    if (result != mnav_success)
    {
        return result;
    }
    memset(query->table, 0xFF, ((size_t)query->tableMask + 1) * sizeof(int32_t));
    Search s = {query,
                navmesh,
                usable,
                Scale(navmesh, usable),
                end,
                (int32_t)endPolygon.slot - 1,
                (int32_t)endPolygon.polygon,
                (double)query->limits.pathLength,
                false,
                false,
                false};
    query->nodes[0] = (mnavSearchNode){start,
                                       start,
                                       start,
                                       0.0,
                                       0.0,
                                       Distance(start, end) * s.cheapest,
                                       (int32_t)startPolygon.slot - 1,
                                       (int32_t)startPolygon.polygon,
                                       MNAV_TAG_START,
                                       0,
                                       MNAV_NO_NODE,
                                       0};
    query->table[Find(query, query->nodes[0].slot, query->nodes[0].polygon, MNAV_TAG_START, 0)] = 0;
    query->heap[0] = 0;
    query->nodeCount = 1;
    query->heapCount = 1;
    int32_t best = 0;
    int32_t found = MNAV_NO_NODE;
    while (query->heapCount > 0 && found == MNAV_NO_NODE)
    {
        int32_t n = Pop(query);
        if (query->nodes[n].tag == MNAV_TAG_END)
        {
            found = n;
            continue;
        }
        best = Nearer(query, n, best) ? n : best;
        Expand(&s, n);
    }
    int32_t last = found != MNAV_NO_NODE ? found : best;
    int32_t linkCount = 0;
    int32_t pointCount = Straighten(query, navmesh, Portals(query, last), &linkCount);
    *pathOut = (mnavPath){found != MNAV_NO_NODE ? mnav_pathFound : EndOf(&s),
                          query->nodes[last].cost,
                          query->nodes[last].length,
                          query->corridor,
                          Corridor(query, navmesh, last),
                          query->points,
                          pointCount,
                          query->links,
                          linkCount};
    return mnav_success;
}
