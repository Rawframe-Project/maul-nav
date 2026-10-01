// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The path search: A* over portals, with named limits (N25).

#include "allocator.h"
#include "navmesh.h"
#include "polymesh.h"
#include "raster.h"

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// Marks a def built by mnavDefaultQueryDef.
#define QUERY_DEF_COOKIE 0x4E415651u

// A node's way in: an edge index below 8, 8 plus the side for a tile link,
// or the start or end point.
#define TAG_LINK  8
#define TAG_START 0xFE
#define TAG_END   0xFF

// No node, or a node that has left the open list.
#define NO_NODE (-1)

// A portal into a polygon, or the start or end point: where the search
// stands, the way it came, and its cost so far.
typedef struct Node
{
    // The portal's ends, in the order of the polygon left, and its midpoint.
    mnavPos3 a;
    mnavPos3 b;
    mnavPos3 at;
    double cost;
    double remaining;
    // The polygon entered: its 0-based slot and index.
    int32_t slot;
    int32_t polygon;
    // The portal: TAG_ values, and the link's start along the side.
    int32_t tag;
    int32_t low;
    int32_t parent;
    // The node's place in the open list, or NO_NODE once closed.
    int32_t heap;
} Node;

// A portal's ends as the funnel sees them, walking into the polygon: the
// left one, then the right one.
typedef struct Portal
{
    mnavPos3 left;
    mnavPos3 right;
} Portal;

struct mnavQuery
{
    mnavMemory memory;
    mnavQueryLimits limits;
    Node* nodes;
    int32_t nodeCount;
    int32_t* heap;
    int32_t heapCount;
    // Open addressing over node keys: node indices, NO_NODE for empty.
    int32_t* table;
    uint32_t tableMask;
    mnavPolygonId* corridor;
    // The corridor's portals, then the straight path; a corridor of n
    // polygons has n + 1 of each at most.
    Portal* portals;
    mnavPos3* points;
};

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
    mnavRelease(memory, query->points, nodes + 1, sizeof(mnavPos3), alignof(mnavPos3));
    mnavRelease(memory, query->portals, nodes + 1, sizeof(Portal), alignof(Portal));
    mnavRelease(memory, query->corridor, nodes, sizeof(mnavPolygonId), alignof(mnavPolygonId));
    mnavRelease(memory, query->table, (size_t)query->tableMask + 1, sizeof(int32_t),
                alignof(int32_t));
    mnavRelease(memory, query->heap, nodes, sizeof(int32_t), alignof(int32_t));
    mnavRelease(memory, query->nodes, nodes, sizeof(Node), alignof(Node));
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
    result =
        mnavAllocate(&query->memory, nodes, sizeof(Node), alignof(Node), (void**)&query->nodes);
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
        result = mnavAllocate(&query->memory, nodes + 1, sizeof(Portal), alignof(Portal),
                              (void**)&query->portals);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&query->memory, nodes + 1, sizeof(mnavPos3), alignof(mnavPos3),
                              (void**)&query->points);
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
        if (n == NO_NODE)
        {
            return cell;
        }
        const Node* node = &query->nodes[n];
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
    const Node* na = &query->nodes[a];
    const Node* nb = &query->nodes[b];
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
    query->nodes[top].heap = NO_NODE;
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

static mnavPos3 World(const mnavFrame* f, const mnavMeshVertex* v)
{
    return (mnavPos3){f->x0 + v->x * f->cell, f->y0 + (v->y - MNAV_HEIGHT_OFFSET) * f->height,
                      f->z0 + v->z * f->cell};
}

// Opens the node behind a portal from node from, or lowers its cost when
// this way is cheaper; closed nodes are final.
static void Open(Search* s, int32_t from, const Node* key, mnavPos3 a, mnavPos3 b)
{
    mnavQuery* query = s->query;
    const Node* parent = &query->nodes[from];
    mnavPos3 at = key->tag == TAG_END ? s->end : Midpoint(a, b);
    double cost = parent->cost + Distance(parent->at, at);
    double remaining = key->tag == TAG_END ? 0.0 : Distance(at, s->end);
    if (cost + remaining > s->limit)
    {
        s->tooLong = true;
        return;
    }
    uint32_t cell = Find(query, key->slot, key->polygon, key->tag, key->low);
    int32_t n = query->table[cell];
    if (n != NO_NODE)
    {
        Node* node = &query->nodes[n];
        if (node->heap != NO_NODE && cost < node->cost)
        {
            node->cost = cost;
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
    query->nodes[n] = (Node){a,        b,        at,   cost,   remaining, key->slot, key->polygon,
                             key->tag, key->low, from, NO_NODE};
    query->table[cell] = n;
    query->heap[query->heapCount] = n;
    SiftUp(query, query->heapCount++);
}

// Opens the polygon across an inner edge j of node n's polygon.
static void ExpandInner(Search* s, int32_t n, const mnavTile* tile, int32_t j)
{
    const Node* node = &s->query->nodes[n];
    const mnavPolygon* polygon = &tile->mesh.polygons[node->polygon];
    int32_t next = polygon->neighbors[j];
    uint16_t from = polygon->vertices[j];
    uint16_t to = polygon->vertices[(j + 1) % polygon->count];
    const mnavPolygon* other = &tile->mesh.polygons[next];
    for (int32_t i = 0; i < other->count; ++i)
    {
        if (other->vertices[i] == to && other->vertices[(i + 1) % other->count] == from)
        {
            mnavFrame f = mnavFrameOf(s->navmesh, s->navmesh->slots[node->slot].x,
                                      s->navmesh->slots[node->slot].z);
            Node key = {.slot = node->slot, .polygon = next, .tag = i, .low = 0};
            Open(s, n, &key, World(&f, &tile->mesh.vertices[from]),
                 World(&f, &tile->mesh.vertices[to]));
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
    const Node* node = &s->query->nodes[n];
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
    mnavPos3 a = World(&f, va);
    mnavPos3 b = World(&f, vb);
    for (int32_t l = tile->firstLink[node->polygon]; l < tile->firstLink[node->polygon + 1]; ++l)
    {
        const mnavLink* link = &tile->links[l];
        bool back = node->tag == TAG_LINK + side && node->low == link->low;
        if (link->edge != j || back)
        {
            continue;
        }
        // The overlap's ends in the edge's own direction.
        bool rising = bu > au;
        mnavPos3 first = AlongSide(a, b, au, bu, rising ? link->low : link->high);
        mnavPos3 second = AlongSide(a, b, au, bu, rising ? link->high : link->low);
        Node key = {.slot = (int32_t)link->target.slot - 1,
                    .polygon = (int32_t)link->target.polygon,
                    .tag = TAG_LINK + facing,
                    .low = link->low};
        Open(s, n, &key, first, second);
    }
}

static void Expand(Search* s, int32_t n)
{
    const Node* node = &s->query->nodes[n];
    const mnavTile* tile = s->navmesh->slots[node->slot].tile;
    const mnavPolygon* polygon = &tile->mesh.polygons[node->polygon];
    if (node->slot == s->endSlot && node->polygon == s->endPolygon)
    {
        Node key = {.slot = node->slot, .polygon = node->polygon, .tag = TAG_END, .low = 0};
        Open(s, n, &key, s->end, s->end);
    }
    // Node pointers stay valid as nodes are added: the array never moves.
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
}

// Whether a polygon id names a polygon of the navmesh now.
static mnavResult CheckPolygon(const mnavNavmesh* navmesh, mnavPolygonId id)
{
    if (id.slot == 0 || id.slot > (uint32_t)navmesh->slotCount || id.generation == 0)
    {
        return mnav_errorInvalid;
    }
    const mnavSlot* slot = &navmesh->slots[id.slot - 1];
    if (id.generation > slot->generation)
    {
        return mnav_errorInvalid;
    }
    if (id.generation != slot->generation || slot->tile == nullptr)
    {
        return mnav_errorStale;
    }
    return id.polygon < (uint32_t)slot->tile->mesh.polygonCount ? mnav_success : mnav_errorInvalid;
}

static bool FinitePoint(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

// Whether node a lies nearer the end than node b: less still to go, then
// the lower cost, then made first.
static bool Nearer(const mnavQuery* query, int32_t a, int32_t b)
{
    const Node* na = &query->nodes[a];
    const Node* nb = &query->nodes[b];
    if (na->remaining != nb->remaining)
    {
        return na->remaining < nb->remaining;
    }
    return na->cost != nb->cost ? na->cost < nb->cost : a < b;
}

// Writes the polygons from the start to node last into the corridor.
static int32_t Corridor(mnavQuery* query, const mnavNavmesh* navmesh, int32_t last)
{
    int32_t count = 0;
    for (int32_t n = last; n != NO_NODE; n = query->nodes[n].parent)
    {
        const Node* node = &query->nodes[n];
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
    for (int32_t n = last; n != NO_NODE; n = query->nodes[n].parent)
    {
        // A portal's ends are in the order of the polygon walked out of,
        // whose inside lies to the right of each edge: the first end is on
        // the walker's left.
        query->portals[count++] = (Portal){query->nodes[n].a, query->nodes[n].b};
    }
    for (int32_t i = 0; i < count / 2; ++i)
    {
        Portal swap = query->portals[i];
        query->portals[i] = query->portals[count - 1 - i];
        query->portals[count - 1 - i] = swap;
    }
    const Node* node = &query->nodes[last];
    if (node->tag != TAG_END)
    {
        query->portals[count++] = (Portal){node->at, node->at};
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

// Pulls the corridor's portals tight into a straight path: a portal end on
// or inside a side of the funnel narrows it; one on or past the other side
// makes that side's point a corner, and the scan goes on from there.
static int32_t Straighten(mnavQuery* query, int32_t portals)
{
    const Portal* p = query->portals;
    int32_t count = 0;
    Push(query, &count, p[0].left);
    Funnel f = {p[0].left, p[0].left, p[0].right, 0, 0, 0};
    for (int32_t i = 1; i < portals; ++i)
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
    Push(query, &count, p[portals - 1].left);
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

mnavResult mnavFindPath(mnavQuery* query, const mnavNavmesh* navmesh, mnavPolygonId startPolygon,
                        mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end, mnavPath* pathOut)
{
    if (query == nullptr || navmesh == nullptr || pathOut == nullptr || !FinitePoint(start) ||
        !FinitePoint(end))
    {
        return mnav_errorInvalid;
    }
    mnavResult result = CheckPolygon(navmesh, startPolygon);
    result = result == mnav_success ? CheckPolygon(navmesh, endPolygon) : result;
    if (result != mnav_success)
    {
        return result;
    }
    memset(query->table, 0xFF, ((size_t)query->tableMask + 1) * sizeof(int32_t));
    Search s = {query,
                navmesh,
                end,
                (int32_t)endPolygon.slot - 1,
                (int32_t)endPolygon.polygon,
                (double)query->limits.pathLength,
                false,
                false,
                false};
    query->nodes[0] = (Node){start,
                             start,
                             start,
                             0.0,
                             Distance(start, end),
                             (int32_t)startPolygon.slot - 1,
                             (int32_t)startPolygon.polygon,
                             TAG_START,
                             0,
                             NO_NODE,
                             0};
    query->table[Find(query, query->nodes[0].slot, query->nodes[0].polygon, TAG_START, 0)] = 0;
    query->heap[0] = 0;
    query->nodeCount = 1;
    query->heapCount = 1;
    int32_t best = 0;
    int32_t found = NO_NODE;
    while (query->heapCount > 0 && found == NO_NODE)
    {
        int32_t n = Pop(query);
        if (query->nodes[n].tag == TAG_END)
        {
            found = n;
            continue;
        }
        best = Nearer(query, n, best) ? n : best;
        Expand(&s, n);
    }
    int32_t last = found != NO_NODE ? found : best;
    *pathOut = (mnavPath){found != NO_NODE ? mnav_pathFound : EndOf(&s),
                          query->nodes[last].cost,
                          query->corridor,
                          Corridor(query, navmesh, last),
                          query->points,
                          Straighten(query, Portals(query, last))};
    return mnav_success;
}
