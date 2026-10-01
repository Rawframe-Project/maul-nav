// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hierarchical paths (mnav-0008), after HPA* (Botea, Mueller and
// Schaeffer, 2004): clusters of tiles, a transition at the middle link of
// each run of tile links leaving a cluster, edges found by the navmesh
// search confined to the cluster entered, A* over the transitions, and a
// refinement confined to the clusters the abstract path crosses.

#include "maul-nav/hierarchy.h"

#include "allocator.h"
#include "navmesh.h"
#include "query.h"
#include "query_filter.h"
#include "sort.h"

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Marks a def built by mnavDefaultHierarchyDef.
#define HIERARCHY_DEF_COOKIE 0x4E415648u

// Coordinates are biased into 32 unsigned bits for sort keys.
#define BIAS 0x80000000u

// A transition: crossing a portal from one cluster into another.
typedef struct Transition
{
    // The portal's midpoint, where the transition stands.
    mnavPos3 at;
    // The search node of the crossing: the polygon entered, its slot, the
    // tag and the link's start along the side.
    int32_t slot;
    int32_t polygon;
    int32_t tag;
    int32_t low;
    // The clusters entered and left, the slot left, its side and the run's
    // start along it.
    int32_t cluster;
    int32_t from;
    int32_t fromSlot;
    int32_t side;
    int32_t runLow;
    // The transition crossing the same portal the other way, or -1.
    int32_t reverse;
} Transition;

typedef struct Edge
{
    int32_t to;
    double cost;
} Edge;

struct mnavHierarchy
{
    mnavMemory memory;
    mnavHierarchyDef def;
    // The graph's navmesh and its commits when built; NULL with no graph.
    const mnavNavmesh* navmesh;
    uint64_t commits;
    mnavQueryFilter filter;
    double scale;
    // Per slot: its cluster, or -1, its place's first transition, and
    // whether a confined search may expand it.
    int32_t* clusterOf;
    int32_t* firstOfSlot;
    uint8_t* inside;
    // Clusters: their sort keys, their slots and the transitions leaving
    // them, each as ranges of the arrays after.
    uint64_t* keys;
    uint64_t* scratch;
    int32_t clusterCount;
    int32_t* firstSlot;
    int32_t* slots;
    int32_t* firstLeaving;
    int32_t* leaving;
    Transition* transitions;
    int32_t transitionCount;
    int32_t* firstEdge;
    Edge* edges;
    int32_t edgeCount;
    // The abstract search: per transition, and one more for the end.
    double* costs;
    double* joins;
    int32_t* parents;
    int32_t* places;
    int32_t* heap;
    int32_t heapCount;
};

mnavHierarchyDef mnavDefaultHierarchyDef(void)
{
    return (mnavHierarchyDef){HIERARCHY_DEF_COOKIE, {0}, {4096, 16384, 262144}, 4};
}

static bool GoodDef(const mnavHierarchyDef* def)
{
    const mnavHierarchyLimits* l = &def->limits;
    return l->tiles >= 1 && l->tiles <= MNAV_MAX_HIERARCHY_TILES && l->transitions >= 1 &&
           l->transitions <= MNAV_MAX_HIERARCHY_TRANSITIONS && l->edges >= 1 &&
           l->edges <= MNAV_MAX_HIERARCHY_EDGES && def->clusterTiles >= 1 &&
           def->clusterTiles <= MNAV_MAX_CLUSTER_TILES;
}

// An array of count items for the hierarchy, kept going on a failure.
static void Take(mnavHierarchy* h, mnavResult* result, size_t count, size_t size, size_t align,
                 void** out)
{
    if (*result == mnav_success)
    {
        *result = mnavAllocate(&h->memory, count, size, align, out);
    }
}

static void Give(mnavHierarchy* h, void* block, size_t count, size_t size, size_t align)
{
    mnavMemory memory = h->memory;
    mnavRelease(&memory, block, count, size, align);
    h->memory = memory;
}

// The sizes each array is allocated with: tiles, transitions and edges.
typedef struct Sizes
{
    size_t tiles;
    size_t transitions;
    size_t edges;
} Sizes;

static Sizes SizesOf(const mnavHierarchyDef* def)
{
    return (Sizes){(size_t)def->limits.tiles, (size_t)def->limits.transitions,
                   (size_t)def->limits.edges};
}

static mnavResult Allocate(mnavHierarchy* h)
{
    Sizes n = SizesOf(&h->def);
    mnavResult r = mnav_success;
    Take(h, &r, n.tiles, sizeof(int32_t), alignof(int32_t), (void**)&h->clusterOf);
    Take(h, &r, n.tiles, sizeof(int32_t), alignof(int32_t), (void**)&h->firstOfSlot);
    Take(h, &r, n.tiles, sizeof(uint8_t), alignof(uint8_t), (void**)&h->inside);
    Take(h, &r, n.tiles, sizeof(uint64_t), alignof(uint64_t), (void**)&h->keys);
    Take(h, &r, n.tiles, sizeof(uint64_t), alignof(uint64_t), (void**)&h->scratch);
    Take(h, &r, n.tiles + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->firstSlot);
    Take(h, &r, n.tiles, sizeof(int32_t), alignof(int32_t), (void**)&h->slots);
    Take(h, &r, n.tiles + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->firstLeaving);
    Take(h, &r, n.transitions, sizeof(int32_t), alignof(int32_t), (void**)&h->leaving);
    Take(h, &r, n.transitions, sizeof(Transition), alignof(Transition), (void**)&h->transitions);
    Take(h, &r, n.transitions + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->firstEdge);
    Take(h, &r, n.edges, sizeof(Edge), alignof(Edge), (void**)&h->edges);
    Take(h, &r, n.transitions + 1, sizeof(double), alignof(double), (void**)&h->costs);
    Take(h, &r, n.transitions, sizeof(double), alignof(double), (void**)&h->joins);
    Take(h, &r, n.transitions + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->parents);
    Take(h, &r, n.transitions + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->places);
    Take(h, &r, n.transitions + 1, sizeof(int32_t), alignof(int32_t), (void**)&h->heap);
    return r;
}

mnavResult mnavCreateHierarchy(const mnavHierarchyDef* def, mnavHierarchy** hierarchyOut)
{
    if (hierarchyOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *hierarchyOut = nullptr;
    if (def == nullptr || def->cookie != HIERARCHY_DEF_COOKIE ||
        (def->allocator.alloc == nullptr) != (def->allocator.free == nullptr))
    {
        return mnav_errorInvalid;
    }
    if (!GoodDef(def))
    {
        return mnav_errorRange;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavHierarchy* h = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavHierarchy), alignof(mnavHierarchy), (void**)&h);
    if (result != mnav_success)
    {
        return result;
    }
    *h = (mnavHierarchy){0};
    h->memory = memory;
    h->def = *def;
    result = Allocate(h);
    if (result != mnav_success)
    {
        mnavDestroyHierarchy(h);
        return result;
    }
    memset(h->inside, 0, (size_t)def->limits.tiles);
    *hierarchyOut = h;
    return mnav_success;
}

void mnavDestroyHierarchy(mnavHierarchy* hierarchy)
{
    if (hierarchy == nullptr)
    {
        return;
    }
    mnavHierarchy* h = hierarchy;
    Sizes n = SizesOf(&h->def);
    Give(h, h->clusterOf, n.tiles, sizeof(int32_t), alignof(int32_t));
    Give(h, h->firstOfSlot, n.tiles, sizeof(int32_t), alignof(int32_t));
    Give(h, h->inside, n.tiles, sizeof(uint8_t), alignof(uint8_t));
    Give(h, h->keys, n.tiles, sizeof(uint64_t), alignof(uint64_t));
    Give(h, h->scratch, n.tiles, sizeof(uint64_t), alignof(uint64_t));
    Give(h, h->firstSlot, n.tiles + 1, sizeof(int32_t), alignof(int32_t));
    Give(h, h->slots, n.tiles, sizeof(int32_t), alignof(int32_t));
    Give(h, h->firstLeaving, n.tiles + 1, sizeof(int32_t), alignof(int32_t));
    Give(h, h->leaving, n.transitions, sizeof(int32_t), alignof(int32_t));
    Give(h, h->transitions, n.transitions, sizeof(Transition), alignof(Transition));
    Give(h, h->firstEdge, n.transitions + 1, sizeof(int32_t), alignof(int32_t));
    Give(h, h->edges, n.edges, sizeof(Edge), alignof(Edge));
    Give(h, h->costs, n.transitions + 1, sizeof(double), alignof(double));
    Give(h, h->joins, n.transitions, sizeof(double), alignof(double));
    Give(h, h->parents, n.transitions + 1, sizeof(int32_t), alignof(int32_t));
    Give(h, h->places, n.transitions + 1, sizeof(int32_t), alignof(int32_t));
    Give(h, h->heap, n.transitions + 1, sizeof(int32_t), alignof(int32_t));
    mnavMemory memory = h->memory;
    mnavRelease(&memory, h, 1, sizeof(mnavHierarchy), alignof(mnavHierarchy));
}

// Floor division, for clusters of tiles at negative places.
static int32_t Down(int32_t v, int32_t by)
{
    int32_t q = v / by;
    return (v % by != 0 && v < 0) ? q - 1 : q;
}

static uint64_t ClusterKey(const mnavHierarchy* h, int32_t x, int32_t z)
{
    uint32_t cx = (uint32_t)Down(x, h->def.clusterTiles) + BIAS;
    uint32_t cz = (uint32_t)Down(z, h->def.clusterTiles) + BIAS;
    return ((uint64_t)cx << 32) | cz;
}

// The index of a key among the sorted unique cluster keys.
static int32_t FindKey(const mnavHierarchy* h, uint64_t key)
{
    int32_t low = 0;
    int32_t high = h->clusterCount - 1;
    while (low < high)
    {
        int32_t middle = low + (high - low) / 2;
        if (h->keys[middle] < key)
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }
    return low;
}

// Numbers the clusters by place and lists each one's slots, in the
// navmesh's place order.
static void FindClusters(mnavHierarchy* h, const mnavNavmesh* navmesh)
{
    for (int32_t i = 0; i < navmesh->placeCount; ++i)
    {
        h->keys[i] = ClusterKey(h, navmesh->places[i].x, navmesh->places[i].z);
    }
    h->clusterCount = (int32_t)mnavSortUnique(h->keys, h->scratch, (size_t)navmesh->placeCount);
    for (int32_t s = 0; s < navmesh->slotCount; ++s)
    {
        h->clusterOf[s] = -1;
        h->firstOfSlot[s] = 0;
    }
    memset(h->firstSlot, 0, ((size_t)h->clusterCount + 1) * sizeof(int32_t));
    for (int32_t i = 0; i < navmesh->placeCount; ++i)
    {
        const mnavPlace* p = &navmesh->places[i];
        int32_t c = FindKey(h, ClusterKey(h, p->x, p->z));
        h->clusterOf[p->slot] = c;
        h->firstSlot[c + 1] += 1;
    }
    for (int32_t c = 0; c < h->clusterCount; ++c)
    {
        h->firstSlot[c + 1] += h->firstSlot[c];
    }
    // Fills each cluster's range in place order, using firstLeaving as
    // the cursors for now.
    memcpy(h->firstLeaving, h->firstSlot, ((size_t)h->clusterCount + 1) * sizeof(int32_t));
    for (int32_t i = 0; i < navmesh->placeCount; ++i)
    {
        int32_t slot = navmesh->places[i].slot;
        h->slots[h->firstLeaving[h->clusterOf[slot]]++] = slot;
    }
}

// The point of edge ab whose coordinate along a tile side is u.
static mnavPos3 AlongSide(mnavPos3 a, mnavPos3 b, double au, double bu, double u)
{
    double t = (u - au) / (bu - au);
    return (mnavPos3){a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

// The transition at tile link l of the tile in slot, which leaves its
// cluster, as the search crosses it.
static Transition Cross(const mnavHierarchy* h, const mnavNavmesh* navmesh, int32_t slot, int32_t l,
                        int32_t runLow)
{
    const mnavSlot* s = &navmesh->slots[slot];
    const mnavTile* tile = s->tile;
    const mnavLink* link = &tile->links[l];
    const mnavPolygon* polygon = &tile->mesh.polygons[link->polygon];
    int32_t side = polygon->sides[link->edge];
    int32_t x = s->x;
    int32_t z = s->z;
    int32_t facing = 0;
    mnavAcross(side, &x, &z, &facing);
    mnavFrame f = mnavFrameOf(navmesh, s->x, s->z);
    const mnavMeshVertex* va = &tile->mesh.vertices[polygon->vertices[link->edge]];
    const mnavMeshVertex* vb =
        &tile->mesh.vertices[polygon->vertices[(link->edge + 1) % polygon->count]];
    bool alongZ = side == 1 || side == 3;
    double au = alongZ ? va->z : va->x;
    double bu = alongZ ? vb->z : vb->x;
    mnavPos3 a = mnavVertexWorld(&f, va);
    mnavPos3 b = mnavVertexWorld(&f, vb);
    mnavPos3 first = AlongSide(a, b, au, bu, link->low);
    mnavPos3 second = AlongSide(a, b, au, bu, link->high);
    int32_t target = (int32_t)link->target.slot - 1;
    return (Transition){
        {(first.x + second.x) * 0.5, (first.y + second.y) * 0.5, (first.z + second.z) * 0.5},
        target,
        (int32_t)link->target.polygon,
        MNAV_TAG_LINK + facing,
        link->low,
        h->clusterOf[target],
        h->clusterOf[slot],
        slot,
        side,
        runLow,
        -1};
}

// Whether tile link l of a tile leaves the cluster across a side.
static bool Leaves(const mnavHierarchy* h, const mnavTile* tile, int32_t slot, int32_t l,
                   int32_t side)
{
    const mnavLink* link = &tile->links[l];
    int32_t target = (int32_t)link->target.slot - 1;
    return tile->mesh.polygons[link->polygon].sides[link->edge] == side &&
           h->clusterOf[target] != h->clusterOf[slot] && h->clusterOf[target] >= 0;
}

// Adds the transitions of one side of the tile in slot: the middle link of
// each run of touching links, by their start along the side.
static mnavResult AddSide(mnavHierarchy* h, const mnavNavmesh* navmesh, int32_t slot, int32_t side,
                          uint64_t* sorted, uint64_t* scratch)
{
    const mnavTile* tile = navmesh->slots[slot].tile;
    size_t count = 0;
    for (int32_t l = 0; l < tile->linkCount; ++l)
    {
        if (Leaves(h, tile, slot, l, side))
        {
            sorted[count++] = ((uint64_t)((uint32_t)tile->links[l].low + BIAS) << 32) | (uint32_t)l;
        }
    }
    count = mnavSortUnique(sorted, scratch, count);
    size_t start = 0;
    while (start < count)
    {
        int32_t high = tile->links[(uint32_t)sorted[start]].high;
        size_t end = start + 1;
        while (end < count && tile->links[(uint32_t)sorted[end]].low <= high)
        {
            int32_t next = tile->links[(uint32_t)sorted[end]].high;
            high = next > high ? next : high;
            end += 1;
        }
        if (h->transitionCount == h->def.limits.transitions)
        {
            return mnav_errorLimit;
        }
        int32_t middle = (int32_t)(uint32_t)sorted[start + (end - start) / 2];
        int32_t runLow = tile->links[(uint32_t)sorted[start]].low;
        h->transitions[h->transitionCount++] = Cross(h, navmesh, slot, middle, runLow);
        start = end;
    }
    return mnav_success;
}

// Finds the transition crossing u's portal the other way.
static int32_t ReverseOf(const mnavHierarchy* h, int32_t u)
{
    const Transition* t = &h->transitions[u];
    int32_t facing = t->side <= 2 ? t->side + 2 : t->side - 2;
    int32_t first = h->firstOfSlot[t->slot];
    for (int32_t v = first; v < h->transitionCount && h->transitions[v].fromSlot == t->slot; ++v)
    {
        const Transition* r = &h->transitions[v];
        if (r->side == facing && r->runLow == t->runLow)
        {
            return v;
        }
    }
    return -1;
}

// Adds every transition, in place order, then side, then position, and
// lists those leaving each cluster.
static mnavResult FindTransitions(mnavHierarchy* h, const mnavNavmesh* navmesh, uint64_t* sorted,
                                  uint64_t* scratch)
{
    h->transitionCount = 0;
    for (int32_t i = 0; i < navmesh->placeCount; ++i)
    {
        int32_t slot = navmesh->places[i].slot;
        h->firstOfSlot[slot] = h->transitionCount;
        for (int32_t side = 1; side <= 4; ++side)
        {
            mnavResult result = AddSide(h, navmesh, slot, side, sorted, scratch);
            if (result != mnav_success)
            {
                return result;
            }
        }
    }
    memset(h->firstLeaving, 0, ((size_t)h->clusterCount + 1) * sizeof(int32_t));
    for (int32_t u = 0; u < h->transitionCount; ++u)
    {
        h->transitions[u].reverse = ReverseOf(h, u);
        h->firstLeaving[h->transitions[u].from + 1] += 1;
    }
    for (int32_t c = 0; c < h->clusterCount; ++c)
    {
        h->firstLeaving[c + 1] += h->firstLeaving[c];
    }
    // Counting sort by the cluster left, keeping transition order; the
    // end costs' slots serve as cursors.
    int32_t* cursors = h->parents;
    memcpy(cursors, h->firstLeaving, ((size_t)h->clusterCount + 1) * sizeof(int32_t));
    for (int32_t u = 0; u < h->transitionCount; ++u)
    {
        h->leaving[cursors[h->transitions[u].from]++] = u;
    }
    return mnav_success;
}

static void Mark(mnavHierarchy* h, int32_t cluster, uint8_t value)
{
    for (int32_t i = h->firstSlot[cluster]; i < h->firstSlot[cluster + 1]; ++i)
    {
        h->inside[h->slots[i]] = value;
    }
}

static mnavPolygonId IdOf(const mnavNavmesh* navmesh, int32_t slot, int32_t polygon)
{
    return (mnavPolygonId){(uint32_t)slot + 1, navmesh->slots[slot].generation, (uint32_t)polygon};
}

// Runs Dijkstra's search from a point in a polygon within a cluster,
// opening the nodes just beyond it; mnav_errorLimit when it runs out of
// nodes.
static mnavResult SearchCluster(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh,
                                int32_t cluster, mnavPolygonId polygon, mnavPos3 point)
{
    mnavResult result = mnavBeginPath(query, navmesh, &h->filter, polygon, point, polygon, point);
    if (result != mnav_success)
    {
        return result;
    }
    Mark(h, cluster, 1);
    mnavConfineSearch(query, h->inside, true, true);
    bool ended = false;
    while (result == mnav_success && !ended)
    {
        result = mnavContinuePath(query, navmesh, INT32_MAX, &ended);
    }
    Mark(h, cluster, 0);
    bool out = query->search.outOfNodes;
    query->search.active = false;
    return result != mnav_success ? result : (out ? mnav_errorLimit : mnav_success);
}

// The cost the last search found to cross transition v's portal, or
// infinity.
static double CostTo(const mnavQuery* query, const Transition* v)
{
    int32_t n = query->table[mnavFindNode(query, v->slot, v->polygon, v->tag, v->low)];
    return n == MNAV_NO_NODE ? (double)INFINITY : query->nodes[n].cost;
}

// Finds each transition's edges to those leaving the cluster it enters.
static mnavResult FindEdges(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh)
{
    h->edgeCount = 0;
    for (int32_t u = 0; u < h->transitionCount; ++u)
    {
        const Transition* t = &h->transitions[u];
        h->firstEdge[u] = h->edgeCount;
        mnavResult result =
            SearchCluster(h, query, navmesh, t->cluster, IdOf(navmesh, t->slot, t->polygon), t->at);
        if (result != mnav_success)
        {
            return result;
        }
        for (int32_t i = h->firstLeaving[t->cluster]; i < h->firstLeaving[t->cluster + 1]; ++i)
        {
            int32_t v = h->leaving[i];
            double cost = CostTo(query, &h->transitions[v]);
            if (v == t->reverse || !isfinite(cost))
            {
                continue;
            }
            if (h->edgeCount == h->def.limits.edges)
            {
                return mnav_errorLimit;
            }
            h->edges[h->edgeCount++] = (Edge){v, cost};
        }
    }
    h->firstEdge[h->transitionCount] = h->edgeCount;
    return mnav_success;
}

// The scratch for sorting one tile's links: the largest link count.
static size_t MostLinks(const mnavNavmesh* navmesh)
{
    size_t most = 1;
    for (int32_t i = 0; i < navmesh->placeCount; ++i)
    {
        size_t count = (size_t)navmesh->slots[navmesh->places[i].slot].tile->linkCount;
        most = count > most ? count : most;
    }
    return most;
}

static mnavResult Build(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh)
{
    if (navmesh->slotCount > h->def.limits.tiles)
    {
        return mnav_errorLimit;
    }
    FindClusters(h, navmesh);
    size_t most = MostLinks(navmesh);
    uint64_t* sorted = nullptr;
    uint64_t* scratch = nullptr;
    mnavResult result = mnav_success;
    Take(h, &result, most, sizeof(uint64_t), alignof(uint64_t), (void**)&sorted);
    Take(h, &result, most, sizeof(uint64_t), alignof(uint64_t), (void**)&scratch);
    if (result == mnav_success)
    {
        result = FindTransitions(h, navmesh, sorted, scratch);
    }
    Give(h, sorted, most, sizeof(uint64_t), alignof(uint64_t));
    Give(h, scratch, most, sizeof(uint64_t), alignof(uint64_t));
    return result == mnav_success ? FindEdges(h, query, navmesh) : result;
}

mnavResult mnavBuildHierarchy(mnavHierarchy* hierarchy, mnavQuery* query,
                              const mnavNavmesh* navmesh, const mnavQueryFilter* filter,
                              mnavHierarchyReport* reportOut)
{
    if (hierarchy == nullptr || query == nullptr || navmesh == nullptr)
    {
        return mnav_errorInvalid;
    }
    hierarchy->navmesh = nullptr;
    const mnavQueryFilter* usable = nullptr;
    mnavResult result = mnavCheckFilter(filter, &usable);
    if (result != mnav_success)
    {
        return result;
    }
    hierarchy->filter = *usable;
    hierarchy->scale = mnavHeuristicScale(navmesh, &hierarchy->filter);
    result = Build(hierarchy, query, navmesh);
    if (result != mnav_success)
    {
        return result;
    }
    hierarchy->navmesh = navmesh;
    hierarchy->commits = navmesh->commits;
    if (reportOut != nullptr)
    {
        *reportOut = (mnavHierarchyReport){hierarchy->clusterCount, hierarchy->transitionCount,
                                           hierarchy->edgeCount};
    }
    return mnav_success;
}

// The abstract search's open list, by cost and estimate, then by index;
// index transitionCount is the end.
static double Total(const mnavHierarchy* h, int32_t u, mnavPos3 end)
{
    if (u == h->transitionCount)
    {
        return h->costs[u];
    }
    mnavPos3 a = h->transitions[u].at;
    double dx = a.x - end.x;
    double dy = a.y - end.y;
    double dz = a.z - end.z;
    return h->costs[u] + sqrt(dx * dx + dy * dy + dz * dz) * h->scale;
}

static bool Sooner(const mnavHierarchy* h, int32_t a, int32_t b, mnavPos3 end)
{
    double ta = Total(h, a, end);
    double tb = Total(h, b, end);
    return ta != tb ? ta < tb : a < b;
}

static void Place(mnavHierarchy* h, int32_t at, int32_t u)
{
    h->heap[at] = u;
    h->places[u] = at;
}

static void SiftUp(mnavHierarchy* h, int32_t at, mnavPos3 end)
{
    int32_t u = h->heap[at];
    while (at > 0)
    {
        int32_t up = (at - 1) / 2;
        if (!Sooner(h, u, h->heap[up], end))
        {
            break;
        }
        Place(h, at, h->heap[up]);
        at = up;
    }
    Place(h, at, u);
}

static int32_t Pop(mnavHierarchy* h, mnavPos3 end)
{
    int32_t top = h->heap[0];
    int32_t last = h->heap[--h->heapCount];
    int32_t at = 0;
    while (h->heapCount > 0)
    {
        int32_t child = 2 * at + 1;
        if (child >= h->heapCount)
        {
            break;
        }
        if (child + 1 < h->heapCount && Sooner(h, h->heap[child + 1], h->heap[child], end))
        {
            child += 1;
        }
        if (!Sooner(h, h->heap[child], last, end))
        {
            break;
        }
        Place(h, at, h->heap[child]);
        at = child;
    }
    if (h->heapCount > 0)
    {
        Place(h, at, last);
    }
    h->places[top] = MNAV_CLOSED;
    return top;
}

// Lowers u's cost to cost by way of parent, opening it if new.
static void Relax(mnavHierarchy* h, int32_t u, double cost, int32_t parent, mnavPos3 end)
{
    if (h->places[u] == MNAV_CLOSED || cost >= h->costs[u])
    {
        return;
    }
    h->costs[u] = cost;
    h->parents[u] = parent;
    if (h->places[u] == MNAV_NO_NODE)
    {
        Place(h, h->heapCount++, u);
    }
    SiftUp(h, h->places[u], end);
}

// A* over the transitions, opened with the start's costs, to the end,
// reached from each transition with its join; whether it got there.
static bool SearchGraph(mnavHierarchy* h, mnavPos3 end)
{
    int32_t goal = h->transitionCount;
    while (h->heapCount > 0)
    {
        int32_t u = Pop(h, end);
        if (u == goal)
        {
            return true;
        }
        for (int32_t e = h->firstEdge[u]; e < h->firstEdge[u + 1]; ++e)
        {
            Relax(h, h->edges[e].to, h->costs[u] + h->edges[e].cost, u, end);
        }
        if (isfinite(h->joins[u]))
        {
            Relax(h, goal, h->costs[u] + h->joins[u], u, end);
        }
    }
    return false;
}

static void ResetGraph(mnavHierarchy* h)
{
    for (int32_t u = 0; u < h->transitionCount; ++u)
    {
        h->joins[u] = (double)INFINITY;
    }
    for (int32_t u = 0; u <= h->transitionCount; ++u)
    {
        h->costs[u] = (double)INFINITY;
        h->parents[u] = -1;
        h->places[u] = MNAV_NO_NODE;
    }
    h->heapCount = 0;
}

// Joins the end to the transitions entering its cluster, and the start to
// those leaving its own, by searches within the clusters; the walk's cost
// is the same either way, so the end's search runs from the end. A search
// out of nodes joins what it reached.
static mnavResult Join(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh,
                       mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon,
                       mnavPos3 end)
{
    int32_t first = h->clusterOf[startPolygon.slot - 1];
    int32_t last = h->clusterOf[endPolygon.slot - 1];
    ResetGraph(h);
    mnavResult result = SearchCluster(h, query, navmesh, last, endPolygon, end);
    if (result != mnav_success && result != mnav_errorLimit)
    {
        return result;
    }
    for (int32_t i = h->firstLeaving[last]; i < h->firstLeaving[last + 1]; ++i)
    {
        const Transition* v = &h->transitions[h->leaving[i]];
        if (v->reverse >= 0)
        {
            h->joins[v->reverse] = CostTo(query, v);
        }
    }
    result = SearchCluster(h, query, navmesh, first, startPolygon, start);
    if (result != mnav_success && result != mnav_errorLimit)
    {
        return result;
    }
    for (int32_t i = h->firstLeaving[first]; i < h->firstLeaving[first + 1]; ++i)
    {
        int32_t v = h->leaving[i];
        double cost = CostTo(query, &h->transitions[v]);
        if (isfinite(cost))
        {
            Relax(h, v, cost, -1, end);
        }
    }
    return mnav_success;
}

// Runs the search begun until it ends; whether it found its end.
static mnavResult Run(mnavQuery* query, const mnavNavmesh* navmesh, bool* foundOut)
{
    mnavResult result = mnav_success;
    bool ended = false;
    while (result == mnav_success && !ended)
    {
        result = mnavContinuePath(query, navmesh, INT32_MAX, &ended);
    }
    *foundOut = query->search.found != MNAV_NO_NODE;
    return result;
}

// One step of the abstract path: from the node the search stands on, the
// way within cluster from to the transition's crossing.
static mnavResult Step(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh, int32_t from,
                       const Transition* t, bool* foundOut)
{
    Mark(h, from, 1);
    mnavConfineSearch(query, h->inside, true, false);
    const int32_t goal[4] = {t->slot, t->polygon, t->tag, t->low};
    mnavAimSearch(query, t->at, -1, -1, goal);
    mnavResult result = Run(query, navmesh, foundOut);
    Mark(h, from, 0);
    if (result == mnav_success && *foundOut)
    {
        mnavRestartSearch(query, query->search.found);
    }
    return result;
}

// Refines the abstract path step by step, each step's search confined to
// one cluster and started over from where the last one ended, so that
// the nodes held are the way so far and one cluster's; then the last
// cluster to the end. Whether every step found its way.
static mnavResult Refine(mnavHierarchy* h, mnavQuery* query, const mnavNavmesh* navmesh,
                         mnavPolygonId startPolygon, mnavPos3 start, mnavPolygonId endPolygon,
                         mnavPos3 end, bool* foundOut)
{
    // The transitions in order, into the abstract search's heap.
    int32_t count = 0;
    for (int32_t u = h->parents[h->transitionCount]; u >= 0; u = h->parents[u])
    {
        h->heap[count++] = u;
    }
    mnavResult result =
        mnavBeginPath(query, navmesh, &h->filter, startPolygon, start, endPolygon, end);
    int32_t cluster = h->clusterOf[startPolygon.slot - 1];
    *foundOut = true;
    for (int32_t i = count - 1; i >= 0 && result == mnav_success && *foundOut; --i)
    {
        const Transition* t = &h->transitions[h->heap[i]];
        result = Step(h, query, navmesh, cluster, t, foundOut);
        cluster = t->cluster;
    }
    if (result != mnav_success || !*foundOut)
    {
        return result;
    }
    Mark(h, cluster, 1);
    mnavConfineSearch(query, h->inside, false, false);
    const int32_t none[4] = {-1, -1, -1, -1};
    mnavAimSearch(query, end, (int32_t)endPolygon.slot - 1, (int32_t)endPolygon.polygon, none);
    result = Run(query, navmesh, foundOut);
    Mark(h, cluster, 0);
    return result;
}

static bool FinitePoint(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

static mnavResult CheckQuery(const mnavQuery* query, const mnavHierarchy* h,
                             const mnavNavmesh* navmesh, mnavPolygonId startPolygon, mnavPos3 start,
                             mnavPolygonId endPolygon, mnavPos3 end)
{
    if (query == nullptr || h == nullptr || navmesh == nullptr || !FinitePoint(start) ||
        !FinitePoint(end) || h->navmesh != navmesh)
    {
        return mnav_errorInvalid;
    }
    if (h->commits != navmesh->commits)
    {
        return mnav_errorStale;
    }
    mnavResult result = mnavCheckPolygon(navmesh, startPolygon);
    return result == mnav_success ? mnavCheckPolygon(navmesh, endPolygon) : result;
}

mnavResult mnavFindHierarchicalPath(mnavQuery* query, mnavHierarchy* hierarchy,
                                    const mnavNavmesh* navmesh, mnavPolygonId startPolygon,
                                    mnavPos3 start, mnavPolygonId endPolygon, mnavPos3 end,
                                    mnavPath* pathOut)
{
    mnavHierarchy* h = hierarchy;
    if (pathOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    mnavResult result = CheckQuery(query, h, navmesh, startPolygon, start, endPolygon, end);
    if (result != mnav_success)
    {
        return result;
    }
    int32_t first = h->clusterOf[startPolygon.slot - 1];
    if (first == h->clusterOf[endPolygon.slot - 1])
    {
        return mnavFindPath(query, navmesh, &h->filter, startPolygon, start, endPolygon, end,
                            pathOut);
    }
    result = Join(h, query, navmesh, startPolygon, start, endPolygon, end);
    if (result != mnav_success)
    {
        return result;
    }
    if (!SearchGraph(h, end))
    {
        return mnavFindPath(query, navmesh, &h->filter, startPolygon, start, endPolygon, end,
                            pathOut);
    }
    bool found = false;
    result = Refine(h, query, navmesh, startPolygon, start, endPolygon, end, &found);
    if (result != mnav_success)
    {
        return result;
    }
    if (!found)
    {
        return mnavFindPath(query, navmesh, &h->filter, startPolygon, start, endPolygon, end,
                            pathOut);
    }
    return mnavFinishPath(query, navmesh, pathOut);
}
