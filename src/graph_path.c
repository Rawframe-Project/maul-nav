// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph paths: A* over a host's graph of points and edges. The heuristic
// is the straight distance to the end times the cheapest included area's
// cost, scaled by the graph's least ratio of an edge's length to its
// straight distance, so it never expects more than the way left costs.
// Nodes, the open list and the node table live in the query context's
// memory, which serves one query at a time.

#include "fpenv.h"
#include "graph.h"
#include "invariant.h"
#include "outputs.h"
#include "query.h"
#include "query_filter.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/graph.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static_assert(sizeof(mnavGraphNode) <= sizeof(mnavSearchNode), "graph nodes fit the nodes");
static_assert(sizeof(int32_t) <= sizeof(mnavPos3), "graph points fit the points");

typedef mnavGraphNode GraphNode;

typedef struct Search
{
    mnavQuery* query;
    GraphNode* nodes;
    const mnavGraph* graph;
    const mnavQueryFilter* filter;
    int32_t end;
    // The heuristic's scale per meter, and the share of the straight
    // distance the way left is at least.
    double cheapest;
    double shortcut;
    double limit;
    bool outOfNodes;
    bool tooLong;
    int32_t found;
    int32_t best;
} Search;

// The straight distance between two points of the graph, in meters.
static double Straight(const Search* s, int32_t a, int32_t b)
{
    mnavPos3 p = s->graph->points[a];
    mnavPos3 q = s->graph->points[b];
    double dx = q.x - p.x;
    double dy = q.y - p.y;
    double dz = q.z - p.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// The open list, a binary heap by total cost, then by node index.
MNAV_NODISCARD static bool Sooner(const GraphNode* nodes, int32_t a, int32_t b)
{
    double ta = nodes[a].cost + nodes[a].remaining;
    double tb = nodes[b].cost + nodes[b].remaining;
    return ta != tb ? ta < tb : a < b;
}

static void Place(Search* s, int32_t at, int32_t node)
{
    s->query->indexHeap[at] = node;
    s->nodes[node].heap = at;
}

static void SiftUp(Search* s, int32_t at)
{
    int32_t node = s->query->indexHeap[at];
    while (at > 0)
    {
        int32_t up = (at - 1) / 2;
        if (!Sooner(s->nodes, node, s->query->indexHeap[up]))
        {
            break;
        }
        Place(s, at, s->query->indexHeap[up]);
        at = up;
    }
    Place(s, at, node);
}

static int32_t Pop(Search* s)
{
    mnavQuery* q = s->query;
    MNAV_ASSERT(q->heapCount > 0 && s->nodes[q->indexHeap[0]].heap == 0);
    int32_t top = q->indexHeap[0];
    int32_t last = q->indexHeap[--q->heapCount];
    int32_t at = 0;
    while (q->heapCount > 0)
    {
        int32_t child = 2 * at + 1;
        if (child >= q->heapCount)
        {
            break;
        }
        if (child + 1 < q->heapCount &&
            Sooner(s->nodes, q->indexHeap[child + 1], q->indexHeap[child]))
        {
            child += 1;
        }
        if (!Sooner(s->nodes, q->indexHeap[child], last))
        {
            break;
        }
        Place(s, at, q->indexHeap[child]);
        at = child;
    }
    if (q->heapCount > 0)
    {
        Place(s, at, last);
    }
    s->nodes[top].heap = MNAV_CLOSED;
    return top;
}

// The table cell for a point's node, or the empty one where it goes.
static uint32_t Slot(const Search* s, int32_t point)
{
    uint32_t at = ((uint32_t)point * 2654435761u) & s->query->tableMask;
    for (;;)
    {
        int32_t node = s->query->table[at];
        if (node == MNAV_NO_NODE || s->nodes[node].point == point)
        {
            return at;
        }
        at = (at + 1) & s->query->tableMask;
    }
}

// The way left from a point to the end: its least length and the
// heuristic.
static double LeastLeft(const Search* s, int32_t point)
{
    return Straight(s, point, s->end) * s->shortcut;
}

// Reaches a point from node from by an arc; opens or improves its node.
static void Reach(Search* s, int32_t from, const mnavGraphArc* arc)
{
    mnavQuery* q = s->query;
    const GraphNode* parent = &s->nodes[from];
    double least = LeastLeft(s, arc->to);
    if (parent->length + arc->length + least > s->limit)
    {
        s->tooLong = true;
        return;
    }
    uint32_t at = Slot(s, arc->to);
    int32_t node = q->table[at];
    if (node == MNAV_NO_NODE)
    {
        if (q->nodeCount == q->limits.nodes)
        {
            s->outOfNodes = true;
            return;
        }
        node = q->nodeCount++;
        q->table[at] = node;
        s->nodes[node] = (GraphNode){arc->to,          MNAV_NO_NODE, MNAV_NO_NODE,
                                     (double)INFINITY, 0.0,          least * s->cheapest};
    }
    GraphNode* n = &s->nodes[node];
    double total = parent->cost + arc->length * (double)s->filter->costs[arc->area];
    // Closed nodes are final, as in the other searches: with a consistent
    // heuristic only rounding finds a cheaper way to one.
    if (n->heap == MNAV_CLOSED || total >= n->cost)
    {
        return;
    }
    n->cost = total;
    n->length = parent->length + arc->length;
    n->parent = from;
    // Each node enters the open list once, and the nodes made are at most
    // its room; one in it is where the open list holds it.
    MNAV_ASSERT(n->heap == MNAV_NO_NODE ? q->heapCount < q->limits.nodes
                                        : q->indexHeap[n->heap] == node);
    if (n->heap == MNAV_NO_NODE)
    {
        Place(s, q->heapCount++, node);
    }
    SiftUp(s, n->heap);
}

static void Expand(Search* s, int32_t node)
{
    const mnavGraph* g = s->graph;
    int32_t point = s->nodes[node].point;
    for (int32_t a = g->first[point]; a < g->first[point + 1]; ++a)
    {
        const mnavGraphArc* arc = &g->arcs[a];
        if (mnavIncludes(s->filter, arc->area))
        {
            Reach(s, node, arc);
        }
    }
}

// Whether node a lies nearer the end than node b: by the heuristic, then
// the cost, then the index.
MNAV_NODISCARD static bool Nearer(const GraphNode* nodes, int32_t a, int32_t b)
{
    if (nodes[a].remaining != nodes[b].remaining)
    {
        return nodes[a].remaining < nodes[b].remaining;
    }
    return nodes[a].cost != nodes[b].cost ? nodes[a].cost < nodes[b].cost : a < b;
}

// Writes the path to a node: its points, from the start.
static mnavGraphPath PathTo(const Search* s, int32_t last, mnavPathEnd end)
{
    int32_t* points = s->query->graphPoints;
    int32_t count = 0;
    for (int32_t n = last; n != MNAV_NO_NODE; n = s->nodes[n].parent)
    {
        // Each node made is on the way once at most.
        MNAV_ASSERT(count < s->query->limits.nodes);
        points[count++] = s->nodes[n].point;
    }
    for (int32_t i = 0; i < count / 2; ++i)
    {
        int32_t swap = points[i];
        points[i] = points[count - 1 - i];
        points[count - 1 - i] = swap;
    }
    return (mnavGraphPath){end, s->nodes[last].cost, s->nodes[last].length, points, count};
}

static MNAV_NOINLINE mnavResult FindGraphPathWork(mnavQuery* query, const mnavGraph* graph,
                                                  const mnavQueryFilter* filter, int32_t start,
                                                  int32_t end, mnavGraphPath* pathOut)
{
    if (query != nullptr && mnavHostCalledBack(&query->host))
    {
        return mnav_errorBusy;
    }
    if (query == nullptr || graph == nullptr || pathOut == nullptr || start < 0 ||
        start >= graph->pointCount || end < 0 || end >= graph->pointCount)
    {
        return mnav_errorInvalid;
    }
    const mnavQueryFilter* usable = nullptr;
    mnavResult checked = mnavCheckFilter(filter, &usable);
    if (checked != mnav_success)
    {
        return checked;
    }
    // The filter is copied: the caller's may change before the next query.
    query->filter = *usable;
    query->search.active = false;
    query->flight.active = false;
    Search s = {query,
                query->graphNodes,
                graph,
                &query->filter,
                end,
                mnavCheapest(&query->filter),
                graph->shortcut,
                (double)query->limits.pathLength,
                false,
                false,
                MNAV_NO_NODE,
                0};
    mnavClearTable(query);
    query->heapCount = 0;
    query->nodeCount = 1;
    s.nodes[0] =
        (GraphNode){start, MNAV_NO_NODE, MNAV_NO_NODE, 0.0, 0.0, LeastLeft(&s, start) * s.cheapest};
    query->table[Slot(&s, start)] = 0;
    Place(&s, query->heapCount++, 0);
    while (query->heapCount > 0)
    {
        int32_t node = Pop(&s);
        s.best = Nearer(s.nodes, node, s.best) ? node : s.best;
        if (s.nodes[node].point == end)
        {
            s.found = node;
            break;
        }
        Expand(&s, node);
    }
    mnavPathEnd ending = s.found != MNAV_NO_NODE ? mnav_pathFound
                         : s.outOfNodes          ? mnav_pathOutOfNodes
                         : s.tooLong             ? mnav_pathTooLong
                                                 : mnav_pathNone;
    *pathOut = PathTo(&s, s.found != MNAV_NO_NODE ? s.found : s.best, ending);
    return mnav_success;
}

static MNAV_NOINLINE mnavResult FindGraphPathCounted(mnavQuery* query, const mnavGraph* graph,
                                                     const mnavQueryFilter* filter, int32_t start,
                                                     int32_t end, mnavGraphPath* pathOut)
{
    mnavGraphPath pathMade = {0};
    mnavResult result = mnavHostCountInvalid(
        MNAV_HOST(query), FindGraphPathWork(query, graph, filter, start, end,
                                            pathOut != nullptr ? &pathMade : nullptr));
    mnavGiveOutput(result, false, pathOut, &pathMade, sizeof(pathMade));
    return result;
}

mnavResult mnavQuery_FindGraphPath(mnavQuery* query, const mnavGraph* graph, int32_t start,
                                   int32_t end, const mnavQueryFilter* filter,
                                   mnavGraphPath* pathOut)
{
    uint64_t env = mnavFloatEnvEnter();
    mnavResult result = FindGraphPathCounted(query, graph, filter, start, end, pathOut);
    mnavFloatEnvLeave(env);
    return result;
}
