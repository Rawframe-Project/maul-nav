// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs: made with the memory their limits need, then given points and
// edges, which are checked whole before any is kept, so that a refused set
// leaves the graph as it was. The edges out of each point lie together,
// in the order given.

#include "graph.h"

#include "allocator.h"
#include "fpenv.h"
#include "host_code.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/graph.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static MNAV_NOINLINE mnavResult CreateGraphWork(const mnavGraphDef* def, mnavGraph** graphOut)
{
    if (graphOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *graphOut = nullptr;
    if (def == nullptr || def->cookie != MNAV_GRAPH_DEF_COOKIE)
    {
        return mnav_errorInvalid;
    }
    if (def->version != MNAV_ABI_VERSION)
    {
        return mnav_errorVersion;
    }
    if ((def->allocator.alloc == nullptr) != (def->allocator.free == nullptr) ||
        def->limits.points < 1 || def->limits.edges < 0)
    {
        return mnav_errorInvalid;
    }
    if (def->limits.points > MNAV_MAX_GRAPH_POINTS || def->limits.edges > MNAV_MAX_GRAPH_EDGES)
    {
        return mnav_errorLimit;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavGraph* g = nullptr;
    mnavResult result = mnavAllocate(&memory, 1, sizeof(mnavGraph), alignof(mnavGraph), (void**)&g);
    if (result != mnav_success)
    {
        return result;
    }
    *g = (mnavGraph){0};
    g->memory = memory;
    g->memory.host = &g->host;
    g->def = *def;
    g->shortcut = 1.0;
    size_t points = (size_t)def->limits.points;
    result =
        mnavAllocate(&g->memory, points, sizeof(mnavPos3), alignof(mnavPos3), (void**)&g->points);
    if (result == mnav_success)
    {
        result = mnavAllocate(&g->memory, points + 1, sizeof(int32_t), alignof(int32_t),
                              (void**)&g->first);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&g->memory, 2 * (size_t)def->limits.edges, sizeof(mnavGraphArc),
                              alignof(mnavGraphArc), (void**)&g->arcs);
    }
    if (result != mnav_success)
    {
        mnavDestroyGraph(g);
        return result;
    }
    g->first[0] = 0;
    *graphOut = g;
    return mnav_success;
}

mnavResult mnavCreateGraph(const mnavGraphDef* def, mnavGraph** graphOut)
{
    uint64_t env = mnavFloatEnvEnter();
    mnavResult result = CreateGraphWork(def, graphOut);
    mnavFloatEnvLeave(env);
    return result;
}

uint64_t mnavGraph_GetMisuse(const mnavGraph* graph)
{
    return graph != nullptr ? mnavHostMisuse(&graph->host) : 0;
}

void mnavDestroyGraph(mnavGraph* graph)
{
    if (graph == nullptr || mnavHostCalledBack(&graph->host))
    {
        return;
    }
    mnavMemory memory = graph->memory;
    size_t points = (size_t)graph->def.limits.points;
    mnavRelease(&memory, graph->arcs, 2 * (size_t)graph->def.limits.edges, sizeof(mnavGraphArc),
                alignof(mnavGraphArc));
    mnavRelease(&memory, graph->first, points + 1, sizeof(int32_t), alignof(int32_t));
    mnavRelease(&memory, graph->points, points, sizeof(mnavPos3), alignof(mnavPos3));
    // The mark lives in the block being freed: it is set for good, so that
    // a call back in from this release is refused too, and the release
    // does not touch it.
    mnavHostEnter(&graph->host);
    memory.host = nullptr;
    mnavRelease(&memory, graph, 1, sizeof(mnavGraph), alignof(mnavGraph));
}

// The straight distance between two points, in meters.
static double Distance(mnavPos3 a, mnavPos3 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = b.z - a.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static mnavResult CheckPoints(const mnavPos3* points, int32_t count)
{
    for (int32_t p = 0; p < count; ++p)
    {
        const mnavPos3* v = &points[p];
        if (!isfinite(v->x) || !isfinite(v->y) || !isfinite(v->z))
        {
            return mnav_errorInvalid;
        }
        if (fabs(v->x) > MNAV_MAX_QUERY_DISTANCE || fabs(v->y) > MNAV_MAX_QUERY_DISTANCE ||
            fabs(v->z) > MNAV_MAX_QUERY_DISTANCE)
        {
            return mnav_errorLimit;
        }
    }
    return mnav_success;
}

static mnavResult CheckEdges(const mnavGraphEdge* edges, int32_t count, int32_t points)
{
    for (int32_t e = 0; e < count; ++e)
    {
        const mnavGraphEdge* edge = &edges[e];
        if (edge->from < 0 || edge->from >= points || edge->to < 0 || edge->to >= points ||
            edge->area == mnav_areaNone || edge->area >= MNAV_AREA_TYPES || !(edge->length >= 0.0))
        {
            return mnav_errorInvalid;
        }
        if (edge->length > (double)MNAV_MAX_PATH_LENGTH)
        {
            return mnav_errorLimit;
        }
    }
    return mnav_success;
}

// Keeps the points and lays the edges out by the point each leaves, every
// point's in the order given: counts first, then each arc at its point's
// next place, then the places shifted back to each point's first.
static void Keep(mnavGraph* g, const mnavPos3* points, int32_t pointCount,
                 const mnavGraphEdge* edges, int32_t edgeCount)
{
    for (int32_t p = 0; p < pointCount; ++p)
    {
        g->points[p] = points[p];
    }
    for (int32_t p = 0; p <= pointCount; ++p)
    {
        g->first[p] = 0;
    }
    for (int32_t e = 0; e < edgeCount; ++e)
    {
        g->first[edges[e].from + 1] += 1;
        g->first[edges[e].to + 1] += edges[e].oneWay ? 0 : 1;
    }
    for (int32_t p = 0; p < pointCount; ++p)
    {
        g->first[p + 1] += g->first[p];
    }
    double shortcut = 1.0;
    for (int32_t e = 0; e < edgeCount; ++e)
    {
        const mnavGraphEdge* edge = &edges[e];
        double distance = Distance(points[edge->from], points[edge->to]);
        double length = edge->length > 0.0 ? edge->length : distance;
        if (length < distance)
        {
            double ratio = length / distance;
            shortcut = ratio < shortcut ? ratio : shortcut;
        }
        g->arcs[g->first[edge->from]++] = (mnavGraphArc){length, edge->to, edge->area};
        if (!edge->oneWay)
        {
            g->arcs[g->first[edge->to]++] = (mnavGraphArc){length, edge->from, edge->area};
        }
    }
    // Each point's place now lies at the next one's first.
    for (int32_t p = pointCount; p > 0; --p)
    {
        g->first[p] = g->first[p - 1];
    }
    g->first[0] = 0;
    g->pointCount = pointCount;
    g->arcCount = g->first[pointCount];
    g->shortcut = shortcut;
}

static MNAV_NOINLINE mnavResult SetGraphWork(mnavGraph* graph, const mnavPos3* points,
                                             int32_t pointCount, const mnavGraphEdge* edges,
                                             int32_t edgeCount)
{
    if (graph != nullptr && mnavHostCalledBack(&graph->host))
    {
        return mnav_errorBusy;
    }
    if (graph == nullptr || pointCount < 0 || edgeCount < 0 ||
        (pointCount > 0 && points == nullptr) || (edgeCount > 0 && edges == nullptr))
    {
        return mnav_errorInvalid;
    }
    mnavResult result = CheckPoints(points, pointCount);
    if (result == mnav_success)
    {
        result = CheckEdges(edges, edgeCount, pointCount);
    }
    if (result != mnav_success)
    {
        return result;
    }
    if (pointCount > graph->def.limits.points || edgeCount > graph->def.limits.edges)
    {
        return mnav_errorCapacity;
    }
    Keep(graph, points, pointCount, edges, edgeCount);
    return mnav_success;
}

static MNAV_NOINLINE mnavResult SetGraphCounted(mnavGraph* graph, const mnavPos3* points,
                                                int32_t pointCount, const mnavGraphEdge* edges,
                                                int32_t edgeCount)
{
    return mnavHostCountInvalid(MNAV_HOST(graph),
                                SetGraphWork(graph, points, pointCount, edges, edgeCount));
}

mnavResult mnavGraph_Set(mnavGraph* graph, const mnavPos3* points, int32_t pointCount,
                         const mnavGraphEdge* edges, int32_t edgeCount)
{
    uint64_t env = mnavFloatEnvEnter();
    mnavResult result = SetGraphCounted(graph, points, pointCount, edges, edgeCount);
    mnavFloatEnvLeave(env);
    return result;
}
