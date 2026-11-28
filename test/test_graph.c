// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph paths: the cheapest way over points and edges, its ties, areas,
// one-way edges and edges shorter than the distance they span, against a
// Dijkstra search on random graphs; the limits and ends; the graph's
// checks, a refused set leaving it as it was.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/graph.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    // The random graphs' points and the edges out of each.
    RANDOM_POINTS = 240,
    RANDOM_OUT = 3,
    RANDOM_EDGES = RANDOM_POINTS * RANDOM_OUT
};

static mnavQuery* MakeQuery(int32_t nodes, float pathLength)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    def.limits.pathLength = pathLength;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

static mnavGraph* MakeGraph(int32_t points, int32_t edges)
{
    mnavGraphDef def = mnavDefaultGraphDef();
    def.limits.points = points;
    def.limits.edges = edges;
    mnavGraph* graph = nullptr;
    CHECK(mnavCreateGraph(&def, &graph) == mnav_success, "graph");
    return graph;
}

static mnavGraphEdge Edge(int32_t from, int32_t to, double length, mnavAreaType area, bool oneWay)
{
    mnavGraphEdge edge = {from, to, length, area, oneWay};
    return edge;
}

static bool PathIs(const mnavGraphPath* path, const int32_t* points, int32_t count)
{
    if (path->pointCount != count)
    {
        return false;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (path->points[i] != points[i])
        {
            return false;
        }
    }
    return true;
}

// A 3 by 4 m rectangle: A (0, 0), B (3, 0), C (3, 4), D (0, 4), its sides
// and the diagonal from A to C, of area 2.
static const mnavPos3 s_box[4] = {
    {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 0.0, 4.0}, {0.0, 0.0, 4.0}};

static void SetBox(mnavGraph* graph)
{
    const mnavGraphEdge edges[5] = {Edge(0, 1, 0.0, 1, false), Edge(1, 2, 0.0, 1, false),
                                    Edge(0, 3, 0.0, 1, false), Edge(3, 2, 0.0, 1, false),
                                    Edge(0, 2, 0.0, 2, false)};
    CHECK(mnavGraph_Set(graph, s_box, 4, edges, 5) == mnav_success, "the box");
}

static void TestCheapestAndTies(void)
{
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavGraph* graph = MakeGraph(8, 8);
    SetBox(graph);
    mnavGraphPath path;
    // At one cost for every area the diagonal, 5 m.
    const int32_t straight[2] = {0, 2};
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 2, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 5.0 && path.length == 5.0 &&
              PathIs(&path, straight, 2),
          "the diagonal");
    // Area 2 at 3 per meter costs 15: round the sides, 7 m either way. B is
    // reached first, by the first edge out of A, and keeps C at the tie.
    mnavQueryFilter dear = mnavDefaultQueryFilter();
    dear.costs[2] = 3.0f;
    const int32_t byB[3] = {0, 1, 2};
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 2, &dear, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 7.0 && path.length == 7.0 &&
              PathIs(&path, byB, 3),
          "round by B at the tie");
    // Area 2 left out: the same.
    mnavQueryFilter without = mnavDefaultQueryFilter();
    without.areas &= ~((uint64_t)1 << 2);
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 2, &without, &path) == mnav_success &&
              path.end == mnav_pathFound && PathIs(&path, byB, 3),
          "area 2 left out");
    // Area 1 at 2 per meter: the diagonal again, at 5.
    mnavQueryFilter sides = mnavDefaultQueryFilter();
    sides.costs[1] = 2.0f;
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 2, &sides, &path) == mnav_success &&
              path.cost == 5.0 && PathIs(&path, straight, 2),
          "dear sides");
    // From C to A, round the sides: C's first edge given is B's, so B is
    // reached first and keeps A at the tie.
    const int32_t back[3] = {2, 1, 0};
    CHECK(mnavQuery_FindGraphPath(query, graph, 2, 0, &without, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 7.0 && PathIs(&path, back, 3),
          "back by B, by the order of the edges");
    // A start at its end: the point alone.
    CHECK(mnavQuery_FindGraphPath(query, graph, 1, 1, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 0.0 && path.pointCount == 1 &&
              path.points[0] == 1,
          "start at the end");
    mnavDestroyGraph(graph);
    mnavDestroyQuery(query);
}

static void TestOneWayAndLengths(void)
{
    mnavQuery* query = MakeQuery(64, 1000.0f);
    mnavGraph* graph = MakeGraph(8, 8);
    const mnavPos3 points[3] = {{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, {50.0, 0.0, 0.0}};
    // 0 to 1 by a lift of 1 m, one way; back by the road through 2, 100 m.
    const mnavGraphEdge edges[3] = {Edge(0, 1, 1.0, 1, true), Edge(1, 2, 0.0, 1, false),
                                    Edge(2, 0, 0.0, 1, false)};
    CHECK(mnavGraph_Set(graph, points, 3, edges, 3) == mnav_success, "set");
    mnavGraphPath path;
    const int32_t lift[2] = {0, 1};
    const int32_t road[3] = {1, 2, 0};
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 1, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 1.0 && path.length == 1.0 &&
              PathIs(&path, lift, 2),
          "the lift, shorter than the distance it spans");
    CHECK(mnavQuery_FindGraphPath(query, graph, 1, 0, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.cost == 100.0 && PathIs(&path, road, 3),
          "back by the road, the lift one way");
    // Only the lift between 0 and 1: no way back.
    CHECK(mnavGraph_Set(graph, points, 3, edges, 1) == mnav_success &&
              mnavQuery_FindGraphPath(query, graph, 1, 0, nullptr, &path) == mnav_success &&
              path.end == mnav_pathNone && path.pointCount == 1 && path.points[0] == 1,
          "no way against a one-way edge");
    // A road given a length past its distance: its length counts.
    const mnavGraphEdge longer[2] = {Edge(1, 2, 500.0, 1, false), Edge(2, 0, 0.0, 1, false)};
    CHECK(mnavGraph_Set(graph, points, 3, longer, 2) == mnav_success &&
              mnavQuery_FindGraphPath(query, graph, 1, 0, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.length == 550.0,
          "an edge longer than its distance");
    mnavDestroyGraph(graph);
    mnavDestroyQuery(query);
}

static void TestLimitsAndEnds(void)
{
    // A chain of 10 points 1 m apart, and point 10 beside point 0 with no
    // edge.
    mnavPos3 points[11];
    mnavGraphEdge edges[9];
    for (int32_t i = 0; i < 10; ++i)
    {
        points[i] = (mnavPos3){(double)i, 0.0, 0.0};
    }
    points[10] = (mnavPos3){0.0, 0.0, 1.0};
    for (int32_t i = 0; i < 9; ++i)
    {
        edges[i] = Edge(i, i + 1, 0.0, 1, false);
    }
    mnavGraph* graph = MakeGraph(16, 16);
    CHECK(mnavGraph_Set(graph, points, 11, edges, 9) == mnav_success, "the chain");
    mnavGraphPath path;
    mnavQuery* query = MakeQuery(4, 1000.0f);
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 9, nullptr, &path) == mnav_success &&
              path.end == mnav_pathOutOfNodes && path.pointCount == 4 && path.points[3] == 3,
          "out of nodes: to the point nearest the end");
    mnavDestroyQuery(query);
    query = MakeQuery(64, 5.0f);
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 9, nullptr, &path) == mnav_success &&
              path.end == mnav_pathTooLong && path.pointCount == 1,
          "too long: no first step leads within the length");
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 5, nullptr, &path) == mnav_success &&
              path.end == mnav_pathFound && path.length == 5.0,
          "a path as long as the limit");
    mnavDestroyQuery(query);
    query = MakeQuery(64, 1000.0f);
    CHECK(mnavQuery_FindGraphPath(query, graph, 10, 9, nullptr, &path) == mnav_success &&
              path.end == mnav_pathNone && path.pointCount == 1 && path.points[0] == 10,
          "no edge: the start alone");
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 10, nullptr, &path) == mnav_success &&
              path.end == mnav_pathNone && path.points[path.pointCount - 1] == 0,
          "unreached: the point nearest the end");
    mnavDestroyQuery(query);
    mnavDestroyGraph(graph);
}

static void TestChecks(void)
{
    mnavGraph* graph = nullptr;
    mnavGraphDef def = mnavDefaultGraphDef();
    CHECK(def.limits.points == 4096 && def.limits.edges == 16384, "the default limits");
    CHECK(mnavCreateGraph(nullptr, &graph) == mnav_errorInvalid && graph == nullptr &&
              mnavCreateGraph(&def, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavGraphDef bad = def;
    bad.limits.points = 0;
    CHECK(mnavCreateGraph(&bad, &graph) == mnav_errorInvalid, "no points");
    bad = def;
    bad.limits.edges = -1;
    CHECK(mnavCreateGraph(&bad, &graph) == mnav_errorInvalid, "negative edges");
    bad = def;
    bad.limits.points = MNAV_MAX_GRAPH_POINTS + 1;
    CHECK(mnavCreateGraph(&bad, &graph) == mnav_errorLimit, "too many points");
    bad = def;
    bad.limits.edges = MNAV_MAX_GRAPH_EDGES + 1;
    CHECK(mnavCreateGraph(&bad, &graph) == mnav_errorLimit, "too many edges");
    bad = def;
    bad.limits.edges = 0;
    CHECK(mnavCreateGraph(&bad, &graph) == mnav_success, "no edges");
    mnavDestroyGraph(graph);
    mnavDestroyGraph(nullptr);
    CHECK(mnavGraph_GetMisuse(nullptr) == 0, "no misuse of nothing");

    graph = MakeGraph(4, 5);
    SetBox(graph);
    mnavQuery* query = MakeQuery(64, 1000.0f);
    const mnavGraphEdge good = Edge(0, 1, 0.0, 1, false);
    mnavPos3 points[5] = {s_box[0], s_box[1], s_box[2], s_box[3], {0.0, 0.0, 0.0}};
    struct
    {
        const char* what;
        int32_t pointCount;
        mnavGraphEdge edge;
        mnavPos3 point;
        mnavResult result;
    } cases[] = {
        {"an edge from outside", 4, Edge(4, 1, 0.0, 1, false), s_box[0], mnav_errorInvalid},
        {"an edge to outside", 4, Edge(0, -1, 0.0, 1, false), s_box[0], mnav_errorInvalid},
        {"area 0", 4, Edge(0, 1, 0.0, 0, false), s_box[0], mnav_errorInvalid},
        {"an area past the last", 4, Edge(0, 1, 0.0, MNAV_AREA_TYPES, false), s_box[0],
         mnav_errorInvalid},
        {"a negative length", 4, Edge(0, 1, -1.0, 1, false), s_box[0], mnav_errorInvalid},
        {"a NaN length", 4, Edge(0, 1, (double)NAN, 1, false), s_box[0], mnav_errorInvalid},
        {"a length past the limit", 4, Edge(0, 1, 2.0e7, 1, false), s_box[0], mnav_errorLimit},
        {"an infinite length", 4, Edge(0, 1, (double)INFINITY, 1, false), s_box[0],
         mnav_errorLimit},
        {"a point not finite", 4, good, (mnavPos3){(double)NAN, 0.0, 0.0}, mnav_errorInvalid},
        {"a point far off", 4, good, (mnavPos3){0.0, 2.0e100, 0.0}, mnav_errorLimit},
        {"more points than the limit", 5, good, s_box[0], mnav_errorCapacity},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        points[0] = cases[c].point;
        const mnavGraphEdge edges[2] = {good, cases[c].edge};
        CHECK(mnavGraph_Set(graph, points, cases[c].pointCount, edges, 2) == cases[c].result,
              cases[c].what);
    }
    points[0] = s_box[0];
    mnavGraphEdge many[6];
    for (int32_t e = 0; e < 6; ++e)
    {
        many[e] = good;
    }
    CHECK(mnavGraph_Set(graph, points, 4, many, 6) == mnav_errorCapacity,
          "more edges than the limit");
    CHECK(mnavGraph_Set(nullptr, points, 4, many, 1) == mnav_errorInvalid &&
              mnavGraph_Set(graph, nullptr, 4, many, 1) == mnav_errorInvalid &&
              mnavGraph_Set(graph, points, 4, nullptr, 1) == mnav_errorInvalid &&
              mnavGraph_Set(graph, points, -1, many, 1) == mnav_errorInvalid &&
              mnavGraph_Set(graph, points, 4, many, -1) == mnav_errorInvalid,
          "NULL arrays and negative counts");
    // Every refusal left the box: the diagonal, then round by B.
    mnavGraphPath path;
    mnavQueryFilter dear = mnavDefaultQueryFilter();
    dear.costs[2] = 3.0f;
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 2, &dear, &path) == mnav_success &&
              path.cost == 7.0 && path.pointCount == 3 && path.points[1] == 1,
          "a refused set leaves the graph");
    // The search's checks; the path is zeroed on each.
    const mnavGraphPath zero = {0};
    memset(&path, 0x5A, sizeof(path));
    CHECK(mnavQuery_FindGraphPath(query, graph, 4, 0, nullptr, &path) == mnav_errorInvalid &&
              memcmp(&path, &zero, sizeof(path)) == 0,
          "a start outside, the path zeroed");
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, -1, nullptr, &path) == mnav_errorInvalid &&
              mnavQuery_FindGraphPath(nullptr, graph, 0, 1, nullptr, &path) == mnav_errorInvalid &&
              mnavQuery_FindGraphPath(query, nullptr, 0, 1, nullptr, &path) == mnav_errorInvalid &&
              mnavQuery_FindGraphPath(query, graph, 0, 1, nullptr, nullptr) == mnav_errorInvalid,
          "an end outside, NULL arguments");
    mnavQueryFilter broken = mnavDefaultQueryFilter();
    broken.cookie = 0;
    mnavQueryFilter far = mnavDefaultQueryFilter();
    far.costs[1] = (float)INFINITY;
    CHECK(mnavQuery_FindGraphPath(query, graph, 0, 1, &broken, &path) == mnav_errorInvalid &&
              mnavQuery_FindGraphPath(query, graph, 0, 1, &far, &path) == mnav_errorLimit,
          "the filter's checks");
    // An empty graph has no point to start at.
    CHECK(mnavGraph_Set(graph, nullptr, 0, nullptr, 0) == mnav_success &&
              mnavQuery_FindGraphPath(query, graph, 0, 0, nullptr, &path) == mnav_errorInvalid,
          "an empty graph");
    mnavDestroyQuery(query);
    mnavDestroyGraph(graph);
}

// The random graphs' state, and a draw below n.
static uint32_t s_state = 12345u;

static uint32_t Next(uint32_t n)
{
    s_state = s_state * 1664525u + 1013904223u;
    return (s_state >> 8) % n;
}

static mnavPos3 s_points[RANDOM_POINTS];
static mnavGraphEdge s_edges[RANDOM_EDGES];

// The cheapest cost from a to every point by Dijkstra's search, scanning
// for the cheapest point not done; INFINITY where none leads.
static void Costs(const mnavQueryFilter* filter, int32_t a, double* costs)
{
    bool done[RANDOM_POINTS];
    for (int32_t p = 0; p < RANDOM_POINTS; ++p)
    {
        costs[p] = (double)INFINITY;
        done[p] = false;
    }
    costs[a] = 0.0;
    for (;;)
    {
        int32_t best = -1;
        for (int32_t p = 0; p < RANDOM_POINTS; ++p)
        {
            if (!done[p] && isfinite(costs[p]) && (best < 0 || costs[p] < costs[best]))
            {
                best = p;
            }
        }
        if (best < 0)
        {
            return;
        }
        done[best] = true;
        for (int32_t e = 0; e < RANDOM_EDGES; ++e)
        {
            const mnavGraphEdge* edge = &s_edges[e];
            int32_t to = edge->from == best                  ? edge->to
                         : !edge->oneWay && edge->to == best ? edge->from
                                                             : -1;
            if (to < 0 || (filter->areas >> edge->area & 1u) == 0)
            {
                continue;
            }
            mnavPos3 p = s_points[edge->from];
            mnavPos3 q = s_points[edge->to];
            double distance = sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) +
                                   (q.z - p.z) * (q.z - p.z));
            double length = edge->length > 0.0 ? edge->length : distance;
            double cost = costs[best] + length * (double)filter->costs[edge->area];
            costs[to] = cost < costs[to] ? cost : costs[to];
        }
    }
}

static void TestMatchesDijkstra(void)
{
    // Random graphs of 240 points in a 100 m square, three edges out of
    // each, a fifth one way, some given lengths shorter or longer than
    // their distance, three areas: the search reaches exactly the points
    // Dijkstra's search reaches, at its cost, over edges the graph has.
    mnavQuery* query = MakeQuery(4096, 1.0e6f);
    mnavGraph* graph = MakeGraph(RANDOM_POINTS, RANDOM_EDGES);
    mnavQueryFilter filters[3] = {mnavDefaultQueryFilter(), mnavDefaultQueryFilter(),
                                  mnavDefaultQueryFilter()};
    filters[1].costs[2] = 2.5f;
    filters[1].costs[3] = 6.0f;
    filters[2].costs[2] = 0.5f;
    filters[2].areas &= ~((uint64_t)1 << 3);
    static double costs[RANDOM_POINTS];
    int32_t found = 0;
    int32_t agree = 0;
    int32_t trials = 0;
    for (int32_t round = 0; round < 6; ++round)
    {
        for (int32_t p = 0; p < RANDOM_POINTS; ++p)
        {
            s_points[p] = (mnavPos3){(double)Next(10000) * 0.01, (double)Next(300) * 0.01,
                                     (double)Next(10000) * 0.01};
        }
        for (int32_t e = 0; e < RANDOM_EDGES; ++e)
        {
            int32_t from = e / RANDOM_OUT;
            uint32_t kind = Next(10);
            double length = kind == 0 ? (double)Next(50) * 0.1 : kind == 1 ? 150.0 : 0.0;
            s_edges[e] = Edge(from, (int32_t)Next(RANDOM_POINTS), length,
                              (mnavAreaType)(1 + Next(3)), Next(5) == 0);
        }
        CHECK(mnavGraph_Set(graph, s_points, RANDOM_POINTS, s_edges, RANDOM_EDGES) == mnav_success,
              "a random graph");
        for (int32_t pair = 0; pair < 30; ++pair)
        {
            int32_t a = (int32_t)Next(RANDOM_POINTS);
            int32_t b = (int32_t)Next(RANDOM_POINTS);
            const mnavQueryFilter* filter = &filters[pair % 3];
            Costs(filter, a, costs);
            mnavGraphPath path;
            CHECK(mnavQuery_FindGraphPath(query, graph, a, b, filter, &path) == mnav_success,
                  "search");
            bool reached = path.end == mnav_pathFound;
            bool ok = reached == isfinite(costs[b]) && path.points[0] == a;
            if (reached)
            {
                ok = ok && fabs(path.cost - costs[b]) <= 1e-9 * (1.0 + costs[b]) &&
                     path.points[path.pointCount - 1] == b;
            }
            found += reached ? 1 : 0;
            agree += ok ? 1 : 0;
            trials += 1;
        }
    }
    printf("graphs: found=%d agree=%d of %d\n", found, agree, trials);
    CHECK(agree == trials && found > trials / 2, "Dijkstra's ends and costs");
    mnavDestroyGraph(graph);
    mnavDestroyQuery(query);
}

int main(void)
{
    TestCheapestAndTies();
    TestOneWayAndLengths();
    TestLimitsAndEnds();
    TestChecks();
    TestMatchesDijkstra();
    return s_failures == 0 ? 0 : 1;
}
