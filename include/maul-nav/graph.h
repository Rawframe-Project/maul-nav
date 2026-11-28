// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs: points and the edges between them that a host lays out itself,
// such as waypoints, a road or rail network or a level graph of rooms and
// doors, and the cheapest path over them. A graph needs no navmesh.

#ifndef MAUL_NAV_GRAPH_H
#define MAUL_NAV_GRAPH_H

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The most points a graph may hold.
#define MNAV_MAX_GRAPH_POINTS 1048576
// The most edges a graph may hold.
#define MNAV_MAX_GRAPH_EDGES 4194304

    // A graph: the memory for its points and edges, and those last set.
    typedef struct mnavGraph mnavGraph;

    // The most a graph holds; its memory is taken when it is made.
    typedef struct mnavGraphLimits
    {
        // Points, 1 to MNAV_MAX_GRAPH_POINTS.
        int32_t points;
        // Edges, 0 to MNAV_MAX_GRAPH_EDGES.
        int32_t edges;
    } mnavGraphLimits;

    // How a graph is made. Build it with mnavDefaultGraphDef.
    typedef struct mnavGraphDef
    {
        uint32_t cookie;
        // MNAV_ABI_VERSION, set by mnavDefaultGraphDef.
        uint32_t version;
        // The allocator the graph uses; zeroed for the C library's.
        mnavAllocator allocator;
        mnavGraphLimits limits;
    } mnavGraphDef;

// The cookie mnavDefaultGraphDef sets and mnavCreateGraph checks.
#define MNAV_GRAPH_DEF_COOKIE 0x4E415647u

    /// Returns the default graph def: up to 4096 points and 16384 edges.
    /// Built in the program, so that it carries the version of the
    /// headers the program includes.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    static inline mnavGraphDef mnavDefaultGraphDef(void)
    {
#ifdef __cplusplus
        mnavGraphDef def = {};
#else
    mnavGraphDef def = {0};
#endif
        def.cookie = MNAV_GRAPH_DEF_COOKIE;
        def.version = MNAV_ABI_VERSION;
        def.limits.points = 4096;
        def.limits.edges = 16384;
        return def;
    }

    // An edge between two points of a graph, by their indices.
    typedef struct mnavGraphEdge
    {
        int32_t from;
        int32_t to;
        // The edge's length in meters, 0 to MNAV_MAX_PATH_LENGTH; 0 for the
        // distance between its points. A length below that distance, as of
        // a lift or a teleport, is allowed: the search then expects less of
        // the way left, so that it stays exact.
        double length;
        // Its area, 1 to MNAV_AREA_TYPES - 1: a filter's cost per meter of
        // the area applies, and an area the filter leaves out closes it.
        mnavAreaType area;
        // True to cross it from `from` to `to` only; both ways otherwise.
        bool oneWay;
    } mnavGraphEdge;

    // A graph path: how its search ended, its cost and length, and the
    // points it passes, the start first and the last point reached last, in
    // a query context's memory until its next query.
    typedef struct mnavGraphPath
    {
        mnavPathEnd end;
        double cost;
        double length;
        const int32_t* points;
        int32_t pointCount;
    } mnavGraphPath;

    /// Makes a graph with the memory its limits need, holding no points.
    ///
    /// @param def      The def, from mnavDefaultGraphDef.
    /// @param graphOut Receives the graph, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a def
    /// not from mnavDefaultGraphDef; `mnav_errorVersion` for one built against
    /// another version of the headers; `mnav_errorInvalid` also for an
    /// allocator with one function, fewer than 1 point or fewer than 0 edges;
    /// `mnav_errorLimit` for more points than MNAV_MAX_GRAPH_POINTS or more
    /// edges than MNAV_MAX_GRAPH_EDGES; `mnav_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavResult mnavCreateGraph(const mnavGraphDef* def,
                                                       mnavGraph** graphOut);

    /// Destroys a graph. Called back in from the graph's allocator, it does
    /// nothing and counts one misuse.
    ///
    /// @param graph The graph, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the graph is used by one thread at a time,
    /// so no search may run on it meanwhile.
    MNAV_API void mnavDestroyGraph(mnavGraph* graph);

    /// Returns how many calls against the graph were refused as misuse since
    /// it was made: an argument refused with `mnav_errorInvalid` when the
    /// graph was the first object the call was given, and a call back in
    /// from host code it ran (`mnav_errorBusy`).
    ///
    /// @param graph The graph.
    /// @return The count; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread. It may be read at any time, also while host
    /// code the object runs calls back in.
    MNAV_API uint64_t mnavGraph_GetMisuse(const mnavGraph* graph);

    /// Gives the graph its points and edges, replacing those it held. Each
    /// point's edges are tried in the order given, a two-way edge from both
    /// its points; any number of edges may join the same two points.
    ///
    /// @param graph      The graph.
    /// @param points     The points, in meters. Only read during the call.
    /// @param pointCount How many, 0 to the graph's point limit.
    /// @param edges      The edges. Only read during the call.
    /// @param edgeCount  How many, 0 to the graph's edge limit.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL graph, a NULL
    /// array with a count above 0, a negative count, a point not finite, or
    /// an edge with a point outside the points given, an area of 0 or past
    /// MNAV_AREA_TYPES - 1, or a length below 0 or NaN; `mnav_errorLimit` for a
    /// point more than MNAV_MAX_QUERY_DISTANCE from the origin on an axis or a
    /// length past MNAV_MAX_PATH_LENGTH, an infinity included;
    /// `mnav_errorCapacity` for more points or edges than the graph's limits.
    /// A refused call leaves the graph as it was. Also `mnav_errorBusy` when
    /// called back in from the graph's allocator, one misuse counted.
    /// @par Thread safety
    /// Safe from any thread; the graph is used by one thread at a time,
    /// so no search may run on it meanwhile.
    MNAV_NODISCARD MNAV_API mnavResult mnavGraph_Set(mnavGraph* graph, const mnavPos3* points,
                                                     int32_t pointCount, const mnavGraphEdge* edges,
                                                     int32_t edgeCount);

    /// Finds the cheapest path over a graph from one point to another with
    /// A*: an edge costs its length times the filter's cost of its area, and
    /// the heuristic is the straight distance to the end times the cheapest
    /// included area's cost, scaled down by the least ratio of an edge's
    /// length to the distance between its points where that is below 1, so
    /// the path found is a cheapest one. The open list takes the least
    /// total, then the node made first; each point's edges are tried in the
    /// order the graph was given them, so the same graph and ends give the
    /// same path on every platform. It keeps the context's node limit, a node
    /// a point reached, and path length, and ends as mnavQuery_FindPath does,
    /// with the path to the point nearest the end by the heuristic when the
    /// end is not reached. A graph search ends any sliced search.
    ///
    /// @param query   The context; its memory holds the path.
    /// @param graph   The graph.
    /// @param start   The start point's index.
    /// @param end     The end point's index.
    /// @param filter  The areas usable and their costs, or NULL.
    /// @param pathOut Receives the path. Zeroed on any failure but
    ///                mnav_errorBusy, which leaves it as it was.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a start
    /// or end outside the graph's points, or a filter not built from
    /// mnavDefaultQueryFilter or with a NaN cost; `mnav_errorLimit` for a
    /// filter cost past MNAV_MIN_AREA_COST or MNAV_MAX_AREA_COST, an infinity
    /// included. Also `mnav_errorBusy` when called back in from host code a
    /// call using the context runs (a link generation's clearance test, a
    /// hierarchy's allocator as it builds), one misuse counted.
    /// @par Thread safety
    /// Safe from any thread; the context is used by one thread at a time.
    /// Any number of contexts may search one graph at once while no other
    /// call uses the graph.
    MNAV_NODISCARD MNAV_API mnavResult mnavQuery_FindGraphPath(mnavQuery* query,
                                                               const mnavGraph* graph,
                                                               int32_t start, int32_t end,
                                                               const mnavQueryFilter* filter,
                                                               mnavGraphPath* pathOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_GRAPH_H
