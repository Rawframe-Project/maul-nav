// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Queries over a navmesh's committed tiles (mnav-0005). Every query reads
// and never changes the navmesh, and works in world coordinates: meters,
// right-handed, +Y up.

#ifndef MAUL_NAV_QUERY_H
#define MAUL_NAV_QUERY_H

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The cheapest and dearest an area's cost may be.
#define MNAV_MIN_AREA_COST 0.001f
#define MNAV_MAX_AREA_COST 1000000.0f

    // Which polygons a query may use and what crossing them costs, by area
    // type (mnav-0002). Build it with mnavDefaultQueryFilter.
    typedef struct mnavQueryFilter
    {
        uint32_t cookie;
        // What a meter costs in each area type, MNAV_MIN_AREA_COST to
        // MNAV_MAX_AREA_COST; 1 by default.
        float costs[MNAV_AREA_TYPES];
        // Bit n set when polygons of area type n may be used; every
        // walkable type by default. Bit 0 is ignored: area 0 is never
        // walkable.
        uint64_t areas;
        // Bit n set when the agent may cross off-mesh links of kind n; every
        // kind by default.
        uint64_t kinds;
    } mnavQueryFilter;

    /// Returns the filter that uses every walkable area at a cost of 1 and
    /// every kind of off-mesh link.
    ///
    /// @return The filter.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavQueryFilter mnavDefaultQueryFilter(void);

    // The nearest point on the navmesh to a query point.
    typedef struct mnavNearest
    {
        // The polygon holding it; slot 0 when no polygon has its nearest
        // point in the box.
        mnavPolygonId polygon;
        // The point, on the polygon's detail surface.
        mnavPos3 point;
        // Whether the query point lies over or under the polygon, so that
        // the point is straight above or below it.
        bool over;
        // Whether part of the search box lies on places with no tile
        // loaded, where a nearer polygon may be.
        bool incomplete;
    } mnavNearest;

    /// Finds the polygon, among those the filter includes, nearest a point
    /// whose nearest point lies within a box round it, and that point. A
    /// point over a polygon scores the height it lies beyond the agent's
    /// step, any other the distance to the polygon; ties go to the shorter
    /// distance, then the tile first by place (x, then z), then the lower
    /// polygon index.
    ///
    /// @param navmesh      The navmesh.
    /// @param filter       The areas usable, or NULL for every walkable one.
    /// @param point        The query point.
    /// @param halfExtents  The box's half sizes, in meters, at least 0.
    /// @param nearestOut   Receives the result.
    /// @return `mnav_success`, also when no polygon's nearest point lies in
    /// the box (the polygon's slot is then 0); `mnav_errorInvalid` for a NULL
    /// argument, a point or extent that is not finite, or a negative
    /// extent; `mnav_errorInvalid` for a filter not built from
    /// mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its
    /// range.
    /// @par Thread safety
    /// Safe from any thread. Any number of queries may run at once between
    /// commits.
    MNAV_NODISCARD MNAV_API mnavResult mnavFindNearest(const mnavNavmesh* navmesh,
                                                       const mnavQueryFilter* filter,
                                                       mnavPos3 point, mnavVec3 halfExtents,
                                                       mnavNearest* nearestOut);

// The largest nodes one search may use.
#define MNAV_MAX_QUERY_NODES 1048576
// The longest path length limit, in meters.
#define MNAV_MAX_PATH_LENGTH 1.0e7f

    // A query context: the scratch memory searches use, sized by its
    // limits when made, so searches never allocate. Made by
    // mnavCreateQuery; one thread uses a context at a time.
    typedef struct mnavQuery mnavQuery;

    // The named limits that bound a search's work.
    typedef struct mnavQueryLimits
    {
        // Nodes one search may open, 1 to MNAV_MAX_QUERY_NODES; a node is
        // a polygon entered through one of its edges.
        int32_t nodes;
        // The longest path searched, in meters, more than 0 and at most
        // MNAV_MAX_PATH_LENGTH: no node is opened whose way from the start
        // and on to the end is longer.
        float pathLength;
    } mnavQueryLimits;

    // How a query context is made. Build it with mnavDefaultQueryDef.
    typedef struct mnavQueryDef
    {
        uint32_t cookie;
        // The allocator the context uses; zeroed for the C library's.
        mnavAllocator allocator;
        // The limits on each search.
        mnavQueryLimits limits;
    } mnavQueryDef;

    // How a path search ended, checked in this order.
    typedef uint8_t mnavPathEnd;

    enum
    {
        // The end point was reached.
        mnav_pathFound = 0,
        // The search used every node of its budget before reaching it.
        mnav_pathOutOfNodes = 1,
        // Every way on was longer than the path length limit.
        mnav_pathTooLong = 2,
        // Every way on ran into places with no tile loaded.
        mnav_pathNotLoaded = 3,
        // The end point cannot be reached from the start.
        mnav_pathNone = 4,
    };

    // An off-mesh link a path crosses: the link, its kind, and the index of
    // its takeoff point in the straight path; the landing point follows it.
    typedef struct mnavPathLink
    {
        mnavLinkId link;
        mnavLinkKind kind;
        int32_t point;
    } mnavPathLink;

    // A path search's result. Short of the end point, the corridor runs to
    // the polygon nearest it.
    typedef struct mnavPath
    {
        mnavPathEnd end;
        // The cost of the way searched, through the midpoints of the edges
        // the corridor crosses: each step's length times the cost of the
        // area it lies in.
        double cost;
        // That way's length in meters; the straight path is never longer.
        double length;
        // The polygons from the start polygon on, in the context's memory
        // until its next search.
        const mnavPolygonId* polygons;
        int32_t polygonCount;
        // The straight path from the start point, the corridor pulled
        // tight: its corners and, last, the end point or, short of it, the
        // midpoint of the last edge crossed. In the context's memory until
        // its next search; never cut short.
        const mnavPos3* points;
        int32_t pointCount;
        // The off-mesh links crossed, in order, in the context's memory
        // until its next search.
        const mnavPathLink* links;
        int32_t linkCount;
    } mnavPath;

    /// Returns a query def with 8,192 nodes per search and paths up to
    /// 1,000 m.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavQueryDef mnavDefaultQueryDef(void);

    /// Makes a query context with the memory its limits need.
    ///
    /// @param def      The def, from mnavDefaultQueryDef.
    /// @param queryOut Receives the context, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a
    /// def not from mnavDefaultQueryDef; `mnav_errorRange` for a limit out
    /// of its range; `mnav_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavResult mnavCreateQuery(const mnavQueryDef* def,
                                                       mnavQuery** queryOut);

    /// Destroys a query context. NULL is ignored.
    ///
    /// @param query    The context.
    /// @par Thread safety
    /// Safe from any thread; the context is used by one thread at a time.
    MNAV_API void mnavDestroyQuery(mnavQuery* query);

    /// Searches for the shortest way from a point on one polygon to a point
    /// on another (mnav-0005): A* over the edges between polygons, its
    /// heuristic the straight distance to the end point; ties go to the
    /// node made first. A step costs its length times the cost of the area
    /// it crosses, and the heuristic is scaled by the cheapest included
    /// area's cost; polygons of excluded areas other than the start
    /// polygon are not entered. Attached off-mesh links of included kinds are
    /// crossed at their declared cost, the heuristic scaled down to the
    /// lowest cost per meter among them (mnav-0005). The corridor found is pulled
    /// tight into a straight path with the funnel algorithm, stretch by
    /// stretch between links.
    ///
    /// @param query        The context; its memory holds the result.
    /// @param navmesh      The navmesh.
    /// @param filter       The areas usable and their costs, or NULL for
    ///                     every walkable area at a cost of 1.
    /// @param startPolygon The polygon the start point lies on, as
    ///                     mnavFindNearest gives it.
    /// @param start        The start point.
    /// @param endPolygon   The polygon the end point lies on.
    /// @param end          The end point.
    /// @param pathOut      Receives the result.
    /// @return `mnav_success` whenever a search ran, however it ended;
    /// `mnav_errorInvalid` for a NULL argument, a point that is not finite
    /// or a polygon id that never existed; `mnav_errorInvalid` for a filter not built from
    /// mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its
    /// range; `mnav_errorStale` for a polygon id whose tile
    /// has been replaced or removed.
    /// @par Thread safety
    /// Safe from any thread; the context is used by one thread at a time.
    /// Any number of contexts may search one navmesh at once between
    /// commits.
    MNAV_NODISCARD MNAV_API mnavResult mnavFindPath(mnavQuery* query, const mnavNavmesh* navmesh,
                                                    const mnavQueryFilter* filter,
                                                    mnavPolygonId startPolygon, mnavPos3 start,
                                                    mnavPolygonId endPolygon, mnavPos3 end,
                                                    mnavPath* pathOut);

    // How a raycast ended.
    typedef uint8_t mnavRayEnd;

    enum
    {
        // The ray reached its end point.
        mnav_rayReached = 0,
        // A wall stopped it.
        mnav_rayWall = 1,
        // It came to a tile side with no tile loaded beyond.
        mnav_rayNotLoaded = 2,
        // The polygons it crossed reached the context's node limit.
        mnav_rayOutOfNodes = 3,
    };

    // A raycast's result.
    typedef struct mnavRay
    {
        mnavRayEnd end;
        // How far the ray went, as a fraction of the way from its start to
        // its end point on the ground: 1 when it reached the end point.
        double t;
        // At a wall, the wall's normal on the ground, pointing back to
        // where the ray came from; 0 otherwise.
        double normalX;
        double normalZ;
        // The polygons crossed, from the start polygon to the one where the
        // ray ended, in the context's memory until its next query.
        const mnavPolygonId* polygons;
        int32_t polygonCount;
    } mnavRay;

    /// Casts a ray along the navmesh on the ground from a point on a polygon
    /// toward an end point (mnav-0005): polygon to polygon through the
    /// first edge it crosses, until it reaches the end point, meets a wall
    /// (an edge into a polygon the filter excludes is a wall too) or a tile
    /// side with no tile loaded, or crosses as many polygons as the
    /// context's node limit. Where it leaves through a corner, it goes
    /// on through an edge that leads on, the lowest-numbered first.
    ///
    /// @param query        The context; its memory holds the polygons.
    /// @param navmesh      The navmesh.
    /// @param filter       The areas usable, or NULL for every walkable one.
    /// @param startPolygon The polygon the start point lies on, as
    ///                     mnavFindNearest gives it.
    /// @param start        The start point.
    /// @param end          The end point; only its ground position counts.
    /// @param rayOut       Receives the result.
    /// @return `mnav_success` whenever the ray was cast, however it ended;
    /// `mnav_errorInvalid` for a NULL argument, a point that is not finite
    /// or a polygon id that never existed; `mnav_errorInvalid` for a filter not built from
    /// mnavDefaultQueryFilter; `mnav_errorRange` for a filter cost out of its
    /// range; `mnav_errorStale` for a polygon id whose tile
    /// has been replaced or removed.
    /// @par Thread safety
    /// Safe from any thread; the context is used by one thread at a time.
    /// Any number of contexts may cast on one navmesh at once between
    /// commits.
    MNAV_NODISCARD MNAV_API mnavResult mnavRaycast(mnavQuery* query, const mnavNavmesh* navmesh,
                                                   const mnavQueryFilter* filter,
                                                   mnavPolygonId startPolygon, mnavPos3 start,
                                                   mnavPos3 end, mnavRay* rayOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_QUERY_H
