// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields over the navmesh (mnav-0013): for every polygon, the cost of
// its cheapest way to the nearest of a set of goal points and the polygon
// to go to next, so that any number of agents sharing the goals find their
// way at the cost of one search.

#ifndef MAUL_NAV_NAVFLOW_H
#define MAUL_NAV_NAVFLOW_H

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The most polygons a navmesh flow field may hold.
#define MNAV_MAX_NAVFLOW_POLYGONS 16777216

    // How a navmesh flow field is made. Build it with
    // mnavDefaultNavFlowDef.
    typedef struct mnavNavFlowDef
    {
        uint32_t cookie;
        // The allocator the field uses; zeroed for the C library's.
        mnavAllocator allocator;
        // The most polygons the navmesh may hold, 1 to
        // MNAV_MAX_NAVFLOW_POLYGONS.
        int32_t polygons;
    } mnavNavFlowDef;

    // A navmesh flow field: the memory for its polygons, and the field last
    // built.
    typedef struct mnavNavFlow mnavNavFlow;

    // A goal: a point on a polygon.
    typedef struct mnavNavFlowGoal
    {
        mnavPolygonId polygon;
        mnavPos3 point;
    } mnavNavFlowGoal;

    // A polygon's way to the goals. An agent on the polygon heads for the
    // portal from left to right, then goes on to the next polygon; where
    // the way leaves by an off-mesh link, both ends are the link's takeoff
    // point and link names it.
    typedef struct mnavPolygonFlow
    {
        // The cost from the portal's midpoint, or the goal point on a goal
        // polygon; infinite where no goal is reached.
        double cost;
        // The next polygon, zeroed on a goal polygon and where no goal is
        // reached.
        mnavPolygonId next;
        mnavPos3 left;
        mnavPos3 right;
        // The link taken, zeroed when none.
        mnavLinkId link;
    } mnavPolygonFlow;

    /// Returns the default navmesh flow field def: up to 65536 polygons.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavNavFlowDef mnavDefaultNavFlowDef(void);

    /// Makes a navmesh flow field with the memory its polygon limit needs.
    ///
    /// @param def      The def, from mnavDefaultNavFlowDef.
    /// @param fieldOut Receives the field, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a
    /// def not from mnavDefaultNavFlowDef; `mnav_errorRange` for a polygon
    /// limit out of its range; `mnav_errorCapacity` when the allocator
    /// fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavResult mnavCreateNavFlow(const mnavNavFlowDef* def,
                                                         mnavNavFlow** fieldOut);

    /// Destroys a navmesh flow field.
    ///
    /// @param field The field, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the field is used by one thread at a time.
    MNAV_API void mnavDestroyNavFlow(mnavNavFlow* field);

    /// Builds the field for a navmesh and a set of goal points by one
    /// search backward from all of them, as path searches price their ways
    /// (mnav-0005): a polygon stands at the midpoint of the portal it is
    /// left by toward the goals, and its cost is the next polygon's plus
    /// the walk between them times the next polygon's area cost, with an
    /// off-mesh link's cost where the way crosses one. Ties go to the lower
    /// cost, then the polygon of the lower slot and index; the same navmesh
    /// and goals give the same field on every platform. Goals on polygons
    /// the filter leaves out are left out.
    ///
    /// @param field     The field; its last field is replaced.
    /// @param navmesh   The navmesh; reads need it unchanged since.
    /// @param filter    The areas usable and their costs, and the link
    ///                  kinds, or NULL; copied.
    /// @param goals     The goals.
    /// @param goalCount How many, at least 0.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a
    /// negative count, a goal polygon id never handed out or a goal point
    /// not finite, or a filter not built from mnavDefaultQueryFilter;
    /// `mnav_errorRange` for a filter cost out of its range;
    /// `mnav_errorStale` for a goal polygon whose tile was replaced or
    /// removed; `mnav_errorLimit` for a navmesh of more polygons than the
    /// field's limit; `mnav_errorCapacity` when the allocator fails. On an
    /// error the field holds nothing.
    /// @par Thread safety
    /// Safe from any thread. Any number of threads may read the navmesh at
    /// once while no commit runs on it; the field is used by one thread at
    /// a time.
    MNAV_NODISCARD MNAV_API mnavResult mnavBuildNavFlow(mnavNavFlow* field,
                                                        const mnavNavmesh* navmesh,
                                                        const mnavQueryFilter* filter,
                                                        const mnavNavFlowGoal* goals,
                                                        int32_t goalCount);

    /// Reads a polygon's way to the goals from the field.
    ///
    /// @param field   The field.
    /// @param navmesh The navmesh the field was built on.
    /// @param polygon The polygon.
    /// @param flowOut Receives its way.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument,
    /// nothing built, or a polygon id never handed out;
    /// `mnav_errorStale` for another navmesh, one committed to since the
    /// build, or a polygon whose tile was replaced or removed.
    /// @par Thread safety
    /// Safe from any thread. Any number of threads may read a field at once
    /// while no build runs on it.
    MNAV_NODISCARD MNAV_API mnavResult mnavNavFlowAt(const mnavNavFlow* field,
                                                     const mnavNavmesh* navmesh,
                                                     mnavPolygonId polygon,
                                                     mnavPolygonFlow* flowOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_NAVFLOW_H
