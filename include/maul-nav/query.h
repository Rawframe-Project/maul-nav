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

    /// Finds the polygon nearest a point whose nearest point lies within a
    /// box round it, and that point. A point over a polygon scores the height it
    /// lies beyond the agent's step, any other the distance to the
    /// polygon; ties go to the shorter distance, then the tile first by
    /// place (x, then z), then the lower polygon index.
    ///
    /// @param navmesh      The navmesh.
    /// @param point        The query point.
    /// @param halfExtents  The box's half sizes, in meters, at least 0.
    /// @param nearestOut   Receives the result.
    /// @return `mnav_success`, also when no polygon's nearest point lies in
    /// the box (the polygon's slot is then 0); `mnav_errorInvalid` for a NULL argument,
    /// a point or extent that is not finite, or a negative extent.
    /// @par Thread safety
    /// Safe from any thread. Any number of queries may run at once between
    /// commits.
    MNAV_NODISCARD MNAV_API mnavResult mnavFindNearest(const mnavNavmesh* navmesh, mnavPos3 point,
                                                       mnavVec3 halfExtents,
                                                       mnavNearest* nearestOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_QUERY_H
