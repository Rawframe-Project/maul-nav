// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A query context's insides, shared by the queries that use it.

#ifndef MAUL_NAV_SRC_QUERY_H
#define MAUL_NAV_SRC_QUERY_H

#include "allocator.h"

#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// A node's way in: an edge index below 8, 8 plus the side for a tile link,
// or the start or end point.
#define MNAV_TAG_LINK  8
#define MNAV_TAG_START 0xFE
#define MNAV_TAG_END   0xFF

// No node, or a node that has left the open list.
#define MNAV_NO_NODE (-1)

// A portal into a polygon, or the start or end point: where the search
// stands, the way it came, and its cost so far.
typedef struct mnavSearchNode
{
    // The portal's ends, in the order of the polygon left, and its midpoint.
    mnavPos3 a;
    mnavPos3 b;
    mnavPos3 at;
    // The cost so far, the length so far in meters, and the heuristic: the
    // distance to the end times the cheapest included area's cost.
    double cost;
    double length;
    double remaining;
    // The polygon entered: its 0-based slot and index.
    int32_t slot;
    int32_t polygon;
    // The portal: TAG_ values, and the link's start along the side.
    int32_t tag;
    int32_t low;
    int32_t parent;
    // The node's place in the open list, or MNAV_NO_NODE once closed.
    int32_t heap;
} mnavSearchNode;

// A portal's ends as the funnel sees them, walking into the polygon: the
// left one, then the right one.
typedef struct mnavPortal
{
    mnavPos3 left;
    mnavPos3 right;
} mnavPortal;

struct mnavQuery
{
    mnavMemory memory;
    mnavQueryLimits limits;
    mnavSearchNode* nodes;
    int32_t nodeCount;
    int32_t* heap;
    int32_t heapCount;
    // Open addressing over node keys: node indices, MNAV_NO_NODE for empty.
    int32_t* table;
    uint32_t tableMask;
    mnavPolygonId* corridor;
    // The corridor's portals, then the straight path; a corridor of n
    // polygons has n + 1 of each at most.
    mnavPortal* portals;
    mnavPos3* points;
};

#endif // MAUL_NAV_SRC_QUERY_H
