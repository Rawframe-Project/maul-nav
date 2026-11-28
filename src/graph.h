// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs: the points and the edges out of each, as the search reads them.

#ifndef MAUL_NAV_SRC_GRAPH_H
#define MAUL_NAV_SRC_GRAPH_H

#include "allocator.h"
#include "host_code.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/graph.h"

#include <stdint.h>

// An edge out of a point: the point it leads to, its length in meters and
// its area.
typedef struct mnavGraphArc
{
    double length;
    int32_t to;
    mnavAreaType area;
} mnavGraphArc;

struct mnavGraph
{
    mnavMemory memory;
    mnavHostCode host;
    mnavGraphDef def;
    // The points set, and the edges out of each: point p's are arcs[first[p]]
    // up to arcs[first[p + 1]], in the order of the edges given. A two-way
    // edge is an arc out of each of its points.
    mnavPos3* points;
    int32_t pointCount;
    int32_t* first;
    mnavGraphArc* arcs;
    int32_t arcCount;
    // The least ratio of an arc's length to the distance between its
    // points, at most 1: how much less than the straight distance the way
    // left may be.
    double shortcut;
};

#endif // MAUL_NAV_SRC_GRAPH_H
