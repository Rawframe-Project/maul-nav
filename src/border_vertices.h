// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Removing the tile-border vertices from a tile's polygons (N19).

#ifndef MAUL_NAV_SRC_BORDER_VERTICES_H
#define MAUL_NAV_SRC_BORDER_VERTICES_H

#include "allocator.h"
#include "polymesh.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// Removes each removable vertex, in index order, whose polygons keep more
// than 2 edges, have at most 2 unshared edges at it and share one area,
// replacing those polygons with the merged triangulation of the hole left.
// A vertex whose replacement cannot be built stays, still removable.
// Polygons must not be linked yet. Returns mnav_errorLimit when the new
// polygons would pass maxPolygons.
mnavResult mnavRemoveBorderVertices(mnavMemory* memory, mnavPolyMesh* mesh, int32_t maxPolygons);

#endif // MAUL_NAV_SRC_BORDER_VERTICES_H
