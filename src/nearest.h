// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The nearest point's surface, shared with the queries that need heights.

#ifndef MAUL_NAV_SRC_NEAREST_H
#define MAUL_NAV_SRC_NEAREST_H

#include "navmesh.h"

#include <stdint.h>

// The world height of a polygon's detail surface at a ground point, at the
// detail triangle nearest it.
double mnavSurfaceHeight(const mnavNavmesh* navmesh, int32_t slot, int32_t polygon, double x,
                         double z);

#endif // MAUL_NAV_SRC_NEAREST_H
