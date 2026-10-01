// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// 2D outlines filled into the fragments of one tile (mnav-0002): every cell
// whose center a walkable outline holds and no obstruction does becomes
// the fragment of a flat triangle at height 0.

#ifndef MAUL_NAV_SRC_OUTLINE_H
#define MAUL_NAV_SRC_OUTLINE_H

#include "allocator.h"
#include "raster.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// Whether an outline's bounds meet the tile's cells, border included.
bool mnavOutlineTouchesTile(const mnavTileFrame* frame, const mnavOutline* outline);

// Adds the fragments of valid outlines for the tile. Returns
// mnav_errorLimit past the tileTriangles limit on outlines touching the
// tile, the tileSpans limit on fragments or the memory limit, and
// mnav_errorCapacity when the allocator fails.
mnavResult mnavCollectOutlines(mnavMemory* memory, const mnavBakeDef* def,
                               const mnavTileFrame* frame, const mnavOutline* outlines,
                               int32_t outlineCount, mnavFragmentList* list);

#endif // MAUL_NAV_SRC_OUTLINE_H
