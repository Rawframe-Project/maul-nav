// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The bake's stages run in order over one tile, for tests that need a
// finished polygon and detail mesh.

#ifndef MAUL_NAV_TEST_BAKE_TILE_H
#define MAUL_NAV_TEST_BAKE_TILE_H

#include "allocator.h"
#include "border_vertices.h"
#include "compact.h"
#include "contour.h"
#include "detail.h"
#include "erode.h"
#include "filter.h"
#include "heightfield.h"
#include "holes.h"
#include "polymesh.h"
#include "region.h"

#include "maul-nav/bake.h"

#include <stdint.h>

// Everything a baked tile holds, and the stages' results it came from.
typedef struct BakedTile
{
    mnavHeightfield heightfield;
    mnavCompactField compact;
    mnavRegionMap regions;
    mnavContourSet set;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
} BakedTile;

// Bakes tile (0, 0) of input; returns the first failing stage's result.
static inline mnavResult BakeTile(mnavMemory* memory, const mnavBakeDef* def,
                                  const mnavBakeCells* cells, const mnavTriangleMesh* input,
                                  BakedTile* tile)
{
    *tile = (BakedTile){0};
    mnavResult result =
        mnavBuildHeightfield(memory, def, cells, input, 1, 0, 0, &tile->heightfield);
    if (result != mnav_success)
    {
        return result;
    }
    mnavFilterWalkable(&tile->heightfield, cells->agentHeight, cells->agentStep);
    result = mnavBuildCompactField(memory, &tile->heightfield, cells->agentHeight, cells->agentStep,
                                   &tile->compact);
    result =
        result == mnav_success ? mnavErode(memory, &tile->compact, cells->agentRadius) : result;
    result = result == mnav_success ? mnavBuildRegions(memory, &tile->compact, cells->border,
                                                       cells->minRegion, &tile->regions)
                                    : result;
    result = result == mnav_success
                 ? mnavBuildContours(memory, &tile->compact, &tile->regions, cells->border,
                                     cells->edgeError, cells->edgeLength, &tile->set)
                 : result;
    result =
        result == mnav_success ? mnavMergeHoles(memory, &tile->set, tile->regions.count) : result;
    result = result == mnav_success
                 ? mnavBuildPolyMesh(memory, &tile->set, def->tileCells, def->limits.tileVertices,
                                     def->limits.tilePolygons, &tile->mesh)
                 : result;
    result = result == mnav_success
                 ? mnavRemoveBorderVertices(memory, &tile->mesh, def->limits.tilePolygons)
                 : result;
    result = result == mnav_success ? mnavLinkPolyMesh(memory, &tile->mesh) : result;
    mnavDetailSettings settings = {cells->detailSample, cells->detailError, 1, cells->border};
    return result == mnav_success ? mnavBuildDetailMesh(memory, &tile->compact, &tile->regions,
                                                        &tile->mesh, settings, &tile->detail)
                                  : result;
}

static inline void ReleaseBakedTile(mnavMemory* memory, BakedTile* tile)
{
    mnavReleaseDetailMesh(memory, &tile->detail);
    mnavReleasePolyMesh(memory, &tile->mesh);
    mnavReleaseContours(memory, &tile->set);
    mnavReleaseRegions(memory, &tile->regions);
    mnavReleaseCompactField(memory, &tile->compact);
    mnavReleaseHeightfield(memory, &tile->heightfield);
}

#endif // MAUL_NAV_TEST_BAKE_TILE_H
