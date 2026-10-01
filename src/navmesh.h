// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The navmesh's insides (N23), for the modules that query it: tiles in
// slots, their links across tile sides, and the sorted index of places.

#ifndef MAUL_NAV_SRC_NAVMESH_H
#define MAUL_NAV_SRC_NAVMESH_H

#include "allocator.h"
#include "detail.h"
#include "polymesh.h"
#include "tile.h"

#include "maul-nav/bake.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>

// A link from a polygon edge on a tile side to a polygon across it, over
// the part of the side from low to high, in cells along the side.
typedef struct mnavLink
{
    uint16_t polygon;
    uint8_t edge;
    uint8_t side;
    int32_t low;
    int32_t high;
    mnavPolygonId target;
} mnavLink;

// A tile as loaded, and its links, polygon by polygon: polygon p's are
// links[firstLink[p]] up to links[firstLink[p + 1]].
typedef struct mnavTile
{
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    mnavLink* links;
    int32_t linkCount;
    int32_t* firstLink;
} mnavTile;

// A slot: its generation, its tile (NULL when free) and the tile's place.
// A retired slot is never used again.
typedef struct mnavSlot
{
    uint32_t generation;
    bool retired;
    int32_t x;
    int32_t z;
    mnavTile* tile;
} mnavSlot;

// A place and the 0-based slot holding its tile.
typedef struct mnavPlace
{
    int32_t x;
    int32_t z;
    int32_t slot;
} mnavPlace;

// A staged change: a tile to install at its place, or NULL to remove.
typedef struct mnavStaged
{
    int32_t x;
    int32_t z;
    mnavTile* tile;
} mnavStaged;

struct mnavNavmesh
{
    mnavBakeDef def;
    mnavBakeCells cells;
    mnavMemory memory;
    mnavSlot* slots;
    int32_t slotCount;
    int32_t slotCapacity;
    // The committed tiles' places, sorted by x, then z.
    mnavPlace* places;
    int32_t placeCount;
    int32_t placeCapacity;
    mnavStaged* staged;
    int32_t stagedCount;
    int32_t stagedCapacity;
};

// The committed tile at a place, or NULL; slotOut receives its 0-based
// slot.
const mnavTile* mnavTileAt(const mnavNavmesh* navmesh, int32_t x, int32_t z, int32_t* slotOut);

// The polygon an id names, or NULL when the id is stale or out of range;
// tileOut receives its tile.
const mnavPolygon* mnavPolygonOf(const mnavNavmesh* navmesh, mnavPolygonId id,
                                 const mnavTile** tileOut);

// The place one step from (x, z) across a side, 1 to 4 for -X, +Z, +X
// and -Z, and the side facing back.
void mnavAcross(int32_t side, int32_t* x, int32_t* z, int32_t* facing);

// A tile's frame: its cell (0, 0) corner in world meters and its cells'
// sizes.
typedef struct mnavFrame
{
    double x0;
    double z0;
    double y0;
    double cell;
    double height;
} mnavFrame;

mnavFrame mnavFrameOf(const mnavNavmesh* navmesh, int32_t x, int32_t z);

// Whether a polygon id names a polygon of the navmesh now:
// mnav_errorInvalid for one never handed out, mnav_errorStale for one whose
// tile was replaced or removed.
mnavResult mnavCheckPolygon(const mnavNavmesh* navmesh, mnavPolygonId id);

// A mesh vertex's world position in a tile's frame.
mnavPos3 mnavVertexWorld(const mnavFrame* f, const mnavMeshVertex* v);

#endif // MAUL_NAV_SRC_NAVMESH_H
