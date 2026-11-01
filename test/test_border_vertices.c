// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Removing tile-border vertices, read from baked tiles: where a floor and
// a ramp over it, two regions of one area, meet on the tile's side, their
// vertex there goes, the side is one edge across their meeting, and the
// detail surface of the polygons that replace the vertex's follows both
// regions. The scene is baked turned to each of the tile's four sides.
// test_border_vertices_rings.c covers the cases no bake reaches.

#include "defs_for.h"
#include "ramp_level.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

enum
{
    TILE_ROOM = 1 << 20,
    DRAW_ROOM = 1 << 14
};

static uint8_t s_tile[TILE_ROOM];
static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawLines[DRAW_ROOM];

// A tile side edge on the ground, turned back to the scene's frame.
typedef struct Side
{
    float x0;
    float z0;
    float x1;
    float z1;
} Side;

// Whether one of the sides runs between two places, either way.
static bool HasSide(const Side* sides, int32_t count, float x0, float z0, float x1, float z1)
{
    for (int32_t s = 0; s < count; ++s)
    {
        const Side* d = &sides[s];
        if ((d->x0 == x0 && d->z0 == z0 && d->x1 == x1 && d->z1 == z1) ||
            (d->x0 == x1 && d->z0 == z1 && d->x1 == x0 && d->z1 == z0))
        {
            return true;
        }
    }
    return false;
}

static void TestFloorAndRampMeetOnEachSide(void)
{
    for (int32_t turns = 0; turns < 4; ++turns)
    {
        mnavVec3 vertices[RAMP_LEVEL_VERTICES];
        int32_t indices[RAMP_LEVEL_INDICES];
        const mnavTriangleMesh mesh = {vertices, RAMP_LEVEL_VERTICES, indices,
                                       MakeRampLevel(vertices, indices, turns), nullptr};
        const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
        mnavBakeDef def = mnavDefaultBakeDef();
        mnavBaker* baker = nullptr;
        mnavBakeReport report;
        size_t size = 0;
        CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
                  mnavBaker_BakeTile(baker, &input, 0, 0, &report) == mnav_success &&
                  mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
              "baked");
        mnavDestroyBaker(baker);
        mnavNavmesh* navmesh = nullptr;
        mnavNavmeshDef meshDef = NavmeshDefFor(&def);
        CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
                  mnavNavmesh_StageTile(navmesh, s_tile, size).result == mnav_success &&
                  mnavNavmesh_Commit(navmesh) == mnav_success,
              "loaded");
        CHECK(report.regionCount == 2, "the floor and the ramp");
        // Along the side, the detail surface of the polygons that replace
        // the vertex's follows the floor, the ramp and the deck: their
        // heights are sampled from both regions, as the polygons mix them.
        bool follows = true;
        for (int32_t k = 0; k < 59; ++k)
        {
            float x = 31.875f;
            float z = 0.125f + 0.5f * (float)k;
            float y = z < 10.0f ? 0.0f : (z > 22.0f ? 3.0f : (z - 10.0f) * 0.25f);
            TurnOnTile(&x, &z, turns);
            mnavNearest n;
            CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){(double)x, (double)y, (double)z},
                                          (mnavVec3){0.01f, 0.4f, 0.01f}, nullptr,
                                          &n) == mnav_success,
                  "nearest");
            double off = n.point.y - (double)y;
            follows = follows && n.over && off >= 0.0 && off <= 0.15;
        }
        CHECK(follows, "the surface on the floor, the ramp and the deck");
        mnavDebugBuffer b = {{0.0, 0.0, 0.0}, s_drawVertices, DRAW_ROOM, 0, nullptr, 0, 0,
                             s_drawLines,     DRAW_ROOM / 2,  0};
        // Polygon fills go uncounted into a buffer with no triangle room.
        (void)mnavNavmesh_Debug(navmesh, &(mnavTileRange){0, 0, 0, 0}, &b);
        CHECK(b.vertexCount <= DRAW_ROOM && b.lineCount <= DRAW_ROOM / 2, "the edges fit");
        Side sides[16];
        int32_t count = 0;
        for (int32_t l = 0; l < b.lineCount; ++l)
        {
            const mnavDebugVertex* p = &s_drawVertices[s_drawLines[2 * l]];
            const mnavDebugVertex* q = &s_drawVertices[s_drawLines[2 * l + 1]];
            if (p->kind == mnav_debugTileSide && count < 16)
            {
                Side side = {p->x, p->z, q->x, q->z};
                TurnOnTile(&side.x0, &side.z0, 4 - turns);
                TurnOnTile(&side.x1, &side.z1, 4 - turns);
                sides[count++] = side;
            }
        }
        // Along +X, the ramp's region runs from the tile's corner past the
        // floor's under the ramp's foot, 12.75 m in, to the deck's end,
        // with no vertex where the regions meet; the floor's region takes
        // the rest beyond the deck. The deck ends at 29.5 m, or a cell
        // short as the cells fall when the scene is turned. The other
        // sides are whole.
        bool across = HasSide(sides, count, 32.0f, 0.0f, 32.0f, 29.5f) ||
                      HasSide(sides, count, 32.0f, 0.0f, 32.0f, 29.25f);
        CHECK(count == 5 && across && HasSide(sides, count, 32.0f, 19.0f, 32.0f, 32.0f) &&
                  HasSide(sides, count, 0.0f, 0.0f, 32.0f, 0.0f) &&
                  HasSide(sides, count, 0.0f, 0.0f, 0.0f, 32.0f) &&
                  HasSide(sides, count, 0.0f, 32.0f, 32.0f, 32.0f),
              "the side one edge across where the regions meet");
        // Every vertex left is a corner of some polygon: the one removed
        // is gone from the tile, not only from its polygons.
        int32_t corners = 0;
        for (int32_t l = 0; l < 2 * b.lineCount; ++l)
        {
            const mnavDebugVertex* v = &s_drawVertices[s_drawLines[l]];
            bool first = v->kind != mnav_debugTileBounds;
            for (int32_t m = 0; m < l && first; ++m)
            {
                const mnavDebugVertex* w = &s_drawVertices[s_drawLines[m]];
                first =
                    w->kind == mnav_debugTileBounds || w->x != v->x || w->y != v->y || w->z != v->z;
            }
            corners += first ? 1 : 0;
        }
        CHECK(corners == report.vertexCount, "every vertex a corner");
        // The two polygons round the vertex are replaced by one: five in
        // all, from six before the removal.
        CHECK(report.polygonCount == 5, "the hole filled by one polygon");
        mnavDestroyNavmesh(navmesh);
    }
}

int main(void)
{
    TestFloorAndRampMeetOnEachSide();
    return s_failures == 0 ? 0 : 1;
}
