// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Removing tile-border vertices, read from baked tiles: where a floor and
// a ramp over it, two regions of one area, meet on the tile's side, their
// vertex there goes, and the side is one edge across their meeting. The
// scene is baked turned to each of the tile's four sides.
// test_border_vertices_rings.c covers the cases no bake reaches.

#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>

enum
{
    TILE_ROOM = 1 << 20,
    DRAW_ROOM = 1 << 14,
    SCENE_VERTICES = 16,
    SCENE_INDICES = 48
};

static uint8_t s_tile[TILE_ROOM];
static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawLines[DRAW_ROOM];

// A place on the ground turned a quarter turn round the tile's middle,
// (16, 16) m, turns times.
static void Turn(float* x, float* z, int32_t turns)
{
    for (int32_t t = 0; t < turns; ++t)
    {
        float nx = 32.0f - *z;
        *z = *x;
        *x = nx;
    }
}

// A floor, and a ramp from it up to a deck 3 m high on a box, both
// crossing the tile's +X side at x = 32 m: the floor and the ramp overlap,
// so they are two regions, and their boundary meets the tile's side.
// Turned a quarter turn turns times. Returns the number of triangles.
static int32_t MakeRampLevel(mnavVec3* vertices, int32_t* indices, int32_t turns)
{
    const mnavVec3 corners[8] = {{-1.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 41.0f}, {41.0f, 0.0f, 41.0f},
                                 {41.0f, 0.0f, -1.0f}, {28.0f, 0.0f, 10.0f}, {28.0f, 3.0f, 22.0f},
                                 {36.0f, 3.0f, 22.0f}, {36.0f, 0.0f, 10.0f}};
    const int32_t quads[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    for (int32_t k = 0; k < 8; ++k)
    {
        vertices[k] = corners[k];
    }
    for (int32_t k = 0; k < 12; ++k)
    {
        indices[k] = quads[k];
    }
    int32_t index = AddBox(vertices, indices, 8, 12, (mnavVec3){28.0f, 2.8f, 22.0f},
                           (mnavVec3){36.0f, 3.0f, 30.0f});
    for (int32_t k = 0; k < SCENE_VERTICES; ++k)
    {
        Turn(&vertices[k].x, &vertices[k].z, turns);
    }
    return index / 3;
}

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
        mnavVec3 vertices[SCENE_VERTICES];
        int32_t indices[SCENE_INDICES];
        const mnavTriangleMesh mesh = {vertices, SCENE_VERTICES, indices,
                                       MakeRampLevel(vertices, indices, turns), nullptr};
        const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
        mnavBakeDef def = mnavDefaultBakeDef();
        mnavBaker* baker = nullptr;
        mnavBakeReport report;
        size_t size = 0;
        CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
                  mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_success &&
                  mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
              "baked");
        mnavDestroyBaker(baker);
        mnavNavmesh* navmesh = nullptr;
        CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
                  mnavStageTile(navmesh, s_tile, size).result == mnav_success &&
                  mnavCommit(navmesh) == mnav_success,
              "loaded");
        CHECK(report.regions == 2, "the floor and the ramp");
        mnavDebugBuffer b = {{0.0, 0.0, 0.0}, s_drawVertices, DRAW_ROOM, 0, nullptr, 0, 0,
                             s_drawLines,     DRAW_ROOM,      0};
        // Polygon fills go uncounted into a buffer with no triangle room.
        (void)mnavDebugNavmesh(navmesh, 0, 0, 0, 0, &b);
        CHECK(b.vertexCount <= DRAW_ROOM && b.lineCount <= DRAW_ROOM, "the edges fit");
        Side sides[16];
        int32_t count = 0;
        for (int32_t l = 0; l < b.lineCount / 2; ++l)
        {
            const mnavDebugVertex* p = &s_drawVertices[s_drawLines[2 * l]];
            const mnavDebugVertex* q = &s_drawVertices[s_drawLines[2 * l + 1]];
            if (p->kind == mnav_debugTileSide && count < 16)
            {
                Side side = {p->x, p->z, q->x, q->z};
                Turn(&side.x0, &side.z0, 4 - turns);
                Turn(&side.x1, &side.z1, 4 - turns);
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
        for (int32_t l = 0; l < b.lineCount; ++l)
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
        CHECK(corners == report.vertices, "every vertex a corner");
        // The two polygons round the vertex are replaced by one: five in
        // all, from six before the removal.
        CHECK(report.polygons == 5, "the hole filled by one polygon");
        mnavDestroyNavmesh(navmesh);
    }
}

int main(void)
{
    TestFloorAndRampMeetOnEachSide();
    return s_failures == 0 ? 0 : 1;
}
