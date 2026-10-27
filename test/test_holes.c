// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Merging hole contours into their outlines, read from 2D bakes of a
// square floor with square holes cut out: every hole is joined, and the
// tile's polygons cover the floor's cells and no hole's, exactly once.
// test_holes_rings.c covers dropping a hole, which no bake reaches.

#include "soup.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>
#include <stdio.h>

// The bridge floors' tile bytes, the same on every platform.
#define BRIDGES_HASH 0x1c1ed59ba8f7749bull

enum
{
    TILE_ROOM = 1 << 20,
    // The floor's lowest corner, in cells of tile (0, 0).
    AT = 8,
    MOST_HOLES = 25,
    DRAW_ROOM = 1 << 16
};

static uint8_t s_tile[TILE_ROOM];
static size_t s_size;
static mnavNavmesh* s_navmesh;
static mnavBakeReport s_report;
static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawTriangles[DRAW_ROOM];

// A floor of side cells with square holes of 2 by 2 cells, each by its
// lowest corner, in cells of the floor.
typedef struct Floor
{
    int32_t side;
    int32_t corners[2 * MOST_HOLES];
    int32_t holes;
} Floor;

static float Meters(int32_t cells)
{
    return (float)(AT + cells) * 0.25f;
}

static bool InHole(const Floor* floor, int32_t x, int32_t z)
{
    for (int32_t h = 0; h < floor->holes; ++h)
    {
        int32_t hx = floor->corners[2 * h];
        int32_t hz = floor->corners[2 * h + 1];
        if (x >= hx && x < hx + 2 && z >= hz && z < hz + 2)
        {
            return true;
        }
    }
    return false;
}

// Bakes the floor in 2D with no agent radius, smallest region or edge
// error, and loads its tile alone in s_navmesh.
static void Load(const Floor* floor)
{
    static mnavVec2 points[4 * (1 + MOST_HOLES)];
    static mnavOutline outlines[1 + MOST_HOLES];
    for (int32_t o = 0; o <= floor->holes; ++o)
    {
        int32_t x = o == 0 ? 0 : floor->corners[2 * (o - 1)];
        int32_t z = o == 0 ? 0 : floor->corners[2 * (o - 1) + 1];
        int32_t side = o == 0 ? floor->side : 2;
        mnavVec2* p = &points[4 * o];
        p[0] = (mnavVec2){Meters(x), Meters(z)};
        p[1] = (mnavVec2){Meters(x), Meters(z + side)};
        p[2] = (mnavVec2){Meters(x + side), Meters(z + side)};
        p[3] = (mnavVec2){Meters(x + side), Meters(z)};
        outlines[o] = (mnavOutline){p, 4, o == 0 ? mnav_areaWalkable : mnav_areaNone};
    }
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, &(mnavBake2DInput){outlines, 1 + floor->holes, nullptr}, 0, 0,
                             &s_report) == mnav_success &&
              mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &s_size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, s_size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
}

// Whether a polygon lies over the center of cell (x, z) of the floor.
static bool Covered(int32_t x, int32_t z)
{
    mnavNearest n;
    CHECK(mnavFindNearest(s_navmesh, nullptr,
                          (mnavPos3){(double)Meters(x) + 0.125, 0.0, (double)Meters(z) + 0.125},
                          (mnavVec3){0.01f, 1.0f, 0.01f}, &n) == mnav_success,
          "nearest");
    return n.polygon.slot != 0 && n.over;
}

// Twice the area of the tile's polygons, in square cells.
static int64_t TwiceArea(void)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0},
                         s_drawVertices,
                         DRAW_ROOM,
                         0,
                         s_drawTriangles,
                         DRAW_ROOM,
                         0,
                         nullptr,
                         0,
                         0};
    // The edges go uncounted into a buffer with no line room.
    (void)mnavDebugNavmesh(s_navmesh, 0, 0, 0, 0, &b);
    CHECK(b.vertexCount <= DRAW_ROOM && b.triangleCount <= DRAW_ROOM, "the polygons fit");
    int64_t area = 0;
    for (int32_t t = 0; t < b.triangleCount / 3; ++t)
    {
        const mnavDebugVertex* p = &s_drawVertices[s_drawTriangles[3 * t]];
        const mnavDebugVertex* q = &s_drawVertices[s_drawTriangles[3 * t + 1]];
        const mnavDebugVertex* r = &s_drawVertices[s_drawTriangles[3 * t + 2]];
        if (p->kind != mnav_debugPolygon)
        {
            continue;
        }
        // Sixteenths of a cell, so 64 to the meter.
        int64_t ax = (int64_t)(p->x * 64.0f);
        int64_t az = (int64_t)(p->z * 64.0f);
        int64_t bx = (int64_t)(q->x * 64.0f) - ax;
        int64_t bz = (int64_t)(q->z * 64.0f) - az;
        int64_t cx = (int64_t)(r->x * 64.0f) - ax;
        int64_t cz = (int64_t)(r->z * 64.0f) - az;
        int64_t twice = bx * cz - cx * bz;
        area += twice < 0 ? -twice : twice;
    }
    return area / 256;
}

// Bakes a floor and checks that every hole joined its outline: none
// dropped, every ring triangulated whole, a polygon over every floor
// cell and none over a hole's, and the polygons' area the floor's.
static bool Merged(const Floor* floor)
{
    Load(floor);
    bool covered = s_report.droppedHoles == 0 && s_report.partialRings == 0;
    int32_t cells = 0;
    for (int32_t z = 0; z < floor->side; ++z)
    {
        for (int32_t x = 0; x < floor->side; ++x)
        {
            bool hole = InHole(floor, x, z);
            covered = covered && Covered(x, z) == !hole;
            cells += hole ? 0 : 1;
        }
    }
    return covered && TwiceArea() == 2 * (int64_t)cells;
}

static void TestHolesMergeIntoTheOutline(void)
{
    // A floor of 25 cells less a hole of 4, and less two of 4.
    Floor one = {5, {2, 2}, 1};
    CHECK(Merged(&one), "one hole");
    Floor two = {7, {1, 1, 4, 4}, 2};
    CHECK(Merged(&two), "two holes");
}

static void TestBridgesTakeTheNearestCorner(void)
{
    // A bridge goes from the hole's lowest-left corner to the nearest
    // outline corner in its cone, the outline's first of equally near
    // ones. Another choice covers the floor alike and moves only the
    // order of the polygons and their edges, so the tiles' bytes are
    // pinned: a hole at (2, 2), nearest to (0, 0); one at (1, 5), nearer
    // to (0, 8) than (0, 0) though as far along X; and one at (5, 5), as
    // far from all four corners, bridged to the lowest-left.
    const Floor floors[3] = {{5, {2, 2}, 1}, {8, {1, 5}, 1}, {10, {5, 5}, 1}};
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t f = 0; f < 3; ++f)
    {
        CHECK(Merged(&floors[f]), "joined");
        hash = mnavHash64(hash, s_tile, (int32_t)s_size);
    }
    printf("BRIDGES_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == BRIDGES_HASH, "the pinned bytes");
}

static void TestBridgeAvoidsAWaitingHole(void)
{
    // The first hole's nearest corner, (20, 0), lies behind the second
    // hole; the bridge must go elsewhere and nothing may cross.
    Floor floor = {20, {14, 4, 16, 1}, 2};
    CHECK(Merged(&floor), "both holes joined");
}

static void TestManyHolesNeverCross(void)
{
    // A grid of 25 holes: later bridges must not cross earlier ones,
    // which are part of the outline by then.
    Floor floor = {30, {0}, 25};
    for (int32_t i = 0; i < 25; ++i)
    {
        floor.corners[2 * i] = 2 + 5 * (i % 5) + (i / 5) % 2;
        floor.corners[2 * i + 1] = 2 + 5 * (i / 5) + (i % 5) % 3;
    }
    CHECK(Merged(&floor), "every hole joined");
}

static void TestBridgeNeverCrossesAnEarlierBridge(void)
{
    // Found by a search: the last hole's nearest candidate lies across a
    // bridge made earlier, which is part of the outline by then.
    Floor floor = {20, {16, 5, 15, 1, 12, 9}, 3};
    CHECK(Merged(&floor), "every hole joined");
}

static void TestSecondBridgeTakesTheCopyFacingIt(void)
{
    // Both holes are nearest to corner (0, 0), 65 squared cells away,
    // nearer than to each other: after the first bridge (0, 0) appears
    // twice, and the second hole, below that bridge, must leave from the
    // copy between the -Z wall and the bridge.
    Floor floor = {40, {1, 8, 8, 1}, 2};
    CHECK(Merged(&floor), "both holes joined");
}

static void TestHolesTiedOnTheirCornerMerge(void)
{
    // Two holes touching at a corner, (4, 4), one above it and one to its
    // +X side below, and the same mirrored: their lowest-left corners tie
    // or nearly, and both join.
    Floor floor = {12, {2, 4, 4, 2}, 2};
    CHECK(Merged(&floor), "touching at a corner");
    Floor mirrored = {12, {4, 4, 2, 2}, 2};
    CHECK(Merged(&mirrored), "touching at the other corner");
}

static void TestLevelMergesEveryHole(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    const mnavTriangleMesh mesh = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile(baker, &input, 0, 0, &report) == mnav_success,
          "baked");
    CHECK(report.droppedHoles == 0 && report.partialRings == 0, "every hole joined");
    mnavDestroyBaker(baker);
}

int main(void)
{
    TestHolesMergeIntoTheOutline();
    TestHolesTiedOnTheirCornerMerge();
    TestBridgesTakeTheNearestCorner();
    TestBridgeAvoidsAWaitingHole();
    TestSecondBridgeTakesTheCopyFacingIt();
    TestManyHolesNeverCross();
    TestBridgeNeverCrossesAnEarlierBridge();
    TestLevelMergesEveryHole();
    mnavDestroyNavmesh(s_navmesh);
    return s_failures == 0 ? 0 : 1;
}
