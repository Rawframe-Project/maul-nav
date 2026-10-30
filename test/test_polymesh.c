// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Triangulating rings and merging them into a linked polygon mesh, read
// from bakes, mostly 2D: a floor over the tile is one quad on its four
// sides, an L is two convex polygons linked, regions weld their shared
// corners and link, stacked floors keep their own, the tile's vertex and
// polygon limits are typed, and rings with holes bridged in, a pinch,
// flat triangles or a diagonal through a vertex triangulate whole, with
// polygons meeting corner to corner, while a ring cut short or left
// unfinished is counted. Several scenes were found by a search of floors
// with blocks cut out or of other areas.

#include "defs_for.h"
#include "soup.h"
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
    // A floor's lowest corner, in cells of tile (0, 0).
    AT = 8,
    DRAW_ROOM = 1 << 16,
    MOST_OUTLINES = 8,
    MOST_POINTS = 16
};

static uint8_t s_tile[TILE_ROOM];
static mnavNavmesh* s_navmesh;
static mnavQuery* s_query;
static mnavBakeReport s_report;
static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawIndices[DRAW_ROOM];

// Outlines in cells from cell AT of tile (0, 0): points as x, z pairs.
typedef struct Shape
{
    int32_t points[2 * MOST_POINTS];
    int32_t count;
    mnavAreaType area;
} Shape;

static float Meters(int32_t cells)
{
    return (float)(AT + cells) * 0.25f;
}

// The default def, but an agent of no radius, regions of any size and no
// edge error, so that the polygons cover exactly the outlines' cells.
static mnavBakeDef Exact(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    return def;
}

// Bakes the shapes in 2D with a def; on success loads the tile alone in
// s_navmesh. Returns the bake's result.
static mnavResult Load(const mnavBakeDef* def, const Shape* shapes, int32_t count)
{
    static mnavVec2 points[MOST_OUTLINES][MOST_POINTS];
    mnavOutline outlines[MOST_OUTLINES];
    for (int32_t o = 0; o < count; ++o)
    {
        for (int32_t k = 0; k < shapes[o].count; ++k)
        {
            points[o][k] =
                (mnavVec2){Meters(shapes[o].points[2 * k]), Meters(shapes[o].points[2 * k + 1])};
        }
        outlines[o] = (mnavOutline){points[o], shapes[o].count, shapes[o].area};
    }
    mnavBaker* baker = nullptr;
    size_t size = 0;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success, "baker");
    mnavResult result =
        mnavBaker_BakeTile2D(baker, &(mnavBake2DInput){outlines, count, nullptr}, 0, 0, &s_report);
    if (result == mnav_success)
    {
        CHECK(mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success, "copied");
        mnavDestroyNavmesh(s_navmesh);
        s_navmesh = nullptr;
        mnavNavmeshDef meshDef = NavmeshDefFor(def);
        CHECK(mnavCreateNavmesh(&meshDef, &s_navmesh).result == mnav_success &&
                  mnavNavmesh_StageTile(s_navmesh, s_tile, size).result == mnav_success &&
                  mnavNavmesh_Commit(s_navmesh) == mnav_success,
              "loaded");
    }
    mnavDestroyBaker(baker);
    return result;
}

// The polygon over the center of cell (x, z) of the floor, or none.
static mnavNearest Over(int32_t x, int32_t z)
{
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(
              s_navmesh, (mnavPos3){(double)Meters(x) + 0.125, 0.0, (double)Meters(z) + 0.125},
              (mnavVec3){0.01f, 1.0f, 0.01f}, nullptr, &n) == mnav_success,
          "nearest");
    if (!n.over)
    {
        n.polygon.slot = 0;
    }
    return n;
}

// Whether a path leads from cell (x0, z0) to cell (x1, z1).
static bool Reaches(int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    mnavNearest a = Over(x0, z0);
    mnavNearest b = Over(x1, z1);
    mnavPath path;
    CHECK(a.polygon.slot != 0 && b.polygon.slot != 0 &&
              mnavQuery_FindPath(s_query, s_navmesh, a.polygon, a.point, b.polygon, b.point,
                                 nullptr, &path) == mnav_success,
          "searched");
    return path.end == mnav_pathFound;
}

// Twice the area of the tile's polygons, in square cells.
static int64_t TwiceArea(void)
{
    mnavDebugBuffer b = {
        {0.0, 0.0, 0.0}, s_drawVertices, DRAW_ROOM, 0, s_drawIndices, DRAW_ROOM, 0, nullptr, 0, 0};
    // The edges go uncounted into a buffer with no line room.
    (void)mnavNavmesh_Debug(s_navmesh, &(mnavTileRange){0, 0, 0, 0}, &b);
    CHECK(b.vertexCount <= DRAW_ROOM && b.triangleCount <= DRAW_ROOM, "the polygons fit");
    int64_t area = 0;
    for (int32_t t = 0; t < b.triangleCount / 3; ++t)
    {
        const mnavDebugVertex* p = &s_drawVertices[s_drawIndices[3 * t]];
        const mnavDebugVertex* q = &s_drawVertices[s_drawIndices[3 * t + 1]];
        const mnavDebugVertex* r = &s_drawVertices[s_drawIndices[3 * t + 2]];
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

// Whether no end of a polygon edge lies inside another edge, between its
// ends: the polygons meet corner to corner.
static bool CornersMeet(void)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0}, s_drawVertices, DRAW_ROOM, 0, nullptr, 0, 0,
                         s_drawIndices,   DRAW_ROOM,      0};
    // Polygon fills go uncounted into a buffer with no triangle room.
    (void)mnavNavmesh_Debug(s_navmesh, &(mnavTileRange){0, 0, 0, 0}, &b);
    bool meet = true;
    for (int32_t l = 0; l < b.lineCount / 2; ++l)
    {
        const mnavDebugVertex* p = &s_drawVertices[s_drawIndices[2 * l]];
        const mnavDebugVertex* q = &s_drawVertices[s_drawIndices[2 * l + 1]];
        for (int32_t m = 0; m < b.lineCount && p->kind != mnav_debugTileBounds; ++m)
        {
            const mnavDebugVertex* v = &s_drawVertices[s_drawIndices[m]];
            // Sixteenths of a cell, so 64 to the meter.
            int64_t ax = (int64_t)(p->x * 64.0f);
            int64_t az = (int64_t)(p->z * 64.0f);
            int64_t bx = (int64_t)(q->x * 64.0f);
            int64_t bz = (int64_t)(q->z * 64.0f);
            int64_t vx = (int64_t)(v->x * 64.0f);
            int64_t vz = (int64_t)(v->z * 64.0f);
            bool inLine = (bx - ax) * (vz - az) == (vx - ax) * (bz - az);
            int64_t t = (vx - ax) * (bx - ax) + (vz - az) * (bz - az);
            int64_t length = (bx - ax) * (bx - ax) + (bz - az) * (bz - az);
            meet = meet && !(v->kind != mnav_debugTileBounds && inLine && t > 0 && t < length);
        }
    }
    return meet;
}

// The tile-side edges, in cells of the tile, and their count.
static int32_t s_sides[16][4];

static int32_t Sides(void)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0}, s_drawVertices, DRAW_ROOM, 0, nullptr, 0, 0,
                         s_drawIndices,   DRAW_ROOM,      0};
    // Polygon fills go uncounted into a buffer with no triangle room.
    (void)mnavNavmesh_Debug(s_navmesh, &(mnavTileRange){0, 0, 0, 0}, &b);
    int32_t count = 0;
    for (int32_t l = 0; l < b.lineCount / 2; ++l)
    {
        const mnavDebugVertex* p = &s_drawVertices[s_drawIndices[2 * l]];
        const mnavDebugVertex* q = &s_drawVertices[s_drawIndices[2 * l + 1]];
        if (p->kind == mnav_debugTileSide && count < 16)
        {
            s_sides[count][0] = (int32_t)(p->x * 4.0f);
            s_sides[count][1] = (int32_t)(p->z * 4.0f);
            s_sides[count][2] = (int32_t)(q->x * 4.0f);
            s_sides[count][3] = (int32_t)(q->z * 4.0f);
            count += 1;
        }
    }
    return count;
}

static bool HasSide(int32_t count, int32_t x0, int32_t z0, int32_t x1, int32_t z1)
{
    for (int32_t s = 0; s < count; ++s)
    {
        const int32_t* d = s_sides[s];
        if ((d[0] == x0 && d[1] == z0 && d[2] == x1 && d[3] == z1) ||
            (d[0] == x1 && d[1] == z1 && d[2] == x0 && d[3] == z0))
        {
            return true;
        }
    }
    return false;
}

static void TestTileIsOneQuadOnFourSides(void)
{
    // A floor past the tile on every side: one quad, its edges the tile's
    // four sides.
    const Shape floor = {{-20, -20, -20, 150, 150, 150, 150, -20}, 4, mnav_areaWalkable};
    mnavBakeDef def = Exact();
    CHECK(Load(&def, &floor, 1) == mnav_success, "baked");
    CHECK(s_report.polygons == 1 && s_report.vertices == 4, "one quad");
    int32_t sides = Sides();
    CHECK(sides == 4 && HasSide(sides, 0, 0, 128, 0) && HasSide(sides, 128, 0, 128, 128) &&
              HasSide(sides, 128, 128, 0, 128) && HasSide(sides, 0, 128, 0, 0),
          "an edge on each of the four sides");
}

// An L of 64 cells: no single convex polygon covers it.
static const Shape s_l = {{0, 0, 0, 10, 4, 10, 4, 4, 10, 4, 10, 0}, 6, mnav_areaWalkable};

static void TestConcaveRingBecomesLinkedConvexPolygons(void)
{
    mnavBakeDef def = Exact();
    CHECK(Load(&def, &s_l, 1) == mnav_success, "baked");
    CHECK(s_report.polygons == 2 && s_report.vertices == 6, "two convex pieces");
    CHECK(Reaches(1, 9, 9, 1), "linked, from one arm to the other");
    CHECK(TwiceArea() == 2 * 64 && CornersMeet(), "the L's area, corner to corner");
}

static void TestRingsWeldAndLinkAcrossRegions(void)
{
    // Two squares side by side, of two areas, so two regions, sharing
    // x = 5: the shared edge's ends weld, and the quads link.
    const Shape squares[2] = {{{0, 0, 0, 5, 5, 5, 5, 0}, 4, mnav_areaWalkable},
                              {{5, 0, 5, 5, 10, 5, 10, 0}, 4, 7}};
    mnavBakeDef def = Exact();
    CHECK(Load(&def, squares, 2) == mnav_success, "baked");
    CHECK(s_report.regions == 2 && s_report.polygons == 2 && s_report.vertices == 6,
          "one quad per region, the shared edge's ends welded");
    CHECK(Reaches(1, 2, 8, 2) && Reaches(8, 2, 1, 2), "linked both ways across the regions");
    mnavAreaType left = 0;
    mnavAreaType right = 0;
    CHECK(mnavPolygon_GetArea(s_navmesh, Over(1, 2).polygon, &left) == mnav_success &&
              mnavPolygon_GetArea(s_navmesh, Over(8, 2).polygon, &right) == mnav_success &&
              left == mnav_areaWalkable && right == 7,
          "each quad its ring's area");
}

static void TestStackedFloorsKeepTheirCorners(void)
{
    // A floor and a deck 3 m over it, of the same square from 2 m to 4 m:
    // their corners share ground places 24 cell heights apart, too far to
    // weld, so each quad keeps four corners of its own.
    const mnavVec3 vertices[8] = {{2, 0, 2}, {2, 0, 4}, {4, 0, 4}, {4, 0, 2},
                                  {2, 3, 2}, {2, 3, 4}, {4, 3, 4}, {4, 3, 2}};
    const int32_t indices[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    const mnavTriangleMesh mesh = {vertices, 8, indices, 4, nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeDef def = Exact();
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBaker_BakeTile(baker, &input, 0, 0, &report) == mnav_success,
          "baked");
    CHECK(report.regions == 2 && report.polygons == 2 && report.vertices == 8,
          "two quads of four corners each");
    mnavDestroyBaker(baker);
}

static void TestLimitsAreTyped(void)
{
    // The L needs six vertices and two polygons.
    mnavBakeDef def = Exact();
    def.limits.tileVertices = 5;
    CHECK(Load(&def, &s_l, 1) == mnav_errorCapacity, "vertices");
    def = Exact();
    def.limits.tilePolygons = 1;
    CHECK(Load(&def, &s_l, 1) == mnav_errorCapacity, "polygons");
    def.limits.tileVertices = 6;
    def.limits.tilePolygons = 2;
    CHECK(Load(&def, &s_l, 1) == mnav_success, "at the limits");
}

static void TestBridgedRingWithAHiddenVertex(void)
{
    // A floor of 20 cells with three holes of 2 by 2 bridged in, found by
    // searching random holes: one hole's corner lies inside an ear whose
    // diagonal runs to a corner the ring passes twice, which the edge test
    // alone let through, losing the rest of the ring.
    const Shape shapes[4] = {{{0, 0, 0, 20, 20, 20, 20, 0}, 4, mnav_areaWalkable},
                             {{1, 14, 1, 16, 3, 16, 3, 14}, 4, mnav_areaNone},
                             {{6, 4, 6, 6, 8, 6, 8, 4}, 4, mnav_areaNone},
                             {{7, 10, 7, 12, 9, 12, 9, 10}, 4, mnav_areaNone}};
    mnavBakeDef def = Exact();
    CHECK(Load(&def, shapes, 4) == mnav_success, "baked");
    CHECK(s_report.partialRings == 0 && s_report.droppedHoles == 0, "the ring is complete");
    CHECK(TwiceArea() == 776 && CornersMeet(), "the floor less three holes, corner to corner");
}

static void TestPinchedRingNeedsTheLooseTest(void)
{
    // Found by a search of floors with single cells cut out or of area 7:
    // a floor of 8 by 8 with cells of area 7 at (0, 5) and (5, 3) and an
    // unwalkable one at (4, 4), within a cell of edge error. The cells at
    // (4, 4) and (5, 3) touch at a corner, and the floor's simplified ring
    // passes a corner twice: no strict ear finishes it, only a diagonal
    // that touches the pinch.
    const Shape shapes[4] = {{{0, 0, 0, 8, 8, 8, 8, 0}, 4, mnav_areaWalkable},
                             {{0, 5, 0, 6, 1, 6, 1, 5}, 4, 7},
                             {{4, 4, 4, 5, 5, 5, 5, 4}, 4, mnav_areaNone},
                             {{5, 3, 5, 4, 6, 4, 6, 3}, 4, 7}};
    mnavBakeDef def = Exact();
    def.maxEdgeError = 0.25f;
    CHECK(Load(&def, shapes, 4) == mnav_success, "baked");
    CHECK(s_report.partialRings == 0 && CornersMeet(), "every ring complete, corner to corner");
}

static void TestFlatTrianglesAreDropped(void)
{
    // Found by a search of floors with blocks cut out or of other areas:
    // within a cell of edge error, a ring of this floor of 4 by 4 leaves
    // triangles with no area. They cover nothing and a loader refuses
    // them, so they are dropped, and the ring still counts as whole.
    const Shape shapes[7] = {{{0, 0, 0, 4, 4, 4, 4, 0}, 4, mnav_areaWalkable},
                             {{1, 1, 1, 2, 3, 2, 3, 1}, 4, mnav_areaNone},
                             {{1, 3, 1, 5, 2, 5, 2, 3}, 4, 8},
                             {{1, 0, 1, 3, 4, 3, 4, 0}, 4, 8},
                             {{1, 0, 1, 2, 2, 2, 2, 0}, 4, 7},
                             {{3, 1, 3, 2, 4, 2, 4, 1}, 4, mnav_areaNone},
                             {{2, 2, 2, 5, 4, 5, 4, 2}, 4, mnav_areaNone}};
    mnavBakeDef def = Exact();
    def.maxEdgeError = 0.25f;
    CHECK(Load(&def, shapes, 7) == mnav_success, "baked and loaded");
    CHECK(s_report.partialRings == 0 && CornersMeet(), "every ring whole, corner to corner");
}

static void TestRingCutShortIsCounted(void)
{
    // Found by the same search: within 4 cells of edge error, a ring of
    // this floor of 7 by 7 is simplified so far that one of its triangles
    // turns the wrong way. It is dropped, and the bake counts the ring as
    // cut short.
    const Shape shapes[5] = {{{0, 0, 0, 7, 7, 7, 7, 0}, 4, mnav_areaWalkable},
                             {{2, 3, 2, 4, 5, 4, 5, 3}, 4, mnav_areaNone},
                             {{2, 2, 2, 5, 4, 5, 4, 2}, 4, 9},
                             {{2, 1, 2, 4, 4, 4, 4, 1}, 4, 9},
                             {{4, 0, 4, 2, 5, 2, 5, 0}, 4, 7}};
    mnavBakeDef def = Exact();
    def.maxEdgeError = 1.0f;
    CHECK(Load(&def, shapes, 5) == mnav_success, "baked and loaded");
    CHECK(s_report.partialRings == 1, "one ring cut short, counted");
}

static void TestEarDiagonalThroughAVertex(void)
{
    // Found by the same search: a floor of 9 by 9 with a cell of area 8
    // at (8, 4) and an unwalkable block from (5, 4), 2 cells by 3. One
    // strict ear's diagonal runs through a vertex of the ring; cut, it
    // would leave a flat triangle and the vertex inside an edge. The
    // strict test refuses a diagonal that touches a vertex: the polygons
    // meet corner to corner and cover the floor exactly.
    const Shape shapes[3] = {{{0, 0, 0, 9, 9, 9, 9, 0}, 4, mnav_areaWalkable},
                             {{8, 4, 8, 5, 9, 5, 9, 4}, 4, 8},
                             {{5, 4, 5, 7, 7, 7, 7, 4}, 4, mnav_areaNone}};
    mnavBakeDef def = Exact();
    CHECK(Load(&def, shapes, 3) == mnav_success, "baked and loaded");
    CHECK(s_report.partialRings == 0 && TwiceArea() == 2 * 75, "the floor covered exactly");
    CHECK(CornersMeet(), "corner to corner");
}

static void TestUnfinishedRingIsCounted(void)
{
    // Found by the same search: a floor of 5 by 5 with a strip of area 7
    // from (3, 1), 3 cells by 1, and an unwalkable cell at (2, 1), within
    // 4 cells of edge error. A simplified ring has no ear left, strict or
    // loose, before it is done, and the bake counts it.
    const Shape shapes[3] = {{{0, 0, 0, 5, 5, 5, 5, 0}, 4, mnav_areaWalkable},
                             {{3, 1, 3, 2, 6, 2, 6, 1}, 4, 7},
                             {{2, 1, 2, 2, 3, 2, 3, 1}, 4, mnav_areaNone}};
    mnavBakeDef def = Exact();
    def.maxEdgeError = 1.0f;
    CHECK(Load(&def, shapes, 3) == mnav_success, "baked and loaded");
    CHECK(s_report.partialRings == 1, "one ring unfinished, counted");
}

static void TestLevelRingsAreComplete(void)
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
              mnavBaker_BakeTile(baker, &input, 0, 0, &report) == mnav_success,
          "baked");
    CHECK(report.partialRings == 0 && report.polygons > 0, "every ring complete");
    mnavDestroyBaker(baker);
}

int main(void)
{
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    CHECK(mnavCreateQuery(&queryDef, &s_query) == mnav_success, "query");
    TestTileIsOneQuadOnFourSides();
    TestConcaveRingBecomesLinkedConvexPolygons();
    TestRingsWeldAndLinkAcrossRegions();
    TestStackedFloorsKeepTheirCorners();
    TestLimitsAreTyped();
    TestBridgedRingWithAHiddenVertex();
    TestPinchedRingNeedsTheLooseTest();
    TestFlatTrianglesAreDropped();
    TestRingCutShortIsCounted();
    TestEarDiagonalThroughAVertex();
    TestUnfinishedRingIsCounted();
    TestLevelRingsAreComplete();
    mnavDestroyNavmesh(s_navmesh);
    mnavDestroyQuery(s_query);
    return s_failures == 0 ? 0 : 1;
}
