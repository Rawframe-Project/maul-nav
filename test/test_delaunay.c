// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The detail mesh's triangulation kept Delaunay by flips, read from baked
// tiles: each polygon's detail triangles are one triangulation, wound
// alike, in which the far corner across each inner edge lies outside the
// circumcircle. A sample on an edge between two triangles splits both.

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
    POLYGON_ROOM = 2048,
    VERTEX_ROOM = 4096,
    // Detail places are sixteenths of a 0.25 m cell.
    PER_METER = 64
};

static uint8_t s_tile[TILE_ROOM];
static mnavNavmesh* s_navmesh;
static mnavDebugVertex s_vertices[VERTEX_ROOM];
static uint32_t s_triangles[VERTEX_ROOM];

// Bakes tile (0, 0) of an input and loads it alone in s_navmesh.
static void Load(const mnavBakeDef* def, const mnavBakeInput* input, mnavBakeReport* report)
{
    mnavBaker* baker = nullptr;
    size_t size = 0;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success &&
              mnavBaker_BakeTile(baker, input, 0, 0, report) == mnav_success &&
              mnavBaker_CopyTile(baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(def, &s_navmesh).result == mnav_success &&
              mnavNavmesh_StageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavNavmesh_Commit(s_navmesh) == mnav_success,
          "loaded");
}

// The polygons over every fourth cell center of the tile, each once.
static int32_t Polygons(mnavPolygonId* polygons)
{
    int32_t count = 0;
    for (int32_t z = 0; z < 128; z += 4)
    {
        for (int32_t x = 0; x < 128; x += 4)
        {
            mnavNearest n;
            CHECK(mnavNavmesh_FindNearest(
                      s_navmesh, (mnavPos3){(x + 0.5) * 0.25, 0.0, (z + 0.5) * 0.25},
                      (mnavVec3){0.01f, 20.0f, 0.01f}, nullptr, &n) == mnav_success,
                  "nearest");
            bool seen = n.polygon.slot == 0 || !n.over;
            for (int32_t k = 0; k < count && !seen; ++k)
            {
                seen = polygons[k].polygon == n.polygon.polygon;
            }
            if (!seen && count < POLYGON_ROOM)
            {
                polygons[count++] = n.polygon;
            }
        }
    }
    return count;
}

typedef struct Point
{
    int64_t x;
    int64_t z;
} Point;

// Corner i of the drawing, in sixteenths of a cell.
static Point At(uint32_t i)
{
    return (Point){(int64_t)(s_vertices[i].x * PER_METER), (int64_t)(s_vertices[i].z * PER_METER)};
}

static bool Same(Point a, Point b)
{
    return a.x == b.x && a.z == b.z;
}

// Twice the signed area of (a, b, c), negative for the detail's winding.
static int64_t Area2(Point a, Point b, Point c)
{
    return (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z);
}

// Whether d lies strictly inside the circumcircle of (a, b, c), wound
// negatively, in exact integers.
static bool InCircle(Point a, Point b, Point c, Point d)
{
    int64_t adx = a.x - d.x;
    int64_t adz = a.z - d.z;
    int64_t bdx = b.x - d.x;
    int64_t bdz = b.z - d.z;
    int64_t cdx = c.x - d.x;
    int64_t cdz = c.z - d.z;
    int64_t determinant = (adx * adx + adz * adz) * (bdx * cdz - cdx * bdz) +
                          (bdx * bdx + bdz * bdz) * (cdx * adz - adx * cdz) +
                          (cdx * cdx + cdz * cdz) * (adx * bdz - bdx * adz);
    return determinant < 0;
}

// One polygon's detail triangles: how many, their distinct corners and
// the edges with no triangle across; whether every triangle is wound
// negatively, every edge appears once with at most one across, and the
// far corner across every inner edge is outside the circumcircle.
typedef struct Shape
{
    int32_t triangles;
    int32_t corners;
    int32_t outline;
    bool wound;
    bool paired;
    bool delaunay;
} Shape;

static Shape Read(mnavPolygonId polygon)
{
    mnavDebugBuffer b = {{0.0, 0.0, 0.0}, s_vertices, VERTEX_ROOM, 0, s_triangles,
                         VERTEX_ROOM,     0,          nullptr,     0, 0};
    CHECK(mnavNavmesh_DebugCorridor(s_navmesh, &polygon, 1, &b) == mnav_success, "drawn");
    Shape shape = {b.triangleCount / 3, 0, 0, true, true, true};
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        bool first = true;
        for (int32_t j = 0; j < i && first; ++j)
        {
            first = !Same(At((uint32_t)i), At((uint32_t)j));
        }
        shape.corners += first ? 1 : 0;
    }
    for (int32_t t = 0; t < shape.triangles; ++t)
    {
        const uint32_t* c = &s_triangles[3 * t];
        shape.wound = shape.wound && Area2(At(c[0]), At(c[1]), At(c[2])) < 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            Point u = At(c[k]);
            Point v = At(c[(k + 1) % 3]);
            Point w = At(c[(k + 2) % 3]);
            int32_t same = 0;
            int32_t across = 0;
            for (int32_t n = 0; n < shape.triangles; ++n)
            {
                const uint32_t* d = &s_triangles[3 * n];
                for (int32_t j = 0; j < 3; ++j)
                {
                    Point p = At(d[j]);
                    Point q = At(d[(j + 1) % 3]);
                    same += Same(p, u) && Same(q, v) ? 1 : 0;
                    if (Same(p, v) && Same(q, u))
                    {
                        across += 1;
                        shape.delaunay = shape.delaunay && !InCircle(u, v, w, At(d[(j + 2) % 3]));
                    }
                }
            }
            shape.paired = shape.paired && same == 1 && across <= 1;
            shape.outline += across == 0 ? 1 : 0;
        }
    }
    return shape;
}

// Whether a shape is one Delaunay triangulation of its corners: a
// triangulation of v points, h of them on its outline, has 2 v - 2 - h
// triangles.
static bool Delaunay(Shape s)
{
    return s.wound && s.paired && s.delaunay && s.triangles == 2 * s.corners - 2 - s.outline;
}

static mnavVec3 s_mesh[9];
static int32_t s_meshIndices[12 * 3];

// Sets triangle t of s_mesh, wound to face up.
static void Face(int32_t t, int32_t a, int32_t b, int32_t c)
{
    mnavVec3 p = s_mesh[a];
    mnavVec3 q = s_mesh[b];
    mnavVec3 r = s_mesh[c];
    bool up = (q.z - p.z) * (r.x - p.x) - (q.x - p.x) * (r.z - p.z) > 0.0f;
    s_meshIndices[3 * t] = a;
    s_meshIndices[3 * t + 1] = up ? b : c;
    s_meshIndices[3 * t + 2] = up ? c : b;
}

static void TestSampleOnADiagonalSplitsBothTriangles(void)
{
    // A pyramid 4 m tall over the tile, its faces meeting along the
    // tile's diagonals, in a flat floor around it.
    for (int32_t k = 0; k < 4; ++k)
    {
        float x = k == 1 || k == 2 ? 1.0f : 0.0f;
        float z = k >= 2 ? 1.0f : 0.0f;
        s_mesh[k] = (mnavVec3){32.0f * x, 0.0f, 32.0f * z};
        s_mesh[4 + k] = (mnavVec3){40.0f * x - 4.0f, 0.0f, 40.0f * z - 4.0f};
    }
    s_mesh[8] = (mnavVec3){16.0f, 4.0f, 16.0f};
    for (int32_t k = 0; k < 4; ++k)
    {
        int32_t next = (k + 1) % 4;
        Face(k, k, next, 8);
        Face(4 + 2 * k, k, 4 + k, 4 + next);
        Face(5 + 2 * k, k, 4 + next, next);
    }
    const mnavTriangleMesh mesh = {s_mesh, 9, s_meshIndices, 12, nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    def.minRegionArea = 0.0f;
    def.maxEdgeError = 0.0f;
    def.detailSampleDistance = 1.0f;
    def.detailMaxError = 0.25f;
    mnavBakeReport report;
    Load(&def, &input, &report);
    // The tile is one square polygon, its outline split in two along a
    // diagonal. The top is the sample farthest off and lies on that
    // diagonal: both triangles split there, into the pyramid's faces,
    // and no other sample is off them.
    mnavNearest n;
    CHECK(mnavNavmesh_FindNearest(s_navmesh, (mnavPos3){16.0, 4.0, 16.0},
                                  (mnavVec3){0.01f, 1.0f, 0.01f}, nullptr, &n) == mnav_success &&
              n.over && report.polygons == 1,
          "one polygon");
    Shape s = Read(n.polygon);
    CHECK(Delaunay(s) && s.triangles == 4 && s.corners == 5 && s.outline == 4, "four faces");
    int32_t top = 0;
    for (int32_t i = 0; i < 3 * s.triangles; ++i)
    {
        const mnavDebugVertex* v = &s_vertices[s_triangles[i]];
        top += v->x == 16.0f && v->y == 4.0f && v->z == 16.0f ? 1 : 0;
    }
    CHECK(top == 4, "every face has the top as a corner");
}

enum
{
    SIDE = 34
};

static float s_heights[SIDE * SIDE];

static void TestBumpyFloorStaysDelaunay(void)
{
    // A terrain over the tile, bumps of 0 to 0.25 m a meter apart; without
    // samples the outlines' triangulations are made Delaunay alone, with
    // them samples go in one by one.
    for (int32_t r = 0; r < SIDE; ++r)
    {
        for (int32_t c = 0; c < SIDE; ++c)
        {
            s_heights[r * SIDE + c] = 0.0625f * (float)((c * 7 + r * 13) % 5);
        }
    }
    const mnavTerrain terrain = {{-1.0f, 0.0f, -1.0f}, 1.0f, 1.0f, SIDE, SIDE, s_heights, nullptr};
    const mnavBakeInput input = {nullptr, 0, &terrain, 1, nullptr, 0, nullptr};
    const float samples[3] = {0.0f, 0.5f, 1.5f};
    const float errors[2] = {0.01f, 0.05f};
    for (int32_t s = 0; s < 3; ++s)
    {
        for (int32_t e = 0; e < 2; ++e)
        {
            mnavBakeDef def = mnavDefaultBakeDef();
            def.detailSampleDistance = samples[s];
            def.detailMaxError = errors[e];
            mnavBakeReport report;
            Load(&def, &input, &report);
            static mnavPolygonId polygons[POLYGON_ROOM];
            int32_t count = Polygons(polygons);
            bool all = count == report.polygons;
            int32_t inner = 0;
            for (int32_t p = 0; p < count; ++p)
            {
                Shape shape = Read(polygons[p]);
                all = all && Delaunay(shape);
                inner += shape.corners - shape.outline;
            }
            CHECK(all, "every polygon Delaunay");
            CHECK((inner > 0) == (s > 0), "samples inside when sampled");
        }
    }
}

int main(void)
{
    TestSampleOnADiagonalSplitsBothTriangles();
    TestBumpyFloorStaysDelaunay();
    mnavDestroyNavmesh(s_navmesh);
    return s_failures == 0 ? 0 : 1;
}
