// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The detail mesh: edge samples and their triangulation, read from baked
// tiles. Each polygon's detail triangles are drawn alone and read in
// sixteenths of a cell on the ground and whole cell heights up.

#include "soup.h"
#include "squares.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/debug.h"

#include <stdint.h>
#include <stdio.h>

// The bytes of the tile with a bump on its edge, the same on every
// platform.
#define BUMP_HASH 0xa66f564a7b20c654ull

enum
{
    DRAW_ROOM = 1 << 14
};

static mnavDebugVertex s_drawVertices[DRAW_ROOM];
static uint32_t s_drawTriangles[DRAW_ROOM];

// A corner of a detail triangle: x and z in sixteenths of a cell of the
// scene, y in cell heights.
typedef struct Corner
{
    int64_t x;
    int64_t y;
    int64_t z;
} Corner;

static Corner At16(uint32_t i)
{
    const mnavDebugVertex* v = &s_drawVertices[i];
    return (Corner){(int64_t)(v->x * 64.0f) - AT * 16, (int64_t)(v->y * 8.0f),
                    (int64_t)(v->z * 64.0f) - AT * 16};
}

static bool SamePlace(Corner a, Corner b)
{
    return a.x == b.x && a.z == b.z;
}

// One polygon's detail: its triangles, its distinct corners and the
// triangle edges with no triangle across.
typedef struct Part
{
    int32_t triangles;
    int32_t vertices;
    int32_t outline;
    // Every triangle wound like the polygons, and each edge once.
    bool sound;
    int64_t twiceArea;
} Part;

static Part Read(mnavPolygonId polygon)
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
    CHECK(mnavDebugCorridor(s_navmesh, &polygon, 1, &b) == mnav_success, "drawn");
    Part part = {b.triangleCount / 3, 0, 0, true, 0};
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        bool first = true;
        for (int32_t j = 0; j < i && first; ++j)
        {
            first = !SamePlace(At16((uint32_t)i), At16((uint32_t)j));
        }
        part.vertices += first ? 1 : 0;
    }
    for (int32_t t = 0; t < part.triangles; ++t)
    {
        const uint32_t* c = &s_drawTriangles[3 * t];
        Corner a = At16(c[0]);
        Corner p = At16(c[1]);
        Corner q = At16(c[2]);
        int64_t twice = (p.x - a.x) * (q.z - a.z) - (q.x - a.x) * (p.z - a.z);
        part.sound = part.sound && twice < 0;
        part.twiceArea -= twice;
        for (int32_t k = 0; k < 3; ++k)
        {
            Corner u = At16(c[k]);
            Corner v = At16(c[(k + 1) % 3]);
            int32_t same = 0;
            int32_t across = 0;
            for (int32_t n = 0; n < part.triangles; ++n)
            {
                for (int32_t j = 0; j < 3; ++j)
                {
                    Corner s = At16(s_drawTriangles[3 * n + j]);
                    Corner e = At16(s_drawTriangles[3 * n + (j + 1) % 3]);
                    same += SamePlace(s, u) && SamePlace(e, v) ? 1 : 0;
                    across += SamePlace(s, v) && SamePlace(e, u) ? 1 : 0;
                }
            }
            part.sound = part.sound && same == 1 && across <= 1;
            part.outline += across == 0 ? 1 : 0;
        }
    }
    return part;
}

// Whether the polygon's detail has a corner at (x, y, z), x and z in
// sixteenths of a cell of the scene, y in cell heights.
static bool HasCorner(mnavPolygonId polygon, int64_t x, int64_t y, int64_t z)
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
    CHECK(mnavDebugCorridor(s_navmesh, &polygon, 1, &b) == mnav_success, "drawn");
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        Corner c = At16((uint32_t)i);
        if (c.x == x && c.y == y && c.z == z)
        {
            return true;
        }
    }
    return false;
}

// The polygon's detail corners on the line x = lineX, or z = lineZ when
// lineX is negative, in sixteenths of a cell of the scene.
static int32_t OnLine(mnavPolygonId polygon, int64_t lineX, int64_t lineZ)
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
    CHECK(mnavDebugCorridor(s_navmesh, &polygon, 1, &b) == mnav_success, "drawn");
    int32_t count = 0;
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        Corner c = At16((uint32_t)i);
        bool first = true;
        for (int32_t j = 0; j < i && first; ++j)
        {
            first = !SamePlace(c, At16((uint32_t)j));
        }
        count += first && (lineX >= 0 ? c.x == lineX : c.z == lineZ) ? 1 : 0;
    }
    return count;
}

// The polygon over the center of cell (x, z) of the scene within a meter
// of a level.
static mnavPolygonId Over(int32_t x, int32_t z, int32_t level)
{
    mnavPos3 center = {(AT + x + 0.5) * 0.25, level * 0.125, (AT + z + 0.5) * 0.25};
    mnavNearest n;
    CHECK(mnavFindNearest(s_navmesh, nullptr, center, (mnavVec3){0.01f, 1.0f, 0.01f}, &n) ==
                  mnav_success &&
              n.polygon.slot != 0 && n.over,
          "a polygon over the cell");
    return n.polygon;
}

// The exact def with detail samples every sample cells within error cell
// heights, both in sixteenths.
static void UseDetail(int32_t sample, int32_t error)
{
    mnavBakeDef def = Exact();
    def.detailSampleDistance = 0.25f * (float)sample / 16.0f;
    def.detailMaxError = 0.125f * (float)error / 16.0f;
    UseDef(&def);
}

// The scene's floor with cell (x, z) raised to a level.
static Scene Bumped(int32_t x, int32_t z, int32_t level)
{
    Scene scene = Flat(0, false);
    if (x >= 0)
    {
        scene.level[z][x] = level;
    }
    return scene;
}

static void TestFlatSquare(void)
{
    // Inside the ledge ring the floor's cells are 1 to 7, one quad from
    // (1, 1) to (8, 8), its top at level 1.
    UseDetail(0, 16);
    Scene scene = Bumped(-1, -1, 0);
    Load(&scene);
    Part part = Read(Over(4, 4, 0));
    CHECK(part.sound && part.vertices == 4 && part.triangles == 2 && part.outline == 4,
          "no samples: the polygon's two triangles");
    CHECK(HasCorner(Over(4, 4, 0), 8 * 16, 1, 16), "corners in sixteenths, at the top");
    UseDetail(16, 16);
    Load(&scene);
    part = Read(Over(4, 4, 0));
    CHECK(part.vertices == 4, "flat samples are dropped");
    CHECK(s_report.fallbackHeights == 0, "every sample found a height");
}

static void TestBumpOnAnEdgeIsSampled(void)
{
    // Cell (4, 1), on the quad's -Z edge, 4 levels up.
    Scene scene = Bumped(4, 1, 4);
    UseDetail(16, 16);
    Load(&scene);
    mnavPolygonId polygon = Over(2, 4, 0);
    // Samples fall every cell; the bump's and, to hold its sides within
    // the error, its two neighbors are kept.
    CHECK(OnLine(polygon, -1, 16) == 5 && HasCorner(polygon, 64, 5, 16) &&
              HasCorner(polygon, 48, 1, 16) && HasCorner(polygon, 80, 1, 16),
          "on the edge, the bump and its neighbors, nothing else");
    Part part = Read(polygon);
    CHECK(part.sound && part.outline == 7, "seven outline edges");
    CHECK(part.vertices > 7, "samples inside hold the floor round the bump");
    // The tile also keeps, per detail triangle, which of its edges lie on
    // the polygon's outline; nothing reads them back, so the bytes are
    // pinned.
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, s_tile, (int32_t)s_tileSize);
    printf("BUMP_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == BUMP_HASH, "the pinned bytes");
    // Within the maximum error the bump is dropped.
    UseDetail(16, 64);
    Load(&scene);
    CHECK(Read(Over(2, 4, 0)).vertices == 4, "4 cell heights is within 64 sixteenths");
}

static void TestHillInsideIsSampled(void)
{
    // A hill at cell (5, 5), 5 levels up; the edges stay flat.
    Scene scene = Bumped(5, 5, 5);
    UseDetail(16, 16);
    Load(&scene);
    mnavPolygonId polygon = Over(2, 2, 0);
    Part part = Read(polygon);
    CHECK(part.sound && part.outline == 4, "the outline unsampled");
    CHECK(HasCorner(polygon, 80, 6, 80), "the hilltop added");
    CHECK(s_report.cappedDetail == 0, "within the cap");
    // A hill exactly the maximum error tall is within it.
    scene = Bumped(5, 5, 1);
    Load(&scene);
    CHECK(Read(Over(2, 2, 0)).vertices == 4, "one cell height is within 16 sixteenths");
    // With samples twice as far apart the grid, every second cell of the
    // tile, misses the hill's cell.
    scene = Bumped(5, 5, 5);
    UseDetail(32, 16);
    Load(&scene);
    CHECK(Read(Over(2, 2, 0)).vertices == 4, "nothing to add");
}

static void TestSharedEdgeIsSampledAlike(void)
{
    // Area 7 from x = 4 on: two quads sharing x = 4, and a bump at (4, 3)
    // on the shared edge.
    Scene scene = Bumped(4, 3, 4);
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 4; x < SIDE; ++x)
        {
            scene.area[z][x] = 7;
        }
    }
    UseDetail(16, 16);
    Load(&scene);
    mnavPolygonId left = Over(2, 3, 0);
    mnavPolygonId right = Over(6, 3, 0);
    CHECK(HasCorner(left, 64, 5, 48) && HasCorner(right, 64, 5, 48),
          "both sides hold the bump's sample");
    CHECK(OnLine(left, 64, -1) == 5 && OnLine(right, 64, -1) == 5,
          "five points on the shared edge each side");
}

static void TestTiesKeepTheFirstFromTheLexicalEnd(void)
{
    // Area 7 from x = 4 on, and a ridge two cells long on the shared edge
    // x = 4, at (4, 2) and (4, 3), 2 levels up. Its two samples stray
    // equally far, and once one is kept the other is within the error.
    // The edge is sampled from its lexically first end, (4, 1), so both
    // polygons keep the sample nearer it, whichever way each runs it.
    Scene scene = Flat(0, false);
    for (int32_t z = 0; z < SIDE; ++z)
    {
        for (int32_t x = 4; x < SIDE; ++x)
        {
            scene.area[z][x] = 7;
        }
    }
    scene.level[2][4] = 2;
    scene.level[3][4] = 2;
    UseDetail(16, 16);
    Load(&scene);
    mnavPolygonId left = Over(2, 5, 0);
    mnavPolygonId right = Over(6, 5, 0);
    CHECK(HasCorner(left, 64, 3, 32) && HasCorner(right, 64, 3, 32) &&
              !HasCorner(left, 64, 3, 48) && !HasCorner(right, 64, 3, 48),
          "the sample nearer (4, 1)");
}

static void TestNoHeightsFallBackAndCount(void)
{
    // Found by a search of 2D floors: a floor of 17 by 17 cells from 2 m,
    // less a block from cell (4, 16), 5 cells by 4, that cuts a notch one
    // cell deep into its +Z side. Within a cell of edge error the notch is
    // simplified away, and a sample over it, 1.5 m apart, finds no floor
    // within the search: it takes the polygon's own height and is
    // counted. Within no detail error a sample off that height would be
    // kept, but the surface stays flat.
    const mnavVec2 floor[4] = {{2.0f, 2.0f}, {2.0f, 6.25f}, {6.25f, 6.25f}, {6.25f, 2.0f}};
    const mnavVec2 block[4] = {{3.0f, 6.0f}, {3.0f, 7.0f}, {4.25f, 7.0f}, {4.25f, 6.0f}};
    const mnavOutline outlines[2] = {{floor, 4, mnav_areaWalkable}, {block, 4, mnav_areaNone}};
    mnavBakeDef def = Exact();
    def.maxEdgeError = 0.25f;
    def.detailSampleDistance = 1.5f;
    def.detailMaxError = 0.0f;
    UseDef(&def);
    size_t size = 0;
    CHECK(mnavBakeTile2D(s_baker, outlines, 2, 0, 0, &s_report) == mnav_success &&
              mnavCopyBakedTile(s_baker, s_tile, TILE_ROOM, &size) == mnav_success,
          "baked");
    mnavDestroyNavmesh(s_navmesh);
    s_navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&s_def, &s_navmesh).result == mnav_success &&
              mnavStageTile(s_navmesh, s_tile, size).result == mnav_success &&
              mnavCommit(s_navmesh) == mnav_success,
          "loaded");
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
    (void)mnavDebugNavmesh(s_navmesh, 0, 0, 0, 0, &b);
    bool level = b.vertexCount > 0;
    for (int32_t i = 0; i < b.vertexCount; ++i)
    {
        level = level && s_drawVertices[i].y == s_drawVertices[0].y;
    }
    CHECK(s_report.fallbackHeights > 0 && level, "counted, the surface flat");
}

static void TestSearchTakesTheHeightNearestTheMean(void)
{
    // Found by a search of scenes: a hole at (4, 8) cuts its neighbor
    // (4, 7) as a ledge, and within a cell of edge error the polygon
    // still covers it. The sample at the corner (4, 7) has no floor of its
    // own, and the cells round it hold the floor's top, 1, and at (5, 7)
    // a step's, 2. Two cells at level 2, (1, 7) and (7, 7), raise the
    // polygon's mean height, and the search takes the height nearest it:
    // the step's, kept within no error.
    Scene scene = Flat(0, false);
    scene.level[7][1] = 2;
    scene.level[7][5] = 1;
    scene.level[7][7] = 2;
    scene.level[8][4] = NO_FLOOR;
    mnavBakeDef def = Exact();
    def.maxEdgeError = 0.25f;
    def.detailSampleDistance = 0.25f;
    def.detailMaxError = 0.0f;
    UseDef(&def);
    Load(&scene);
    CHECK(HasCorner(Over(4, 4, 0), 64, 2, 112), "the step's height at (4, 7)");
}

enum
{
    TERRAIN_QUADS = 32,
    TERRAIN_VERTICES = (TERRAIN_QUADS + 1) * (TERRAIN_QUADS + 1),
    TERRAIN_TRIANGLES = TERRAIN_QUADS * TERRAIN_QUADS * 2
};

// Rolling ground over the tile: 1 m quads, heights from 0 to 0.3 m from
// a fixed generator.
static void MakeTerrain(mnavVec3* vertices, int32_t* indices)
{
    uint32_t state = 5;
    for (int32_t z = 0; z <= TERRAIN_QUADS; ++z)
    {
        for (int32_t x = 0; x <= TERRAIN_QUADS; ++x)
        {
            float h = (float)(Next(&state) % 31u) / 100.0f;
            vertices[x + z * (TERRAIN_QUADS + 1)] = (mnavVec3){(float)x, h, (float)z};
        }
    }
    int32_t k = 0;
    for (int32_t z = 0; z < TERRAIN_QUADS; ++z)
    {
        for (int32_t x = 0; x < TERRAIN_QUADS; ++x)
        {
            int32_t a = x + z * (TERRAIN_QUADS + 1);
            int32_t b = a + TERRAIN_QUADS + 1;
            const int32_t quad[6] = {a, b, b + 1, a, b + 1, a + 1};
            for (int32_t q = 0; q < 6; ++q)
            {
                indices[k++] = quad[q];
            }
        }
    }
}

static void TestBumpyGroundReachesTheCap(void)
{
    static mnavVec3 vertices[TERRAIN_VERTICES];
    static int32_t indices[TERRAIN_TRIANGLES * 3];
    MakeTerrain(vertices, indices);
    const mnavTriangleMesh mesh = {vertices, TERRAIN_VERTICES, indices, TERRAIN_TRIANGLES, nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    // Samples every cell within no error.
    mnavBakeDef def = mnavDefaultBakeDef();
    def.detailSampleDistance = 0.25f;
    def.detailMaxError = 0.0f;
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
    CHECK(report.cappedDetail > 0 && report.partialRings == 0, "some polygons reach the cap");
}

static void TestLevelDetailIsSound(void)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    const mnavTriangleMesh mesh = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    const mnavBakeInput input = {&mesh, 1, nullptr, 0, nullptr, 0, nullptr};
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        // The default def, then samples every cell within no error.
        mnavBakeDef def = mnavDefaultBakeDef();
        if (pass == 1)
        {
            def.detailSampleDistance = 0.25f;
            def.detailMaxError = 0.0f;
        }
        mnavBakeReport report;
        mnavBaker* baker = nullptr;
        CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
                  mnavBakeTileInput(baker, &input, 0, 0, &report) == mnav_success,
              "baked");
        mnavDestroyBaker(baker);
        CHECK(report.partialRings == 0 && report.detailTriangles >= report.polygons,
              "every polygon's detail whole");
    }
}

int main(void)
{
    TestFlatSquare();
    TestBumpOnAnEdgeIsSampled();
    TestHillInsideIsSampled();
    TestSharedEdgeIsSampledAlike();
    TestTiesKeepTheFirstFromTheLexicalEnd();
    TestNoHeightsFallBackAndCount();
    TestSearchTakesTheHeightNearestTheMean();
    TestLevelDetailIsSound();
    TestBumpyGroundReachesTheCap();
    Done();
    return s_failures == 0 ? 0 : 1;
}
