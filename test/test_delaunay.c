// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The detail mesh's triangulation kept Delaunay by flips (N20).

#include "delaunay.h"
#include "test_harness.h"

#include <stdint.h>

enum
{
    CAPACITY = 32
};

typedef struct Setup
{
    mnavDetailVertex vertices[CAPACITY];
    mnavDetailTriangle triangles[CAPACITY];
    int32_t stack[CAPACITY];
    uint8_t changed[CAPACITY];
    mnavTriangulation tr;
} Setup;

static void Prepare(Setup* setup, int32_t count)
{
    setup->tr = (mnavTriangulation){setup->vertices, setup->triangles, count,         CAPACITY,
                                    setup->stack,    CAPACITY,         setup->changed};
}

static bool HasTriangle(const mnavTriangulation* tr, int32_t a, int32_t b, int32_t c)
{
    for (int32_t t = 0; t < tr->count; ++t)
    {
        const uint8_t* k = tr->triangles[t].corners;
        for (int32_t r = 0; r < 3; ++r)
        {
            if (k[r] == a && k[(r + 1) % 3] == b && k[(r + 2) % 3] == c)
            {
                return true;
            }
        }
    }
    return false;
}

// Every triangle wound negatively, and no vertex strictly inside any
// triangle's circumcircle (checked in exact integers like the module).
static bool Delaunay(const mnavTriangulation* tr, int32_t vertexCount)
{
    for (int32_t t = 0; t < tr->count; ++t)
    {
        const mnavDetailVertex* a = &tr->vertices[tr->triangles[t].corners[0]];
        const mnavDetailVertex* b = &tr->vertices[tr->triangles[t].corners[1]];
        const mnavDetailVertex* c = &tr->vertices[tr->triangles[t].corners[2]];
        if (mnavDetailArea2(a, b, c) >= 0)
        {
            return false;
        }
        for (int32_t v = 0; v < vertexCount; ++v)
        {
            const mnavDetailVertex* d = &tr->vertices[v];
            int64_t adx = a->x - d->x;
            int64_t adz = a->z - d->z;
            int64_t bdx = b->x - d->x;
            int64_t bdz = b->z - d->z;
            int64_t cdx = c->x - d->x;
            int64_t cdz = c->z - d->z;
            int64_t det = (adx * adx + adz * adz) * (bdx * cdz - cdx * bdz) +
                          (bdx * bdx + bdz * bdz) * (cdx * adz - adx * cdz) +
                          (cdx * cdx + cdz * cdz) * (adx * bdz - bdx * adz);
            if (det < 0)
            {
                return false;
            }
        }
    }
    return true;
}

static void TestLongDiagonalFlips(void)
{
    // A parallelogram split along its long diagonal, 0 to 2.
    static Setup setup;
    setup.vertices[0] = (mnavDetailVertex){0, 0, 0};
    setup.vertices[1] = (mnavDetailVertex){40, 0, 100};
    setup.vertices[2] = (mnavDetailVertex){200, 0, 100};
    setup.vertices[3] = (mnavDetailVertex){160, 0, 0};
    setup.triangles[0] = (mnavDetailTriangle){{0, 1, 2}, 0};
    setup.triangles[1] = (mnavDetailTriangle){{0, 2, 3}, 0};
    Prepare(&setup, 2);
    mnavMakeDelaunay(&setup.tr);
    CHECK(setup.tr.count == 2 && Delaunay(&setup.tr, 4), "Delaunay");
    CHECK(HasTriangle(&setup.tr, 0, 1, 3) && HasTriangle(&setup.tr, 1, 2, 3),
          "split along the short diagonal");
    // Made Delaunay again, nothing moves.
    mnavMakeDelaunay(&setup.tr);
    CHECK(HasTriangle(&setup.tr, 0, 1, 3) && HasTriangle(&setup.tr, 1, 2, 3), "stable");
}

static void TestConcaveQuadNeverFlips(void)
{
    // An arrowhead: across a concave quadrilateral's diagonal the far
    // vertex lies outside the circumcircle, so the edge 0 to 2 stays.
    static Setup setup;
    setup.vertices[0] = (mnavDetailVertex){0, 0, 0};
    setup.vertices[1] = (mnavDetailVertex){0, 0, 100};
    setup.vertices[2] = (mnavDetailVertex){40, 0, 40};
    setup.vertices[3] = (mnavDetailVertex){100, 0, 0};
    setup.triangles[0] = (mnavDetailTriangle){{0, 1, 2}, 0};
    setup.triangles[1] = (mnavDetailTriangle){{0, 2, 3}, 0};
    Prepare(&setup, 2);
    mnavMakeDelaunay(&setup.tr);
    CHECK(HasTriangle(&setup.tr, 0, 1, 2) && HasTriangle(&setup.tr, 0, 2, 3), "unchanged");
}

static void TestInsertionsStayDelaunay(void)
{
    // A square split in two, then points inside and one on the diagonal.
    static Setup setup;
    setup.vertices[0] = (mnavDetailVertex){0, 0, 0};
    setup.vertices[1] = (mnavDetailVertex){0, 0, 160};
    setup.vertices[2] = (mnavDetailVertex){160, 0, 160};
    setup.vertices[3] = (mnavDetailVertex){160, 0, 0};
    setup.triangles[0] = (mnavDetailTriangle){{0, 1, 2}, 0};
    setup.triangles[1] = (mnavDetailTriangle){{0, 2, 3}, 0};
    Prepare(&setup, 2);
    const int32_t points[10] = {80, 80, 30, 120, 130, 20, 50, 40, 110, 140};
    int32_t edge = 0;
    for (int32_t k = 0; k < 5; ++k)
    {
        int32_t v = 4 + k;
        setup.vertices[v] = (mnavDetailVertex){points[2 * k], 0, points[2 * k + 1]};
        int32_t t = mnavLocate(&setup.tr, points[2 * k], points[2 * k + 1], &edge);
        CHECK(t >= 0, "located");
        if (k == 0)
        {
            CHECK(edge >= 0, "the first lies on the diagonal");
        }
        CHECK(mnavInsertVertex(&setup.tr, v, t, edge), "inserted");
        CHECK(Delaunay(&setup.tr, v + 1), "Delaunay after each insertion");
    }
    CHECK(setup.tr.count == 2 * 9 - 2 - 4, "2n - 2 - h triangles");
}

static void TestOutlineEdgeRefuses(void)
{
    static Setup setup;
    setup.vertices[0] = (mnavDetailVertex){0, 0, 0};
    setup.vertices[1] = (mnavDetailVertex){0, 0, 100};
    setup.vertices[2] = (mnavDetailVertex){100, 0, 100};
    setup.vertices[3] = (mnavDetailVertex){0, 0, 50};
    setup.triangles[0] = (mnavDetailTriangle){{0, 1, 2}, 0};
    Prepare(&setup, 1);
    int32_t edge = 0;
    CHECK(mnavLocate(&setup.tr, 0, 50, &edge) == 0 && edge == 0, "on the edge 0 to 1");
    CHECK(!mnavInsertVertex(&setup.tr, 3, 0, edge) && setup.tr.count == 1,
          "nothing across it: refused, unchanged");
    CHECK(mnavLocate(&setup.tr, 200, 50, &edge) == -1 && edge == -1, "outside");
    // Room for two more triangles is needed.
    setup.tr.capacity = 2;
    setup.vertices[3] = (mnavDetailVertex){30, 0, 60};
    CHECK(mnavLocate(&setup.tr, 30, 60, &edge) == 0 && edge == -1, "inside");
    CHECK(!mnavInsertVertex(&setup.tr, 3, 0, edge), "no room");
}

int main(void)
{
    TestLongDiagonalFlips();
    TestConcaveQuadNeverFlips();
    TestInsertionsStayDelaunay();
    TestOutlineEdgeRefuses();
    return s_failures == 0 ? 0 : 1;
}
