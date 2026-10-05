// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential corpus (mnav-0011): the test world, the two-floor
// building, the terrain bowl and the plateaus with two bridges, each as
// one triangle mesh, as the scenario tests build them.

#include "diff.h"
#include "world.h"

#include <stdint.h>
#include <string.h>

enum
{
    SAMPLES = 129,
    MOST_VERTICES = SAMPLES * SAMPLES,
    MOST_TRIANGLES = (SAMPLES - 1) * (SAMPLES - 1) * 2
};

static float s_out[MOST_VERTICES * 3];
static int32_t s_tris[MOST_TRIANGLES * 3];
static int32_t s_v;
static int32_t s_t;

static void Vertex(float x, float y, float z)
{
    s_out[s_v * 3 + 0] = x;
    s_out[s_v * 3 + 1] = y;
    s_out[s_v * 3 + 2] = z;
    s_v += 1;
}

static void Triangle(int32_t a, int32_t b, int32_t c)
{
    s_tris[s_t * 3 + 0] = a;
    s_tris[s_t * 3 + 1] = b;
    s_tris[s_t * 3 + 2] = c;
    s_t += 1;
}

// The quad from (x0, z0) to (x1, z1), its height y0 along z0 and y1 along
// z1, facing up.
static void Quad(float x0, float z0, float x1, float z1, float y0, float y1)
{
    int32_t v = s_v;
    Vertex(x0, y0, z0);
    Vertex(x0, y1, z1);
    Vertex(x1, y1, z1);
    Vertex(x1, y0, z0);
    Triangle(v, v + 1, v + 2);
    Triangle(v, v + 2, v + 3);
}

static void TestWorld(void)
{
    mnavTriangleMesh world = World();
    memcpy(s_out, world.vertices, (size_t)world.vertexCount * sizeof(mnavVec3));
    memcpy(s_tris, world.indices, (size_t)world.triangleCount * 3 * sizeof(int32_t));
    s_v = world.vertexCount;
    s_t = world.triangleCount;
}

static void Building(void)
{
    Quad(-2, -2, 26, 26, 0, 0);
    Quad(0, 10, 20, 20, 3, 3);
    for (int32_t k = 1; k <= 12; ++k)
    {
        float z0 = 10.0f - (float)(13 - k) * 0.4f;
        Quad(2, z0, 4, z0 + 0.4f, 0.25f * (float)k, 0.25f * (float)k);
    }
    Quad(16, 2, 18, 10, 0, 3);
}

static void Bowl(void)
{
    for (int32_t r = 0; r < SAMPLES; ++r)
    {
        for (int32_t c = 0; c < SAMPLES; ++c)
        {
            int32_t dx = c - 64;
            int32_t dz = r - 64;
            Vertex((float)c,
                   (float)(dx * dx + dz * dz) * 0.0005f + (float)((c * 7 + r * 13) % 5) * 0.02f,
                   (float)r);
        }
    }
    for (int32_t r = 0; r + 1 < SAMPLES; ++r)
    {
        for (int32_t c = 0; c + 1 < SAMPLES; ++c)
        {
            int32_t a = r * SAMPLES + c;
            Triangle(a, a + SAMPLES, a + SAMPLES + 1);
            Triangle(a, a + SAMPLES + 1, a + 1);
        }
    }
}

static void Plateaus(void)
{
    Quad(0, 0, 12, 32, 5, 5);
    Quad(20, 0, 32, 32, 5, 5);
    Quad(12, 26, 20, 28, 5, 5);
    Quad(12, 4, 20, 6, 5, 5);
}

int32_t DiffSceneCount(void)
{
    return 4;
}

DiffScene DiffGetScene(int32_t i)
{
    static const char* const names[4] = {"world", "building", "bowl", "plateaus"};
    s_v = 0;
    s_t = 0;
    if (i == 0)
    {
        TestWorld();
    }
    else if (i == 1)
    {
        Building();
    }
    else if (i == 2)
    {
        Bowl();
    }
    else
    {
        Plateaus();
    }
    return (DiffScene){names[i], s_out, s_v, s_tris, s_t};
}
