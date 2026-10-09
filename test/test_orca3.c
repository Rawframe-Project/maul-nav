// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// ORCA's linear programs in space (mnav-0006): planes laid out by hand,
// parallel ones included, and random ones against a search of sampled
// velocities.

#include "orca3.h"
#include "test_harness.h"

#include "maul-nav/base.h"

#include <math.h>
#include <stdint.h>

enum
{
    MAX_PLANES = 8,
    SAMPLES = 20000
};

static uint64_t s_state = 0x9E3779B97F4A7C15ull;

// A draw in [0, 1), splitmix64's.
static double Random(void)
{
    s_state += 0x9E3779B97F4A7C15ull;
    uint64_t z = s_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return (double)(z >> 11) * 0x1p-53;
}

static double Between(double low, double high)
{
    return low + (high - low) * Random();
}

static mnavPlane Plane(double px, double py, double pz, double nx, double ny, double nz)
{
    return (mnavPlane){{px, py, pz}, {nx, ny, nz}};
}

static void TestFacingPlanes(void)
{
    // y at least 1 and y at most 0: nothing allowed; the 3D program fails
    // at the second plane, and the 4D one splits the difference.
    const mnavPlane planes[2] = {Plane(0, 1, 0, 0, 1, 0), Plane(0, 0, 0, 0, -1, 0)};
    mnavPlane scratch[2];
    mnavPos3 result = {0.0, 0.0, 0.0};
    int32_t failed = mnavPlaneProgram3(planes, 2, 5.0, (mnavPos3){0.0, 0.5, 0.0}, false, &result);
    CHECK(failed == 1 && result.y == 1.0, "facing planes leave nothing");
    mnavPlaneProgram4(planes, 2, 0, failed, 5.0, scratch, &result);
    CHECK(result.y == 0.5, "halfway between them");
}

static void TestSameWayPlanes(void)
{
    // y at least 1 and y at least 2: the second covers the first.
    const mnavPlane planes[2] = {Plane(0, 1, 0, 0, 1, 0), Plane(0, 2, 0, 0, 1, 0)};
    mnavPos3 result = {0.0, 0.0, 0.0};
    CHECK(mnavPlaneProgram3(planes, 2, 5.0, (mnavPos3){1.0, 0.0, -1.0}, false, &result) == 2 &&
              result.x == 1.0 && result.y == 2.0 && result.z == -1.0,
          "the nearest point of the stricter plane");
}

static void TestCorners(void)
{
    // x at least 1, then y at least 1, then z at least 1: the nearest
    // velocity to 0 moves onto a plane, a line and a point.
    const mnavPlane planes[3] = {Plane(1, 0, 0, 1, 0, 0), Plane(0, 1, 0, 0, 1, 0),
                                 Plane(0, 0, 1, 0, 0, 1)};
    mnavPos3 result = {0.0, 0.0, 0.0};
    mnavPos3 zero = {0.0, 0.0, 0.0};
    CHECK(mnavPlaneProgram3(planes, 1, 5.0, zero, false, &result) == 1 && result.x == 1.0 &&
              result.y == 0.0 && result.z == 0.0,
          "onto a plane");
    CHECK(mnavPlaneProgram3(planes, 2, 5.0, zero, false, &result) == 2 && result.x == 1.0 &&
              result.y == 1.0 && result.z == 0.0,
          "onto a line");
    CHECK(mnavPlaneProgram3(planes, 3, 5.0, zero, false, &result) == 3 && result.x == 1.0 &&
              result.y == 1.0 && result.z == 1.0,
          "onto a point");
}

static void TestPastTheSpeed(void)
{
    // x at least 3 within speed 2: the plane misses the sphere, and the
    // 4D program goes as far toward it as the speed allows.
    const mnavPlane planes[1] = {Plane(3, 0, 0, 1, 0, 0)};
    mnavPlane scratch[1];
    mnavPos3 result = {0.0, 0.0, 0.0};
    int32_t failed = mnavPlaneProgram3(planes, 1, 2.0, (mnavPos3){0.0, 1.0, 0.0}, false, &result);
    CHECK(failed == 0, "out of reach");
    mnavPlaneProgram4(planes, 1, 0, failed, 2.0, scratch, &result);
    CHECK(result.x == 2.0 && result.y == 0.0 && result.z == 0.0, "as far as the speed goes");
}

static void TestWantedPastTheSpeed(void)
{
    // No planes: the wanted velocity cut to the speed.
    mnavPos3 result = {0.0, 0.0, 0.0};
    CHECK(mnavPlaneProgram3(nullptr, 0, 2.0, (mnavPos3){0.0, 0.0, -8.0}, false, &result) == 0 &&
              result.x == 0.0 && result.y == 0.0 && result.z == -2.0,
          "cut to the speed");
}

static double Violation(const mnavPlane* planes, int32_t count, mnavPos3 v)
{
    double worst = -(double)INFINITY;
    for (int32_t i = 0; i < count; ++i)
    {
        double d = mnavDot3(planes[i].normal, mnavSub3(planes[i].point, v));
        worst = d > worst ? d : worst;
    }
    return worst;
}

static double DistanceSq(mnavPos3 a, mnavPos3 b)
{
    mnavPos3 d = mnavSub3(a, b);
    return mnavDot3(d, d);
}

static mnavPos3 InBall(double radius)
{
    for (;;)
    {
        double x = Between(-1.0, 1.0);
        double y = Between(-1.0, 1.0);
        double z = Between(-1.0, 1.0);
        if (x * x + y * y + z * z <= 1.0)
        {
            return (mnavPos3){x * radius, y * radius, z * radius};
        }
    }
}

// Random planes: the 3D program's result keeps every plane and no
// sampled velocity that does is nearer; when it fails, no sampled
// velocity breaks the planes less than the 4D program's.
static void TestAgainstSamples(void)
{
    const double radius = 2.0;
    int32_t failures = 0;
    int32_t solved = 0;
    for (int32_t round = 0; round < 200; ++round)
    {
        int32_t count = 1 + (int32_t)(Random() * MAX_PLANES);
        mnavPlane planes[MAX_PLANES];
        mnavPlane scratch[MAX_PLANES];
        for (int32_t i = 0; i < count; ++i)
        {
            mnavPos3 normal = mnavNormalize3(InBall(1.0));
            planes[i] = (mnavPlane){InBall(radius * 1.2), normal};
        }
        mnavPos3 wanted = InBall(radius * 1.5);
        mnavPos3 result = {0.0, 0.0, 0.0};
        int32_t failed = mnavPlaneProgram3(planes, count, radius, wanted, false, &result);
        if (failed == count)
        {
            solved += 1;
            bool kept = Violation(planes, count, result) <= 1e-9 &&
                        mnavDot3(result, result) <= radius * radius * (1.0 + 1e-12);
            double best = DistanceSq(result, wanted);
            for (int32_t s = 0; s < SAMPLES && kept; ++s)
            {
                mnavPos3 v = InBall(radius);
                kept = Violation(planes, count, v) > 0.0 || DistanceSq(v, wanted) >= best - 1e-9;
            }
            failures += kept ? 0 : 1;
            continue;
        }
        mnavPlaneProgram4(planes, count, 0, failed, radius, scratch, &result);
        double least = Violation(planes, count, result);
        bool kept = mnavDot3(result, result) <= radius * radius * (1.0 + 1e-12);
        for (int32_t s = 0; s < SAMPLES && kept; ++s)
        {
            kept = Violation(planes, count, InBall(radius)) >= least - 1e-9;
        }
        failures += kept ? 0 : 1;
    }
    printf("sampled: %d solved of 200, %d failures\n", solved, failures);
    CHECK(failures == 0, "no sampled velocity does better");
    CHECK(solved > 20 && solved < 180, "both programs ran");
}

static void TestHeadOnPair(void)
{
    // Straight at each other: every side is as near; each takes the
    // other's opposite.
    mnavPos3 a = {0.0, 0.0, 0.0};
    mnavPos3 b = {10.0, 0.0, 0.0};
    mnavPlane pa = mnavPairPlane(a, (mnavPos3){1.0, 0.0, 0.0}, b, (mnavPos3){-1.0, 0.0, 0.0}, 1.0,
                                 0.5, 10.0, 0.1, true);
    mnavPlane pb = mnavPairPlane(b, (mnavPos3){-1.0, 0.0, 0.0}, a, (mnavPos3){1.0, 0.0, 0.0}, 1.0,
                                 0.5, 10.0, 0.1, false);
    CHECK(isfinite(pa.normal.x) && isfinite(pa.normal.y) && isfinite(pa.normal.z),
          "a finite normal");
    CHECK(pa.normal.x == -pb.normal.x && pa.normal.y == -pb.normal.y && pa.normal.z == -pb.normal.z,
          "opposite sides");
    CHECK(fabs(pa.normal.x) < 1e-12, "square to the line between them");
    // Vertical: across X instead of up.
    mnavPlane pv = mnavPairPlane(a, (mnavPos3){0.0, 1.0, 0.0}, (mnavPos3){0.0, 10.0, 0.0},
                                 (mnavPos3){0.0, -1.0, 0.0}, 1.0, 0.5, 10.0, 0.1, true);
    CHECK(isfinite(pv.normal.x) && fabs(pv.normal.y) < 1e-12 &&
              fabs(mnavDot3(pv.normal, pv.normal) - 1.0) < 1e-12,
          "a vertical pair turns aside");
}

static void TestOnOneSpot(void)
{
    // The same place and velocity: the lower id one way along X, the
    // other the opposite.
    mnavPos3 p = {1.0, 2.0, 3.0};
    mnavPos3 v = {0.0, 0.0, 0.0};
    mnavPlane low = mnavPairPlane(p, v, p, v, 1.0, 0.5, 2.0, 0.1, true);
    mnavPlane high = mnavPairPlane(p, v, p, v, 1.0, 0.5, 2.0, 0.1, false);
    CHECK(low.normal.x == -1.0 && high.normal.x == 1.0, "opposite ways along X");
    // Bound for the other's center within the step: away from it.
    mnavPlane away = mnavPairPlane(p, (mnavPos3){5.0, 0.0, 0.0}, (mnavPos3){1.5, 2.0, 3.0}, v, 1.0,
                                   0.5, 2.0, 0.1, true);
    CHECK(away.normal.x == -1.0 && away.normal.y == 0.0 && away.normal.z == 0.0, "away from it");
}

int main(void)
{
    TestFacingPlanes();
    TestSameWayPlanes();
    TestCorners();
    TestPastTheSpeed();
    TestWantedPastTheSpeed();
    TestAgainstSamples();
    TestHeadOnPair();
    TestOnOneSpot();
    return s_failures == 0 ? 0 : 1;
}
