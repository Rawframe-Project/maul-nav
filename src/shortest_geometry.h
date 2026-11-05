// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The geometry on the ground that the exact shortest-path search measures
// its intervals with.

#ifndef MAUL_NAV_SRC_SHORTEST_GEOMETRY_H
#define MAUL_NAV_SRC_SHORTEST_GEOMETRY_H

#include "maul-nav/base.h"

#include <math.h>

// Twice the signed area of triangle o, a, b on the ground: positive when
// b lies left of the way from o to a.
static inline double mnavCross(mnavPos3 o, mnavPos3 a, mnavPos3 b)
{
    return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

static inline double mnavFlat(mnavPos3 a, mnavPos3 b)
{
    double dx = a.x - b.x;
    double dz = a.z - b.z;
    return sqrt(dx * dx + dz * dz);
}

static inline mnavPos3 mnavLerp(mnavPos3 a, mnavPos3 b, double t)
{
    return (mnavPos3){a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

// The point of segment a, b nearest p on the ground.
static inline mnavPos3 mnavNearestOn(mnavPos3 p, mnavPos3 a, mnavPos3 b)
{
    double dx = b.x - a.x;
    double dz = b.z - a.z;
    double length2 = dx * dx + dz * dz;
    double t = length2 > 0.0 ? ((p.x - a.x) * dx + (p.z - a.z) * dz) / length2 : 0.0;
    return mnavLerp(a, b, t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t));
}

// The shortest distance on the ground from root r through the interval
// from a to b to point e: an e on r's side of the interval's line is
// mirrored through it first, then the way is straight when it meets the
// interval and turns at the nearer end otherwise.
static inline double mnavThrough(mnavPos3 r, mnavPos3 a, mnavPos3 b, mnavPos3 e)
{
    double side = mnavCross(a, b, r);
    double eSide = mnavCross(a, b, e);
    double dx = b.x - a.x;
    double dz = b.z - a.z;
    double length2 = dx * dx + dz * dz;
    if (length2 > 0.0 && ((side > 0.0 && eSide > 0.0) || (side < 0.0 && eSide < 0.0)))
    {
        double t = ((e.x - a.x) * dx + (e.z - a.z) * dz) / length2;
        e.x = 2.0 * (a.x + t * dx) - e.x;
        e.z = 2.0 * (a.z + t * dz) - e.z;
    }
    if (mnavCross(r, a, e) >= 0.0 && mnavCross(r, b, e) <= 0.0)
    {
        return mnavFlat(r, e);
    }
    double viaA = mnavFlat(r, a) + mnavFlat(a, e);
    double viaB = mnavFlat(r, b) + mnavFlat(b, e);
    return viaA < viaB ? viaA : viaB;
}

#endif // MAUL_NAV_SRC_SHORTEST_GEOMETRY_H
