// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// ORCA's linear programs (mnav-0006): RVO2's linearProgram1 to 3 (RVO2
// src/Agent.cc, lines 63 to 268) in binary64. The only function taken from
// libm is sqrt, which is exactly rounded.

#include "orca.h"

#include "maul-nav/avoidance.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static mnavPos2 Add(mnavPos2 a, mnavPos2 b)
{
    return (mnavPos2){a.x + b.x, a.y + b.y};
}

static mnavPos2 Sub(mnavPos2 a, mnavPos2 b)
{
    return (mnavPos2){a.x - b.x, a.y - b.y};
}

static mnavPos2 Scale(mnavPos2 a, double s)
{
    return (mnavPos2){a.x * s, a.y * s};
}

static double Dot(mnavPos2 a, mnavPos2 b)
{
    return a.x * b.x + a.y * b.y;
}

static double Det(mnavPos2 a, mnavPos2 b)
{
    return a.x * b.y - a.y * b.x;
}

static mnavPos2 Normalize(mnavPos2 a)
{
    return Scale(a, 1.0 / sqrt(Dot(a, a)));
}

// Narrows [left, right] along line n to where the lines before it allow;
// false when nothing is left.
static bool Bound(const mnavLine* lines, int32_t n, double* left, double* right)
{
    const mnavLine* line = &lines[n];
    for (int32_t i = 0; i < n; ++i)
    {
        double denominator = Det(line->direction, lines[i].direction);
        double numerator = Det(lines[i].direction, Sub(line->point, lines[i].point));
        if (fabs(denominator) <= MNAV_ORCA_PARALLEL)
        {
            if (numerator < 0.0)
            {
                return false;
            }
            continue;
        }
        double t = numerator / denominator;
        if (denominator >= 0.0)
        {
            *right = t < *right ? t : *right;
        }
        else
        {
            *left = t > *left ? t : *left;
        }
        if (*left > *right)
        {
            return false;
        }
    }
    return true;
}

// The 1D program on line n, bounded by the lines before it and the
// circle: false when they leave nothing of it.
static bool Program1(const mnavLine* lines, int32_t n, double radius, mnavPos2 wanted,
                     bool directionOpt, mnavPos2* result)
{
    const mnavLine* line = &lines[n];
    double dot = Dot(line->point, line->direction);
    double discriminant = dot * dot + radius * radius - Dot(line->point, line->point);
    if (discriminant < 0.0)
    {
        return false;
    }
    double root = sqrt(discriminant);
    double left = -dot - root;
    double right = -dot + root;
    if (!Bound(lines, n, &left, &right))
    {
        return false;
    }
    double t = 0.0;
    if (directionOpt)
    {
        t = Dot(wanted, line->direction) > 0.0 ? right : left;
    }
    else
    {
        t = Dot(line->direction, Sub(wanted, line->point));
        t = t < left ? left : (t > right ? right : t);
    }
    *result = Add(line->point, Scale(line->direction, t));
    return true;
}

int32_t mnavLinearProgram2(const mnavLine* lines, int32_t count, double radius, mnavPos2 wanted,
                           bool directionOpt, mnavPos2* result)
{
    if (directionOpt)
    {
        *result = Scale(wanted, radius);
    }
    else if (Dot(wanted, wanted) > radius * radius)
    {
        *result = Scale(Normalize(wanted), radius);
    }
    else
    {
        *result = wanted;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (Det(lines[i].direction, Sub(lines[i].point, *result)) > 0.0)
        {
            mnavPos2 before = *result;
            if (!Program1(lines, i, radius, wanted, directionOpt, result))
            {
                *result = before;
                return i;
            }
        }
    }
    return count;
}

// The line where lines i and j are equally broken, along which the 3D
// program's projection runs; false when they point the same way.
static bool Bisector(const mnavLine* i, const mnavLine* j, mnavLine* out)
{
    double determinant = Det(i->direction, j->direction);
    if (fabs(determinant) <= MNAV_ORCA_PARALLEL)
    {
        if (Dot(i->direction, j->direction) > 0.0)
        {
            return false;
        }
        out->point = Scale(Add(i->point, j->point), 0.5);
    }
    else
    {
        double t = Det(j->direction, Sub(i->point, j->point)) / determinant;
        out->point = Add(i->point, Scale(i->direction, t));
    }
    out->direction = Normalize(Sub(j->direction, i->direction));
    return true;
}

void mnavLinearProgram3(const mnavLine* lines, int32_t count, int32_t fixed, int32_t first,
                        double radius, mnavLine* scratch, mnavPos2* result)
{
    double distance = 0.0;
    for (int32_t i = first; i < count; ++i)
    {
        if (Det(lines[i].direction, Sub(lines[i].point, *result)) <= distance)
        {
            continue;
        }
        int32_t projected = 0;
        for (int32_t k = 0; k < fixed; ++k)
        {
            scratch[projected++] = lines[k];
        }
        for (int32_t j = fixed; j < i; ++j)
        {
            projected += Bisector(&lines[i], &lines[j], &scratch[projected]) ? 1 : 0;
        }
        mnavPos2 before = *result;
        mnavPos2 across = {-lines[i].direction.y, lines[i].direction.x};
        if (mnavLinearProgram2(scratch, projected, radius, across, true, result) < projected)
        {
            // In principle impossible: the result already lies in this
            // program's region. Rounding can say otherwise; keep it then.
            *result = before;
        }
        distance = Det(lines[i].direction, Sub(lines[i].point, *result));
    }
}
