// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// ORCA's linear programs (mnav-0006), after RVO2's linearProgram1 to 3 and
// in binary64: the velocity nearest a wanted one within a speed that
// keeps to the left of every line, or that breaks the lines least.

#ifndef MAUL_NAV_SRC_ORCA_H
#define MAUL_NAV_SRC_ORCA_H

#include "maul-nav/avoidance.h"

#include <stdbool.h>
#include <stdint.h>

// Lines whose directions' cross product is within this count as parallel.
#define MNAV_ORCA_PARALLEL 1e-9

// A half-plane of allowed velocities: those on the left of the line
// through point along direction, a unit vector.
typedef struct mnavLine
{
    mnavPos2 point;
    mnavPos2 direction;
} mnavLine;

// The 2D program: the velocity within radius on the left of every line
// nearest wanted, or with directionOpt the farthest along wanted, a unit
// vector. Returns the count when it holds, else the first line it could
// not keep, result then being the best before it.
int32_t mnavLinearProgram2(const mnavLine* lines, int32_t count, double radius, mnavPos2 wanted,
                           bool directionOpt, mnavPos2* result);

// The 3D program, after the 2D one failed at line first: keeps the first
// fixed lines and finds the velocity that breaks the others least. Needs
// scratch room for count lines.
void mnavLinearProgram3(const mnavLine* lines, int32_t count, int32_t fixed, int32_t first,
                        double radius, mnavLine* scratch, mnavPos2* result);

#endif // MAUL_NAV_SRC_ORCA_H
