// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A suite's dense form. A suite with sweeps or trials too slow for a unit
// test runs less of them: a subset of the dense form's (flight_query,
// hierarchy, scenarios, tile_cache), or a coarser bake of the same scene
// that reaches the same allocation stacks (alloc_failures). With the
// argument `dense` it runs them all. The dense test (`<suite>_dense`,
// labeled dense) runs on every cell but AddressSanitizer's on a pull
// request, and under AddressSanitizer at night.

#ifndef MAUL_NAV_TEST_DENSE_H
#define MAUL_NAV_TEST_DENSE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static bool s_dense = false;

// Reads a suite's arguments: none, or `dense`. Whether they are known.
static inline bool ReadDense(int argc, char** argv)
{
    s_dense = argc == 2 && strcmp(argv[1], "dense") == 0;
    if (argc > 1 && !s_dense)
    {
        printf("unknown arguments: none, or dense\n");
        return false;
    }
    return true;
}

// Whether trial k of a sweep runs: every one in the dense form, the first
// `plain` otherwise. A sweep draws its inputs for every trial either way,
// so the trials that run are the dense form's first ones.
static inline bool Runs(int32_t k, int32_t plain)
{
    return s_dense || k < plain;
}

#endif // MAUL_NAV_TEST_DENSE_H
