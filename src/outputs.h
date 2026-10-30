// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What a public call leaves in its outputs. A call whose first parameter is
// an owner object clears them before it checks anything, and again when it
// fails, so that a failure never leaves an earlier call's values, or part
// of its own, for the caller to read. A call that answers "no" without
// failing writes nothing past the cleared outputs but the answer. The
// outputs a failure defines (the
// size or count a buffer too small needed, a bake's report) are left to the
// call (docs/internals/overview.md).

#ifndef MAUL_NAV_SRC_OUTPUTS_H
#define MAUL_NAV_SRC_OUTPUTS_H

#include "maul-nav/base.h"

#include <stddef.h>
#include <string.h>

// Sets an output's bytes to 0; NULL is no output.
static inline void mnavClearOutput(void* output, size_t size)
{
    if (output != nullptr)
    {
        memset(output, 0, size);
    }
}

// Clears an output when the call failed, and returns its result.
static inline mnavResult mnavClearOnFailure(mnavResult result, void* output, size_t size)
{
    if (result != mnav_success)
    {
        mnavClearOutput(output, size);
    }
    return result;
}

// Clears a count or size when the call failed for another reason than a
// buffer too small, which defines it.
static inline mnavResult mnavClearUnlessCapacity(mnavResult result, void* output, size_t size)
{
    if (result != mnav_success && result != mnav_errorCapacity)
    {
        mnavClearOutput(output, size);
    }
    return result;
}

#endif // MAUL_NAV_SRC_OUTPUTS_H
