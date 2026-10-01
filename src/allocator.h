// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation through the caller's allocator, counted against a byte
// limit (N10).

#ifndef MAUL_NAV_SRC_ALLOCATOR_H
#define MAUL_NAV_SRC_ALLOCATOR_H

#include "maul-nav/base.h"

#include <stddef.h>
#include <stdint.h>

typedef struct mnavMemory
{
    mnavAllocator allocator;
    // The most bytes held at once; reaching past it is mnav_errorLimit.
    uint64_t limit;
    // The bytes held now, and the most held so far.
    uint64_t used;
    uint64_t peak;
} mnavMemory;

// A memory counter over an allocator; a zeroed allocator means the C
// library's functions.
mnavMemory mnavMakeMemory(mnavAllocator allocator, uint64_t limit);

// Allocates count elements of size bytes each, aligned to alignment, a
// power of two. Returns mnav_errorLimit past the limit, mnav_errorCapacity
// when the size overflows or the allocator fails.
mnavResult mnavAllocate(mnavMemory* memory, size_t count, size_t size, size_t alignment,
                        void** out);

// Frees what mnavAllocate returned for the same count, size and alignment.
// NULL is ignored.
void mnavRelease(mnavMemory* memory, void* block, size_t count, size_t size, size_t alignment);

#endif // MAUL_NAV_SRC_ALLOCATOR_H
