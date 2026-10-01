// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An allocator for tests that counts the bytes it holds, so a test can see
// an object give every byte back. It aligns by hand on malloc, since not
// every C library has aligned_alloc.

#ifndef MAUL_NAV_TEST_COUNTING_ALLOCATOR_H
#define MAUL_NAV_TEST_COUNTING_ALLOCATOR_H

#include "maul-nav/base.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

static size_t s_held;

// Allocates size bytes at an alignment, a power of two; the block malloc
// returned is kept just before the aligned one.
static inline void* CountingAlloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    unsigned char* base = malloc(size + alignment + sizeof(void*));
    if (base == nullptr)
    {
        return nullptr;
    }
    uintptr_t start = (uintptr_t)(base + sizeof(void*));
    unsigned char* block = base + sizeof(void*) + ((alignment - start % alignment) % alignment);
    ((void**)block)[-1] = base;
    s_held += size;
    return block;
}

static inline void CountingFree(void* block, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held -= size;
    free(((void**)block)[-1]);
}

static inline mnavAllocator CountingAllocator(void)
{
    return (mnavAllocator){CountingAlloc, CountingFree, nullptr};
}

#endif // MAUL_NAV_TEST_COUNTING_ALLOCATOR_H
