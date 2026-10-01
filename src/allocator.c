// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation through the caller's allocator, counted against a byte
// limit; the one place a zeroed allocator reaches the C library.

#include "allocator.h"

#include <stdckdint.h>
#include <stddef.h>
#include <stdlib.h>

mnavMemory mnavMakeMemory(mnavAllocator allocator, uint64_t limit)
{
    return (mnavMemory){allocator, limit, 0, 0};
}

mnavResult mnavAllocate(mnavMemory* memory, size_t count, size_t size, size_t alignment, void** out)
{
    *out = nullptr;
    size_t bytes = 0;
    uint64_t total = 0;
    if (ckd_mul(&bytes, count, size) || ckd_add(&total, memory->used, (uint64_t)bytes))
    {
        return mnav_errorCapacity;
    }
    if (total > memory->limit)
    {
        return mnav_errorLimit;
    }
    if (bytes == 0)
    {
        return mnav_success;
    }
    void* block = nullptr;
    if (memory->allocator.alloc != nullptr)
    {
        block = memory->allocator.alloc(bytes, alignment, memory->allocator.context);
    }
    else if (alignment <= alignof(max_align_t))
    {
        block = malloc(bytes);
    }
    if (block == nullptr)
    {
        return mnav_errorCapacity;
    }
    memory->used = total;
    memory->peak = total > memory->peak ? total : memory->peak;
    *out = block;
    return mnav_success;
}

void mnavRelease(mnavMemory* memory, void* block, size_t count, size_t size, size_t alignment)
{
    if (block == nullptr)
    {
        return;
    }
    size_t bytes = count * size;
    memory->used -= bytes;
    if (memory->allocator.free != nullptr)
    {
        memory->allocator.free(block, bytes, alignment, memory->allocator.context);
        return;
    }
    free(block);
}
