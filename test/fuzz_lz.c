// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the tile cache's LZ coder (mnav-0016): any bytes coded must
// decode to themselves within the bound, and any bytes decoded as a
// coding must be refused or fill exactly the length asked for, nothing
// read or written out of range.

#include "lz.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    MOST = 1 << 16
};

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    static uint8_t coded[MOST + MOST / 255 + 16];
    static uint8_t out[MOST];
    static uint32_t table[MNAV_LZ_TABLE];
    if (size == 0 || size > MOST)
    {
        return 0;
    }
    size_t length = mnavLzEncode(data, size, coded, table);
    Expect(length <= mnavLzBound(size) && mnavLzDecode(coded, length, out, size) &&
           memcmp(out, data, size) == 0);
    // The bytes as a coding, decoded to a length their first two give.
    size_t want = ((size_t)data[0] | (size_t)(size > 1 ? data[1] : 0) << 8) % MOST;
    uint8_t* exact = malloc(want > 0 ? want : 1);
    Expect(exact != nullptr);
    (void)mnavLzDecode(data, size, exact, want);
    free(exact);
    return 0;
}
