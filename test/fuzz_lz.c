// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the tile cache's LZ coder: any bytes coded must
// decode to themselves within the bound, every assertion of the
// decoder holding.

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
    Expect(length <= mnavLzBound(size));
    mnavLzDecode(coded, length, out, size);
    Expect(memcmp(out, data, size) == 0);
    return 0;
}
