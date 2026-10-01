// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the tile loader (mnav-0003), the bytes a program reads from disk or a
// network and hands the library. The input's payload size and hash are
// set to match before the load, so that mutations reach the sections and
// the mesh checks instead of stopping at the seal, which the unit tests
// cover. A tile that loads must encode back to the same bytes, and every
// load, refused or not, must give back all it took.

#include "allocator.h"
#include "detail.h"
#include "polymesh.h"
#include "tile.h"

#include "maul-nav/base.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

static void PutLittle(uint8_t* at, uint64_t value, int32_t bytes)
{
    for (int32_t k = 0; k < bytes; ++k)
    {
        at[k] = (uint8_t)(value >> (8 * k));
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size > ((size_t)1 << 24))
    {
        return 0;
    }
    uint8_t* bytes = malloc(size > 0 ? size : 1);
    Expect(bytes != nullptr);
    if (size > 0)
    {
        memcpy(bytes, data, size);
    }
    if (size >= MNAV_TILE_HEADER_BYTES)
    {
        size_t payload = size - MNAV_TILE_HEADER_BYTES;
        PutLittle(bytes + 32, payload, 4);
        PutLittle(bytes + 24,
                  mnavHash64(MNAV_HASH_INIT, bytes + MNAV_TILE_HEADER_BYTES, (int32_t)payload), 8);
    }
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, (uint64_t)1 << 28);
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    mnavTileResult result = mnavDecodeTile(&memory, bytes, size, &info, &mesh, &detail);
    if (result.result == mnav_success)
    {
        uint8_t* again = nullptr;
        size_t againSize = 0;
        Expect(mnavEncodeTile(&memory, &info, &mesh, &detail, &again, &againSize) == mnav_success);
        Expect(againSize == size && memcmp(again, bytes, size) == 0);
        mnavReleaseTileBytes(&memory, again, againSize);
        mnavReleaseDetailMesh(&memory, &detail);
        mnavReleasePolyMesh(&memory, &mesh);
    }
    else
    {
        Expect(result.result == mnav_errorInvalid || result.result == mnav_errorVersion ||
               result.result == mnav_errorLimit);
    }
    Expect(memory.used == 0);
    free(bytes);
    return 0;
}
