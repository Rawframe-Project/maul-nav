// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The LZ coder of the tile cache: bytes coded and decoded
// come back the same, at lengths past each count's first byte, offsets
// at the window's end and matches overlapping what they write. A packed
// field unpacks to its spans.
//
// White-box: the tile cache codes its fields only for itself; no public
// call takes or gives coded bytes, so no public input reaches the coder.

#include "allocator.h"
#include "compact.h"
#include "field_pack.h"
#include "lz.h"
#include "test_harness.h"

#include "maul-nav/base.h"

#include <stdint.h>
#include <string.h>

enum
{
    ROOM = 1 << 18
};

static uint8_t s_in[ROOM];
static uint8_t s_coded[ROOM + ROOM / 255 + 16];
static uint8_t s_out[ROOM];
static uint32_t s_table[MNAV_LZ_TABLE];

// Codes and decodes size bytes of s_in; returns the coded size, or 0
// when they do not come back the same.
static size_t RoundTrip(size_t size)
{
    size_t coded = mnavLzEncode(s_in, size, s_coded, s_table);
    if (coded > mnavLzBound(size))
    {
        return 0;
    }
    mnavLzDecode(s_coded, coded, s_out, size);
    return memcmp(s_in, s_out, size) == 0 ? coded : 0;
}

static uint32_t s_state = 12345;

static uint8_t Random(void)
{
    s_state = s_state * 1664525u + 1013904223u;
    return (uint8_t)(s_state >> 24);
}

static void TestRoundTrips(void)
{
    CHECK(RoundTrip(0) == 1 && s_coded[0] == 0, "nothing: one empty token");
    s_in[0] = 7;
    CHECK(RoundTrip(1) == 2, "one byte: a token and the literal");
    for (size_t i = 0; i < ROOM; ++i)
    {
        s_in[i] = Random();
    }
    size_t noise = RoundTrip(ROOM);
    CHECK(noise > ROOM && noise <= mnavLzBound(ROOM), "noise: a little longer, within the bound");
    memset(s_in, 9, ROOM);
    size_t run = RoundTrip(ROOM);
    printf("lz: %zu bytes of noise in %zu, of one value in %zu\n", (size_t)ROOM, noise, run);
    CHECK(run > 0 && run < ROOM / 200, "a run: a match whose length runs on for many bytes");
    // Literal and match counts at 14, 15, 15 + 255 and past, each side
    // of where a count's bytes begin and continue.
    const size_t counts[] = {14, 15, 16, 269, 270, 271, 524, 525, 1000};
    bool all = true;
    for (size_t a = 0; a < sizeof(counts) / sizeof(counts[0]); ++a)
    {
        for (size_t b = 0; b < sizeof(counts) / sizeof(counts[0]); ++b)
        {
            size_t at = 0;
            for (size_t k = 0; k < counts[a]; ++k)
            {
                s_in[at++] = Random();
            }
            size_t pattern = at;
            memcpy(&s_in[at], &s_in[0], 8);
            at += 8;
            memset(&s_in[at], 0xAB, counts[b]);
            at += counts[b];
            all = all && RoundTrip(at) > 0 && pattern > 0;
        }
    }
    CHECK(all, "counts each side of 15 and 15 + 255");
    // A match 65,535 bytes back is reached; one 65,536 back is not. Zeros
    // between keep the table's other places off the pattern's.
    memset(s_in, 0, ROOM);
    for (size_t i = 0; i < 64; ++i)
    {
        s_in[i] = (uint8_t)(1 + Random() % 255);
    }
    memcpy(&s_in[65535], &s_in[0], 64);
    size_t near = RoundTrip(65535 + 64);
    memset(&s_in[64], 0, ROOM - 64);
    memcpy(&s_in[65536], &s_in[0], 64);
    size_t far = RoundTrip(65536 + 64);
    CHECK(near > 0 && far > 0 && near + 50 < far, "the window's last offset used, the next not");
}

static void TestLengths(void)
{
    for (size_t i = 0; i < 4096; ++i)
    {
        s_in[i] = (uint8_t)(i % 7 == 0 ? Random() : i % 13);
    }
    CHECK(RoundTrip(4096) > 0, "short matches among literals");
    // One run: a match a byte behind itself, its length continued past
    // its first byte.
    memset(s_in, 'a', 1000);
    size_t run = RoundTrip(1000);
    CHECK(run > 0 && run < 16, "a match overlapping what it writes");
    // Literals continued past their first byte.
    for (size_t i = 0; i < 1000; ++i)
    {
        s_in[i] = Random();
    }
    CHECK(RoundTrip(1000) > 1000, "literals only");
}

// Planes of a field of four columns and one span, its counts given.
static mnavResult Unpack(uint8_t first, uint8_t second, mnavCompactField* field, mnavMemory* memory)
{
    // Counts' low and high bytes, the floor's, the height's, the area.
    const uint8_t raw[13] = {first, second, 0, 0, 0, 0, 0, 0, 5, 0, 9, 0, 1};
    static uint8_t coded[64];
    const mnavPackedField packed = {
        {.width = 2}, 1, coded, mnavLzEncode(raw, sizeof(raw), coded, s_table)};
    return mnavUnpackField(memory, &packed, 4, 2, field);
}

static void TestUnpack(void)
{
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, 1 << 20);
    mnavCompactField field;
    CHECK(Unpack(1, 0, &field, &memory) == mnav_success && field.spanCount == 1 &&
              field.columns[1] == 1 && field.spans[0].floor == 5 && field.spans[0].height == 9 &&
              field.areas[0] == 1 && field.spans[0].links[2] == MNAV_NO_LINK,
          "a field unpacked");
    mnavReleaseCompactField(&memory, &field);
    CHECK(memory.used == 0, "nothing held once released");
}

int main(void)
{
    TestRoundTrips();
    TestLengths();
    TestUnpack();
    return s_failures == 0 ? 0 : 1;
}
