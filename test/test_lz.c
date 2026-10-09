// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The LZ coder of the tile cache (mnav-0016): bytes coded and decoded
// come back the same, at lengths past each count's first byte and
// offsets at the window's end; codings cut, padded or bent are refused
// without reading or writing out of range.

#include "lz.h"
#include "test_harness.h"

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
    if (coded > mnavLzBound(size) || !mnavLzDecode(s_coded, coded, s_out, size) ||
        memcmp(s_in, s_out, size) != 0)
    {
        return 0;
    }
    return coded;
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

static void TestRefusals(void)
{
    for (size_t i = 0; i < 4096; ++i)
    {
        s_in[i] = (uint8_t)(i % 7 == 0 ? Random() : i % 13);
    }
    size_t coded = RoundTrip(4096);
    CHECK(coded > 0, "coded");
    bool cut = true;
    for (size_t size = 0; size < coded; ++size)
    {
        cut = cut && !mnavLzDecode(s_coded, size, s_out, 4096);
    }
    CHECK(cut, "every cut coding refused");
    CHECK(!mnavLzDecode(s_coded, coded, s_out, 4095) && !mnavLzDecode(s_coded, coded, s_out, 4097),
          "a coding of other length refused");
    // A match before the first byte, at offset 0, or past the end.
    const uint8_t before[] = {0x10, 'a', 0x02, 0x00};
    const uint8_t zero[] = {0x10, 'a', 0x00, 0x00, 0x00};
    const uint8_t past[] = {0x10, 'a', 0x01, 0x00, 0x00};
    CHECK(!mnavLzDecode(before, sizeof(before), s_out, 5) &&
              !mnavLzDecode(zero, sizeof(zero), s_out, 5) &&
              !mnavLzDecode(past, sizeof(past), s_out, 4),
          "matches out of range refused");
    const uint8_t fine[] = {0x10, 'a', 0x01, 0x00, 0x00};
    CHECK(mnavLzDecode(fine, sizeof(fine), s_out, 5) && memcmp(s_out, "aaaaa", 5) == 0,
          "a match overlapping what it writes");
    // Literals past the bytes, a count's bytes cut, a last token with a
    // match's length.
    const uint8_t literals[] = {0x30, 'a', 'b'};
    const uint8_t length[] = {0xF0, 255};
    const uint8_t last[] = {0x11, 'a'};
    CHECK(!mnavLzDecode(literals, sizeof(literals), s_out, 3) &&
              !mnavLzDecode(length, sizeof(length), s_out, 1000) &&
              !mnavLzDecode(last, sizeof(last), s_out, 1),
          "literals, counts and tokens cut refused");
    // A count past what the output holds stops before reading it all.
    uint8_t huge[64];
    memset(huge, 255, sizeof(huge));
    huge[0] = 0xF0;
    CHECK(!mnavLzDecode(huge, sizeof(huge), s_out, 100), "a count past the output refused");
}

int main(void)
{
    TestRoundTrips();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
