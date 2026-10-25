// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writes hand_tiles.h: the tiles the query suites stage, built by hand
// from squares (hand_tile.h) so that their polygons, corners and links lie
// where a test needs them. The suites load the bytes through the public
// loader and need no internal header.
//
// `test_hand_tiles --write > test/hand_tiles.h` writes the header after a
// change of the tiles below or of the tile format; `test_hand_tiles
// --check test/hand_tiles.h` writes it again and fails when the file
// differs, as the checks job runs it. The writer uses fopen, which the
// Microsoft C library deprecates.
#define _CRT_SECURE_NO_WARNINGS

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A tile: its name in the header, what it is built to exercise, its place
// along x, the square whose first detail triangle is degenerate (-1 for
// none) and its squares, in cells of 0.25 m from the tile's corner.
typedef struct HandTile
{
    const char* name;
    const char* about;
    int32_t place;
    int32_t degenerate;
    int32_t count;
    HandSquare squares[HAND_SQUARES];
} HandTile;

static const HandTile s_tiles[] = {
    {"s_handSquare",
     "One square of 10 m from the origin, its corners (0, 0), (0, 10), (10, 10), (10, 0) in that "
     "order: walls at exact distances from a point inside it.",
     0,
     -1,
     1,
     {{0, 0, 40, 40, {0}, 0}}},
    {"s_handSeamWest",
     "Tile 0's east side: a square from x = 24 to 32 m and z = 0 to 10 m, its east edge on the "
     "seam with tile 1.",
     0,
     -1,
     1,
     {{96, 0, 128, 40, {0}, 0}}},
    {"s_handSeamSouth",
     "Tile 1's west side: a square across the seam from z = 0 to 5 m only, so the seam's part "
     "from z = 5 to 10 m has nothing across it.",
     1,
     -1,
     1,
     {{0, 0, 32, 20, {0}, 0}}},
    {"s_handSeamNorth",
     "Tile 1's west side the other way: a square across the seam from z = 5 to 10 m only.",
     1,
     -1,
     1,
     {{0, 20, 32, 40, {0}, 0}}},
    {"s_handRowAndGap",
     "A row of three squares from (5, 5) to (20, 10), and a fourth beyond a gap, from (25, 5) to "
     "(30, 10): area changes and links across the gap through each tier's commit.",
     0,
     -1,
     4,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {100, 20, 120, 40, {0}, 0}}},
    {"s_handBridge",
     "A bridge of area 2 from (10, 5) to (15, 10) between two squares of area 1, and a way round "
     "it above, three squares of area 1, all clear of the tile's sides: a filter that costs or "
     "leaves out the bridge chooses between the two ways.",
     0,
     -1,
     6,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 2},
      {60, 20, 80, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0}}},
};

enum
{
    TILE_BYTES = 1 << 16,
    HEADER_BYTES = 1 << 20
};

static char s_text[HEADER_BYTES];
static size_t s_length;

static void Emit(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    int written = vsnprintf(s_text + s_length, sizeof(s_text) - s_length, format, args);
    va_end(args);
    CHECK(written >= 0 && (size_t)written < sizeof(s_text) - s_length, "the header fits");
    s_length += written > 0 ? (size_t)written : 0;
}

// A comment of the text, its lines filled to 76 columns.
static void EmitComment(const char* text)
{
    const char* at = text;
    while (*at != '\0')
    {
        size_t line = strlen(at);
        if (line > 73)
        {
            line = 73;
            while (line > 0 && at[line] != ' ')
            {
                line -= 1;
            }
        }
        Emit("// %.*s\n", (int)line, at);
        at += line;
        at += *at == ' ' ? 1 : 0;
    }
}

static void EmitBytes(const char* name, const uint8_t* bytes, size_t size)
{
    Emit("[[maybe_unused]] static const uint8_t %s[%zu] = {\n", name, size);
    for (size_t i = 0; i < size; ++i)
    {
        Emit("%s0x%02x,%s", i % 16 == 0 ? "    " : " ", bytes[i],
             i % 16 == 15 || i + 1 == size ? "\n" : "");
    }
    Emit("};\n\n");
}

static void Write(void)
{
    static uint8_t bytes[TILE_BYTES];
    Emit("// SPDX-License-Identifier: MIT\n"
         "// Copyright (c) 2026 Sirac Ozmen\n"
         "//\n"
         "// Tiles built by hand, in tile format %d, written by\n"
         "// `test_hand_tiles --write`; see test_hand_tiles.c. Cells are 0.25 m\n"
         "// and the default def's other settings hold.\n"
         "\n"
         "#ifndef MAUL_NAV_TEST_HAND_TILES_H\n"
         "#define MAUL_NAV_TEST_HAND_TILES_H\n"
         "\n"
         "#include <stdint.h>\n"
         "\n",
         MNAV_TILE_FORMAT);
    for (size_t t = 0; t < sizeof(s_tiles) / sizeof(s_tiles[0]); ++t)
    {
        const HandTile* tile = &s_tiles[t];
        size_t size =
            HandTileBytesWith(bytes, tile->place, tile->squares, tile->count, tile->degenerate);
        EmitComment(tile->about);
        EmitBytes(tile->name, bytes, size);
    }
    Emit("#endif // MAUL_NAV_TEST_HAND_TILES_H\n");
}

// Whether the file holds what Write wrote.
static bool Same(const char* path)
{
    FILE* file = fopen(path, "rb");
    if (file == nullptr)
    {
        printf("%s: cannot be read\n", path);
        return false;
    }
    static char read[HEADER_BYTES];
    size_t size = fread(read, 1, sizeof(read), file);
    fclose(file);
    bool same = size == s_length && memcmp(read, s_text, size) == 0;
    if (!same)
    {
        printf("%s differs from what test_hand_tiles writes: run\n"
               "test_hand_tiles --write > %s\n",
               path, path);
    }
    return same;
}

int main(int argc, char** argv)
{
    bool write = argc == 2 && strcmp(argv[1], "--write") == 0;
    bool check = argc == 3 && strcmp(argv[1], "--check") == 0;
    if (!write && !check)
    {
        printf("usage: test_hand_tiles --write | --check <header>\n");
        return 2;
    }
    Write();
    if (write)
    {
        fwrite(s_text, 1, s_length, stdout);
        return s_failures == 0 ? 0 : 1;
    }
    return s_failures == 0 && Same(argv[2]) ? 0 : 1;
}
