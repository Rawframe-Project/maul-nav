// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writes hand_tiles.h: the tiles the query suites stage, built by hand
// from squares (hand_tile.h) so that their polygons, corners and links lie
// where a test needs them. The suites load the bytes through the public
// loader and need no internal header.
//
// White-box: the polygons are laid by hand and encoded with the tile
// format's own writer (polymesh.h, detail.h and tile.h), since a bake
// gives only the polygons its stages make from geometry.
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
    {"s_handRowFromOrigin",
     "A row of four squares of 5 m along x from the origin, z = 0 to 5 m: rays that reach their "
     "end, stop at a wall or leave through a side.",
     0,
     -1,
     4,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 0},
      {60, 0, 80, 20, {0}, 0}}},
    {"s_handEll",
     "An L of three squares of 5 m: (0, 0) to (10, 5) and (5, 5) to (10, 10), so a way from the "
     "first square to the third turns at the corner (5, 5).",
     0,
     -1,
     3,
     {{0, 0, 20, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}}},
    {"s_handTallWest",
     "Tile 0's east side: a tall square from x = 25 to 32 m and z = 0 to 20 m, its east edge on "
     "the seam with tile 1.",
     0,
     -1,
     1,
     {{100, 0, 128, 80, {0}, 0}}},
    {"s_handShortEast",
     "Tile 1's west side: a square from x = 32 to 37 m and z = 0 to 10 m, so the link across the "
     "seam covers z = 0 to 10 m only.",
     1,
     -1,
     1,
     {{0, 0, 20, 40, {0}, 0}}},
    {"s_handTwoFloors8",
     "A low floor from x = 0 to 10 m and a high one from 10.25 to 20 m, both from z = 0 to 10 m, "
     "the high one 8 cell heights (1 m) up: the nearest point's score against the agent's step "
     "height.",
     0,
     -1,
     2,
     {{0, 0, 40, 40, {0}, 0}, {41, 0, 80, 40, {8, 8, 8, 8}, 0}}},
    {"s_handTwoFloors10",
     "A low floor from x = 0 to 10 m and a high one from 10.25 to 20 m, both from z = 0 to 10 m, "
     "the high one 10 cell heights (1.25 m) up: the nearest point's score against the agent's step "
     "height.",
     0,
     -1,
     2,
     {{0, 0, 40, 40, {0}, 0}, {41, 0, 80, 40, {10, 10, 10, 10}, 0}}},
    {"s_handTwoFloors24",
     "A low floor from x = 0 to 10 m and a high one from 10.25 to 20 m, both from z = 0 to 10 m, "
     "the high one 24 cell heights (3 m) up: the nearest point's score against the agent's step "
     "height.",
     0,
     -1,
     2,
     {{0, 0, 40, 40, {0}, 0}, {41, 0, 80, 40, {24, 24, 24, 24}, 0}}},
    {"s_handSlopedSquare",
     "One square from (0, 0) to (20, 12) whose corners stand 3, 9, 1 and 7 cell heights up: the "
     "detail surface's height between its two triangles.",
     0,
     -1,
     1,
     {{0, 0, 80, 48, {3, 9, 1, 7}, 0}}},
    {"s_handDegenerateSquare",
     "One square from (0, 0) to (10, 10), its corners 0, 4, 8 and 16 cell heights up, whose first "
     "detail triangle is the degenerate (0, 2, 2), as a tile from elsewhere may hold: the nearest "
     "point on the other triangle.",
     0,
     0,
     1,
     {{0, 0, 40, 40, {0, 4, 8, 16}, 0}}},
    {"s_handRoundAWall",
     "A U of seven squares round a wall, the bottom middle one, from (10, 0) to (15, 5), of area "
     "2: a path's area runs enter it where the path leaves the wall's corner.",
     0,
     -1,
     7,
     {{20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {60, 0, 80, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 2},
      {20, 0, 40, 20, {0}, 0}}},
    {"s_handRowWithBridge",
     "A row of three squares from (5, 5) to (20, 10), the middle one of area 2, and a fourth "
     "beyond a gap, from (25, 5) to (30, 10): corridors under filters and across a jump.",
     0,
     -1,
     4,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 2},
      {60, 20, 80, 40, {0}, 0},
      {100, 20, 120, 40, {0}, 0}}},
    {"s_handRowOfFive",
     "A row of five squares from (5, 5) to (30, 10), clear of the tile's sides: corridors whose "
     "ends move along it, and links along it.",
     0,
     -1,
     5,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {80, 20, 100, 40, {0}, 0},
      {100, 20, 120, 40, {0}, 0}}},
    {"s_handGridWithCentre",
     "A 3 by 3 grid of squares from (5, 5) to (20, 20), its centre of area 2: a corridor planned "
     "round the centre, then shortened across it.",
     0,
     -1,
     9,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 2},
      {60, 40, 80, 60, {0}, 0},
      {20, 60, 40, 80, {0}, 0},
      {40, 60, 60, 80, {0}, 0},
      {60, 60, 80, 80, {0}, 0}}},
    {"s_handApartBridge",
     "Two squares 5 m apart, from (5, 5) to (10, 10) and from (15, 5) to (20, 10), the second of "
     "area 2: paths across a link between them, and the area at the link's end.",
     0,
     -1,
     2,
     {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 2}}},
    {"s_handGrid4",
     "A 4 by 4 grid of squares from (5, 5) to (25, 25): from corner to corner many ways cost the "
     "same, so the search opens nearly every portal when its heuristic is weak.",
     0,
     -1,
     16,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {80, 20, 100, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0},
      {80, 40, 100, 60, {0}, 0},
      {20, 60, 40, 80, {0}, 0},
      {40, 60, 60, 80, {0}, 0},
      {60, 60, 80, 80, {0}, 0},
      {80, 60, 100, 80, {0}, 0},
      {20, 80, 40, 100, {0}, 0},
      {40, 80, 60, 100, {0}, 0},
      {60, 80, 80, 100, {0}, 0},
      {80, 80, 100, 100, {0}, 0}}},
    {"s_handEllMirrored",
     "The L of s_handEll mirrored, (5, 0) to (15, 5) and (5, 5) to (10, 10), so the corner (10, 5) "
     "falls on the funnel's other side.",
     0,
     -1,
     3,
     {{40, 0, 60, 20, {0}, 0}, {20, 0, 40, 20, {0}, 0}, {20, 20, 40, 40, {0}, 0}}},
    {"s_handU",
     "A U of five squares round the cells (20, 0) to (40, 20), an obstacle from (5, 0) to (10, 5): "
     "a way across it turns at two corners.",
     0,
     -1,
     5,
     {{0, 0, 20, 20, {0}, 0},
      {0, 20, 20, 40, {0}, 0},
      {20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {40, 0, 60, 20, {0}, 0}}},
    {"s_handN",
     "The U of s_handU mirrored, the obstacle above, from (5, 5) to (10, 10): the corners change "
     "sides.",
     0,
     -1,
     5,
     {{0, 20, 20, 40, {0}, 0},
      {0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 0},
      {40, 20, 60, 40, {0}, 0}}},
    {"s_handSideWest",
     "Tile 0's east side: a square from x = 25 to 32 m and z = 0 to 10 m, its east edge on the "
     "seam with tile 1.",
     0,
     -1,
     1,
     {{100, 0, 128, 40, {0}, 0}}},
    {"s_handSideEastPair",
     "Tile 1's west side: two squares across the seam, z = 0 to 10 m and 10 to 20 m, the second "
     "reaching past s_handSideWest's end, so a way across bends at its corner on the seam, (32, "
     "10).",
     1,
     -1,
     2,
     {{0, 0, 20, 40, {0}, 0}, {0, 40, 20, 80, {0}, 0}}},
    {"s_handGrid4FromOrigin",
     "A 4 by 4 grid of squares of 5 m from the origin: from corner to corner many ways cost the "
     "same, and the search's ties decide the work and the corridor.",
     0,
     -1,
     16,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 0},
      {60, 0, 80, 20, {0}, 0},
      {0, 20, 20, 40, {0}, 0},
      {20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {0, 40, 20, 60, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0},
      {0, 60, 20, 80, {0}, 0},
      {20, 60, 40, 80, {0}, 0},
      {40, 60, 60, 80, {0}, 0},
      {60, 60, 80, 80, {0}, 0}}},
    {"s_handApartSquares",
     "Two squares inside one tile, from (2.5, 2.5) to (7.5, 7.5) and from (15, 15) to (22.5, "
     "22.5), apart and touching none of its sides: a search between them ends with no path.",
     0,
     -1,
     2,
     {{10, 10, 30, 30, {0}, 0}, {60, 60, 90, 90, {0}, 0}}},
    {"s_handApartPair",
     "Two squares 5 m apart in tile (0, 0), from (5, 5) to (10, 10) and from (15, 5) to (20, 10), "
     "joined by nothing: only an off-mesh link crosses between them.",
     0,
     -1,
     2,
     {{20, 20, 40, 40, {0}, 0}, {60, 20, 80, 40, {0}, 0}}},
    {"s_handSquareEast",
     "Tile (1, 0): one square from (37, 5) to (42, 10), where a link from s_handApartPair "
     "lands once the tile is loaded.",
     1,
     -1,
     1,
     {{20, 20, 40, 40, {0}, 0}}},
    {"s_handRowMiddleArea2",
     "s_handRowOfFive with its middle square, from (15, 5) to (20, 10), of area 2: a move stopped "
     "where a filter leaves that area out.",
     0,
     -1,
     5,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 2},
      {80, 20, 100, 40, {0}, 0},
      {100, 20, 120, 40, {0}, 0}}},
    {"s_handSlopeAlongX",
     "One square from (5, 5) to (10, 10) rising along x from height 0 to 8 cell heights, 1 m: "
     "the height of a point moved up it.",
     0,
     -1,
     1,
     {{20, 20, 40, 40, {0, 0, 8, 8}, 0}}},
    {"s_handBorderWest",
     "Tile (0, 0)'s +X side: one square from (27, 0) to (32, 5), the L's first leg up to the "
     "border; s_handBorderEastPair goes on across it.",
     0,
     -1,
     1,
     {{108, 0, 128, 20, {0}, 0}}},
    {"s_handBorderEastPair",
     "Tile (1, 0)'s -X side: squares from (32, 0) to (37, 5) and north of it to (37, 10), the L's "
     "corner on the border with s_handBorderWest.",
     1,
     -1,
     2,
     {{0, 0, 20, 20, {0}, 0}, {0, 20, 20, 40, {0}, 0}}},
    {"s_handRingAboutHole",
     "Eight squares from (0, 0) to (15, 15) about a hole from (5, 5) to (10, 10): the shortest way "
     "round goes by the nearer side.",
     0,
     -1,
     8,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {0, 40, 20, 60, {0}, 0},
      {0, 20, 20, 40, {0}, 0}}},
    {"s_handFourAtVertex",
     "Four squares from (0, 0) to (10, 10) meeting at (5, 5): a start on the vertex sees all of "
     "them.",
     0,
     -1,
     4,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {20, 20, 40, 40, {0}, 0},
      {0, 20, 20, 40, {0}, 0}}},
    {"s_handGrid3",
     "Nine squares from (0, 0) to (15, 15), row by row: the diagonals pass through vertices where "
     "four squares meet and no wall.",
     0,
     -1,
     9,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {40, 0, 60, 20, {0}, 0},
      {0, 20, 20, 40, {0}, 0},
      {20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {0, 40, 20, 60, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0}}},
    {"s_handEllAndFar",
     "s_handEll and a square far off, from (25, 25) to (30, 30), where a long cheap link starts.",
     0,
     -1,
     4,
     {{0, 0, 20, 20, {0}, 0},
      {20, 0, 40, 20, {0}, 0},
      {20, 20, 40, 40, {0}, 0},
      {100, 100, 120, 120, {0}, 0}}},
    {"s_handNineAndIsland",
     "Nine squares from (5, 5) to (20, 20) and an island from (25, 5) to (30, 10): every way to "
     "the island tried, none found.",
     0,
     -1,
     10,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0},
      {20, 60, 40, 80, {0}, 0},
      {40, 60, 60, 80, {0}, 0},
      {60, 60, 80, 80, {0}, 0},
      {100, 20, 120, 40, {0}, 0}}},
    {"s_handConesWest",
     "Tile (0, 0) of a scene fuzz_shortest found, thirteen squares, some of area 2: a way at 45 "
     "degrees through vertices, with s_handConesEast.",
     0,
     -1,
     13,
     {{68, 24, 88, 44, {0}, 0},
      {108, 24, 128, 44, {0}, 0},
      {48, 44, 68, 64, {0}, 0},
      {68, 44, 88, 64, {0}, 0},
      {88, 44, 108, 64, {0}, 0},
      {108, 44, 128, 64, {0}, 0},
      {48, 64, 68, 84, {0}, 2},
      {68, 64, 88, 84, {0}, 2},
      {108, 64, 128, 84, {0}, 0},
      {48, 84, 68, 104, {0}, 2},
      {68, 84, 88, 104, {0}, 2},
      {88, 84, 108, 104, {0}, 0},
      {108, 84, 128, 104, {0}, 0}}},
    {"s_handConesEast",
     "Tile (1, 0) of the scene of s_handConesWest: four squares.",
     1,
     -1,
     4,
     {{0, 24, 20, 44, {0}, 0},
      {40, 44, 60, 64, {0}, 0},
      {60, 44, 80, 64, {0}, 0},
      {40, 64, 60, 84, {0}, 2}}},
    {"s_handFourAndIsland",
     "Four squares from (5, 5) to (15, 15) meeting at (10, 10), and an island from (25, 25) to "
     "(30, 30): every way from the vertex to the island tried.",
     0,
     -1,
     5,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {40, 40, 60, 60, {0}, 0},
      {100, 100, 120, 120, {0}, 0}}},
    {"s_handRingAndIsland",
     "Eight squares from (5, 5) to (20, 20) about a hole from (10, 10) to (15, 15), and an island "
     "from (25, 25) to (30, 30): every way round the hole to the island tried.",
     0,
     -1,
     9,
     {{20, 20, 40, 40, {0}, 0},
      {40, 20, 60, 40, {0}, 0},
      {60, 20, 80, 40, {0}, 0},
      {20, 40, 40, 60, {0}, 0},
      {60, 40, 80, 60, {0}, 0},
      {20, 60, 40, 80, {0}, 0},
      {40, 60, 60, 80, {0}, 0},
      {60, 60, 80, 80, {0}, 0},
      {100, 100, 120, 120, {0}, 0}}},
    {"s_handStepWestFlat",
     "Tile (0, 0)'s +X side: a strip from (25, 0) to (32, 2.5), level: its edge on the side at "
     "height 0 at both ends.",
     0,
     -1,
     1,
     {{100, 0, 128, 10, {0, 0, 0, 0}, 0}}},
    {"s_handStepWestRising",
     "s_handStepWestFlat with its side edge rising from 0 at z = 0 to 7 cell heights at z = 2.5 m.",
     0,
     -1,
     1,
     {{100, 0, 128, 10, {0, 0, 7, 0}, 0}}},
    {"s_handStepEastFlat",
     "Tile (1, 0)'s -X side: a strip from (32, 0) to (37, 2.5), level, facing s_handStepWest*.",
     1,
     -1,
     1,
     {{0, 0, 20, 10, {0, 0, 0, 0}, 0}}},
    {"s_handStepEastStep",
     "s_handStepEastFlat with its side edge 6 cell heights up at both ends: one agent step.",
     1,
     -1,
     1,
     {{0, 0, 20, 10, {6, 6, 0, 0}, 0}}},
    {"s_handStepEastPast",
     "s_handStepEastFlat with its side edge 7 cell heights up at both ends: past a step.",
     1,
     -1,
     1,
     {{0, 0, 20, 10, {7, 7, 0, 0}, 0}}},
    {"s_handStepEastRising",
     "s_handStepEastFlat with its side edge rising from 0 at z = 0 to 7 cell heights at z = 2.5 m.",
     1,
     -1,
     1,
     {{0, 0, 20, 10, {0, 7, 0, 0}, 0}}},
};

// path_rules' layouts (HandLayout, hand_tile.h): the layouts stored are
// those LAYOUTS names: of the first 60, the set a greedy choice found to
// fail every mutant of the funnel and the path search that all 60 fail,
// and to reach every line and branch of funnel.c and path.c that all 60
// reach. path_rules_dense builds all 60.
static const int32_t LAYOUTS[] = {0, 10, 31};

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
    for (size_t l = 0; l < sizeof(LAYOUTS) / sizeof(LAYOUTS[0]); ++l)
    {
        HandSquare squares[16];
        uint32_t mask = 0;
        int32_t k = LAYOUTS[l];
        CHECK(k >= 0 && k < HAND_LAYOUT_COUNT, "a layout of the first 60");
        int32_t count = HandLayout(k, squares, &mask);
        size_t size = HandTileBytes(bytes, 0, squares, count);
        char name[64];
        snprintf(name, sizeof(name), "s_handLayout%d", k);
        Emit("// path_rules' layout %d (HandLayout, hand_tile.h), its squares kept as\n"
             "// s_handLayout%dSquares, bit i for the square at (i %% 4, i / 4).\n",
             k, k);
        Emit("[[maybe_unused]] static const uint16_t s_handLayout%dSquares = 0x%04x;\n", k, mask);
        EmitBytes(name, bytes, size);
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
