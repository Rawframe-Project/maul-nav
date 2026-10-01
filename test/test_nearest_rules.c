// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The nearest point's score on hand-built tiles (mnav-0005): the height beyond
// the agent's step over a polygon, the distance beside one.

#include "hand_tile.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>

// A navmesh of one hand tile at (0, 0): a low floor from x = 0 to 40 and a
// high one from 41 to 80, both from z = 0 to 40, the high one high cell
// heights up.
static mnavNavmesh* TwoFloors(int32_t high)
{
    const HandSquare squares[2] = {{0, 0, 40, 40, {0, 0, 0, 0}, 0},
                                   {41, 0, 80, 40, {high, high, high, high}, 0}};
    static uint8_t bytes[2048];
    size_t size = HandTileBytes(bytes, 0, squares, 2);
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavStageTile(navmesh, bytes, size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed");
    return navmesh;
}

static uint32_t NearestPolygon(const mnavNavmesh* navmesh, double x, double y)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, y, 5.0}, (mnavVec3){4.0f, 4.0f, 4.0f},
                          &n) == mnav_success,
          "queried");
    return n.polygon.slot != 0 ? n.polygon.polygon : 99;
}

static void TestStepDecidesOverAgainstBeside(void)
{
    // Cells of 0.25 m and 0.125 m; the step is 0.75 m. The low floor ends
    // at x = 10 m, the high one begins at 10.25 m.
    mnavNavmesh* navmesh = TwoFloors(24);
    // At the high floor's height (3 m), over the low floor 0.35 m from the
    // high one's edge: the low floor scores (3 - 0.75)^2, the high one
    // 0.35^2 by distance.
    CHECK(NearestPolygon(navmesh, 9.9, 3.0) == 1, "the high floor beside beats one far below");
    // 0.8 m over the low floor: it scores (0.8 - 0.75)^2; the high floor
    // lies 2.2 m up and 0.35 m away.
    CHECK(NearestPolygon(navmesh, 9.9, 0.8) == 0, "the floor within a step below");
    mnavDestroyNavmesh(navmesh);
    // With the high floor 1 m up, 1 m over the low floor 0.65 m from the
    // high floor's edge: the low floor scores (1 - 0.75)^2 = 0.0625 and
    // wins over the high floor's 0.65^2, though its plain distance of 1 m
    // would lose.
    navmesh = TwoFloors(8);
    CHECK(NearestPolygon(navmesh, 9.6, 1.0) == 0, "the step, not the distance, scores over");
    // 1.6 m over the low floor, 0.35 m from the high one's edge and 0.6 m
    // above it: (1.6 - 0.75)^2 = 0.7225 loses to 0.35^2 + 0.6^2 = 0.4825.
    CHECK(NearestPolygon(navmesh, 9.9, 1.6) == 1, "beyond the step, the floor beside wins");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestStepDecidesOverAgainstBeside();
    return s_failures == 0 ? 0 : 1;
}
