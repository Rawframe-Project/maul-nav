// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh link slots at their last generation. White-box: a slot's
// generation rises once for each link staged in it, so the API reaches
// the last one only after 2^32 links staged and removed; the suite sets
// it in the navmesh instead.

#include "defs_for.h"
#include "navmesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdint.h>

static void TestAWornSlotIsNotReused(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "created");
    const mnavLinkDef link = {
        {7.5, 0.0, 7.5}, {17.5, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &link, &id) == mnav_success &&
              mnavStageLinkRemoval(navmesh, id) == mnav_success,
          "staged and removed, the slot free");
    mnavLinkId again = {0, 0};
    CHECK(mnavStageLink(navmesh, &link, &again) == mnav_success && again.slot == id.slot &&
              again.generation == id.generation + 1 &&
              mnavStageLinkRemoval(navmesh, again) == mnav_success,
          "the slot reused under the next generation");
    navmesh->links[id.slot - 1].generation = UINT32_MAX;
    mnavLinkId fresh = {0, 0};
    CHECK(mnavStageLink(navmesh, &link, &fresh) == mnav_success && fresh.slot != id.slot &&
              fresh.generation == 1,
          "a new slot past a worn one");
    mnavDestroyNavmesh(navmesh);
}

// An attachment's key holds slots below MNAV_MAX_LINKS. The navmesh keeps
// that limit, which the test lowers to four slots: with the three below
// the last worn, the last takes one link, and a crossing past it refuses
// its whole link.
static void TestNoSlotPastTheLast(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success &&
              navmesh->linkSlotLimit == MNAV_MAX_LINKS,
          "created with the keys' limit");
    navmesh->linkSlotLimit = 4;
    const mnavLinkDef point = {
        {7.5, 0.0, 7.5}, {17.5, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.0f};
    mnavLinkId ids[3] = {{0, 0}, {0, 0}, {0, 0}};
    for (int32_t k = 0; k < 3; ++k)
    {
        CHECK(mnavStageLink(navmesh, &point, &ids[k]) == mnav_success, "staged");
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        CHECK(mnavStageLinkRemoval(navmesh, ids[k]) == mnav_success, "removed");
        navmesh->links[ids[k].slot - 1].generation = UINT32_MAX;
    }
    CHECK(navmesh->linkSlots == 3 && navmesh->linksHeld == 0, "three slots, all worn");
    // Two crossings: the width is under the agent's radius.
    const mnavLinkDef edge = {
        {7.5, 0.0, 7.5}, {17.5, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.25f};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &edge, &id) == mnav_errorLimit && navmesh->linkSlots == 3 &&
              navmesh->linksHeld == 0,
          "an edge link whose second crossing passes the last slot refused whole");
    CHECK(mnavStageLink(navmesh, &point, &id) == mnav_success && id.slot == 4 && id.generation == 1,
          "a point link in the last slot");
    mnavLinkId past = {0, 0};
    CHECK(mnavStageLink(navmesh, &point, &past) == mnav_errorLimit && navmesh->linkSlots == 4 &&
              navmesh->linksHeld == 1,
          "no slot past the last");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestAWornSlotIsNotReused();
    TestNoSlotPastTheLast();
    return s_failures == 0 ? 0 : 1;
}
