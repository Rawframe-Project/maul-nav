// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh link slots at their last generation. White-box: a slot's
// generation rises once for each link staged in it, so the API reaches
// the last one only after 2^32 links staged and removed; the suite sets
// it in the navmesh instead.

#include "allocator.h"
#include "navmesh.h"
#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdalign.h>
#include <stdint.h>

static void TestAWornSlotIsNotReused(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
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

// An attachment's key holds slots below MNAV_MAX_LINKS: with every slot
// below the last worn, the last takes one link, and a crossing past it
// refuses its whole link.
static void TestNoSlotPastTheLast(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    // Room for the slots to grow past the last, so that only the slot
    // refuses a link.
    def.limits.memoryBytes = 1ull << 30;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    CHECK(mnavReserve(&navmesh->memory, (void**)&navmesh->links, &navmesh->linkCapacity, 0,
                      MNAV_MAX_LINKS, sizeof(mnavOffLink), alignof(mnavOffLink)) == mnav_success,
          "room for every slot");
    for (int32_t s = 0; s < MNAV_MAX_LINKS - 1; ++s)
    {
        navmesh->links[s] = (mnavOffLink){.generation = UINT32_MAX, .phase = MNAV_LINK_FREE};
    }
    navmesh->linkSlots = MNAV_MAX_LINKS - 1;
    // Two crossings: the width is under the agent's radius.
    const mnavLinkDef edge = {
        {7.5, 0.0, 7.5}, {17.5, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.25f};
    mnavLinkId id = {0, 0};
    CHECK(mnavStageLink(navmesh, &edge, &id) == mnav_errorLimit &&
              navmesh->linkSlots == MNAV_MAX_LINKS - 1 && navmesh->linksHeld == 0,
          "an edge link whose second crossing passes the last slot refused whole");
    const mnavLinkDef point = {
        {7.5, 0.0, 7.5}, {17.5, 0.0, 7.5}, 0.5f, 4.0f, mnav_linkJump, false, 0.0f};
    CHECK(mnavStageLink(navmesh, &point, &id) == mnav_success && id.slot == MNAV_MAX_LINKS &&
              id.generation == 1,
          "a point link in the last slot");
    mnavLinkId past = {0, 0};
    CHECK(mnavStageLink(navmesh, &point, &past) == mnav_errorLimit &&
              navmesh->linkSlots == MNAV_MAX_LINKS && navmesh->linksHeld == 1,
          "no slot past the last");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestAWornSlotIsNotReused();
    TestNoSlotPastTheLast();
    return s_failures == 0 ? 0 : 1;
}
