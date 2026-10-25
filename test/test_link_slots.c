// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh link slots at their last generation. White-box: a slot's
// generation rises once for each link staged in it, so the API reaches
// the last one only after 2^32 links staged and removed; the suite sets
// it in the navmesh instead.

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

int main(void)
{
    TestAWornSlotIsNotReused();
    return s_failures == 0 ? 0 : 1;
}
