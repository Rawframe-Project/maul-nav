// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh links (N29): their attachments to polygons, made at each
// commit.

#ifndef MAUL_NAV_SRC_OFFMESH_H
#define MAUL_NAV_SRC_OFFMESH_H

#include "navmesh.h"

#include "maul-nav/base.h"

#include <stdint.h>

// An attached link seen from the polygon it leaves: the polygon's 0-based
// slot and index, the link's 0-based slot, and whether it is crossed from
// its end to its start. Packed into a key, it sorts by those in turn.
typedef struct mnavAttachment
{
    int32_t slot;
    int32_t polygon;
    int32_t link;
    bool reverse;
} mnavAttachment;

uint64_t mnavAttachmentKey(mnavAttachment attachment);
mnavAttachment mnavAttachmentOf(uint64_t key);

// The memory the next commit's attachments need, allocated before
// anything changes: room for both directions of every link it keeps, and
// as much scratch for sorting.
typedef struct mnavAttachmentPlan
{
    uint64_t* keys;
    int32_t capacity;
} mnavAttachmentPlan;

mnavResult mnavPlanAttachments(mnavNavmesh* navmesh, mnavAttachmentPlan* plan);
void mnavDropAttachmentPlan(mnavNavmesh* navmesh, mnavAttachmentPlan* plan);

// After the tiles are applied: commits the staged links and snaps every
// link's ends again into the planned memory; nothing can fail.
void mnavApplyAttachments(mnavNavmesh* navmesh, mnavAttachmentPlan* plan);

// The attachments leaving a polygon: first receives the index of the
// first in navmesh->attachments; returns how many.
int32_t mnavAttachmentsFrom(const mnavNavmesh* navmesh, int32_t slot, int32_t polygon,
                            int32_t* first);

#endif // MAUL_NAV_SRC_OFFMESH_H
