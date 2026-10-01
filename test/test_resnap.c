// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Off-mesh links (mnav-0004) snapped again only near the places a commit
// changes, yet always as if snapped from scratch.

#include "navmesh.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>
#include <string.h>

// Whether a link's committed state is what snapping it now would give.
static bool SnappedNow(const mnavNavmesh* navmesh, const mnavOffLink* link)
{
    float step = (float)((double)navmesh->cells.agentStep * (double)navmesh->def.cellHeight);
    mnavPos3 ends[2] = {link->def.start, link->def.end};
    mnavPolygonId polygons[2] = {{0}, {0}};
    mnavPos3 points[2] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    bool attached = true;
    for (int32_t k = 0; k < 2 && attached; ++k)
    {
        mnavNearest n;
        float r = link->def.radius;
        attached = mnavFindNearest(navmesh, nullptr, ends[k], (mnavVec3){r, step, r}, &n) ==
                       mnav_success &&
                   n.polygon.slot != 0;
        double dx = n.point.x - ends[k].x;
        double dz = n.point.z - ends[k].z;
        attached = attached && dx * dx + dz * dz <= (double)r * (double)r;
        polygons[k] = n.polygon;
        points[k] = n.point;
    }
    const mnavLinkState* s = &link->state;
    if (!attached)
    {
        return !s->attached;
    }
    return s->attached && memcmp(&s->startPolygon, &polygons[0], sizeof(mnavPolygonId)) == 0 &&
           memcmp(&s->endPolygon, &polygons[1], sizeof(mnavPolygonId)) == 0 &&
           memcmp(&s->start, &points[0], sizeof(mnavPos3)) == 0 &&
           memcmp(&s->end, &points[1], sizeof(mnavPos3)) == 0;
}

static void TestOnlyChangedPlacesResnap(void)
{
    // Tiles stream in, out and are replaced, areas change and links
    // toggle at random: after every commit, every link's snaps are what
    // snapping it from scratch gives, though only links near a changed
    // place were snapped again.
    BakeWorld();
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tier = mnav_tierDynamic;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    uint32_t state = 21;
    mnavLinkId ids[120];
    for (int32_t l = 0; l < 120; ++l)
    {
        double p[4];
        for (int32_t k = 0; k < 4; ++k)
        {
            state = state * 1664525u + 1013904223u;
            p[k] = (double)(state >> 8 & 0xFFFFu) / 65536.0 * 64.0;
        }
        float radius = 0.25f + (float)(l % 12);
        mnavLinkDef def2 = {{p[0], 0.0, p[1]}, {p[2], 0.0, p[3]}, radius, 2.0f,
                            mnav_linkJump,     (l & 1) != 0};
        CHECK(mnavStageLink(navmesh, &def2, &ids[l]) == mnav_success, "staged");
    }
    bool loaded[4] = {false, false, false, false};
    bool matched = true;
    int32_t attachedSeen = 0;
    for (int32_t step = 0; step < 80; ++step)
    {
        state = state * 1664525u + 1013904223u;
        int32_t t = (int32_t)(state >> 8 & 3u);
        int32_t what = (int32_t)(state >> 12 & 7u);
        mnavTileId tile;
        if (what < 3)
        {
            CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "tile in");
            loaded[t] = true;
        }
        else if (what < 5 && loaded[t])
        {
            CHECK(mnavStageTileRemoval(navmesh, t & 1, t >> 1) == mnav_success, "tile out");
            loaded[t] = false;
        }
        else if (what < 7 && mnavGetTile(navmesh, t & 1, t >> 1, &tile) == mnav_success)
        {
            uint32_t count = (uint32_t)navmesh->slots[tile.slot - 1].tile->mesh.polygonCount;
            mnavPolygonId polygon = {tile.slot, tile.generation, (state >> 16) % count};
            mnavAreaType area = (state >> 20 & 1u) != 0 ? mnav_areaNone : mnav_areaWalkable;
            CHECK(mnavStageArea(navmesh, polygon, area) == mnav_success, "area");
        }
        else
        {
            CHECK(mnavStageLinkEnabled(navmesh, ids[state >> 16 & 63u], (state >> 22 & 1u) != 0) ==
                      mnav_success,
                  "toggled");
        }
        CHECK(mnavCommit(navmesh) == mnav_success, "committed");
        for (int32_t s = 0; s < navmesh->linkSlots; ++s)
        {
            const mnavOffLink* link = &navmesh->links[s];
            matched = matched && SnappedNow(navmesh, link) && link->state.enabled == link->enabled;
            attachedSeen += link->state.attached ? 1 : 0;
        }
    }
    CHECK(matched, "every link as if snapped from scratch");
    CHECK(attachedSeen > 2000, "most links attached most of the time");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestOnlyChangedPlacesResnap();
    return s_failures == 0 ? 0 : 1;
}
