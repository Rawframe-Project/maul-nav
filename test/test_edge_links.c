// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Edge-to-edge links (mnav-0004): a ledge 3 m high over a gap of 2 m,
// joined by one link 16 m wide; its crossings, how they attach, paths
// across it near the place they need, the link limit, removal, toggles and
// the checks.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

enum
{
    TILE_ROOM = 1 << 18
};

static uint8_t s_tile[TILE_ROOM];
static size_t s_size;

// A ledge from x 0 to 10 at 3 m, ground from x 12 to 22 at 0, both 20 m
// along z.
static void BakeLedge(void)
{
    const mnavVec3 vertices[8] = {{0, 3, 0},  {0, 3, 20},  {10, 3, 20}, {10, 3, 0},
                                  {12, 0, 0}, {12, 0, 20}, {22, 0, 20}, {22, 0, 0}};
    const int32_t indices[12] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    const mnavTriangleMesh mesh = {vertices, 8, indices, 4, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile(baker, &mesh, 1, 0, 0, nullptr) == mnav_success &&
              mnavCopyBakedTile(baker, s_tile, TILE_ROOM, &s_size) == mnav_success,
          "baked");
    mnavDestroyBaker(baker);
}

static mnavNavmesh* Load(int32_t links, mnavTier tier)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    def.limits.links = links;
    def.tier = tier;
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_tile, s_size).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    return navmesh;
}

// The link across the gap, centered at z, this wide.
static mnavLinkDef Ledge(double z, float width)
{
    return (mnavLinkDef){{9.3, 3.0, z}, {12.7, 0.0, z}, 0.5f, 1.0f, mnav_linkDrop, false, width};
}

// A path from the ledge to the ground at one z; the takeoff point's z, or
// NAN when no path crosses.
static double Across(mnavQuery* query, const mnavNavmesh* navmesh, mnavLinkId link, double z)
{
    mnavNearest a;
    mnavNearest b;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){5, 3, z}, (mnavVec3){0.5f, 1, 0.5f}, &a) ==
                  mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){17, 0, z}, (mnavVec3){0.5f, 1, 0.5f},
                              &b) == mnav_success,
          "ends");
    mnavPath path;
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    if (path.end != mnav_pathFound)
    {
        return (double)NAN;
    }
    CHECK(path.linkCount == 1 && path.links[0].link.slot == link.slot &&
              path.links[0].link.generation == link.generation &&
              path.links[0].kind == mnav_linkDrop,
          "across the link, named by its id");
    return path.points[path.links[0].point].z;
}

static void TestCrossings(mnavQuery* query)
{
    mnavNavmesh* navmesh = Load(4096, mnav_tierStatic);
    mnavLinkId link;
    mnavLinkDef def = Ledge(10.0, 16.0f);
    CHECK(mnavStageLink(navmesh, &def, &link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "staged");
    mnavLinkState state;
    CHECK(mnavGetLink(navmesh, link, &state) == mnav_success && state.attached &&
              state.crossings == 33 && fabs(state.start.z - 2.0) < 1e-9,
          "33 crossings 0.5 m apart, all attached, the first at one side");
    // Each path crosses near its own z, within half a spacing.
    const double places[4] = {3.0, 7.3, 12.6, 17.9};
    int32_t near = 0;
    for (int32_t i = 0; i < 4; ++i)
    {
        double z = Across(query, navmesh, link, places[i]);
        near += fabs(z - (places[i] < 18.0 ? places[i] : 18.0)) <= 0.25 + 1e-9 ? 1 : 0;
    }
    CHECK(near == 4, "crossed where the path needs");
    double beyond = Across(query, navmesh, link, 19.5);
    CHECK(fabs(beyond - 18.0) < 1e-9, "past the width, at its end");
    // A link wider than the ledge: the crossings off it stay detached.
    mnavLinkId wide;
    def = Ledge(10.0, 30.0f);
    CHECK(mnavStageLink(navmesh, &def, &wide) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetLink(navmesh, wide, &state) == mnav_success && state.attached &&
              state.crossings > 30 && state.crossings < 61 && state.start.z > 0.0,
          "partly attached");
    // A child slot is named by no id.
    mnavLinkId child = {link.slot + 1, link.generation};
    CHECK(mnavGetLink(navmesh, child, &state) == mnav_errorInvalid &&
              mnavStageLinkRemoval(navmesh, child) == mnav_errorInvalid,
          "a crossing's own slot is no link");
    // The widest link is crossed at 64 points.
    def = Ledge(10.0, MNAV_MAX_LINK_WIDTH);
    mnavLinkId widest;
    CHECK(mnavStageLink(navmesh, &def, &widest) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the widest");
    // Removed, all crossings go: no way across is left but the others'.
    CHECK(mnavStageLinkRemoval(navmesh, wide) == mnav_success &&
              mnavStageLinkRemoval(navmesh, widest) == mnav_success &&
              mnavStageLinkRemoval(navmesh, link) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              mnavGetLink(navmesh, link, &state) == mnav_errorStale,
          "removed");
    mnavNearest a;
    mnavNearest b;
    mnavPath path;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){5, 3, 5}, (mnavVec3){0.5f, 1, 0.5f}, &a) ==
                  mnav_success &&
              mnavFindNearest(navmesh, nullptr, (mnavPos3){17, 0, 5}, (mnavVec3){0.5f, 1, 0.5f},
                              &b) == mnav_success &&
              mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &path) == mnav_success &&
              path.end != mnav_pathFound,
          "no way across");
    mnavDestroyNavmesh(navmesh);
}

static void TestLimitsAndToggles(mnavQuery* query)
{
    // 40 links: a point link and an edge link of 33 crossings fit, a
    // second edge link does not.
    mnavNavmesh* navmesh = Load(40, mnav_tierModifiers);
    mnavLinkId point;
    mnavLinkId edge;
    mnavLinkId more;
    mnavLinkDef def = Ledge(10.0, 0.0f);
    CHECK(mnavStageLink(navmesh, &def, &point) == mnav_success, "a point link");
    def = Ledge(10.0, 16.0f);
    CHECK(mnavStageLink(navmesh, &def, &edge) == mnav_success &&
              mnavStageLink(navmesh, &def, &more) == mnav_errorLimit,
          "crossings count toward the limit");
    CHECK(mnavStageLinkRemoval(navmesh, point) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "committed without the point link");
    // Toggled off and on again, as a door.
    mnavLinkState state;
    CHECK(mnavStageLinkEnabled(navmesh, edge, false) == mnav_success &&
              mnavCommit(navmesh) == mnav_success && isnan(Across(query, navmesh, edge, 6.0)) &&
              mnavGetLink(navmesh, edge, &state) == mnav_success && !state.enabled &&
              state.crossings == 33,
          "disabled: still snapped, never crossed");
    CHECK(mnavStageLinkEnabled(navmesh, edge, true) == mnav_success &&
              mnavCommit(navmesh) == mnav_success &&
              fabs(Across(query, navmesh, edge, 6.0) - 6.0) < 1e-9,
          "enabled again");
    mnavDestroyNavmesh(navmesh);
}

static void TestChecks(void)
{
    mnavNavmesh* navmesh = Load(4096, mnav_tierStatic);
    mnavLinkId id;
    mnavLinkDef def = Ledge(10.0, -1.0f);
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_errorRange, "a negative width");
    def.width = (float)NAN;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_errorRange, "no width");
    def.width = MNAV_MAX_LINK_WIDTH * 1.01f;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_errorRange, "too wide");
    // A ladder straight up: a point link only.
    def = (mnavLinkDef){{5, 0, 5}, {5, 3, 5}, 0.5f, 1.0f, mnav_linkLadder, true, 2.0f};
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_errorInvalid, "an edge link straight up");
    def.width = 0.0f;
    CHECK(mnavStageLink(navmesh, &def, &id) == mnav_success, "a point link straight up");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    BakeLedge();
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    TestCrossings(query);
    TestLimitsAndToggles(query);
    TestChecks();
    mnavDestroyQuery(query);
    return s_failures == 0 ? 0 : 1;
}
