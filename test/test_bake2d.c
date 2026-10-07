// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// 2D bakes (mnav-0002): outlines filled into the 3D pipeline, and queries
// on the flat navmesh they give. Default def: 0.25 m cells, 32 m tiles,
// an agent 0.5 m in radius.

#include "test_harness.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of the room's tile bytes, the same on every platform.
#define ROOM_HASH 0x46541fd7503690f8ull

enum
{
    CAPACITY = 1 << 16
};

static uint8_t s_bytes[2][CAPACITY];
static size_t s_sizes[2];

// A room from (2, 2) to (22, 22) with a pillar from (10, 10) to (14,
// 14), a carpet of area 3 over its left half and a rug of area 5 inside
// the carpet; the rug's corner overlaps the carpet.
static const mnavVec2 s_room[4] = {{2, 2}, {22, 2}, {22, 22}, {2, 22}};
static const mnavVec2 s_pillar[4] = {{10, 10}, {14, 10}, {14, 14}, {10, 14}};
static const mnavVec2 s_carpet[4] = {{2, 2}, {8, 2}, {8, 22}, {2, 22}};
static const mnavVec2 s_rug[4] = {{3, 3}, {7, 3}, {7, 7}, {3, 7}};
// Far away, beyond and before the tile: reach no tile baked here.
static const mnavVec2 s_far[3] = {{500, 500}, {501, 500}, {500, 501}};
static const mnavVec2 s_left[3] = {{-500, 5}, {-499, 5}, {-500, 6}};

static const mnavOutline s_outlines[6] = {
    {s_pillar, 4, mnav_areaNone},  {s_room, 4, mnav_areaWalkable}, {s_rug, 4, 5}, {s_carpet, 4, 3},
    {s_far, 3, mnav_areaWalkable}, {s_left, 3, mnav_areaWalkable},
};

static mnavNavmesh* Load(int32_t tiles)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < tiles; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_bytes[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavNearest On(const mnavNavmesh* navmesh, double x, double y, float box)
{
    mnavNearest n;
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 0.0, y}, (mnavVec3){box, 1.0f, box},
                          &n) == mnav_success,
          "nearest");
    return n;
}

static void TestTheRoom(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, s_outlines, 6, 0, 0, &report) == mnav_success &&
              report.triangles == 4 && report.polygons > 0 &&
              mnavCopyBakedTile(baker, s_bytes[0], CAPACITY, &s_sizes[0]) == mnav_success,
          "baked, four outlines reaching the tile");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, s_bytes[0], (int32_t)s_sizes[0]);
    printf("ROOM_HASH=%016llx polygons=%d bytes=%zu\n", (unsigned long long)hash, report.polygons,
           s_sizes[0]);
    CHECK(hash == ROOM_HASH, "the pinned hash");
    mnavNavmesh* navmesh = Load(1);
    // The floor keeps the agent's radius from walls, less the error its
    // simplified outline may have (0.3 m), as in 3D.
    mnavNearest wall = On(navmesh, 1.0, 12.0, 3.0f);
    mnavNearest pillar = On(navmesh, 12.0, 12.0, 3.0f);
    CHECK(wall.polygon.slot != 0 && wall.point.x >= 2.5 && wall.point.x <= 2.75,
          "the room's side a radius away");
    CHECK(pillar.polygon.slot != 0 && (pillar.point.x <= 9.8 || pillar.point.x >= 14.2 ||
                                       pillar.point.z <= 9.8 || pillar.point.z >= 14.2),
          "the pillar a radius away, less the outline's error");
    CHECK(fabs(wall.point.y) <= (double)def.cellHeight, "a flat floor at height 0");
    // Areas: the highest outline holding a cell wins.
    mnavAreaType area = 0;
    CHECK(mnavGetArea(navmesh, On(navmesh, 5.0, 5.0, 0.1f).polygon, &area) == mnav_success &&
              area == 5,
          "the rug");
    CHECK(mnavGetArea(navmesh, On(navmesh, 5.0, 15.0, 0.1f).polygon, &area) == mnav_success &&
              area == 3,
          "the carpet");
    CHECK(mnavGetArea(navmesh, On(navmesh, 18.0, 5.0, 0.1f).polygon, &area) == mnav_success &&
              area == mnav_areaWalkable,
          "the bare floor");
    // Across the pillar: a straight ray stops, a path goes round.
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success, "query");
    mnavNearest a = On(navmesh, 12.0, 4.0, 0.1f);
    mnavNearest b = On(navmesh, 12.0, 20.0, 0.1f);
    mnavRay ray;
    mnavPath path;
    CHECK(mnavRaycast(query, navmesh, nullptr, a.polygon, a.point, b.point, &ray) == mnav_success &&
              ray.end == mnav_rayWall,
          "the pillar stops a ray");
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
                  mnav_success &&
              path.end == mnav_pathFound && path.pointCount >= 4 && path.length > 16.5,
          "a path round it");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    mnavDestroyBaker(baker);
}

static void TestAcrossTiles(void)
{
    // A corridor 3 m wide from x = 20 to 44 crosses from tile 0 into tile
    // 1, and a path runs along it.
    const mnavVec2 hall[4] = {{20, 10}, {44, 10}, {44, 13}, {20, 13}};
    const mnavOutline outline = {hall, 4, mnav_areaWalkable};
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    for (int32_t t = 0; t < 2; ++t)
    {
        CHECK(mnavBakeTile2D(baker, &outline, 1, t, 0, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_bytes[t], CAPACITY, &s_sizes[t]) == mnav_success,
              "baked");
    }
    mnavNavmesh* navmesh = Load(2);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    mnavNearest a = On(navmesh, 21.0, 11.5, 0.1f);
    mnavNearest b = On(navmesh, 43.0, 11.5, 0.1f);
    mnavPath path;
    CHECK(mnavCreateQuery(&queryDef, &query) == mnav_success &&
              mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point,
                           &path) == mnav_success &&
              path.end == mnav_pathFound && path.pointCount == 2 &&
              a.polygon.slot != b.polygon.slot,
          "straight along the hall, over the tile side");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    mnavDestroyBaker(baker);
}

static void TestFillRules(void)
{
    // A ring drawn round twice in one outline holds nothing by the even-
    // odd rule; the same ring once holds its cells. A thin sliver between
    // cell centers holds none.
    const mnavVec2 twice[8] = {{2, 2}, {22, 2}, {22, 22}, {2, 22},
                               {2, 2}, {22, 2}, {22, 22}, {2, 22}};
    const mnavVec2 sliver[4] = {{2, 2}, {22, 2}, {22, 2.1f}, {2, 2.1f}};
    const mnavOutline outlines[3] = {
        {twice, 8, mnav_areaWalkable}, {twice, 4, mnav_areaWalkable}, {sliver, 4, 1}};
    mnavBakeDef def = mnavDefaultBakeDef();
    def.agent.radius = 0.0f;
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, &outlines[0], 1, 0, 0, &report) == mnav_success &&
              report.polygons == 0,
          "twice round: empty");
    CHECK(mnavBakeTile2D(baker, &outlines[1], 1, 0, 0, &report) == mnav_success &&
              report.polygons > 0 && report.spans == 80 * 80,
          "once round: every cell, 20 m by 20 m");
    CHECK(mnavBakeTile2D(baker, &outlines[2], 1, 0, 0, &report) == mnav_success &&
              report.spans == 0,
          "a sliver between centers: no cell");
    // A strip whose bottom edge lies on a row of centers (they sit at
    // 0.125 m past each quarter here) holds that row: an edge end on the
    // line counts as below it.
    const mnavVec2 strip[4] = {{2, 2.125f}, {22, 2.125f}, {22, 2.3f}, {2, 2.3f}};
    const mnavOutline onLine = {strip, 4, 1};
    CHECK(mnavBakeTile2D(baker, &onLine, 1, 0, 0, &report) == mnav_success && report.spans == 80,
          "a bottom edge on the centers' line holds its row");
    // An outline past the tile on every side fills all its cells, border
    // included, and none beyond.
    const mnavVec2 huge[4] = {{-100, -100}, {100, -100}, {100, 100}, {-100, 100}};
    const mnavOutline all = {huge, 4, 1};
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success &&
              mnavBakeTile2D(baker, &all, 1, 0, 0, &report) == mnav_success &&
              report.spans ==
                  (def.tileCells + 2 * cells.border) * (def.tileCells + 2 * cells.border),
          "every cell of the tile and its border");
    CHECK(mnavBakeTile2D(baker, nullptr, 0, 0, 0, &report) == mnav_success &&
              report.polygons == 0 && report.triangles == 0,
          "nothing: an empty tile");
    mnavDestroyBaker(baker);
}

static void TestChecks(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    const mnavVec2 bad[3] = {{0, 0}, {NAN, 0}, {1, 1}};
    const mnavVec2 far[3] = {{0, 0}, {1, 0}, {1e9f, 1}};
    const mnavVec2 fine[3] = {{0, 0}, {1, 0}, {1, 1}};
    mnavOutline outline = {bad, 3, 1};
    mnavInputResult checked = mnavValidateOutline(&def, &outline);
    CHECK(checked.result == mnav_errorInvalid && checked.element == mnav_elementPoint &&
              checked.index == 1,
          "a point that is not finite");
    outline.points = far;
    checked = mnavValidateOutline(&def, &outline);
    CHECK(checked.result == mnav_errorRange && checked.index == 2, "a point past the extent");
    outline.points = fine;
    outline.pointCount = 2;
    CHECK(mnavValidateOutline(&def, &outline).result == mnav_errorInvalid, "two points");
    outline.pointCount = 3;
    outline.area = MNAV_AREA_TYPES;
    CHECK(mnavValidateOutline(&def, &outline).result == mnav_errorInvalid, "an area past range");
    outline.area = 1;
    CHECK(mnavValidateOutline(&def, &outline).result == mnav_success &&
              mnavValidateOutline(nullptr, &outline).result == mnav_errorInvalid &&
              mnavValidateOutline(&def, nullptr).result == mnav_errorInvalid,
          "fine, and NULL arguments");
    def.limits.inputTriangles = 2;
    CHECK(mnavValidateOutline(&def, &outline).result == mnav_errorLimit, "past the limit");
    def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    mnavBakeReport report;
    outline.points = bad;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, &outline, 1, 0, 0, &report) == mnav_errorInvalid &&
              report.mesh == 0 && report.input.index == 1,
          "the bake names the outline and point");
    mnavDestroyBaker(baker);
    // Four outlines reach the room's tile: past a limit of three.
    def.limits.tileTriangles = 3;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, s_outlines, 6, 0, 0, &report) == mnav_errorLimit &&
              report.stage == mnav_stageRasterize,
          "more outlines on the tile than the limit");
    def.limits.tileTriangles = 4;
    mnavDestroyBaker(baker);
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success &&
              mnavBakeTile2D(baker, s_outlines, 6, 0, 0, &report) == mnav_success,
          "as many as the limit");
    CHECK(mnavBakeTile2D(baker, nullptr, 1, 0, 0, &report) == mnav_errorInvalid &&
              mnavBakeTile2D(baker, &outline, -1, 0, 0, &report) == mnav_errorInvalid &&
              mnavBakeTile2D(nullptr, &outline, 1, 0, 0, &report) == mnav_errorInvalid,
          "bad arguments");
    mnavDestroyBaker(baker);
}

static void TestIslands(void)
{
    // A patch apart from the floor: 1.6 m wide it is under the minimum
    // region area once the radius is kept from its edges, and dropped and
    // counted; 3 m wide it is kept.
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    const mnavVec2 floor[4] = {{0, 0}, {20, 0}, {20, 20}, {0, 20}};
    const float sides[2] = {1.6f, 3.0f};
    for (int32_t k = 0; k < 2; ++k)
    {
        float s = sides[k];
        const mnavVec2 patch[4] = {{24, 24}, {24 + s, 24}, {24 + s, 24 + s}, {24, 24 + s}};
        const mnavOutline outlines[2] = {{floor, 4, mnav_areaWalkable},
                                         {patch, 4, mnav_areaWalkable}};
        mnavBakeReport report;
        CHECK(mnavBakeTile2D(baker, outlines, 2, 0, 0, &report) == mnav_success, "baked");
        CHECK(report.regions == 1 + k && report.droppedRegions == 1 - k,
              "a small island dropped and counted, a larger one kept");
    }
    mnavDestroyBaker(baker);
}

int main(void)
{
    TestIslands();
    TestTheRoom();
    TestAcrossTiles();
    TestFillRules();
    TestChecks();
    return s_failures == 0 ? 0 : 1;
}
