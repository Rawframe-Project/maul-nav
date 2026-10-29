// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The query context's node table after a move along the surface. A move
// takes its nodes out of the table again and marks it clean, so the next
// search clears nothing. White-box: a table left dirty costs the next
// search a clear, and no result shows it; the results the next search
// gives are surface's and path's to check.

#include "query.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdint.h>
#include <stdio.h>

static mnavQuery* MakeQuery(int32_t nodes)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    def.limits.nodes = nodes;
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    return query;
}

// Sixteen ways round, about 6 m long, in exact numbers.
static const double s_fan[16][2] = {{6, 0},  {5.5, 2.3},   {4.2, 4.2},   {2.3, 5.5},
                                    {0, 6},  {-2.3, 5.5},  {-4.2, 4.2},  {-5.5, 2.3},
                                    {-6, 0}, {-5.5, -2.3}, {-4.2, -4.2}, {-2.3, -5.5},
                                    {0, -6}, {2.3, -5.5},  {4.2, -4.2},  {5.5, -2.3}};

static mnavNavmesh* WorldNavmesh(void)
{
    LoadWorldTiles();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "created");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static void TestTableLeftEmpty(void)
{
    // A move takes its nodes out of the context's node table again, so
    // that the next search has nothing to clear: in a table of 64 cells,
    // where keys often share a probe chain, every cell is empty after
    // each move that says so.
    mnavNavmesh* navmesh = WorldNavmesh();
    mnavQuery* query = MakeQuery(32);
    int32_t clean = 0;
    int32_t left = 0;
    for (int32_t i = 0; i < 64; ++i)
    {
        mnavNearest a;
        CHECK(mnavNavmesh_FindNearest(navmesh,
                                      (mnavPos3){4.0 + (i % 8) * 7.5, 0.0, 4.0 + (i / 8) * 7.5},
                                      (mnavVec3){3.0f, 2.0f, 3.0f}, nullptr, &a) == mnav_success,
              "start");
        for (int32_t k = 0; k < 16 && a.polygon.slot != 0; ++k)
        {
            mnavPos3 wanted = {a.point.x + s_fan[k][0], 0.0, a.point.z + s_fan[k][1]};
            mnavMove move = {0};
            CHECK(mnavQuery_MoveAlongSurface(query, navmesh, a.polygon, a.point, wanted, nullptr,
                                             &move) == mnav_success,
                  "moved");
            if (!query->tableClean)
            {
                continue;
            }
            clean += 1;
            for (uint32_t c = 0; c <= query->tableMask; ++c)
            {
                left += query->table[c] != MNAV_NO_NODE ? 1 : 0;
            }
        }
    }
    printf("table: %d moves left it clean, %d cells left\n", clean, left);
    CHECK(clean > 500, "most moves leave it clean");
    CHECK(left == 0, "every cell empty");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestTableLeftEmpty();
    return s_failures == 0 ? 0 : 1;
}
