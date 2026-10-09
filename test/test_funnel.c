// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The funnel (mnav-0005) on portals written by hand: a start on a portal
// passes it over, as on a seam; a start on a portal's line beyond its
// ends does not, and the path turns at the near end; a start at a vertex
// several portals meet at passes them all over.

#include "funnel.h"
#include "query.h"
#include "test_harness.h"

#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <stdint.h>

static mnavPos3 At(double x, double z)
{
    return (mnavPos3){x, 0.0, z};
}

// Pulls a start, portals given as (left, right) pairs and an end; returns
// the points.
static int32_t PullThrough(mnavQuery* query, mnavPos3 start, const mnavPos3 (*ends)[2],
                           int32_t count, mnavPos3 end)
{
    int32_t n = 0;
    query->portals[n++] = (mnavPortal){start, start, -1};
    for (int32_t k = 0; k < count; ++k)
    {
        query->portals[n++] = (mnavPortal){ends[k][0], ends[k][1], -1};
    }
    query->portals[n++] = (mnavPortal){end, end, -1};
    int32_t links = 0;
    return mnavPullPortals(query, nullptr, n, &links);
}

static bool IsAt(mnavPos3 p, double x, double z)
{
    return p.x == x && p.z == z;
}

int main(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    // Walking south (-z), a walker's left is +x. A seam along z = 0 from
    // x = 0 to 10, the start on it and the end on it too.
    const mnavPos3 seam[1][2] = {{At(10, 0), At(0, 0)}};
    int32_t n = PullThrough(query, At(6, 0), seam, 1, At(2, 0));
    CHECK(n == 2 && IsAt(query->points[0], 6, 0) && IsAt(query->points[1], 2, 0),
          "on the seam: straight along it");
    // The start on the portal's line, short of its ends: the way south
    // passes through the portal, round its near end.
    const mnavPos3 gap[1][2] = {{At(4, 0), At(2, 0)}};
    n = PullThrough(query, At(0, 0), gap, 1, At(3, -2));
    CHECK(n == 3 && IsAt(query->points[1], 2, 0) && IsAt(query->points[2], 3, -2),
          "beside the portal's line: round its end");
    // A start at a vertex three portals meet at, fanning round it.
    const mnavPos3 fan[3][2] = {
        {At(0, 0), At(-2, -1)}, {At(0, 0), At(-1, -2)}, {At(0, 0), At(1, -2)}};
    n = PullThrough(query, At(0, 0), fan, 3, At(0.5, -3));
    CHECK(n == 2 && IsAt(query->points[1], 0.5, -3), "at a fan's vertex: straight out");
    mnavDestroyQuery(query);
    return s_failures == 0 ? 0 : 1;
}
