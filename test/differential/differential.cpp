// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential harness (mnav-0011): bakes each scene with Recast and
// with Maul Nav at the same settings, then for a fixed set of point pairs
// on the navmesh compares whether each library reaches the end and how
// long its straight path is. Recast builds one untiled mesh with layer
// regions, as Maul Nav partitions; Maul Nav bakes 32 m tiles. Test only:
// neither Recast nor Detour is ever linked into the library.

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "diff.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

// Prints what Recast reports going wrong.
class Context : public rcContext
{
  protected:
    void doLog(const rcLogCategory category, const char* message, const int) override
    {
        if (category != RC_LOG_PROGRESS)
        {
            std::fprintf(stderr, "recast: %s\n", message);
        }
    }
};

// The pairs per scene, and the floors the shares must reach: reachability
// agreeing; lengths within 5% or 0.25 m of each other; Maul Nav's no
// longer than that beyond Detour's; and the scene's summed lengths within
// 2% of each other.
constexpr int kPairs = 200;
constexpr double kReachFloor = 0.97;
constexpr double kLengthFloor = 0.90;
constexpr double kNotLongerFloor = 0.95;
constexpr double kSumBound = 0.02;

struct Detour
{
    dtNavMesh* mesh = nullptr;
    dtNavMeshQuery* query = nullptr;
    ~Detour()
    {
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(mesh);
    }
};

// Recast's solo build at Maul Nav's default def: cells of 0.25 by 0.125
// m, an agent 2 m tall, 0.5 m wide, stepping 0.75 m, on slopes to 45
// degrees; edges to 12 m and 0.3 m off; regions of 2 square meters;
// detail samples every 1.5 m within 0.125 m.
bool Build(const DiffScene& scene, Detour& out)
{
    rcConfig cfg{};
    cfg.cs = 0.25f;
    cfg.ch = 0.125f;
    cfg.walkableSlopeAngle = 45.0f;
    cfg.walkableHeight = 16;
    cfg.walkableClimb = 6;
    cfg.walkableRadius = 2;
    cfg.maxEdgeLen = 48;
    cfg.maxSimplificationError = 1.2f;
    cfg.minRegionArea = 32;
    cfg.mergeRegionArea = 400;
    cfg.maxVertsPerPoly = 6;
    cfg.detailSampleDist = 1.5f;
    cfg.detailSampleMaxError = 0.125f;
    rcCalcBounds(scene.vertices, scene.vertexCount, cfg.bmin, cfg.bmax);
    rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
    Context ctx;
    rcHeightfield* hf = rcAllocHeightfield();
    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    rcContourSet* cset = rcAllocContourSet();
    rcPolyMesh* pmesh = rcAllocPolyMesh();
    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    std::vector<unsigned char> areas(static_cast<size_t>(scene.triangleCount), 0);
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, scene.vertices, scene.vertexCount,
                            scene.indices, scene.triangleCount, areas.data());
    bool good =
        rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch) &&
        rcRasterizeTriangles(&ctx, scene.vertices, scene.vertexCount, scene.indices, areas.data(),
                             scene.triangleCount, *hf, cfg.walkableClimb);
    if (good)
    {
        rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
        rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
        rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);
    }
    good = good &&
           rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf) &&
           rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf) &&
           rcBuildLayerRegions(&ctx, *chf, 0, cfg.minRegionArea) &&
           rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset) &&
           rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *pmesh) &&
           rcBuildPolyMeshDetail(&ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError,
                                 *dmesh);
    unsigned char* data = nullptr;
    int size = 0;
    if (good)
    {
        for (int i = 0; i < pmesh->npolys; ++i)
        {
            pmesh->flags[i] = 1;
        }
        dtNavMeshCreateParams params{};
        params.verts = pmesh->verts;
        params.vertCount = pmesh->nverts;
        params.polys = pmesh->polys;
        params.polyAreas = pmesh->areas;
        params.polyFlags = pmesh->flags;
        params.polyCount = pmesh->npolys;
        params.nvp = pmesh->nvp;
        params.detailMeshes = dmesh->meshes;
        params.detailVerts = dmesh->verts;
        params.detailVertsCount = dmesh->nverts;
        params.detailTris = dmesh->tris;
        params.detailTriCount = dmesh->ntris;
        params.walkableHeight = 2.0f;
        params.walkableRadius = 0.5f;
        params.walkableClimb = 0.75f;
        rcVcopy(params.bmin, pmesh->bmin);
        rcVcopy(params.bmax, pmesh->bmax);
        params.cs = cfg.cs;
        params.ch = cfg.ch;
        params.buildBvTree = true;
        good = dtCreateNavMeshData(&params, &data, &size);
    }
    rcFreePolyMeshDetail(dmesh);
    rcFreePolyMesh(pmesh);
    rcFreeContourSet(cset);
    rcFreeCompactHeightfield(chf);
    rcFreeHeightField(hf);
    if (!good)
    {
        return false;
    }
    out.mesh = dtAllocNavMesh();
    out.query = dtAllocNavMeshQuery();
    if (dtStatusFailed(out.mesh->init(data, size, DT_TILE_FREE_DATA)))
    {
        dtFree(data);
        return false;
    }
    // Detour's node pool indexes with 16 bits.
    return dtStatusSucceed(out.query->init(out.mesh, 32768));
}

// Detour's answer, as DiffMaulPath gives Maul Nav's.
int DetourPath(const Detour& d, const float a[3], const float b[3], double* length)
{
    dtQueryFilter filter;
    const float extents[3] = {0.5f, 2.0f, 0.5f};
    dtPolyRef start = 0;
    dtPolyRef end = 0;
    float s[3];
    float e[3];
    d.query->findNearestPoly(a, extents, &filter, &start, s);
    d.query->findNearestPoly(b, extents, &filter, &end, e);
    if (start == 0 || end == 0)
    {
        return -1;
    }
    static dtPolyRef path[4096];
    int count = 0;
    dtStatus status = d.query->findPath(start, end, s, e, &filter, path, &count, 4096);
    if (dtStatusFailed(status) || (status & DT_PARTIAL_RESULT) != 0 || count == 0 ||
        path[count - 1] != end)
    {
        return 0;
    }
    static float points[256 * 3];
    int pointCount = 0;
    d.query->findStraightPath(s, e, path, count, points, nullptr, nullptr, &pointCount, 256);
    double sum = 0.0;
    for (int i = 0; i + 1 < pointCount; ++i)
    {
        const float* p = &points[i * 3];
        const float* q = &points[(i + 1) * 3];
        double dx = q[0] - p[0];
        double dy = q[1] - p[1];
        double dz = q[2] - p[2];
        sum += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    *length = sum;
    return 1;
}

struct Tally
{
    int snapped = 0;
    int unsnapped = 0;
    int reachAgree = 0;
    int bothReached = 0;
    int lengthAgree = 0;
    int notLonger = 0;
    double maulSum = 0.0;
    double detourSum = 0.0;
};

void Compare(DiffMaul* maul, const Detour& d, Tally& t)
{
    for (int i = 0; i < kPairs; ++i)
    {
        float a[3];
        float b[3];
        if (!DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 1u, a) ||
            !DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 2u, b))
        {
            continue;
        }
        double ml = 0.0;
        double dl = 0.0;
        int m = DiffMaulPath(maul, a, b, &ml);
        int r = DetourPath(d, a, b, &dl);
        if (m < 0 || r < 0)
        {
            t.unsnapped += 1;
            continue;
        }
        t.snapped += 1;
        t.reachAgree += m == r ? 1 : 0;
        if (m == 1 && r == 1)
        {
            t.bothReached += 1;
            double gap = std::fabs(ml - dl);
            double bound = std::fmax(0.05 * std::fmax(ml, dl), 0.25);
            t.lengthAgree += gap <= bound ? 1 : 0;
            t.notLonger += ml - dl <= bound ? 1 : 0;
            t.maulSum += ml;
            t.detourSum += dl;
        }
    }
}

} // namespace

int main()
{
    bool pass = true;
    std::printf("%-10s %6s %6s %8s %6s %8s %9s %8s\n", "scene", "pairs", "fringe", "reach", "both",
                "length", "notlonger", "sums");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        bool built = maul != nullptr && Build(scene, d);
        if (!built)
        {
            std::printf("%s: %s build failed\n", scene.name,
                        maul == nullptr ? "Maul Nav's" : "Recast's");
            DiffMaulDestroy(maul);
            return 1;
        }
        Tally t;
        Compare(maul, d, t);
        DiffMaulDestroy(maul);
        double reach = t.snapped > 0 ? static_cast<double>(t.reachAgree) / t.snapped : 0.0;
        double both = t.bothReached > 0 ? static_cast<double>(t.bothReached) : 1.0;
        double length = t.lengthAgree / both;
        double notLonger = t.notLonger / both;
        double sums = t.detourSum > 0.0 ? t.maulSum / t.detourSum - 1.0 : 0.0;
        std::printf("%-10s %6d %6d %7.1f%% %6d %7.1f%% %8.1f%% %+7.2f%%\n", scene.name, t.snapped,
                    t.unsnapped, 100.0 * reach, t.bothReached, 100.0 * length, 100.0 * notLonger,
                    100.0 * sums);
        pass = pass && t.snapped >= kPairs * 9 / 10 && reach >= kReachFloor &&
               length >= kLengthFloor && notLonger >= kNotLongerFloor &&
               std::fabs(sums) <= kSumBound;
    }
    std::printf(pass ? "agreement within the floors\n" : "agreement below a floor\n");
    return pass ? 0 : 1;
}
