// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential harness: bakes each scene with Recast and
// with Maul Nav at the same settings, then for a fixed set of point pairs
// on the navmesh compares whether each library reaches the end and how
// long its straight path is. Recast builds one untiled mesh with layer
// regions, as Maul Nav partitions; Maul Nav bakes 32 m tiles. Test only:
// neither Recast nor Detour is ever linked into the library.
//
// It also measures what each costs, reported and never gated: a scene's
// bake, a pair's path (nearest polygons, search and straight path), the
// nodes a search makes and the navmesh's bytes. Each is counted in
// user-space instructions retired where Linux's perf_event_open allows,
// steady on a loaded machine, and timed, the best of five runs.

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "Recast.h"
#include "diff.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#if defined(__linux__)
#include <linux/perf_event.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

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
// Recast's bake of a scene into Detour's navmesh data, which the caller
// frees with dtFree.
bool Bake(const DiffScene& scene, unsigned char** dataOut, int* sizeOut)
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
    *dataOut = data;
    *sizeOut = size;
    return good;
}

// Recast's bake of a scene, loaded into Detour.
bool Build(const DiffScene& scene, Detour& out)
{
    unsigned char* data = nullptr;
    int size = 0;
    if (!Bake(scene, &data, &size))
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
int DetourPath(const Detour& d, const float a[3], const float b[3], double* length, int* nodes)
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
    if (nodes != nullptr)
    {
        *nodes = d.query->getNodePool()->getNodeCount();
    }
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
        int m = DiffMaulPath(maul, a, b, &ml, nullptr);
        int r = DetourPath(d, a, b, &dl, nullptr);
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

// User-space instructions retired by this thread, where perf_event_open
// allows; otherwise unavailable, and only time is measured.
class Counter
{
  public:
    Counter()
    {
#if defined(__linux__)
        perf_event_attr attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.type = PERF_TYPE_HARDWARE;
        attr.size = sizeof(attr);
        attr.config = PERF_COUNT_HW_INSTRUCTIONS;
        attr.exclude_kernel = 1;
        attr.exclude_hv = 1;
        fd_ = static_cast<int>(syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0));
#endif
    }
    ~Counter()
    {
#if defined(__linux__)
        if (fd_ >= 0)
        {
            close(fd_);
        }
#endif
    }
    Counter(const Counter&) = delete;
    Counter& operator=(const Counter&) = delete;
    bool available() const
    {
        return fd_ >= 0;
    }
    uint64_t read() const
    {
        uint64_t count = 0;
#if defined(__linux__)
        if (fd_ >= 0 && ::read(fd_, &count, sizeof(count)) != static_cast<ssize_t>(sizeof(count)))
        {
            count = 0;
        }
#endif
        return count;
    }

  private:
    int fd_ = -1;
};

// The best of five runs of a piece of work: instructions and seconds.
struct Cost
{
    double instructions = 0.0;
    double seconds = 0.0;
};

template <typename Work> Cost Measure(const Counter& counter, Work work)
{
    Cost best{1e300, 1e300};
    for (int run = 0; run < 5; ++run)
    {
        uint64_t before = counter.read();
        auto start = std::chrono::steady_clock::now();
        work();
        std::chrono::duration<double> spent = std::chrono::steady_clock::now() - start;
        double instructions = static_cast<double>(counter.read() - before);
        best.instructions = std::fmin(best.instructions, instructions);
        best.seconds = std::fmin(best.seconds, spent.count());
    }
    return best;
}

// The pairs both libraries snap, as Compare counts them: a pair either
// side finds no polygon near is left out, so that both sides search every
// pair measured.
std::vector<std::vector<float>> Pairs(DiffMaul* maul, const Detour& d)
{
    std::vector<std::vector<float>> pairs;
    for (int i = 0; i < kPairs; ++i)
    {
        std::vector<float> p(6);
        double length = 0.0;
        if (DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 1u, p.data()) &&
            DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 2u, p.data() + 3) &&
            DiffMaulPath(maul, p.data(), p.data() + 3, &length, nullptr) >= 0 &&
            DetourPath(d, p.data(), p.data() + 3, &length, nullptr) >= 0)
        {
            pairs.push_back(p);
        }
    }
    return pairs;
}

void Row(const char* scene, const char* what, double maul, double detour, double scale,
         const char* unit)
{
    std::printf("%-10s %-12s %12.2f %12.2f %-6s %6.2f\n", scene, what, maul / scale, detour / scale,
                unit, detour > 0.0 ? maul / detour : 0.0);
}

// Measures one scene's costs on both sides and prints them.
void Costs(const Counter& counter, const DiffScene& scene, DiffMaul* maul, const Detour& d)
{
    int64_t maulBytes = 0;
    Cost maulBake = Measure(counter, [&] { maulBytes = DiffMaulBake(&scene); });
    int detourBytes = 0;
    Cost detourBake = Measure(counter,
                              [&]
                              {
                                  unsigned char* data = nullptr;
                                  if (Bake(scene, &data, &detourBytes))
                                  {
                                      dtFree(data);
                                  }
                              });
    std::vector<std::vector<float>> pairs = Pairs(maul, d);
    double n = pairs.empty() ? 1.0 : static_cast<double>(pairs.size());
    double maulNodes = 0.0;
    double detourNodes = 0.0;
    for (const std::vector<float>& p : pairs)
    {
        double length = 0.0;
        int32_t mn = 0;
        int dn = 0;
        DiffMaulPath(maul, p.data(), p.data() + 3, &length, &mn);
        DetourPath(d, p.data(), p.data() + 3, &length, &dn);
        maulNodes += mn;
        detourNodes += dn;
    }
    Cost maulPaths = Measure(counter,
                             [&]
                             {
                                 double length = 0.0;
                                 for (const std::vector<float>& p : pairs)
                                 {
                                     DiffMaulPath(maul, p.data(), p.data() + 3, &length, nullptr);
                                 }
                             });
    Cost detourPaths = Measure(counter,
                               [&]
                               {
                                   double length = 0.0;
                                   for (const std::vector<float>& p : pairs)
                                   {
                                       DetourPath(d, p.data(), p.data() + 3, &length, nullptr);
                                   }
                               });
    if (counter.available())
    {
        Row(scene.name, "bake", maulBake.instructions, detourBake.instructions, 1e6, "M ins");
    }
    Row(scene.name, "bake", maulBake.seconds, detourBake.seconds, 1e-3, "ms");
    if (counter.available())
    {
        Row(scene.name, "path", maulPaths.instructions / n, detourPaths.instructions / n, 1e3,
            "k ins");
    }
    Row(scene.name, "path", maulPaths.seconds / n, detourPaths.seconds / n, 1e-6, "us");
    Row(scene.name, "nodes", maulNodes / n, detourNodes / n, 1.0, "");
    Row(scene.name, "bytes", static_cast<double>(maulBytes), detourBytes, 1e3, "kB");
}

// The machine's load over the last minute, where Linux reports it.
double Load()
{
    double load = -1.0;
#if defined(__linux__)
    if (std::FILE* f = std::fopen("/proc/loadavg", "r"))
    {
        if (std::fscanf(f, "%lf", &load) != 1)
        {
            load = -1.0;
        }
        std::fclose(f);
    }
#endif
    return load;
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
    Counter counter;
    std::printf("\ncosts, best of five, reported and not gated; Maul Nav in 32 m tiles, Recast "
                "untiled; %s; load average %.2f\n",
                counter.available() ? "instructions are user space's, retired"
                                    : "no instruction counter here, time only",
                Load());
    std::printf("%-10s %-12s %12s %12s %-6s %6s\n", "scene", "measure", "maul nav", "detour",
                "unit", "ratio");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        if (maul != nullptr && Build(scene, d))
        {
            Costs(counter, scene, maul, d);
        }
        DiffMaulDestroy(maul);
    }
    return pass ? 0 : 1;
}
