// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The differential harness: bakes each scene with Recast and
// with Maul Nav at the same settings, then for a fixed set of point pairs
// on the navmesh compares whether each library reaches the end and how
// long its straight path is, and whether a ray from one point toward the
// other stops at a wall, and where. Recast builds one untiled mesh with layer
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

#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

// GitHub's step summary when the run has one (GITHUB_STEP_SUMMARY, which
// a job's container sees too), so that the report shows on the job's page
// though the test passes and its log keeps no output; NULL otherwise.
std::FILE* s_summary = nullptr;

// Prints the report's text, and appends it to the step summary.
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
void Say(const char* format, ...)
{
    std::va_list args;
    va_start(args, format);
    if (s_summary != nullptr)
    {
        std::va_list again;
        va_copy(again, args);
        std::vfprintf(s_summary, format, again);
        va_end(again);
    }
    std::vprintf(format, args);
    va_end(args);
}

// Opens the step summary, the report in a block of fixed width.
void OpenSummary()
{
    const char* path = std::getenv("GITHUB_STEP_SUMMARY");
    s_summary = path != nullptr && path[0] != '\0' ? std::fopen(path, "a") : nullptr;
    if (s_summary != nullptr)
    {
        std::fprintf(s_summary,
                     "### The differential harness against Recast and Detour\n\n```text\n");
    }
}

void CloseSummary()
{
    if (s_summary != nullptr)
    {
        std::fprintf(s_summary, "```\n");
        std::fclose(s_summary);
        s_summary = nullptr;
    }
}

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

// Detour's ray, as DiffMaulRaycast gives Maul Nav's.
int DetourRaycast(const Detour& d, const float a[3], const float b[3], float start[3], double* t,
                  float normal[3])
{
    dtQueryFilter filter;
    const float extents[3] = {0.5f, 2.0f, 0.5f};
    dtPolyRef ref = 0;
    d.query->findNearestPoly(a, extents, &filter, &ref, start);
    if (ref == 0)
    {
        return -1;
    }
    float hit = 0.0f;
    static dtPolyRef path[4096];
    int count = 0;
    dtStatus status = d.query->raycast(ref, start, b, &filter, &hit, normal, path, &count, 4096);
    if (dtStatusFailed(status) || (status & DT_BUFFER_TOO_SMALL) != 0)
    {
        return 2;
    }
    if (hit == FLT_MAX)
    {
        return 0;
    }
    *t = static_cast<double>(hit);
    return 1;
}

// The walls of the polygons Detour's ray from a toward b went through,
// as getPolyWallSegments gives them: their count, up to capacity, with
// each wall's two ends as six floats in segments; -1 when a has no
// polygon near or the ray ended another way; -2 when more lie there than
// capacity holds.
int DetourRayWalls(const Detour& d, const float a[3], const float b[3], float* segments,
                   int capacity)
{
    dtQueryFilter filter;
    const float extents[3] = {0.5f, 2.0f, 0.5f};
    dtPolyRef ref = 0;
    float start[3];
    d.query->findNearestPoly(a, extents, &filter, &ref, start);
    if (ref == 0)
    {
        return -1;
    }
    float hit = 0.0f;
    float normal[3];
    static dtPolyRef path[4096];
    int count = 0;
    dtStatus status = d.query->raycast(ref, start, b, &filter, &hit, normal, path, &count, 4096);
    if (dtStatusFailed(status) || (status & DT_BUFFER_TOO_SMALL) != 0)
    {
        return -1;
    }
    int total = 0;
    for (int i = 0; i < count; ++i)
    {
        float walls[DT_VERTS_PER_POLYGON * 6];
        int n = 0;
        status = d.query->getPolyWallSegments(path[i], &filter, walls, nullptr, &n,
                                              DT_VERTS_PER_POLYGON);
        if (dtStatusFailed(status) || total + n > capacity)
        {
            return -2;
        }
        std::memcpy(&segments[total * 6], walls, sizeof(float) * 6 * static_cast<std::size_t>(n));
        total += n;
    }
    return total;
}

// Where a ray from start toward b stopped on the ground, t of the way.
void Along(const float start[3], const float b[3], double t, double out[2])
{
    double sx = static_cast<double>(start[0]);
    double sz = static_cast<double>(start[2]);
    out[0] = sx + t * (static_cast<double>(b[0]) - sx);
    out[1] = sz + t * (static_cast<double>(b[2]) - sz);
}

// Rays both libraries cast from the same pair's first point toward its
// second: whether both stop at a wall or both reach the end, and, when
// both stop, whether the points they stop at lie within 0.25 m of each
// other on the ground. Every pair they disagree on is printed, with
// whether the other library has a wall within kWallBand of each point a
// ray stopped at: each simplifies its walls to within 0.3 m of the cells
// they follow (Recast's maxSimplificationError of 1.2 cells, Maul Nav's
// maxEdgeError), so the same wall of the two navmeshes may stand up to
// 0.6 m apart, and a ray may stop at one and pass the other's corner.
// A pair both stop on is banded when, besides, the ray that went farther
// stops within kWallBand past the other's stop; a pair one stops on and
// the other passes is grazed when, besides, the ray that passed crossed
// none of its own library's walls near the other's stop, so that it
// passes a wall only the other has. So that this check is seen to work,
// every ray that stopped must cross its own library's walls near its stop
// when it goes on to the end, and walls that cannot be read fail both. A pair passes only as
// kRayAllowed lists it, by scene, index and kind, so a change in which pairs disagree fails until
// it is explained, and so does a listed pair that no longer disagrees, so that the list stays the
// set of known disagreements.
struct RayTally
{
    int cast = 0;
    int endAgree = 0;
    int bothHit = 0;
    int hitClose = 0;
    int disagree = 0;
    int banded = 0;
    int grazed = 0;
    int listed = 0;
    int stale = 0;
    int stops = 0;
    int crossed = 0;
};

constexpr double kRayClose = 0.25;
constexpr double kWallBand = 0.6;

// The pairs that may disagree, each explained in the pull request that
// listed it: by scene, the pair's index and whether it is grazed rather
// than banded. A scene not listed allows none.
struct RayAllowed
{
    const char* scene;
    int pair;
    bool grazed;
};
constexpr RayAllowed kRayAllowed[] = {
    {"world", 8, true},    {"world", 41, false},  {"world", 61, false},  {"world", 64, false},
    {"world", 83, false},  {"world", 100, false}, {"world", 132, false}, {"world", 155, false},
    {"world", 181, false}, {"world", 191, false}, {"world", 198, false},
};
constexpr int kRayAllowedCount = static_cast<int>(sizeof(kRayAllowed) / sizeof(kRayAllowed[0]));

// The index in kRayAllowed of a scene's pair, or -1.
int AllowedOf(const char* scene, int pair)
{
    for (int k = 0; k < kRayAllowedCount; ++k)
    {
        if (kRayAllowed[k].pair == pair && std::strcmp(kRayAllowed[k].scene, scene) == 0)
        {
            return k;
        }
    }
    return -1;
}

// Whether one library has a wall within kWallBand of the point on the
// ground the other's ray from start toward b stopped at, t of the way,
// against a wall of normal n: its ray from kWallBand out of that point
// along the normal to kWallBand in stops within kWallBand of it.
bool WallNear(DiffMaul* maul, const Detour& d, bool onMaul, const float start[3], const float b[3],
              double t, const float n[3])
{
    double p[2];
    Along(start, b, t, p);
    double y = static_cast<double>(start[1]) +
               t * (static_cast<double>(b[1]) - static_cast<double>(start[1]));
    double nx = static_cast<double>(n[0]);
    double nz = static_cast<double>(n[2]);
    double length = std::hypot(nx, nz);
    if (length == 0.0)
    {
        return false;
    }
    // Face the normal back to the ray's start, whichever way each library
    // turns it.
    double back =
        (static_cast<double>(start[0]) - p[0]) * nx + (static_cast<double>(start[2]) - p[1]) * nz;
    double sign = back < 0.0 ? -1.0 : 1.0;
    nx = sign * nx / length;
    nz = sign * nz / length;
    auto F = [](double v) { return static_cast<float>(v); };
    const float from[3] = {F(p[0] + kWallBand * nx), F(y), F(p[1] + kWallBand * nz)};
    const float to[3] = {F(p[0] - kWallBand * nx), F(y), F(p[1] - kWallBand * nz)};
    float probe[3];
    float normal[3];
    double at = 0.0;
    int end = onMaul ? DiffMaulRaycast(maul, from, to, probe, &at, normal)
                     : DetourRaycast(d, from, to, probe, &at, normal);
    if (end != 1)
    {
        return false;
    }
    double h[2];
    Along(probe, to, at, h);
    return std::hypot(h[0] - p[0], h[1] - p[1]) <= kWallBand;
}

// Whether a library's ray from a toward b, which went past the point s
// on the ground the other's stopped at, heeded the wall there: its ray
// from a toward kWallBand past s stops at a wall within kWallBand of s. A
// ray that went through a wall both navmeshes have goes on and fails.
bool StopsNear(DiffMaul* maul, const Detour& d, bool onMaul, const float a[3], const float b[3],
               const double s[2])
{
    double ux = static_cast<double>(b[0]) - static_cast<double>(a[0]);
    double uz = static_cast<double>(b[2]) - static_cast<double>(a[2]);
    double length = std::hypot(ux, uz);
    if (length == 0.0)
    {
        return false;
    }
    auto F = [](double v) { return static_cast<float>(v); };
    const float to[3] = {F(s[0] + kWallBand * ux / length), b[1],
                         F(s[1] + kWallBand * uz / length)};
    float start[3];
    float normal[3];
    double at = 0.0;
    int end = onMaul ? DiffMaulRaycast(maul, a, to, start, &at, normal)
                     : DetourRaycast(d, a, to, start, &at, normal);
    if (end != 1)
    {
        return false;
    }
    double h[2];
    Along(start, to, at, h);
    return std::hypot(h[0] - s[0], h[1] - s[1]) <= kWallBand;
}

// Which side of the line on the ground from p through q the point r lies
// on: 1 left, -1 right, 0 on it.
int Side(const double p[2], const double q[2], const double r[2])
{
    double c = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    return c > 0.0 ? 1 : (c < 0.0 ? -1 : 0);
}

// The distance on the ground from s to the segment from p to q.
double SegmentDistance(const double s[2], const double p[2], const double q[2])
{
    double ux = q[0] - p[0];
    double uz = q[1] - p[1];
    double length2 = ux * ux + uz * uz;
    double t = length2 > 0.0 ? ((s[0] - p[0]) * ux + (s[1] - p[1]) * uz) / length2 : 0.0;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return std::hypot(p[0] + t * ux - s[0], p[1] + t * uz - s[1]);
}

// What the way on the ground from a library's ray start to b meets
// near the point s on it.
enum class Crossing
{
    // None of its own walls within kWallBand of s lies across it.
    none,
    // One does.
    wall,
    // Its walls near s could not be read: no polygon of the ray under s
    // or just behind it, the node limit, or more walls than the room.
    unread,
};

// Whether the way on the ground from a library's ray start, cast from a,
// to b crosses one of its own walls within kWallBand of the point s on
// it: Maul Nav's walls mnavQuery_FindWalls lists around s from the
// polygon its ray crossed there, and Detour's of the polygons its ray went
// through. A wall the way only touches does not count. A ray that reached
// b through a wall of its own navmesh near the other's stop fails, though
// the other's probe finds that wall near.
Crossing Cross(DiffMaul* maul, const Detour& d, bool onMaul, const float a[3], const float start[3],
               const float b[3], const double s[2])
{
    constexpr int kRoom = 1024;
    static float walls[kRoom * 6];
    int count = onMaul ? DiffMaulRayWalls(maul, a, b, s, kWallBand, walls, kRoom)
                       : DetourRayWalls(d, a, b, walls, kRoom);
    if (count < 0)
    {
        return Crossing::unread;
    }
    auto D = [](float v) { return static_cast<double>(v); };
    const double from[2] = {D(start[0]), D(start[2])};
    const double to[2] = {D(b[0]), D(b[2])};
    for (int i = 0; i < count; ++i)
    {
        const float* w = &walls[i * 6];
        const double p[2] = {D(w[0]), D(w[2])};
        const double q[2] = {D(w[3]), D(w[5])};
        if (SegmentDistance(s, p, q) > kWallBand)
        {
            continue;
        }
        if (Side(from, to, p) * Side(from, to, q) < 0 && Side(p, q, from) * Side(p, q, to) < 0)
        {
            return Crossing::wall;
        }
    }
    return Crossing::none;
}

// How far along the pair's direction from a the point p on the ground lies.
double Ahead(const float a[3], const float b[3], const double p[2])
{
    double ux = static_cast<double>(b[0]) - static_cast<double>(a[0]);
    double uz = static_cast<double>(b[2]) - static_cast<double>(a[2]);
    return (p[0] - static_cast<double>(a[0])) * ux + (p[1] - static_cast<double>(a[2])) * uz;
}

void CompareRays(const char* scene, DiffMaul* maul, const Detour& d, RayTally& t)
{
    bool seen[kRayAllowedCount] = {};
    for (int i = 0; i < kPairs; ++i)
    {
        float a[3];
        float b[3];
        if (!DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 1u, a) ||
            !DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 2u, b))
        {
            continue;
        }
        float ms[3];
        float ds[3];
        float mn[3] = {0.0f, 0.0f, 0.0f};
        float dn[3] = {0.0f, 0.0f, 0.0f};
        double mt = 0.0;
        double dt = 0.0;
        int m = DiffMaulRaycast(maul, a, b, ms, &mt, mn);
        int r = DetourRaycast(d, a, b, ds, &dt, dn);
        if (m < 0 || r < 0)
        {
            continue;
        }
        t.cast += 1;
        double mp[2];
        double dp[2];
        Along(ms, b, m == 1 ? mt : 1.0, mp);
        Along(ds, b, r == 1 ? dt : 1.0, dp);
        double gap = std::hypot(mp[0] - dp[0], mp[1] - dp[1]);
        // A ray that stopped, gone on to the end, crosses a wall of its own
        // near its stop.
        for (int k = 0; k < 2; ++k)
        {
            bool onMaul = k == 0;
            if ((onMaul ? m : r) != 1)
            {
                continue;
            }
            const double* p = onMaul ? mp : dp;
            t.stops += 1;
            Crossing c = Cross(maul, d, onMaul, a, onMaul ? ms : ds, b, p);
            if (c == Crossing::wall)
            {
                t.crossed += 1;
            }
            else
            {
                Say("ray %s %d: %s's ray stopped at (%.4f %.4f) but %s\n", scene, i,
                    onMaul ? "maul" : "detour", p[0], p[1],
                    c == Crossing::none ? "crosses none of its walls near there"
                                        : "its walls near there could not be read");
            }
        }
        bool agree = m == r;
        t.endAgree += agree ? 1 : 0;
        if (m == 1 && r == 1)
        {
            t.bothHit += 1;
            t.hitClose += gap <= kRayClose ? 1 : 0;
        }
        if (!agree || (m == 1 && r == 1 && gap > kRayClose))
        {
            // Every point a ray stopped at has the other library's wall
            // near, and neither ended another way; when both stopped, the
            // one that went farther stops within the band past the other.
            bool near = m != 2 && r != 2 && (m != 1 || WallNear(maul, d, false, ms, b, mt, mn)) &&
                        (r != 1 || WallNear(maul, d, true, ds, b, dt, dn));
            bool both = m == 1 && r == 1;
            bool mauler = Ahead(a, b, mp) > Ahead(a, b, dp);
            bool banded =
                near && both &&
                (mauler ? StopsNear(maul, d, true, a, b, dp) : StopsNear(maul, d, false, a, b, mp));
            // When one stopped, the one that reached the end crossed none
            // of its own walls near the other's stop.
            Crossing passed = !near || both ? Crossing::none
                              : m == 0      ? Cross(maul, d, true, a, ms, b, dp)
                                            : Cross(maul, d, false, a, ds, b, mp);
            bool grazed = near && !both && passed == Crossing::none;
            int k = AllowedOf(scene, i);
            bool listed =
                k >= 0 && ((banded && !kRayAllowed[k].grazed) || (grazed && kRayAllowed[k].grazed));
            if (k >= 0)
            {
                seen[k] = true;
            }
            t.disagree += 1;
            t.banded += banded ? 1 : 0;
            t.grazed += grazed ? 1 : 0;
            t.listed += listed ? 1 : 0;
            const char* verdict = listed   ? (banded ? "banded, listed" : "grazed, listed")
                                  : banded ? "banded, not listed as banded"
                                  : grazed ? "grazed, not listed as grazed"
                                  : near && !both && passed == Crossing::unread
                                      ? "its walls near the other's stop could not be read"
                                  : near && !both ? "through its own wall near the other's stop"
                                  : near          ? "past the other's stop"
                                                  : "no wall within 0.6 m";
            auto D = [](float v) { return static_cast<double>(v); };
            Say("ray %s %d: (%.4f %.4f %.4f) to (%.4f %.4f %.4f): maul %d at (%.4f %.4f) "
                "t %.6f, detour %d at (%.4f %.4f) t %.6f, %.4f m apart, %s\n",
                scene, i, D(a[0]), D(a[1]), D(a[2]), D(b[0]), D(b[1]), D(b[2]), m, mp[0], mp[1],
                m == 1 ? mt : 1.0, r, dp[0], dp[1], r == 1 ? dt : 1.0, gap, verdict);
        }
    }
    for (int k = 0; k < kRayAllowedCount; ++k)
    {
        if (!seen[k] && std::strcmp(kRayAllowed[k].scene, scene) == 0)
        {
            Say("ray %s %d: the rays no longer disagree; remove it from kRayAllowed\n", scene,
                kRayAllowed[k].pair);
            t.stale += 1;
        }
    }
}

// Detour's nearest wall within radius, as DiffMaulWallDistance gives
// Maul Nav's.
int DetourWallDistance(const Detour& d, const float at[3], double radius, float center[3],
                       double* distance, float wall[3])
{
    dtQueryFilter filter;
    const float extents[3] = {0.5f, 2.0f, 0.5f};
    dtPolyRef ref = 0;
    d.query->findNearestPoly(at, extents, &filter, &ref, center);
    if (ref == 0)
    {
        return -1;
    }
    float hit = 0.0f;
    float normal[3];
    // Detour writes the wall's point only when it finds one.
    wall[0] = FLT_MAX;
    dtStatus status = d.query->findDistanceToWall(ref, center, static_cast<float>(radius), &filter,
                                                  &hit, wall, normal);
    if (dtStatusFailed(status) || (status & DT_OUT_OF_NODES) != 0)
    {
        return 2;
    }
    if (wall[0] == FLT_MAX)
    {
        return 0;
    }
    *distance = static_cast<double>(hit);
    return 1;
}

// The nearest wall within kWallRadius of each pair's two points, found by
// both libraries from the polygon nearest the point: whether both find
// one, and how far apart their distances are. They may differ by
// kWallBand plus how far apart the two snap the point: each library's
// walls lie within 0.3 m of the cells they follow, as the rays' band
// explains, so the distance from a point to the nearest of them moves by
// at most that much with each. A wall one finds and the other does not
// is within the band when the one found lies within kWallBand of the
// radius. Every point where they differ, by whether they find one or by
// more than 0.25 m, is printed with whether it lies within the band, and
// passes only within the band and as kWallAllowed lists it, with the
// wall Maul Nav finds there; a listed point that no longer differs
// fails, as with the rays.
struct WallTally
{
    int searched = 0;
    int foundAgree = 0;
    int bothFound = 0;
    int close = 0;
    int differ = 0;
    int listed = 0;
    int stale = 0;
};

constexpr double kWallRadius = 5.0;

// The points whose walls may differ, each explained in the pull request
// that listed it: by scene, the point's seed and the wall Maul Nav finds
// there, its distance and z within kWallMatch, so that a change of Maul
// Nav's wall at a listed point fails until it is explained. A scene not
// listed allows none.
struct WallAllowed
{
    const char* scene;
    int seed;
    double distance;
    double z;
};
constexpr WallAllowed kWallAllowed[] = {{"world", 12, 4.9209, 30.5}, {"world", 156, 4.8772, 30.5}};
constexpr int kWallAllowedCount = static_cast<int>(sizeof(kWallAllowed) / sizeof(kWallAllowed[0]));
constexpr double kWallMatch = 0.001;

void CompareWalls(const char* scene, DiffMaul* maul, const Detour& d, WallTally& t)
{
    bool seen[kWallAllowedCount] = {};
    for (int seed = 1; seed <= 2 * kPairs; ++seed)
    {
        float at[3];
        if (!DiffMaulPoint(maul, static_cast<uint64_t>(seed), at))
        {
            continue;
        }
        float mc[3];
        float dc[3];
        float mw[3] = {0.0f, 0.0f, 0.0f};
        float dw[3] = {0.0f, 0.0f, 0.0f};
        double md = 0.0;
        double dd = 0.0;
        int m = DiffMaulWallDistance(maul, at, kWallRadius, mc, &md, mw);
        int r = DetourWallDistance(d, at, kWallRadius, dc, &dd, dw);
        if (m < 0 || r < 0)
        {
            continue;
        }
        t.searched += 1;
        t.foundAgree += m == r ? 1 : 0;
        double snap = std::hypot(static_cast<double>(mc[0]) - static_cast<double>(dc[0]),
                                 static_cast<double>(mc[2]) - static_cast<double>(dc[2]));
        double band = kWallBand + snap;
        double gap = m == 1 && r == 1 ? std::fabs(md - dd) : 0.0;
        bool within = false;
        if (m == 1 && r == 1)
        {
            t.bothFound += 1;
            t.close += gap <= kRayClose ? 1 : 0;
            within = gap <= band;
        }
        else if (m == 0 && r == 0)
        {
            within = true;
        }
        else if (m != 2 && r != 2)
        {
            within = (m == 1 ? md : dd) >= kWallRadius - band;
        }
        if ((m == 0 && r == 0) || (m == 1 && r == 1 && gap <= kRayClose))
        {
            continue;
        }
        int k = -1;
        for (int j = 0; j < kWallAllowedCount; ++j)
        {
            if (kWallAllowed[j].seed == seed && std::strcmp(kWallAllowed[j].scene, scene) == 0)
            {
                k = j;
            }
        }
        if (k >= 0)
        {
            seen[k] = true;
        }
        auto D = [](float v) { return static_cast<double>(v); };
        bool matches = k >= 0 && m == 1 && std::fabs(md - kWallAllowed[k].distance) <= kWallMatch &&
                       std::fabs(D(mw[2]) - kWallAllowed[k].z) <= kWallMatch;
        bool listed = within && matches;
        t.differ += 1;
        t.listed += listed ? 1 : 0;
        // A library's wall, or that it found none within the radius.
        auto Found = [&](char* out, int end, double distance, const float wall[3])
        {
            if (end == 1)
            {
                std::snprintf(out, 64, "at %.4f m (%.4f %.4f)", distance, D(wall[0]), D(wall[2]));
            }
            else
            {
                std::snprintf(out, 64, "none within %.1f m", kWallRadius);
            }
        };
        char mf[64];
        char df[64];
        Found(mf, m, md, mw);
        Found(df, r, dd, dw);
        Say("wall %s %d: (%.4f %.4f %.4f): maul %d %s, detour %d %s, %.4f m apart, snapped "
            "%.4f m apart, %s\n",
            scene, seed, D(at[0]), D(at[1]), D(at[2]), m, mf, r, df, gap, snap,
            within ? (listed   ? "within the band, listed"
                      : k >= 0 ? "within the band, listed with another Maul Nav wall"
                               : "within the band, not listed")
                   : (k >= 0 ? "past the band, though listed" : "past the band, not listed"));
    }
    for (int k = 0; k < kWallAllowedCount; ++k)
    {
        if (!seen[k] && std::strcmp(kWallAllowed[k].scene, scene) == 0)
        {
            Say("wall %s %d: the walls no longer differ; remove it from kWallAllowed\n", scene,
                kWallAllowed[k].seed);
            t.stale += 1;
        }
    }
}

// Detour's move along the surface, as DiffMaulMove gives Maul Nav's.
// Detour copies the wanted point when it reaches it, and a wall point
// otherwise.
int DetourMove(const Detour& d, const float a[3], const float to[3], float start[3], float end[3])
{
    dtQueryFilter filter;
    const float extents[3] = {0.5f, 2.0f, 0.5f};
    dtPolyRef ref = 0;
    d.query->findNearestPoly(a, extents, &filter, &ref, start);
    if (ref == 0)
    {
        return -1;
    }
    static dtPolyRef visited[256];
    int count = 0;
    dtStatus status = d.query->moveAlongSurface(ref, start, to, &filter, end, visited, &count, 256);
    if (dtStatusFailed(status) || (status & DT_BUFFER_TOO_SMALL) != 0)
    {
        return 2;
    }
    return end[0] == to[0] && end[2] == to[2] ? 0 : 1;
}

// A move along the surface from each pair's first point toward its
// second, at most kMoveLength on the ground, by both libraries from the
// polygon nearest the first point: whether each reaches the wanted
// point, and how far apart they end. Detour's move keeps at most 48
// polygons to visit and 64 nodes, one for each neighbour of a polygon it
// visits, and passes over the rest without a status, so a move is kept
// short enough to stay within them: at 5 m, Maul Nav's walk visits at
// most 10 polygons of up to 6 edges on these scenes, and Detour's
// untiled navmesh splits none of its polygons at tile sides. Detour
// searches round half the move's length in three dimensions, so the
// wanted point is at the first point's height. Each move ends at the
// wanted point or at the point nearest it on the walls met, so how far
// each ends from the wanted point moves by at most kWallBand plus how
// far apart the two snap the start, as for the walls. Every pair whose
// ends lie more than 0.25 m apart is printed with whether its two
// distances from the wanted point lie within that band, and passes only
// within the band and as kMoveAllowed lists it, with the point Maul
// Nav's move ends at; a listed pair that no longer differs fails, as
// with the rays.
struct MoveTally
{
    int moved = 0;
    int endAgree = 0;
    int bothStopped = 0;
    int close = 0;
    int differ = 0;
    int listed = 0;
    int stale = 0;
};

constexpr double kMoveLength = 5.0;

// The pairs whose moves may end apart, each explained in the pull
// request that listed it: by scene, the pair's index and the point Maul
// Nav's move ends at, its x and z within kWallMatch, so that a change of
// Maul Nav's end fails until it is explained. A scene not listed allows
// none.
struct MoveAllowed
{
    const char* scene;
    int pair;
    double x;
    double z;
};
// An entry no scene names keeps the array from being empty.
constexpr MoveAllowed kMoveAllowed[] = {{"", -1, 0.0, 0.0}};
constexpr int kMoveAllowedCount = static_cast<int>(sizeof(kMoveAllowed) / sizeof(kMoveAllowed[0]));

void CompareMoves(const char* scene, DiffMaul* maul, const Detour& d, MoveTally& t)
{
    bool seen[kMoveAllowedCount] = {};
    auto D = [](float v) { return static_cast<double>(v); };
    auto Apart = [&](const float p[3], const float q[3])
    { return std::hypot(D(p[0]) - D(q[0]), D(p[2]) - D(q[2])); };
    for (int i = 0; i < kPairs; ++i)
    {
        float a[3];
        float b[3];
        if (!DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 1u, a) ||
            !DiffMaulPoint(maul, 2u * static_cast<uint64_t>(i) + 2u, b))
        {
            continue;
        }
        double length = Apart(a, b);
        double scale = length > kMoveLength ? kMoveLength / length : 1.0;
        float to[3] = {static_cast<float>(D(a[0]) + (D(b[0]) - D(a[0])) * scale), a[1],
                       static_cast<float>(D(a[2]) + (D(b[2]) - D(a[2])) * scale)};
        float ms[3];
        float ds[3];
        float me[3];
        float de[3];
        int m = DiffMaulMove(maul, a, to, ms, me);
        int r = DetourMove(d, a, to, ds, de);
        if (m < 0 || r < 0)
        {
            continue;
        }
        t.moved += 1;
        t.endAgree += m == r ? 1 : 0;
        double apart = m == 2 || r == 2 ? 0.0 : Apart(me, de);
        if (m == 1 && r == 1)
        {
            t.bothStopped += 1;
            t.close += apart <= kRayClose ? 1 : 0;
        }
        if (m != 2 && r != 2 && apart <= kRayClose)
        {
            continue;
        }
        double snap = Apart(ms, ds);
        double band = kWallBand + snap;
        double gap = m == 2 || r == 2 ? 0.0 : std::fabs(Apart(me, to) - Apart(de, to));
        bool within = m != 2 && r != 2 && gap <= band;
        int k = -1;
        for (int j = 0; j < kMoveAllowedCount; ++j)
        {
            if (kMoveAllowed[j].pair == i && std::strcmp(kMoveAllowed[j].scene, scene) == 0)
            {
                k = j;
            }
        }
        if (k >= 0)
        {
            seen[k] = true;
        }
        bool matches = k >= 0 && std::fabs(D(me[0]) - kMoveAllowed[k].x) <= kWallMatch &&
                       std::fabs(D(me[2]) - kMoveAllowed[k].z) <= kWallMatch;
        bool listed = within && matches;
        t.differ += 1;
        t.listed += listed ? 1 : 0;
        Say("move %s %d: (%.4f %.4f) toward (%.4f %.4f): maul %d at (%.4f %.4f), detour %d at "
            "(%.4f %.4f), %.4f m apart, %.4f and %.4f m short, snapped %.4f m apart, %s\n",
            scene, i, D(a[0]), D(a[2]), D(to[0]), D(to[2]), m, D(me[0]), D(me[2]), r, D(de[0]),
            D(de[2]), apart, Apart(me, to), Apart(de, to), snap,
            within ? (listed   ? "within the band, listed"
                      : k >= 0 ? "within the band, listed with another Maul Nav end"
                               : "within the band, not listed")
                   : (k >= 0 ? "past the band, though listed" : "past the band, not listed"));
    }
    for (int k = 0; k < kMoveAllowedCount; ++k)
    {
        if (!seen[k] && std::strcmp(kMoveAllowed[k].scene, scene) == 0)
        {
            Say("move %s %d: the moves no longer differ; remove it from kMoveAllowed\n", scene,
                kMoveAllowed[k].pair);
            t.stale += 1;
        }
    }
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
    Say("%-10s %-12s %12.2f %12.2f %-6s %6.2f\n", scene, what, maul / scale, detour / scale, unit,
        detour > 0.0 ? maul / detour : 0.0);
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
    OpenSummary();
    bool pass = true;
    Say("%-10s %6s %6s %8s %6s %8s %9s %8s\n", "scene", "pairs", "fringe", "reach", "both",
        "length", "notlonger", "sums");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        bool built = maul != nullptr && Build(scene, d);
        if (!built)
        {
            Say("%s: %s build failed\n", scene.name, maul == nullptr ? "Maul Nav's" : "Recast's");
            DiffMaulDestroy(maul);
            CloseSummary();
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
        Say("%-10s %6d %6d %7.1f%% %6d %7.1f%% %8.1f%% %+7.2f%%\n", scene.name, t.snapped,
            t.unsnapped, 100.0 * reach, t.bothReached, 100.0 * length, 100.0 * notLonger,
            100.0 * sums);
        pass = pass && t.snapped >= kPairs * 9 / 10 && reach >= kReachFloor &&
               length >= kLengthFloor && notLonger >= kNotLongerFloor &&
               std::fabs(sums) <= kSumBound;
    }
    Say(pass ? "agreement within the floors\n" : "agreement below a floor\n");
    // Rays: each pair agrees, or is banded or grazed as kRayAllowed lists
    // it, and every ray that stopped crosses its own wall near its stop.
    bool rays = true;
    Say("\n%-10s %6s %8s %6s %8s %8s %6s %6s %6s %6s %7s\n", "scene", "rays", "ends", "hits",
        "close", "differ", "banded", "grazed", "listed", "stops", "crossed");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        if (maul != nullptr && Build(scene, d))
        {
            RayTally t;
            CompareRays(scene.name, maul, d, t);
            double cast = t.cast > 0 ? static_cast<double>(t.cast) : 1.0;
            double hits = t.bothHit > 0 ? static_cast<double>(t.bothHit) : 1.0;
            Say("%-10s %6d %7.1f%% %6d %7.1f%% %8d %6d %6d %6d %6d %7d\n", scene.name, t.cast,
                100.0 * t.endAgree / cast, t.bothHit, 100.0 * t.hitClose / hits, t.disagree,
                t.banded, t.grazed, t.listed, t.stops, t.crossed);
            rays = rays && t.listed == t.disagree && t.stale == 0 && t.crossed == t.stops;
        }
        DiffMaulDestroy(maul);
    }
    Say(rays ? "rays agree, or differ as listed within the wall band\n"
             : "rays differ past the wall band or the list\n");
    pass = pass && rays;
    // Walls: each point agrees, or differs as kWallAllowed lists it.
    bool walls = true;
    Say("\n%-10s %6s %8s %6s %8s %8s %6s\n", "scene", "points", "found", "both", "close", "differ",
        "listed");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        if (maul != nullptr && Build(scene, d))
        {
            WallTally t;
            CompareWalls(scene.name, maul, d, t);
            double searched = t.searched > 0 ? static_cast<double>(t.searched) : 1.0;
            double both = t.bothFound > 0 ? static_cast<double>(t.bothFound) : 1.0;
            Say("%-10s %6d %7.1f%% %6d %7.1f%% %8d %6d\n", scene.name, t.searched,
                100.0 * t.foundAgree / searched, t.bothFound, 100.0 * t.close / both, t.differ,
                t.listed);
            walls = walls && t.listed == t.differ && t.stale == 0;
        }
        DiffMaulDestroy(maul);
    }
    Say(walls ? "walls agree, or differ as listed\n" : "walls differ past the list\n");
    pass = pass && walls;
    // Moves: each pair's moves end together, or apart as kMoveAllowed
    // lists it.
    bool moves = true;
    Say("\n%-10s %6s %8s %6s %8s %8s %6s\n", "scene", "moves", "ends", "stops", "close", "differ",
        "listed");
    for (int32_t i = 0; i < DiffSceneCount(); ++i)
    {
        DiffScene scene = DiffGetScene(i);
        Detour d;
        DiffMaul* maul = DiffMaulBuild(&scene);
        if (maul != nullptr && Build(scene, d))
        {
            MoveTally t;
            CompareMoves(scene.name, maul, d, t);
            double moved = t.moved > 0 ? static_cast<double>(t.moved) : 1.0;
            double stops = t.bothStopped > 0 ? static_cast<double>(t.bothStopped) : 1.0;
            Say("%-10s %6d %7.1f%% %6d %7.1f%% %8d %6d\n", scene.name, t.moved,
                100.0 * t.endAgree / moved, t.bothStopped, 100.0 * t.close / stops, t.differ,
                t.listed);
            moves = moves && t.listed == t.differ && t.stale == 0;
        }
        DiffMaulDestroy(maul);
    }
    Say(moves ? "moves end together, or apart as listed\n" : "moves end apart past the list\n");
    pass = pass && moves;
    Counter counter;
    Say("\ncosts, best of five, reported and not gated; Maul Nav in 32 m tiles, Recast "
        "untiled; %s; load average %.2f\n",
        counter.available() ? "instructions are user space's, retired"
                            : "no instruction counter here, time only",
        Load());
    Say("%-10s %-12s %12s %12s %-6s %6s\n", "scene", "measure", "maul nav", "detour", "unit",
        "ratio");
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
    CloseSummary();
    return pass ? 0 : 1;
}
