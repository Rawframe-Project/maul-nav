// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Scenarios from game shapes (requirements section 5): a building of two
// floors joined by stairs and a ramp, a bridge that collapses under a
// dynamic navmesh while an agent's corridor runs over it, a large
// terrain streamed in and out around an agent crossing it, and an RTS
// group led by a flow field through a gap, steered apart by avoidance.

#include "test_harness.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The hash of the building's paths, the same on every platform.
#define BUILDING_HASH 0x64041a557af8393bull

// The hash of the terrain's sixteen tiles.
#define TERRAIN_HASH 0x18be2cb00b4aec43ull

// The hash of the group's final places.
#define GROUP_HASH 0xb16f4123b3485eedull

enum
{
    MOST_QUADS = 64,
    TILE_ROOM = 1 << 18,
    MOST_TILES = 16
};

// A mesh of quads, each two triangles facing up.
typedef struct Quads
{
    mnavVec3 vertices[MOST_QUADS * 4];
    int32_t indices[MOST_QUADS * 6];
    int32_t count;
} Quads;

// Adds the quad from (x0, z0) to (x1, z1), its height y0 along z0 and y1
// along z1.
static void AddQuad(Quads* q, float x0, float z0, float x1, float z1, float y0, float y1)
{
    int32_t v = q->count * 4;
    q->vertices[v + 0] = (mnavVec3){x0, y0, z0};
    q->vertices[v + 1] = (mnavVec3){x0, y1, z1};
    q->vertices[v + 2] = (mnavVec3){x1, y1, z1};
    q->vertices[v + 3] = (mnavVec3){x1, y0, z0};
    const int32_t corners[6] = {v, v + 1, v + 2, v, v + 2, v + 3};
    memcpy(&q->indices[q->count * 6], corners, sizeof(corners));
    q->count += 1;
}

static mnavTriangleMesh Mesh(const Quads* q)
{
    return (mnavTriangleMesh){q->vertices, q->count * 4, q->indices, q->count * 2, nullptr};
}

static uint8_t s_tiles[MOST_TILES][TILE_ROOM];
static size_t s_sizes[MOST_TILES];

// Bakes tiles (x0, z0) to (x1, z1) of a mesh into s_tiles, row by row.
static void Bake(const mnavBakeDef* def, const Quads* q, int32_t x0, int32_t z0, int32_t x1,
                 int32_t z1)
{
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(def, &baker).result == mnav_success, "baker");
    mnavTriangleMesh mesh = Mesh(q);
    int32_t n = 0;
    for (int32_t z = z0; z <= z1; ++z)
    {
        for (int32_t x = x0; x <= x1; ++x, ++n)
        {
            CHECK(mnavBakeTile(baker, &mesh, 1, x, z, nullptr) == mnav_success &&
                      mnavCopyBakedTile(baker, s_tiles[n], TILE_ROOM, &s_sizes[n]) == mnav_success,
                  "baked");
        }
    }
    mnavDestroyBaker(baker);
}

static mnavNearest Nearest(const mnavNavmesh* navmesh, double x, double y, double z)
{
    mnavNearest n = {0};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, y, z}, (mnavVec3){0.5f, 1.0f, 0.5f},
                          &n) == mnav_success,
          "looked");
    return n;
}

static mnavPath Path(mnavQuery* query, const mnavNavmesh* navmesh, mnavNearest a, mnavNearest b)
{
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, nullptr, a.polygon, a.point, b.polygon, b.point, &path) ==
              mnav_success,
          "searched");
    return path;
}

// Whether a path runs through a box on the ground: its segments sampled
// every 0.1 m.
static bool Passes(const mnavPath* path, double x0, double z0, double x1, double z1)
{
    for (int32_t i = 0; i + 1 < path->pointCount; ++i)
    {
        mnavPos3 a = path->points[i];
        mnavPos3 b = path->points[i + 1];
        double length = sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
        int32_t steps = 1 + (int32_t)(length / 0.1);
        for (int32_t k = 0; k <= steps; ++k)
        {
            double t = (double)k / (double)steps;
            double x = a.x + (b.x - a.x) * t;
            double z = a.z + (b.z - a.z) * t;
            if (x >= x0 && x <= x1 && z >= z0 && z <= z1)
            {
                return true;
            }
        }
    }
    return false;
}

static double Highest(const mnavPath* path)
{
    double top = -1e9;
    for (int32_t i = 0; i < path->pointCount; ++i)
    {
        top = path->points[i].y > top ? path->points[i].y : top;
    }
    return top;
}

// A ground floor, an upper floor 3 m up over half of it, twelve stairs of
// 0.25 m on one side and a ramp of 20.6 degrees on the other.
static void Building(Quads* q, bool stairs)
{
    q->count = 0;
    AddQuad(q, -2, -2, 26, 26, 0, 0);
    AddQuad(q, 0, 10, 20, 20, 3, 3);
    for (int32_t k = 1; k <= 12 && stairs; ++k)
    {
        float z0 = 10.0f - (float)(13 - k) * 0.4f;
        AddQuad(q, 2, z0, 4, z0 + 0.4f, 0.25f * (float)k, 0.25f * (float)k);
    }
    AddQuad(q, 16, 2, 18, 10, 0, 3);
}

static mnavNavmesh* LoadOne(const mnavBakeDef* def)
{
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(def, &navmesh).result == mnav_success &&
              mnavStageTile(navmesh, s_tiles[0], s_sizes[0]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "loaded");
    return navmesh;
}

static void TestBuilding(mnavQuery* query)
{
    static Quads q;
    Building(&q, true);
    mnavBakeDef def = mnavDefaultBakeDef();
    Bake(&def, &q, 0, 0, 0, 0);
    mnavNavmesh* navmesh = LoadOne(&def);
    // Two levels at one place.
    mnavNearest up = Nearest(navmesh, 10, 3, 15);
    mnavNearest down = Nearest(navmesh, 10, 0, 15);
    CHECK(up.polygon.slot != 0 && down.polygon.slot != 0 && fabs(up.point.y - 3.0) < 0.3 &&
              fabs(down.point.y) < 0.3,
          "the upper floor over the ground floor");
    // From the ground to the upper floor and back, by the stairs or the
    // ramp, whichever is cheaper.
    mnavNearest a = Nearest(navmesh, 4, 0, 2);
    mnavNearest b = Nearest(navmesh, 4, 3, 16);
    mnavPath path = Path(query, navmesh, a, b);
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, path.points,
                               (int32_t)((size_t)path.pointCount * sizeof(mnavPos3)));
    CHECK(path.end == mnav_pathFound && Highest(&path) > 2.7 && Passes(&path, 1.5, 5, 4.5, 10.5),
          "up the stairs");
    mnavNearest c = Nearest(navmesh, 17, 3, 16);
    mnavNearest d = Nearest(navmesh, 17, 0, 0);
    path = Path(query, navmesh, c, d);
    hash = mnavHash64(hash, path.points, (int32_t)((size_t)path.pointCount * sizeof(mnavPos3)));
    CHECK(path.end == mnav_pathFound && Passes(&path, 15.5, 3, 18.5, 9.5), "down the ramp");
    // Under the upper floor to a place under it: never climbing.
    mnavNearest e = Nearest(navmesh, 10, 0, 18);
    path = Path(query, navmesh, a, e);
    CHECK(path.end == mnav_pathFound && Highest(&path) < 0.5, "along the ground under it");
    // Without the stairs, the way up runs by the ramp.
    Building(&q, false);
    Bake(&def, &q, 0, 0, 0, 0);
    mnavDestroyNavmesh(navmesh);
    navmesh = LoadOne(&def);
    a = Nearest(navmesh, 4, 0, 2);
    b = Nearest(navmesh, 4, 3, 16);
    path = Path(query, navmesh, a, b);
    hash = mnavHash64(hash, path.points, (int32_t)((size_t)path.pointCount * sizeof(mnavPos3)));
    CHECK(path.end == mnav_pathFound && Passes(&path, 15.5, 3, 18.5, 9.5) &&
              !Passes(&path, 1.5, 5, 4.5, 9.5),
          "up the ramp when the stairs are gone");
    printf("BUILDING_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == BUILDING_HASH, "the pinned hash");
    mnavDestroyNavmesh(navmesh);
}

// Two plateaus 5 m up over nothing, joined by a near bridge and a far
// one, 8 m tiles.
static void Plateaus(Quads* q, bool near)
{
    q->count = 0;
    AddQuad(q, 0, 0, 12, 32, 5, 5);
    AddQuad(q, 20, 0, 32, 32, 5, 5);
    AddQuad(q, 12, 26, 20, 28, 5, 5);
    if (near)
    {
        AddQuad(q, 12, 4, 20, 6, 5, 5);
    }
}

static void TestCollapsingBridge(mnavQuery* query)
{
    static Quads q;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.tileCells = 32;
    def.tier = mnav_tierDynamic;
    Plateaus(&q, true);
    Bake(&def, &q, 0, 0, 3, 3);
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    for (int32_t t = 0; t < 16; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    mnavNearest a = Nearest(navmesh, 2, 5, 5);
    mnavNearest b = Nearest(navmesh, 30, 5, 5);
    mnavPath path = Path(query, navmesh, a, b);
    CHECK(path.end == mnav_pathFound && path.length < 30.0 && Passes(&path, 11, 3.5, 21, 6.5),
          "over the near bridge");
    static mnavPolygonId buffer[256];
    mnavCorridor corridor;
    CHECK(mnavResetCorridor(&corridor, buffer, 256, a.polygon, a.point) == mnav_success &&
              mnavSetCorridor(&corridor, &path) == mnav_success,
          "a corridor along it");
    // The bridge falls: its two tiles are baked again without it.
    static uint8_t keep[2][TILE_ROOM];
    memcpy(keep[0], s_tiles[1], s_sizes[1]);
    memcpy(keep[1], s_tiles[2], s_sizes[2]);
    size_t keepSizes[2] = {s_sizes[1], s_sizes[2]};
    Plateaus(&q, false);
    Bake(&def, &q, 1, 0, 2, 0);
    CHECK(mnavStageTile(navmesh, s_tiles[0], s_sizes[0]).result == mnav_success &&
              mnavStageTile(navmesh, s_tiles[1], s_sizes[1]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the bridge's tiles replaced");
    int32_t valid = 0;
    CHECK(mnavCheckCorridor(navmesh, nullptr, &corridor, &valid) == mnav_success &&
              valid < corridor.count,
          "the corridor broken where the bridge was");
    mnavNearest middle = {0};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){16, 5, 5}, (mnavVec3){0.5f, 1.0f, 0.5f},
                          &middle) == mnav_success &&
              middle.polygon.slot == 0,
          "nothing where the bridge stood");
    mnavPath replanned = {0};
    CHECK(mnavReplanCorridor(query, navmesh, nullptr, &corridor, (mnavVec3){1, 1, 1}, &replanned) ==
                  mnav_success &&
              replanned.end == mnav_pathFound,
          "replanned");
    double around = replanned.length;
    path = Path(query, navmesh, a, b);
    CHECK(path.end == mnav_pathFound && around > 60.0 && fabs(path.length - around) < 1e-9 &&
              Passes(&path, 11, 25.5, 21, 28.5),
          "around by the far bridge, as a new search finds");
    // Built again, the bridge carries the way once more.
    CHECK(mnavStageTile(navmesh, keep[0], keepSizes[0]).result == mnav_success &&
              mnavStageTile(navmesh, keep[1], keepSizes[1]).result == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "the bridge's tiles back");
    path = Path(query, navmesh, a, b);
    CHECK(path.end == mnav_pathFound && path.length < 30.0, "over the bridge again");
    mnavDestroyNavmesh(navmesh);
}

enum
{
    SAMPLES = 129
};

static float s_heights[SAMPLES * SAMPLES];

// A bowl 4 m deep with ripples of up to 8 cm, sampled every meter over
// 128 by 128 m: four by four tiles of 32 m.
static mnavTerrain Terrain(void)
{
    for (int32_t r = 0; r < SAMPLES; ++r)
    {
        for (int32_t c = 0; c < SAMPLES; ++c)
        {
            int32_t dx = c - 64;
            int32_t dz = r - 64;
            s_heights[r * SAMPLES + c] =
                (float)(dx * dx + dz * dz) * 0.0005f + (float)((c * 7 + r * 13) % 5) * 0.02f;
        }
    }
    return (mnavTerrain){{0, 0, 0}, 1.0f, 1.0f, SAMPLES, SAMPLES, s_heights, nullptr};
}

// The tiles within one of the agent's, of the four by four.
static bool Wanted(int32_t x, int32_t z, double ax, double az)
{
    int32_t tx = (int32_t)(ax / 32.0);
    int32_t tz = (int32_t)(az / 32.0);
    return abs(x - tx) <= 1 && abs(z - tz) <= 1;
}

// Moves the window of loaded tiles to the agent's; returns how many are
// loaded.
static int32_t Stream(mnavNavmesh* navmesh, bool loaded[4][4], double ax, double az)
{
    int32_t count = 0;
    for (int32_t z = 0; z < 4; ++z)
    {
        for (int32_t x = 0; x < 4; ++x)
        {
            bool want = Wanted(x, z, ax, az);
            if (loaded[z][x] && !want)
            {
                CHECK(mnavStageTileRemoval(navmesh, x, z) == mnav_success, "streamed out");
            }
            else if (!loaded[z][x] && want)
            {
                CHECK(mnavStageTile(navmesh, s_tiles[z * 4 + x], s_sizes[z * 4 + x]).result ==
                          mnav_success,
                      "streamed in");
            }
            loaded[z][x] = want;
            count += want ? 1 : 0;
        }
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return count;
}

static mnavNearest Ground(const mnavNavmesh* navmesh, double x, double z)
{
    mnavNearest n = {0};
    CHECK(mnavFindNearest(navmesh, nullptr, (mnavPos3){x, 2.0, z}, (mnavVec3){0.5f, 6.0f, 0.5f},
                          &n) == mnav_success,
          "looked");
    return n;
}

static void TestStreamedTerrain(mnavQuery* query)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavTerrain terrain = Terrain();
    const mnavBakeInput input = {nullptr, 0, &terrain, 1, nullptr, 0, nullptr};
    mnavBaker* baker = nullptr;
    CHECK(mnavCreateBaker(&def, &baker).result == mnav_success, "baker");
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t t = 0; t < 16; ++t)
    {
        CHECK(mnavBakeTileInput(baker, &input, t % 4, t / 4, nullptr) == mnav_success &&
                  mnavCopyBakedTile(baker, s_tiles[t], TILE_ROOM, &s_sizes[t]) == mnav_success,
              "baked");
        hash = mnavHash64(hash, s_tiles[t], (int32_t)s_sizes[t]);
    }
    mnavDestroyBaker(baker);
    printf("TERRAIN_HASH=%016llx\n", (unsigned long long)hash);
    CHECK(hash == TERRAIN_HASH, "the pinned hash");
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "navmesh");
    bool loaded[4][4] = {{false}};
    int32_t most = 0;
    int32_t outCount = 0;
    int32_t ahead = 0;
    int32_t steps = 0;
    for (double a = 16.0; a <= 112.0; a += 8.0, ++steps)
    {
        int32_t count = Stream(navmesh, loaded, a, a);
        most = count > most ? count : most;
        mnavNearest here = Ground(navmesh, a, a);
        double b = a + 12.0 < 124.0 ? a + 12.0 : 124.0;
        mnavNearest next = Ground(navmesh, b, b);
        mnavNearest corner = Ground(navmesh, 124.0, 124.0);
        mnavPath path = {0};
        CHECK(here.polygon.slot != 0 && next.polygon.slot != 0, "ground around the agent");
        CHECK(mnavFindPath(query, navmesh, nullptr, here.polygon, here.point, next.polygon,
                           next.point, &path) == mnav_success,
              "searched ahead");
        ahead += path.end == mnav_pathFound ? 1 : 0;
        if (corner.polygon.slot == 0)
        {
            // The far corner's tile is out: a way toward it ends at the
            // window's edge, not at a wall.
            mnavRay ray;
            mnavPos3 far = {124.0, here.point.y, 124.0};
            CHECK(mnavRaycast(query, navmesh, nullptr, here.polygon, here.point, far, &ray) ==
                          mnav_success &&
                      ray.end == mnav_rayNotLoaded,
                  "a ray stops where tiles are out");
            outCount += 1;
        }
        else
        {
            CHECK(mnavFindPath(query, navmesh, nullptr, here.polygon, here.point, corner.polygon,
                               corner.point, &path) == mnav_success &&
                      path.end == mnav_pathFound,
                  "the corner reached once loaded");
        }
    }
    printf("terrain: %d steps, at most %d tiles loaded, %d with the corner out\n", steps, most,
           outCount);
    CHECK(most == 9 && ahead == steps && outCount > 0 && outCount < steps,
          "a window of nine tiles carried the agent across");
    // Back at the start, the window streams back with the agent and a way
    // within it is found again.
    Stream(navmesh, loaded, 16.0, 16.0);
    mnavNearest start = Ground(navmesh, 16.0, 16.0);
    mnavNearest edge = Ground(navmesh, 60.0, 60.0);
    mnavPath path = {0};
    CHECK(mnavFindPath(query, navmesh, nullptr, start.polygon, start.point, edge.polygon,
                       edge.point, &path) == mnav_success &&
              path.end == mnav_pathFound,
          "within the window");
    mnavDestroyNavmesh(navmesh);
}

enum
{
    FIELD = 64,
    GROUP = 60
};

static mnavAreaType s_field[FIELD * FIELD];

// Whether a place lies in the wall at x 32 to 33, open from y 28 to 36.
static bool InWall(mnavPos2 p)
{
    return p.x >= 32.0 && p.x < 33.0 && (p.y < 28.0 || p.y >= 36.0);
}

// The velocity an agent prefers: toward the center of the next cell the
// field gives its cell, at its speed; slowing into the goal's center.
static mnavPos2 Preferred(const mnavFlowField* field, const mnavAgent* a)
{
    mnavCell cell = {(int32_t)floor(a->position.x), (int32_t)floor(a->position.y)};
    mnavFlow flow = {0};
    if (mnavFlowAt(field, cell, &flow) != mnav_success || !isfinite(flow.cost))
    {
        return (mnavPos2){0, 0};
    }
    mnavPos2 to = {flow.next.x + 0.5, flow.next.y + 0.5};
    if (flow.cost == 0.0)
    {
        to = (mnavPos2){56.5, 32.5};
    }
    double dx = to.x - a->position.x;
    double dy = to.y - a->position.y;
    double length = sqrt(dx * dx + dy * dy);
    double speed = length < a->maxSpeed ? length : a->maxSpeed;
    return length > 1e-9 ? (mnavPos2){dx / length * speed, dy / length * speed} : (mnavPos2){0, 0};
}

static void TestGroup(void)
{
    for (int32_t y = 0; y < FIELD; ++y)
    {
        for (int32_t x = 0; x < FIELD; ++x)
        {
            s_field[y * FIELD + x] = InWall((mnavPos2){x + 0.5, y + 0.5}) ? 0 : 1;
        }
    }
    mnavGrid grid = {s_field, FIELD, FIELD, 1.0f};
    mnavCell goals[25];
    for (int32_t i = 0; i < 25; ++i)
    {
        goals[i] = (mnavCell){54 + i % 5, 30 + i / 5};
    }
    mnavFlowFieldDef fieldDef = mnavDefaultFlowFieldDef();
    fieldDef.cells = FIELD * FIELD;
    mnavFlowField* field = nullptr;
    CHECK(mnavCreateFlowField(&fieldDef, &field) == mnav_success &&
              mnavBuildFlowField(field, &grid, nullptr, goals, 25) == mnav_success,
          "a field to the goal");
    static const mnavPos2 lower[4] = {{32, 0}, {33, 0}, {33, 28}, {32, 28}};
    static const mnavPos2 upper[4] = {{32, 36}, {33, 36}, {33, 64}, {32, 64}};
    const mnavObstacle walls[2] = {{lower, 4, 0.0, {0, 0}, 1}, {upper, 4, 0.0, {0, 0}, 2}};
    mnavAvoidanceDef avoidDef = mnavDefaultAvoidanceDef();
    avoidDef.limits.agents = GROUP;
    avoidDef.neighborDistance = 3.0;
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&avoidDef, &avoidance) == mnav_success, "avoidance");
    static mnavAgent agents[GROUP];
    for (int32_t i = 0; i < GROUP; ++i)
    {
        agents[i] = (mnavAgent){{6.0 + 1.6 * (i % 6), 24.0 + 1.6 * (i / 6)},
                                {0, 0},
                                {0, 0},
                                0.4,
                                1.5,
                                1.0,
                                (uint64_t)i + 1};
    }
    static mnavPos2 velocities[GROUP];
    double closest = 1e9;
    int32_t walled = 0;
    const double step = 0.1;
    for (int32_t tick = 0; tick < 900; ++tick)
    {
        for (int32_t i = 0; i < GROUP; ++i)
        {
            agents[i].preferred = Preferred(field, &agents[i]);
        }
        CHECK(mnavAvoid(avoidance, agents, GROUP, walls, 2, step, velocities) == mnav_success,
              "avoided");
        for (int32_t i = 0; i < GROUP; ++i)
        {
            agents[i].velocity = velocities[i];
            agents[i].position.x += velocities[i].x * step;
            agents[i].position.y += velocities[i].y * step;
            walled += InWall(agents[i].position) ? 1 : 0;
        }
        for (int32_t i = 0; i < GROUP; ++i)
        {
            for (int32_t j = i + 1; j < GROUP; ++j)
            {
                double dx = agents[i].position.x - agents[j].position.x;
                double dy = agents[i].position.y - agents[j].position.y;
                double d = sqrt(dx * dx + dy * dy);
                closest = d < closest ? d : closest;
            }
        }
    }
    int32_t arrived = 0;
    uint64_t hash = MNAV_HASH_INIT;
    for (int32_t i = 0; i < GROUP; ++i)
    {
        double dx = agents[i].position.x - 56.5;
        double dy = agents[i].position.y - 32.5;
        arrived += dx * dx + dy * dy < 8.0 * 8.0 ? 1 : 0;
        hash = mnavHash64(hash, &agents[i].position, (int32_t)sizeof(mnavPos2));
    }
    printf("group: %d of %d arrived, closest pair %.3f m apart, %d places in the wall; "
           "GROUP_HASH=%016llx\n",
           arrived, GROUP, closest, walled, (unsigned long long)hash);
    CHECK(arrived >= GROUP * 9 / 10, "the group through the gap to the goal");
    CHECK(closest > 0.8 * 0.95 && walled == 0, "no agents overlapping or in the wall");
    CHECK(hash == GROUP_HASH, "the pinned hash");
    mnavDestroyAvoidance(avoidance);
    mnavDestroyFlowField(field);
}

int main(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "query");
    TestBuilding(query);
    TestCollapsingBridge(query);
    TestStreamedTerrain(query);
    TestGroup();
    mnavDestroyQuery(query);
    return s_failures == 0 ? 0 : 1;
}
