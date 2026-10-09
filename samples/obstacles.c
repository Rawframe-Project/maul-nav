// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Obstacles on a floor: a floor of 64 m by 64 m is baked into four tiles
// through a tile cache and loaded into a navmesh of the dynamic tier,
// and eight agents cross it from west to east. Then crates come and go:
// a barricade falls across the floor, leaving a gap at its north end; a
// crate drops in one tile's middle; later the barricade is lifted. Each
// change rebuilds only the tiles the changed crates reach, from the
// cache, with every crate standing on them, and commits them together;
// the agents whose corridors the change breaks plan again. Prints each
// change and the walk; returns 0 when every agent arrives without
// standing in a crate.

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

enum
{
    TILES = 2,
    AGENTS = 8,
    // The barricade's crates, then one more.
    BARRICADE = 27,
    CRATES = BARRICADE + 1,
    CORRIDOR = 256,
    STEPS = 600,
    TILE_ROOM = 1 << 18
};

static const float TILE_SIZE = 32.0f;
// A step's seconds and the agents' speed: half a meter a step.
static const double STEP = 0.1;
static const double SPEED = 5.0;

// A crate: a square of 2 m on the ground, standing or not.
typedef struct Crate
{
    mnavVec2 ring[4];
    bool standing;
} Crate;

static Crate s_crates[CRATES];

static void Check(mnavResult result, const char* what)
{
    if (result != mnav_success)
    {
        fprintf(stderr, "%s: %s\n", what, mnavResultName(result));
        exit(1);
    }
}

static void PlaceCrates(void)
{
    // The barricade: x from 30 to 32, z from 0 to 54; the gap is north.
    for (int32_t k = 0; k < BARRICADE; ++k)
    {
        float z = 2.0f * (float)k;
        s_crates[k].ring[0] = (mnavVec2){30.0f, z};
        s_crates[k].ring[1] = (mnavVec2){32.0f, z};
        s_crates[k].ring[2] = (mnavVec2){32.0f, z + 2.0f};
        s_crates[k].ring[3] = (mnavVec2){30.0f, z + 2.0f};
    }
    // A crate in the middle of the tile at (1, 0).
    Crate* one = &s_crates[BARRICADE];
    one->ring[0] = (mnavVec2){47.0f, 9.0f};
    one->ring[1] = (mnavVec2){49.0f, 9.0f};
    one->ring[2] = (mnavVec2){49.0f, 11.0f};
    one->ring[3] = (mnavVec2){47.0f, 11.0f};
}

// The obstacles standing: each crate as an exclude volume from below
// the floor to above an agent.
static int32_t Standing(mnavBakeVolume* obstacles)
{
    int32_t count = 0;
    for (int32_t k = 0; k < CRATES; ++k)
    {
        if (s_crates[k].standing)
        {
            obstacles[count++] =
                (mnavBakeVolume){s_crates[k].ring, 4, -1.0f, 3.0f, mnav_volumeExclude, 0};
        }
    }
    return count;
}

// Marks the tiles a crate reaches: its square widened by a tile's border,
// the agent's radius in whole cells and three cells more, which the bake
// reads past each tile's edge.
static void Reach(const mnavBakeDef* def, const Crate* crate, bool reached[TILES][TILES])
{
    float border = (ceilf(def->agent.radius / def->cellSize) + 3.0f) * def->cellSize;
    float low[2] = {crate->ring[0].x - border, crate->ring[0].y - border};
    float high[2] = {crate->ring[2].x + border, crate->ring[2].y + border};
    for (int32_t z = 0; z < TILES; ++z)
    {
        for (int32_t x = 0; x < TILES; ++x)
        {
            float x0 = (float)x * TILE_SIZE;
            float z0 = (float)z * TILE_SIZE;
            reached[z][x] = reached[z][x] || (low[0] <= x0 + TILE_SIZE && high[0] >= x0 &&
                                              low[1] <= z0 + TILE_SIZE && high[1] >= z0);
        }
    }
}

typedef struct World
{
    mnavBakeDef def;
    mnavBaker* baker;
    mnavTileCache* cache;
    mnavNavmesh* navmesh;
    uint8_t* bytes;
} World;

static void Stage(World* w)
{
    size_t size = 0;
    Check(mnavCopyBakedTile(w->baker, w->bytes, TILE_ROOM, &size), "copy");
    Check(mnavStageTile(w->navmesh, w->bytes, size).result, "stage");
}

static void Load(World* w)
{
    static const mnavVec3 vertices[4] = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 64.0f}, {64.0f, 0.0f, 64.0f}, {64.0f, 0.0f, 0.0f}};
    static const int32_t indices[6] = {0, 1, 2, 0, 2, 3};
    const mnavTriangleMesh floor = {vertices, 4, indices, 2, NULL};
    const mnavBakeInput input = {&floor, 1, NULL, 0, NULL, 0, NULL};
    w->def = mnavDefaultBakeDef();
    // Tiles are replaced at runtime: the dynamic tier.
    w->def.tier = mnav_tierDynamic;
    mnavTileCacheDef cacheDef = mnavDefaultTileCacheDef();
    Check(mnavCreateBaker(&w->def, &w->baker).result, "baker");
    Check(mnavCreateTileCache(&cacheDef, &w->cache), "cache");
    Check(mnavCreateNavmesh(&w->def, &w->navmesh).result, "navmesh");
    w->bytes = malloc(TILE_ROOM);
    for (int32_t t = 0; t < TILES * TILES; ++t)
    {
        Check(mnavBakeTileCached(w->baker, w->cache, &input, t % TILES, t / TILES, NULL),
              "cached bake");
        Stage(w);
    }
    Check(mnavCommit(w->navmesh), "commit");
}

// Sets crates first to last standing or not, and rebuilds the tiles they
// reach with every crate standing, committed together.
static void Change(World* w, int32_t first, int32_t last, bool standing)
{
    bool reached[TILES][TILES] = {{false}};
    for (int32_t k = first; k <= last; ++k)
    {
        s_crates[k].standing = standing;
        Reach(&w->def, &s_crates[k], reached);
    }
    static mnavBakeVolume obstacles[CRATES];
    int32_t count = Standing(obstacles);
    int32_t rebuilt = 0;
    for (int32_t z = 0; z < TILES; ++z)
    {
        for (int32_t x = 0; x < TILES; ++x)
        {
            if (reached[z][x])
            {
                Check(mnavRebuildTile(w->baker, w->cache, x, z, obstacles, count, NULL), "rebuild");
                Stage(w);
                rebuilt += 1;
            }
        }
    }
    Check(mnavCommit(w->navmesh), "commit");
    printf("%s %d crate%s: %d tile%s rebuilt, %d crate%s standing\n",
           standing ? "dropped" : "lifted", last - first + 1, last > first ? "s" : "", rebuilt,
           rebuilt > 1 ? "s" : "", count, count == 1 ? "" : "s");
}

typedef struct Agent
{
    mnavPolygonId buffer[CORRIDOR];
    mnavCorridor corridor;
    bool arrived;
} Agent;

static Agent s_agents[AGENTS];

static const mnavVec3 BOX = {1.0f, 2.0f, 1.0f};

static void Start(World* w, mnavQuery* query)
{
    for (int32_t a = 0; a < AGENTS; ++a)
    {
        // The fifth row walks the seam between the tiles at z = 32 m.
        double z = 4.0 + 7.0 * a;
        mnavNearest start;
        Check(mnavFindNearest(w->navmesh, NULL, (mnavPos3){4.0, 0.0, z}, BOX, &start), "start");
        Agent* agent = &s_agents[a];
        Check(mnavResetCorridor(&agent->corridor, agent->buffer, CORRIDOR, start.polygon,
                                start.point),
              "reset");
        agent->corridor.target = (mnavPos3){60.0, 0.0, z};
        mnavPath path;
        Check(mnavReplanCorridor(query, w->navmesh, NULL, &agent->corridor, BOX, &path), "plan");
    }
}

// Plans again the corridors a commit broke; returns how many.
static int32_t Replan(World* w, mnavQuery* query)
{
    int32_t replanned = 0;
    for (int32_t a = 0; a < AGENTS; ++a)
    {
        Agent* agent = &s_agents[a];
        int32_t valid = 0;
        Check(mnavCheckCorridor(w->navmesh, NULL, &agent->corridor, &valid), "check");
        if (!agent->arrived && valid < agent->corridor.count)
        {
            mnavPath path;
            Check(mnavReplanCorridor(query, w->navmesh, NULL, &agent->corridor, BOX, &path),
                  "replan");
            replanned += 1;
        }
    }
    return replanned;
}

// Whether a point stands in a standing crate.
static bool InCrate(mnavPos3 p)
{
    for (int32_t k = 0; k < CRATES; ++k)
    {
        const Crate* c = &s_crates[k];
        if (c->standing && p.x > (double)c->ring[0].x && p.x < (double)c->ring[2].x &&
            p.z > (double)c->ring[0].y && p.z < (double)c->ring[2].y)
        {
            return true;
        }
    }
    return false;
}

// Moves each agent a step of STEP seconds at the velocity mnavSteer
// gives along its corridor; returns how many stood in a crate.
static int32_t Walk(World* w, mnavQuery* query)
{
    mnavSteerDef steer = mnavDefaultSteerDef();
    steer.maxSpeed = (float)SPEED;
    steer.arriveDistance = 0.01f;
    int32_t inside = 0;
    for (int32_t a = 0; a < AGENTS; ++a)
    {
        Agent* agent = &s_agents[a];
        if (agent->arrived)
        {
            continue;
        }
        mnavCorners corners;
        mnavSteering steering;
        Check(mnavCorridorCorners(query, w->navmesh, &agent->corridor, &corners), "corners");
        Check(mnavSteer(&corners, &steer, &steering), "steer");
        if (steering.state == mnav_steerArrived)
        {
            agent->arrived = true;
            continue;
        }
        mnavPos3 at = agent->corridor.position;
        mnavPos3 wanted = {at.x + steering.velocity.x * STEP, at.y,
                           at.z + steering.velocity.z * STEP};
        Check(mnavMoveCorridor(query, w->navmesh, NULL, &agent->corridor, wanted, NULL), "move");
        inside += InCrate(agent->corridor.position) ? 1 : 0;
    }
    return inside;
}

int main(void)
{
    World w = {0};
    PlaceCrates();
    Load(&w);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    Start(&w, query);
    int32_t inside = 0;
    int32_t arrived = 0;
    int32_t step = 0;
    for (; step < STEPS && arrived < AGENTS; ++step)
    {
        if (step == 20 || step == 40 || step == 120)
        {
            if (step == 20)
            {
                Change(&w, 0, BARRICADE - 1, true);
            }
            else if (step == 40)
            {
                Change(&w, BARRICADE, BARRICADE, true);
            }
            else
            {
                Change(&w, 0, BARRICADE - 1, false);
            }
            printf("  step %d: %d agents planned again\n", step, Replan(&w, query));
        }
        inside += Walk(&w, query);
        arrived = 0;
        for (int32_t a = 0; a < AGENTS; ++a)
        {
            arrived += s_agents[a].arrived ? 1 : 0;
        }
    }
    printf("%d of %d agents arrived in %d steps; %d steps in a crate\n", arrived, AGENTS, step,
           inside);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(w.navmesh);
    mnavDestroyTileCache(w.cache);
    mnavDestroyBaker(w.baker);
    free(w.bytes);
    return arrived == AGENTS && inside == 0 ? 0 : 1;
}
