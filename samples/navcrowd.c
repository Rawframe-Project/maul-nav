// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A crowd on a navmesh, the whole way Detour's crowd goes: a room of
// 32 m by 32 m with four pillars is baked into one tile, and two groups
// of eight agents cross it from opposite walls. Each step, every agent's
// corridor gives its corners, mnavSteer the velocity it wants, mnavAvoid
// a velocity clear of the others and of the pillars, the host limits the
// change by an acceleration and moves the agent along the surface with
// mnavMoveCorridor. Prints how close any two came; returns 0 when every
// agent arrives with no two overlapping by more than 2 cm: an agent
// sliding along a pillar does not go quite where avoidance planned.

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

enum
{
    AGENTS = 16,
    PILLARS = 4,
    CORRIDOR = 256,
    STEPS = 1200,
    TILE_ROOM = 1 << 18
};

static const double STEP = 0.1;
static const double RADIUS = 0.4;
static const double SPEED = 1.5;
static const double ACCELERATION = 8.0;

// The pillars' centers; each is 2 m square.
static const double s_pillars[PILLARS][2] = {{12, 12}, {20, 12}, {12, 20}, {20, 20}};

static void Check(mnavResult result, const char* what)
{
    if (result != mnav_success)
    {
        fprintf(stderr, "%s: %s\n", what, mnavResultName(result));
        exit(1);
    }
}

// The floor, and each pillar as a box 2 m tall.
static mnavVec3 s_vertices[4 + 8 * PILLARS];
static int32_t s_indices[6 + 36 * PILLARS];

static mnavTriangleMesh Room(void)
{
    const mnavVec3 floor[4] = {{0, 0, 0}, {0, 0, 32}, {32, 0, 32}, {32, 0, 0}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    const int32_t faces[36] = {0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                               2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
    for (int32_t k = 0; k < 4; ++k)
    {
        s_vertices[k] = floor[k];
    }
    for (int32_t k = 0; k < 6; ++k)
    {
        s_indices[k] = quad[k];
    }
    for (int32_t p = 0; p < PILLARS; ++p)
    {
        mnavVec3* v = &s_vertices[4 + 8 * p];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] =
                (mnavVec3){(float)s_pillars[p][0] + ((k & 1) ? 1.0f : -1.0f), (k & 4) ? 2.0f : 0.0f,
                           (float)s_pillars[p][1] + ((k & 2) ? 1.0f : -1.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * p + k] = 4 + 8 * p + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, 4 + 8 * PILLARS, s_indices, 2 + 12 * PILLARS, NULL};
}

static mnavNavmesh* Load(void)
{
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = NULL;
    mnavNavmesh* navmesh = NULL;
    Check(mnavCreateBaker(&def, &baker).result, "baker");
    Check(mnavCreateNavmesh(&def, &navmesh).result, "navmesh");
    mnavTriangleMesh room = Room();
    uint8_t* bytes = malloc(TILE_ROOM);
    size_t size = 0;
    Check(mnavBakeTile(baker, &room, 1, 0, 0, NULL), "bake");
    Check(mnavCopyBakedTile(baker, bytes, TILE_ROOM, &size), "copy");
    Check(mnavStageTile(navmesh, bytes, size).result, "stage");
    Check(mnavCommit(navmesh), "commit");
    free(bytes);
    mnavDestroyBaker(baker);
    return navmesh;
}

typedef struct Walker
{
    mnavPolygonId buffer[CORRIDOR];
    mnavCorridor corridor;
    bool arrived;
} Walker;

static Walker s_walkers[AGENTS];
static mnavAgent s_agents[AGENTS];

static void Start(mnavQuery* query, const mnavNavmesh* navmesh)
{
    const mnavVec3 box = {1.0f, 2.0f, 1.0f};
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        // Group one from the west wall, group two from the east, in rows
        // a meter and a half apart.
        double z = 10.75 + 1.5 * (double)(i % 8);
        double x = i < 8 ? 2.0 : 30.0;
        mnavNearest start;
        mnavNearest end;
        Check(mnavFindNearest(navmesh, NULL, (mnavPos3){x, 0.0, z}, box, &start), "start");
        Check(mnavFindNearest(navmesh, NULL, (mnavPos3){32.0 - x, 0.0, z}, box, &end), "end");
        Walker* w = &s_walkers[i];
        mnavPath path;
        Check(mnavFindPath(query, navmesh, NULL, start.polygon, start.point, end.polygon, end.point,
                           &path),
              "path");
        Check(mnavResetCorridor(&w->corridor, w->buffer, CORRIDOR, start.polygon, start.point),
              "reset");
        Check(mnavSetCorridor(&w->corridor, &path), "corridor");
        s_agents[i] = (mnavAgent){
            {start.point.x, start.point.z}, {0, 0}, {0, 0}, RADIUS, SPEED, 1.0, (uint64_t)i + 1};
    }
}

// Each agent's preferred velocity from its corners; returns how many
// have arrived.
static int32_t Steer(mnavQuery* query, const mnavNavmesh* navmesh)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    def.maxSpeed = (float)SPEED;
    def.arriveDistance = 0.05f;
    int32_t arrived = 0;
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        mnavCorners corners;
        mnavSteering steering;
        Check(mnavCorridorCorners(query, navmesh, &s_walkers[i].corridor, &corners), "corners");
        Check(mnavSteer(&corners, &def, &steering), "steer");
        s_walkers[i].arrived = steering.state == mnav_steerArrived;
        arrived += s_walkers[i].arrived ? 1 : 0;
        s_agents[i].preferred = (mnavPos2){steering.velocity.x, steering.velocity.z};
    }
    return arrived;
}

// Moves each agent at its avoided velocity, its change limited by the
// acceleration, along the surface.
static void Move(mnavQuery* query, const mnavNavmesh* navmesh, const mnavPos2* velocities)
{
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        mnavAgent* a = &s_agents[i];
        double dx = velocities[i].x - a->velocity.x;
        double dz = velocities[i].y - a->velocity.y;
        double change = sqrt(dx * dx + dz * dz);
        double most = ACCELERATION * STEP;
        double t = change > most ? most / change : 1.0;
        a->velocity = (mnavPos2){a->velocity.x + dx * t, a->velocity.y + dz * t};
        mnavCorridor* c = &s_walkers[i].corridor;
        mnavPos3 wanted = {c->position.x + a->velocity.x * STEP, c->position.y,
                           c->position.z + a->velocity.y * STEP};
        Check(mnavMoveCorridor(query, navmesh, NULL, c, wanted, NULL), "move");
        a->position = (mnavPos2){c->position.x, c->position.z};
    }
}

// The smallest gap between two agents.
static double Closest(void)
{
    double closest = (double)INFINITY;
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        for (int32_t j = i + 1; j < AGENTS; ++j)
        {
            double dx = s_agents[j].position.x - s_agents[i].position.x;
            double dz = s_agents[j].position.y - s_agents[i].position.y;
            double gap = sqrt(dx * dx + dz * dz) - 2.0 * RADIUS;
            closest = gap < closest ? gap : closest;
        }
    }
    return closest;
}

int main(void)
{
    mnavNavmesh* navmesh = Load();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    mnavAvoidanceDef avoidanceDef = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = NULL;
    Check(mnavCreateAvoidance(&avoidanceDef, &avoidance), "avoidance");
    // The pillars as polygons, counterclockwise.
    static mnavPos2 squares[PILLARS][4];
    mnavObstacle pillars[PILLARS];
    for (int32_t p = 0; p < PILLARS; ++p)
    {
        double x = s_pillars[p][0];
        double z = s_pillars[p][1];
        squares[p][0] = (mnavPos2){x - 1, z - 1};
        squares[p][1] = (mnavPos2){x + 1, z - 1};
        squares[p][2] = (mnavPos2){x + 1, z + 1};
        squares[p][3] = (mnavPos2){x - 1, z + 1};
        pillars[p] = (mnavObstacle){squares[p], 4, 0.0, {0, 0}, (uint64_t)p + 1};
    }
    Start(query, navmesh);
    double closest = (double)INFINITY;
    int32_t arrived = 0;
    int32_t step = 0;
    for (; step < STEPS; ++step)
    {
        arrived = Steer(query, navmesh);
        if (arrived == AGENTS)
        {
            break;
        }
        static mnavPos2 velocities[AGENTS];
        Check(mnavAvoid(avoidance, s_agents, AGENTS, pillars, PILLARS, STEP, velocities), "avoid");
        Move(query, navmesh, velocities);
        double gap = Closest();
        closest = gap < closest ? gap : closest;
    }
    printf("%d of %d agents arrived in %.1f s; the closest two came %.3f m apart\n", arrived,
           AGENTS, step * STEP, closest);
    mnavDestroyAvoidance(avoidance);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return arrived == AGENTS && closest > -0.02 ? 0 : 1;
}
