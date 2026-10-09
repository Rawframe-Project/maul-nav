// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A crowd on a navmesh, the whole way Detour's crowd goes: a room of
// 32 m by 32 m split by two blocks around a corridor 4 m wide is baked
// into one tile, and two groups of eight agents cross it from opposite
// walls, meeting head on in the corridor. Each step, every agent's
// corridor gives its corners, mnavSteer the velocity it wants, mnavAvoid
// a velocity clear of the others and of the walls near them, which
// mnavFindWalls finds on the navmesh, the host limits the change by an
// acceleration and moves the agent along the surface with
// mnavMoveCorridor. Prints how close any two came and how far the moves
// fell short of where agents meant to go, pressed against walls; returns
// 0 when every agent arrives, no two overlapping by more than 5 cm (in a
// packed corridor avoidance keeps clear of the walls first and gives way
// between agents) and the moves falling short by under half a meter in
// all. Without the walls, the agents press over 20 m into them.

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
    BLOCKS = 2,
    CORRIDOR = 256,
    STEPS = 1200,
    TILE_ROOM = 1 << 18,
    // The nearest walls each agent keeps.
    WALLS = 8
};

static const double STEP = 0.1;
static const double RADIUS = 0.4;
static const double SPEED = 1.5;
static const double ACCELERATION = 8.0;
// How far an agent looks for walls; it looks again when it has moved a
// quarter of that from where it last looked, as Detour's crowd does.
static const double WALL_RANGE = 2.0;

// The blocks from (x0, z0) to (x1, z1), the corridor between them.
static const double s_blocks[BLOCKS][4] = {{10, 0, 22, 14}, {10, 18, 22, 32}};

static void Check(mnavResult result, const char* what)
{
    if (result != mnav_success)
    {
        fprintf(stderr, "%s: %s\n", what, mnavResultName(result));
        exit(1);
    }
}

// The floor, and each block as a box 2 m tall.
static mnavVec3 s_vertices[4 + 8 * BLOCKS];
static int32_t s_indices[6 + 36 * BLOCKS];

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
    for (int32_t p = 0; p < BLOCKS; ++p)
    {
        mnavVec3* v = &s_vertices[4 + 8 * p];
        for (int32_t k = 0; k < 8; ++k)
        {
            v[k] = (mnavVec3){(float)s_blocks[p][(k & 1) ? 2 : 0], (k & 4) ? 2.0f : 0.0f,
                              (float)s_blocks[p][(k & 2) ? 3 : 1]};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + 36 * p + k] = 4 + 8 * p + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, 4 + 8 * BLOCKS, s_indices, 2 + 12 * BLOCKS, NULL};
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
    mnavWallSegment walls[WALLS];
    int32_t wallCount;
    mnavPos3 wallsAt;
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
        w->wallCount = -1;
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

// Finds each agent's walls again when it has moved a quarter of the range
// from where it last looked; a full buffer keeps the nearest.
static void FindWalls(mnavQuery* query, const mnavNavmesh* navmesh)
{
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        Walker* w = &s_walkers[i];
        double dx = w->corridor.position.x - w->wallsAt.x;
        double dz = w->corridor.position.z - w->wallsAt.z;
        double quarter = WALL_RANGE * 0.25;
        if (w->wallCount >= 0 && dx * dx + dz * dz < quarter * quarter)
        {
            continue;
        }
        mnavWallsFound found;
        mnavResult result =
            mnavFindWalls(query, navmesh, NULL, w->corridor.polygons[0], w->corridor.position,
                          WALL_RANGE, w->walls, WALLS, &found);
        Check(result == mnav_errorCapacity ? mnav_success : result, "walls");
        w->wallCount = found.count < WALLS ? found.count : WALLS;
        w->wallsAt = w->corridor.position;
    }
}

// Every agent's walls as avoidance's segments, each once, moved out by
// the agent's radius: the navmesh keeps an agent's center a radius from
// the real walls already, and avoidance keeps an agent a radius from every
// obstacle. Walls of no length on the ground are left out, as avoidance
// refuses them. Returns how many.
static int32_t GatherWalls(mnavObstacle* obstacles, mnavPos2 (*ends)[2])
{
    int32_t count = 0;
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        for (int32_t k = 0; k < s_walkers[i].wallCount; ++k)
        {
            const mnavWallSegment* wall = &s_walkers[i].walls[k];
            if (wall->normal.x == 0.0 && wall->normal.z == 0.0)
            {
                continue;
            }
            mnavPos2 a = {wall->start.x - wall->normal.x * RADIUS,
                          wall->start.z - wall->normal.z * RADIUS};
            mnavPos2 b = {wall->end.x - wall->normal.x * RADIUS,
                          wall->end.z - wall->normal.z * RADIUS};
            bool seen = false;
            for (int32_t o = 0; o < count && !seen; ++o)
            {
                seen = ends[o][0].x == a.x && ends[o][0].y == a.y && ends[o][1].x == b.x &&
                       ends[o][1].y == b.y;
            }
            if (!seen)
            {
                ends[count][0] = a;
                ends[count][1] = b;
                obstacles[count] = (mnavObstacle){ends[count], 2, 0.0, {0, 0}, (uint64_t)count + 1};
                count += 1;
            }
        }
    }
    return count;
}

// Moves each agent at its avoided velocity, its change limited by the
// acceleration, along the surface; returns how far the moves fell short
// of where the agents meant to go, pressed against walls.
static double Move(mnavQuery* query, const mnavNavmesh* navmesh, const mnavPos2* velocities)
{
    double pressed = 0.0;
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
        pressed += hypot(wanted.x - c->position.x, wanted.z - c->position.z);
        a->position = (mnavPos2){c->position.x, c->position.z};
    }
    return pressed;
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
    static mnavObstacle walls[AGENTS * WALLS];
    static mnavPos2 ends[AGENTS * WALLS][2];
    int32_t mostWalls = 0;
    double pressed = 0.0;
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
        FindWalls(query, navmesh);
        int32_t wallCount = GatherWalls(walls, ends);
        mostWalls = wallCount > mostWalls ? wallCount : mostWalls;
        static mnavPos2 velocities[AGENTS];
        Check(mnavAvoid(avoidance, s_agents, AGENTS, walls, wallCount, STEP, velocities), "avoid");
        pressed += Move(query, navmesh, velocities);
        double gap = Closest();
        closest = gap < closest ? gap : closest;
    }
    printf("%d of %d agents arrived in %.1f s; the closest two came %.3f m apart; their moves "
           "fell %.2f m short against walls; at most %d walls at once\n",
           arrived, AGENTS, step * STEP, closest, pressed, mostWalls);
    mnavDestroyAvoidance(avoidance);
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
    return arrived == AGENTS && closest > -0.05 && pressed < 0.5 ? 0 : 1;
}
