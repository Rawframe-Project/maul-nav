// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fliers among pillars: a ground of 32 m by 32 m with four pillars 12 m
// tall is baked into a flight volume of four tiles, and two streams of
// twelve fliers cross it at right angles at different heights. Each
// flier gets a path through the volume and follows it, a point at a
// time, while avoidance in space keeps the fliers apart and a raycast
// each step keeps them in open space. Prints the paths' lengths and how
// close any two fliers came; returns 0 when every flier arrives without
// overlapping another or leaving open space.

#include "maul-nav/flight.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    FLIERS = 24,
    MOST_POINTS = 64,
    STEPS = 1200,
    TILE_ROOM = 1 << 18
};

static const double STEP = 0.1;
static const double RADIUS = 0.4;
static const double SPEED = 2.0;

// The ground, then four pillars of 2 m by 2 m.
static mnavVec3 s_vertices[4 + 4 * 8];
static int32_t s_indices[6 + 4 * 36];

static void Check(mnavResult result, const char* what)
{
    if (result != mnav_success)
    {
        fprintf(stderr, "%s: %s\n", what, mnavResultName(result));
        exit(1);
    }
}

static mnavTriangleMesh World(void)
{
    static const float pillars[4][2] = {
        {11.0f, 11.0f}, {19.0f, 11.0f}, {11.0f, 19.0f}, {19.0f, 19.0f}};
    static const int32_t faces[36] = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                                      2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    const mnavVec3 ground[4] = {{0, 0, 0}, {0, 0, 32}, {32, 0, 32}, {32, 0, 0}};
    const int32_t quad[6] = {0, 1, 2, 0, 2, 3};
    memcpy(s_vertices, ground, sizeof(ground));
    memcpy(s_indices, quad, sizeof(quad));
    for (int32_t p = 0; p < 4; ++p)
    {
        for (int32_t c = 0; c < 8; ++c)
        {
            s_vertices[4 + p * 8 + c] = (mnavVec3){pillars[p][0] + ((c & 1) != 0 ? 2.0f : 0.0f),
                                                   (c & 2) != 0 ? 12.0f : 0.0f,
                                                   pillars[p][1] + ((c & 4) != 0 ? 2.0f : 0.0f)};
        }
        for (int32_t k = 0; k < 36; ++k)
        {
            s_indices[6 + p * 36 + k] = 4 + p * 8 + faces[k];
        }
    }
    return (mnavTriangleMesh){s_vertices, 4 + 4 * 8, s_indices, 2 + 4 * 12, NULL};
}

// Bakes the four tiles of 16 m, half-meter voxels up to 16 m, and loads
// them into a volume.
static mnavFlightVolume* BakeAndLoad(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.voxelSize = 0.5f;
    def.ceiling = 16.0f;
    def.radius = (float)RADIUS + 0.1f;
    mnavFlightBaker* baker = NULL;
    mnavFlightVolume* volume = NULL;
    Check(mnavCreateFlightBaker(&def, &baker).result, "baker");
    Check(mnavCreateFlightVolume(&def, &volume).result, "volume");
    mnavTriangleMesh mesh = World();
    mnavBakeInput input = {&mesh, 1, NULL, 0, NULL, 0, NULL};
    uint8_t* bytes = malloc(TILE_ROOM);
    for (int32_t t = 0; t < 4 && bytes != NULL; ++t)
    {
        size_t size = 0;
        Check(mnavBakeFlightTile(baker, &input, t % 2, t / 2, NULL), "bake");
        Check(mnavCopyFlightTile(baker, bytes, TILE_ROOM, &size), "copy");
        Check(mnavStageFlightTile(volume, bytes, size).result, "stage");
    }
    Check(bytes != NULL ? mnavCommitFlight(volume) : mnav_errorCapacity, "commit");
    free(bytes);
    mnavDestroyFlightBaker(baker);
    return volume;
}

// A flier's path, copied out of the query context, and the point it
// heads for.
typedef struct Route
{
    mnavPos3 points[MOST_POINTS];
    int32_t count;
    int32_t next;
} Route;

static double FindRoute(mnavQuery* query, const mnavFlightVolume* volume, mnavPos3 from,
                        mnavPos3 to, Route* route)
{
    bool found = false;
    Check(mnavFindNearestFlightPoint(volume, from, 2.0f, &from, &found), "nearest");
    Check(mnavFindNearestFlightPoint(volume, to, 2.0f, &to, &found), "nearest");
    mnavFlightPath path;
    Check(mnavFindFlightPath(query, volume, from, to, &path), "path");
    if (path.end != mnav_pathFound || path.pointCount > MOST_POINTS)
    {
        fprintf(stderr, "no path\n");
        exit(1);
    }
    memcpy(route->points, path.points, (size_t)path.pointCount * sizeof(mnavPos3));
    route->count = path.pointCount;
    route->next = 1;
    return path.length;
}

static double Distance(mnavPos3 a, mnavPos3 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = b.z - a.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Heads each flier for its route's next point, moving on to the one
// after once within a meter of it, and slowing into the last.
static void Prefer(mnavAgent3D* fliers, Route* routes)
{
    for (int32_t i = 0; i < FLIERS; ++i)
    {
        Route* r = &routes[i];
        mnavPos3 at = fliers[i].position;
        while (r->next + 1 < r->count && Distance(at, r->points[r->next]) < 1.0)
        {
            r->next += 1;
        }
        mnavPos3 aim = r->points[r->next];
        double d = Distance(at, aim);
        double speed = d < SPEED * STEP ? d / STEP : SPEED;
        double scale = d > 0.0 ? speed / d : 0.0;
        fliers[i].preferred =
            (mnavPos3){(aim.x - at.x) * scale, (aim.y - at.y) * scale, (aim.z - at.z) * scale};
    }
}

// Whether a flier may go straight from one point to another.
static bool Clear(const mnavFlightVolume* volume, mnavPos3 from, mnavPos3 to)
{
    mnavFlightHit hit;
    Check(mnavFlightRaycast(volume, from, to, &hit), "raycast");
    return hit.stop == mnav_flightClear;
}

// Moves a flier by its velocity for a step as far as the volume lets it:
// avoidance knows the other fliers, not the pillars, and may push a
// flier toward one. A step whose ray meets a voxel the flier may not
// enter slides instead: its parts along X, Y and Z are taken one at a
// time, each only when its own ray is clear.
static void Move(const mnavFlightVolume* volume, mnavAgent3D* flier, mnavPos3 velocity)
{
    mnavPos3 from = flier->position;
    mnavPos3 to = {from.x + velocity.x * STEP, from.y + velocity.y * STEP,
                   from.z + velocity.z * STEP};
    if (!Clear(volume, from, to))
    {
        to = from;
        mnavPos3 x = {from.x + velocity.x * STEP, to.y, to.z};
        to = Clear(volume, to, x) ? x : to;
        mnavPos3 y = {to.x, from.y + velocity.y * STEP, to.z};
        to = Clear(volume, to, y) ? y : to;
        mnavPos3 z = {to.x, to.y, from.z + velocity.z * STEP};
        to = Clear(volume, to, z) ? z : to;
    }
    flier->velocity =
        (mnavPos3){(to.x - from.x) / STEP, (to.y - from.y) / STEP, (to.z - from.z) / STEP};
    flier->position = to;
}

int main(void)
{
    mnavFlightVolume* volume = BakeAndLoad();
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    mnavQuery* query = NULL;
    Check(mnavCreateQuery(&queryDef, &query), "query");
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    def.neighborDistance = 4.0;
    mnavAvoidance* avoidance = NULL;
    Check(mnavCreateAvoidance(&def, &avoidance), "avoidance");
    // One stream flies west to east 3 m up, the other south to north
    // 4 m up, through the pillars.
    static mnavAgent3D fliers[FLIERS];
    static Route routes[FLIERS];
    double length = 0.0;
    for (int32_t i = 0; i < FLIERS; ++i)
    {
        double lane = 4.0 + (double)(i % 12) * 2.0;
        mnavPos3 from = i < 12 ? (mnavPos3){1.5, 3.0, lane} : (mnavPos3){lane, 4.0, 1.5};
        mnavPos3 to = i < 12 ? (mnavPos3){30.5, 3.0, lane} : (mnavPos3){lane, 4.0, 30.5};
        length += FindRoute(query, volume, from, to, &routes[i]);
        fliers[i] =
            (mnavAgent3D){routes[i].points[0], {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, RADIUS, SPEED, 1.0,
                          (uint64_t)i + 1};
    }
    printf("%d paths found, %.1f m in all\n", FLIERS, length);
    double closest = (double)INFINITY;
    int32_t arrived = 0;
    int32_t blocked = 0;
    for (int32_t step = 0; step < STEPS && arrived < FLIERS; ++step)
    {
        Prefer(fliers, routes);
        mnavPos3 velocities[FLIERS];
        Check(mnavAvoid3D(avoidance, fliers, FLIERS, NULL, 0, STEP, velocities), "avoid");
        arrived = 0;
        for (int32_t i = 0; i < FLIERS; ++i)
        {
            Move(volume, &fliers[i], velocities[i]);
            arrived += Distance(fliers[i].position, routes[i].points[routes[i].count - 1]) < 0.05;
            bool open = false;
            Check(mnavIsFlightOpen(volume, fliers[i].position, &open), "open");
            blocked += open ? 0 : 1;
        }
        for (int32_t i = 0; i < FLIERS; ++i)
        {
            for (int32_t j = i + 1; j < FLIERS; ++j)
            {
                double gap = Distance(fliers[i].position, fliers[j].position) - 2.0 * RADIUS;
                closest = gap < closest ? gap : closest;
            }
        }
    }
    printf("%d of %d fliers arrived; the closest two came %.3f m apart; %d steps in blocked "
           "space\n",
           arrived, FLIERS, closest, blocked);
    mnavDestroyAvoidance(avoidance);
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);
    return arrived == FLIERS && closest > -0.01 && blocked == 0 ? 0 : 1;
}
