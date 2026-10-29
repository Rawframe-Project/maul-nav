// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What a failure leaves in a call's outputs. A call whose first parameter
// is an owner object clears them on any failure, before it checks the
// owner, so that a refused call never leaves an earlier call's values, or
// part of its own, for the caller to read. Each output here is filled with
// a byte pattern first. The outputs a failure defines keep what it
// gives: the size or count a buffer too small needed, and a bake's report.

#include "stored_tiles.h"
#include "test_harness.h"
#include "world.h"

#include "maul-nav/avoidance.h"
#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/flight.h"
#include "maul-nav/flow.h"
#include "maul-nav/hierarchy.h"
#include "maul-nav/linkgen.h"
#include "maul-nav/navflow.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum
{
    PATTERN = 0xA5,
    ROOM = 8
};

static void Fill(void* output, size_t size)
{
    memset(output, PATTERN, size);
}

// Whether every byte of an output is 0, read as bytes so that a bool or a
// pointer the pattern filled is never read as one.
static bool Zeroed(const void* output, size_t size)
{
    const unsigned char* bytes = output;
    for (size_t i = 0; i < size; ++i)
    {
        if (bytes[i] != 0)
        {
            return false;
        }
    }
    return true;
}

// The outputs the calls below write, each filled again before a call.
typedef struct Outputs
{
    mnavPath path;
    mnavMove move;
    mnavCorners corners;
    mnavRay ray;
    mnavNearest nearest;
    mnavFound found;
    mnavNearestWall wall;
    mnavWallsFound walls;
    mnavRandomPoint point;
    mnavGridPath gridPath;
    mnavFlightPath flightPath;
    mnavFlightHit hit;
    mnavFlow flow;
    mnavPolygonFlow polygonFlow;
    mnavLinkState link;
    mnavLinkId linkId;
    mnavTileId tile;
    mnavHierarchyReport hierarchy;
    mnavPos3 position;
    mnavPathEnd end;
    mnavAreaType area;
    uint64_t fingerprint;
    double height;
    size_t size;
    int32_t count;
    bool flag;
    bool second;
} Outputs;

static Outputs s_out;

static void FillAll(void)
{
    Fill(&s_out, sizeof(s_out));
}

#define CLEARED(field) Zeroed(&s_out.field, sizeof(s_out.field))

// The world's four stored tiles in a navmesh.
static mnavNavmesh* WorldNavmesh(void)
{
    LoadWorldTiles();
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavNavmesh* navmesh = nullptr;
    CHECK(mnavCreateNavmesh(&def, &navmesh).result == mnav_success, "a navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success, "staged");
    }
    CHECK(mnavCommit(navmesh) == mnav_success, "committed");
    return navmesh;
}

static mnavQuery* Query(void)
{
    mnavQueryDef def = mnavDefaultQueryDef();
    mnavQuery* query = nullptr;
    CHECK(mnavCreateQuery(&def, &query) == mnav_success, "a query context");
    return query;
}

// Polygon 0 of the tile at (0, 0).
static mnavPolygonId FirstPolygon(const mnavNavmesh* navmesh)
{
    mnavTileId tile = {0};
    CHECK(mnavGetTile(navmesh, 0, 0, &tile) == mnav_success, "the first tile");
    return (mnavPolygonId){tile.slot, tile.generation, 0};
}

// Every call that takes an owner first, given none: the outputs are
// cleared although the call checks nothing else.
static void TestNoOwner(void)
{
    mnavNavmesh* navmesh = WorldNavmesh();
    mnavQuery* query = Query();
    mnavPolygonId polygon = FirstPolygon(navmesh);
    mnavPos3 at = {1.0, 0.0, 1.0};
    mnavVec3 extents = {1.0f, 2.0f, 1.0f};
    mnavPolygonId polygons[ROOM];
    mnavCorridor corridor = {0};
    CHECK(mnavResetCorridor(&corridor, polygons, ROOM, polygon, at) == mnav_success, "a corridor");
    mnavPolygonId found[ROOM];
    mnavWall walls[ROOM];
    mnavAreaRun runs[ROOM];
    mnavLinkDef links[ROOM];
    mnavLinkDef linkDef = {at, at, 0.5f, 1.0f, mnav_linkJump, false, 0.0f};
    mnavLinkGenDef genDef = mnavDefaultLinkGenDef();
    mnavAreaType areas[4] = {1, 1, 1, 1};
    mnavGrid grid = {areas, 2, 2, 1.0f};

    FillAll();
    CHECK(mnavGetArea(nullptr, polygon, &s_out.area) == mnav_errorInvalid && CLEARED(area),
          "mnavGetArea");
    FillAll();
    CHECK(mnavGetTile(nullptr, 0, 0, &s_out.tile) == mnav_errorInvalid && CLEARED(tile),
          "mnavGetTile");
    FillAll();
    CHECK(mnavStageLink(nullptr, &linkDef, &s_out.linkId) == mnav_errorInvalid && CLEARED(linkId),
          "mnavStageLink");
    FillAll();
    CHECK(mnavGetLink(nullptr, (mnavLinkId){1, 1}, &s_out.link) == mnav_errorInvalid &&
              CLEARED(link),
          "mnavGetLink");
    FillAll();
    CHECK(mnavFindPolygons(nullptr, nullptr, at, extents, found, ROOM, &s_out.found) ==
                  mnav_errorInvalid &&
              CLEARED(found),
          "mnavFindPolygons");
    FillAll();
    CHECK(mnavFindNearest(nullptr, nullptr, at, extents, &s_out.nearest) == mnav_errorInvalid &&
              CLEARED(nearest),
          "mnavFindNearest");
    FillAll();
    CHECK(mnavFindPath(nullptr, navmesh, nullptr, polygon, at, polygon, at, &s_out.path) ==
                  mnav_errorInvalid &&
              CLEARED(path),
          "mnavFindPath");
    FillAll();
    CHECK(mnavFindShortestPath(nullptr, navmesh, nullptr, polygon, at, polygon, at, &s_out.path) ==
                  mnav_errorInvalid &&
              CLEARED(path),
          "mnavFindShortestPath");
    FillAll();
    CHECK(mnavContinuePath(nullptr, navmesh, 1, &s_out.flag) == mnav_errorInvalid && CLEARED(flag),
          "mnavContinuePath");
    FillAll();
    CHECK(mnavFinishPath(nullptr, navmesh, &s_out.path) == mnav_errorInvalid && CLEARED(path),
          "mnavFinishPath");
    FillAll();
    CHECK(mnavMoveAlongSurface(nullptr, navmesh, nullptr, polygon, at, at, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavMoveAlongSurface");
    FillAll();
    CHECK(mnavCheckCorridor(nullptr, nullptr, &corridor, &s_out.count) == mnav_errorInvalid &&
              CLEARED(count),
          "mnavCheckCorridor");
    FillAll();
    CHECK(mnavCorridorCorners(nullptr, navmesh, &corridor, &s_out.corners) == mnav_errorInvalid &&
              CLEARED(corners),
          "mnavCorridorCorners");
    FillAll();
    CHECK(mnavCorridorAreas(nullptr, navmesh, &corridor, &s_out.corners, runs, ROOM,
                            &s_out.count) == mnav_errorInvalid &&
              CLEARED(corners) && CLEARED(count),
          "mnavCorridorAreas");
    FillAll();
    CHECK(mnavMoveCorridor(nullptr, navmesh, nullptr, &corridor, at, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavMoveCorridor");
    FillAll();
    CHECK(mnavMoveCorridorTarget(nullptr, navmesh, nullptr, &corridor, at, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavMoveCorridorTarget");
    FillAll();
    CHECK(mnavShortcutCorridor(nullptr, navmesh, nullptr, &corridor, at, &s_out.flag) ==
                  mnav_errorInvalid &&
              CLEARED(flag),
          "mnavShortcutCorridor");
    FillAll();
    CHECK(mnavReplanCorridor(nullptr, navmesh, nullptr, &corridor, extents, &s_out.path) ==
                  mnav_errorInvalid &&
              CLEARED(path),
          "mnavReplanCorridor");
    FillAll();
    CHECK(mnavFindGridPath(nullptr, &grid, nullptr, (mnavCell){0, 0}, (mnavCell){1, 1},
                           &s_out.gridPath) == mnav_errorInvalid &&
              CLEARED(gridPath),
          "mnavFindGridPath");
    FillAll();
    CHECK(mnavGetHeight(nullptr, polygon, 1.0, 1.0, &s_out.height) == mnav_errorInvalid &&
              CLEARED(height),
          "mnavGetHeight");
    FillAll();
    CHECK(mnavFindWallDistance(nullptr, navmesh, nullptr, polygon, at, 1.0, &s_out.wall) ==
                  mnav_errorInvalid &&
              CLEARED(wall),
          "mnavFindWallDistance");
    FillAll();
    CHECK(mnavFindWalls(nullptr, navmesh, nullptr, polygon, at, 1.0, walls, ROOM, &s_out.walls) ==
                  mnav_errorInvalid &&
              CLEARED(walls),
          "mnavFindWalls");
    FillAll();
    CHECK(mnavFindRandomPoint(nullptr, nullptr, 1, &s_out.point) == mnav_errorInvalid &&
              CLEARED(point),
          "mnavFindRandomPoint");
    FillAll();
    CHECK(mnavFindRandomPointAround(nullptr, navmesh, nullptr, polygon, at, 1.0, 1, &s_out.point) ==
                  mnav_errorInvalid &&
              CLEARED(point),
          "mnavFindRandomPointAround");
    FillAll();
    CHECK(mnavCheckReachable(nullptr, navmesh, nullptr, polygon, at, polygon, at, &s_out.end) ==
                  mnav_errorInvalid &&
              CLEARED(end),
          "mnavCheckReachable");
    FillAll();
    CHECK(mnavRaycast(nullptr, navmesh, nullptr, polygon, at, at, &s_out.ray) ==
                  mnav_errorInvalid &&
              CLEARED(ray),
          "mnavRaycast");
    FillAll();
    CHECK(mnavGenerateLinks(nullptr, navmesh, nullptr, &genDef, 0, 0, 0, 0, links, ROOM,
                            &s_out.count) == mnav_errorInvalid &&
              CLEARED(count),
          "mnavGenerateLinks");
    FillAll();
    CHECK(mnavFindHierarchicalPath(nullptr, nullptr, navmesh, polygon, at, polygon, at,
                                   &s_out.path) == mnav_errorInvalid &&
              CLEARED(path),
          "mnavFindHierarchicalPath");
    FillAll();
    CHECK(mnavBuildHierarchy(nullptr, query, navmesh, nullptr, &s_out.hierarchy) ==
                  mnav_errorInvalid &&
              CLEARED(hierarchy),
          "mnavBuildHierarchy");
    FillAll();
    CHECK(mnavUpdateHierarchy(nullptr, query, navmesh, &s_out.hierarchy) == mnav_errorInvalid &&
              CLEARED(hierarchy),
          "mnavUpdateHierarchy");
    FillAll();
    CHECK(mnavContinueFlowField(nullptr, &grid, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavContinueFlowField");
    FillAll();
    CHECK(mnavFlowAt(nullptr, (mnavCell){0, 0}, &s_out.flow) == mnav_errorInvalid && CLEARED(flow),
          "mnavFlowAt");
    FillAll();
    CHECK(mnavContinueNavFlow(nullptr, navmesh, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavContinueNavFlow");
    FillAll();
    CHECK(mnavNavFlowAt(nullptr, navmesh, polygon, &s_out.polygonFlow) == mnav_errorInvalid &&
              CLEARED(polygonFlow),
          "mnavNavFlowAt");
    FillAll();
    CHECK(mnavCopyBakedTile(nullptr, nullptr, 0, &s_out.size) == mnav_errorInvalid && CLEARED(size),
          "mnavCopyBakedTile");
    FillAll();
    CHECK(mnavCopyFlightTile(nullptr, nullptr, 0, &s_out.size) == mnav_errorInvalid &&
              CLEARED(size),
          "mnavCopyFlightTile");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// The flight calls given no volume or no context.
static void TestNoFlightOwner(void)
{
    mnavQuery* query = Query();
    mnavPos3 at = {1.0, 1.0, 1.0};
    mnavPos3 points[2] = {at, at};
    FillAll();
    CHECK(mnavGetFlightTile(nullptr, 0, 0, &s_out.fingerprint) == mnav_errorInvalid &&
              CLEARED(fingerprint),
          "mnavGetFlightTile");
    FillAll();
    CHECK(mnavIsFlightOpen(nullptr, at, &s_out.flag) == mnav_errorInvalid && CLEARED(flag),
          "mnavIsFlightOpen");
    FillAll();
    CHECK(mnavFindFlightPath(nullptr, nullptr, at, at, &s_out.flightPath) == mnav_errorInvalid &&
              CLEARED(flightPath),
          "mnavFindFlightPath");
    FillAll();
    CHECK(mnavContinueFlightPath(nullptr, nullptr, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavContinueFlightPath");
    FillAll();
    CHECK(mnavFinishFlightPath(nullptr, nullptr, &s_out.flightPath) == mnav_errorInvalid &&
              CLEARED(flightPath),
          "mnavFinishFlightPath");
    FillAll();
    CHECK(mnavFlightRaycast(nullptr, at, at, &s_out.hit) == mnav_errorInvalid && CLEARED(hit),
          "mnavFlightRaycast");
    FillAll();
    CHECK(mnavCheckFlightPath(nullptr, points, 2, &s_out.count, &s_out.hit) == mnav_errorInvalid &&
              CLEARED(count) && CLEARED(hit),
          "mnavCheckFlightPath");
    FillAll();
    CHECK(mnavFindNearestFlightPoint(nullptr, at, 1.0f, &s_out.position, &s_out.flag) ==
                  mnav_errorInvalid &&
              CLEARED(position) && CLEARED(flag),
          "mnavFindNearestFlightPoint");
    mnavDestroyQuery(query);
}

// Avoidance given no set: the velocities are not written, since the set's
// limit is what bounds them.
static void TestAvoidanceOutputs(void)
{
    mnavAvoidanceDef def = mnavDefaultAvoidanceDef();
    mnavAvoidance* avoidance = nullptr;
    CHECK(mnavCreateAvoidance(&def, &avoidance) == mnav_success, "a set");
    mnavAgent agents[2] = {
        {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, 0.5, 2.0, 1.0, 1, 0, 0},
        {{5.0, 0.0}, {0.0, 0.0}, {-1.0, 0.0}, 0.0, 2.0, 1.0, 2, 0, 0},
    };
    mnavPos2 velocities[2];
    Fill(velocities, sizeof(velocities));
    CHECK(mnavAvoid(avoidance, agents, 2, nullptr, 0, 0.1, velocities) == mnav_errorInvalid &&
              Zeroed(velocities, sizeof(velocities)),
          "an agent of no radius: every velocity cleared");
    mnavAgent3D fliers[2] = {
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.5, 2.0, 1.0, 1, 0, 0},
        {{5.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, 0.5, 2.0, 0.0, 2, 0, 0},
    };
    mnavPos3 velocities3[2];
    Fill(velocities3, sizeof(velocities3));
    CHECK(mnavAvoid3D(avoidance, fliers, 2, nullptr, 0, 0.1, velocities3) == mnav_errorInvalid &&
              Zeroed(velocities3, sizeof(velocities3)),
          "a flier of no priority: every velocity cleared");
    Fill(velocities, sizeof(velocities));
    CHECK(mnavAvoid(nullptr, agents, 2, nullptr, 0, 0.1, velocities) == mnav_errorInvalid &&
              velocities[0].x != 0.0,
          "no set: nothing written");
    mnavDestroyAvoidance(avoidance);
}

// Failures past the owner's check: a stale id, a place outside the
// object, the wrong state, a bad argument after a good owner.
static void TestPastTheOwner(void)
{
    mnavNavmesh* navmesh = WorldNavmesh();
    mnavQuery* query = Query();
    mnavPolygonId polygon = FirstPolygon(navmesh);
    // A polygon of the tile at (1, 1), which a commit then removes.
    mnavTileId gone = {0};
    CHECK(mnavGetTile(navmesh, 1, 1, &gone) == mnav_success &&
              mnavStageTileRemoval(navmesh, 1, 1) == mnav_success &&
              mnavCommit(navmesh) == mnav_success,
          "a tile removed");
    mnavPolygonId stale = {gone.slot, gone.generation, 0};
    mnavPos3 at = {1.0, 0.0, 1.0};
    FillAll();
    CHECK(mnavGetArea(navmesh, stale, &s_out.area) == mnav_errorStale && CLEARED(area),
          "a stale area");
    FillAll();
    CHECK(mnavGetHeight(navmesh, polygon, 1000.0, 1000.0, &s_out.height) == mnav_errorOutside &&
              CLEARED(height),
          "a height off the polygon");
    FillAll();
    CHECK(mnavFindPath(query, navmesh, nullptr, stale, at, polygon, at, &s_out.path) ==
                  mnav_errorStale &&
              CLEARED(path),
          "a path from a stale polygon");
    FillAll();
    CHECK(mnavRaycast(query, navmesh, nullptr, stale, at, at, &s_out.ray) == mnav_errorStale &&
              CLEARED(ray),
          "a ray from a stale polygon");
    FillAll();
    CHECK(mnavContinuePath(query, navmesh, 1, &s_out.flag) == mnav_errorState && CLEARED(flag),
          "no search begun");
    FillAll();
    CHECK(mnavFinishPath(query, navmesh, &s_out.path) == mnav_errorState && CLEARED(path),
          "no search to finish");
    FillAll();
    CHECK(mnavFindNearest(navmesh, nullptr, at, (mnavVec3){-1.0f, 1.0f, 1.0f}, &s_out.nearest) ==
                  mnav_errorInvalid &&
              CLEARED(nearest),
          "a negative extent");
    FillAll();
    CHECK(mnavGetLink(navmesh, (mnavLinkId){7, 1}, &s_out.link) == mnav_errorInvalid &&
              CLEARED(link),
          "a link never handed out");
    // A buffer too small defines its count: the total, past the room.
    mnavPolygonId one[1];
    FillAll();
    CHECK(mnavFindPolygons(navmesh, nullptr, (mnavPos3){32.0, 0.0, 32.0},
                           (mnavVec3){64.0f, 64.0f, 64.0f}, one, 1,
                           &s_out.found) == mnav_errorCapacity &&
              s_out.found.count > 1,
          "a count past the room is kept");
    mnavDestroyQuery(query);
    mnavDestroyNavmesh(navmesh);
}

// A grid flow field read outside its region, and a navmesh flow field
// read outside its tiles.
static void TestFlowOutside(void)
{
    mnavFlowFieldDef def = mnavDefaultFlowFieldDef();
    mnavFlowField* field = nullptr;
    CHECK(mnavCreateFlowField(&def, &field) == mnav_success, "a field");
    mnavAreaType areas[16];
    memset(areas, 1, sizeof(areas));
    mnavGrid grid = {areas, 4, 4, 1.0f};
    mnavFlowRegion region = {0, 0, 2, 2};
    mnavCell goal = {0, 0};
    bool ended = false;
    CHECK(mnavBeginFlowField(field, &grid, nullptr, &region, &goal, 1) == mnav_success &&
              mnavContinueFlowField(field, &grid, 64, &ended) == mnav_success && ended,
          "built over a corner");
    FillAll();
    CHECK(mnavFlowAt(field, (mnavCell){3, 3}, &s_out.flow) == mnav_errorOutside && CLEARED(flow),
          "a cell outside the region");
    mnavDestroyFlowField(field);

    mnavNavmesh* navmesh = WorldNavmesh();
    mnavNavFlowDef navDef = mnavDefaultNavFlowDef();
    mnavNavFlow* navFlow = nullptr;
    CHECK(mnavCreateNavFlow(&navDef, &navFlow) == mnav_success, "a navmesh field");
    mnavPolygonId polygon = FirstPolygon(navmesh);
    mnavNavFlowGoal navGoal = {polygon, {1.0, 0.0, 1.0}};
    mnavNavFlowRegion tiles = {0, 0, 0, 0};
    CHECK(mnavBeginNavFlow(navFlow, navmesh, nullptr, &tiles, &navGoal, 1) == mnav_success &&
              mnavContinueNavFlow(navFlow, navmesh, 1 << 16, &ended) == mnav_success && ended,
          "built over one tile");
    mnavTileId far = {0};
    CHECK(mnavGetTile(navmesh, 1, 1, &far) == mnav_success, "a tile outside the region");
    FillAll();
    CHECK(mnavNavFlowAt(navFlow, navmesh, (mnavPolygonId){far.slot, far.generation, 0},
                        &s_out.polygonFlow) == mnav_errorOutside &&
              CLEARED(polygonFlow),
          "a polygon outside the region");
    mnavDestroyNavFlow(navFlow);
    mnavDestroyNavmesh(navmesh);
}

// The stored flight tiles' def (test_stored.c's FlightDef).
static mnavFlightDef FlightDef(void)
{
    mnavFlightDef def = mnavDefaultFlightDef();
    def.voxelSize = 0.5f;
    def.tileVoxels = 64;
    def.floor = -2.0f;
    def.ceiling = 20.0f;
    def.radius = 0.5f;
    def.groundBelow = true;
    return def;
}

// A flight volume asked about a point below it, and a baker with no tile.
static void TestFlightOutside(void)
{
    mnavFlightDef def = FlightDef();
    mnavFlightVolume* volume = nullptr;
    CHECK(mnavCreateFlightVolume(&def, &volume).result == mnav_success, "a volume");
    const uint8_t* stored[4] = {s_flight0, s_flight1, s_flight2, s_flight3};
    const size_t sizes[4] = {sizeof(s_flight0), sizeof(s_flight1), sizeof(s_flight2),
                             sizeof(s_flight3)};
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavStageFlightTile(volume, stored[t], sizes[t]).result == mnav_success,
              "a flight tile");
    }
    CHECK(mnavCommitFlight(volume) == mnav_success, "committed");
    mnavQuery* query = Query();
    mnavPos3 below = {1.0, -100.0, 1.0};
    mnavPos3 inside = {1.0, 5.0, 1.0};
    FillAll();
    CHECK(mnavIsFlightOpen(volume, below, &s_out.flag) == mnav_errorOutside && CLEARED(flag),
          "open below the volume");
    FillAll();
    CHECK(mnavFindFlightPath(query, volume, below, inside, &s_out.flightPath) ==
                  mnav_errorOutside &&
              CLEARED(flightPath),
          "a path from below the volume");
    FillAll();
    CHECK(mnavGetFlightTile(volume, 9, 9, &s_out.fingerprint) == mnav_errorNotLoaded &&
              CLEARED(fingerprint),
          "a flight tile not loaded");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);

    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "a baker");
    FillAll();
    CHECK(mnavCopyFlightTile(baker, nullptr, 0, &s_out.size) != mnav_success && CLEARED(size),
          "no tile to copy");
    mnavDestroyFlightBaker(baker);
}

int main(void)
{
    TestNoOwner();
    TestNoFlightOwner();
    TestAvoidanceOutputs();
    TestPastTheOwner();
    TestFlowOutside();
    TestFlightOutside();
    return s_failures == 0 ? 0 : 1;
}
