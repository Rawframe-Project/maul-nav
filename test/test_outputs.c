// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What a failure leaves in a call's outputs. A call whose first parameter
// is an owner object clears them on any failure, before it checks the
// owner, so that a refused call never leaves an earlier call's values, or
// part of its own, for the caller to read; so does one that answers "no"
// without failing. Each output here is filled with
// a byte pattern first. The outputs a failure defines keep what it
// gives: the size or count a buffer too small needed, and a bake's report.

#include "defs_for.h"
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
    mnavCellFlow flow;
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
    mnavNavmeshDef meshDef = NavmeshDefFor(&def);
    CHECK(mnavCreateNavmesh(&meshDef, &navmesh).result == mnav_success, "a navmesh");
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavNavmesh_StageTile(navmesh, s_tiles[t], s_sizes[t]).result == mnav_success,
              "staged");
    }
    CHECK(mnavNavmesh_Commit(navmesh) == mnav_success, "committed");
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
    CHECK(mnavNavmesh_GetTile(navmesh, 0, 0, &tile) == mnav_success, "the first tile");
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
    CHECK(mnavPolygon_GetArea(nullptr, polygon, &s_out.area) == mnav_errorInvalid && CLEARED(area),
          "mnavPolygon_GetArea");
    FillAll();
    CHECK(mnavNavmesh_GetTile(nullptr, 0, 0, &s_out.tile) == mnav_errorInvalid && CLEARED(tile),
          "mnavNavmesh_GetTile");
    FillAll();
    CHECK(mnavStageLink(nullptr, &linkDef, &s_out.linkId) == mnav_errorInvalid && CLEARED(linkId),
          "mnavStageLink");
    FillAll();
    CHECK(mnavLink_GetState(nullptr, (mnavLinkId){1, 1}, &s_out.link) == mnav_errorInvalid &&
              CLEARED(link),
          "mnavLink_GetState");
    FillAll();
    CHECK(mnavLink_IsEnabled(nullptr, (mnavLinkId){1, 1}, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavLink_IsEnabled");
    FillAll();
    CHECK(mnavNavmesh_FindPolygons(nullptr, at, extents, nullptr, found, ROOM, &s_out.found) ==
                  mnav_errorInvalid &&
              CLEARED(found),
          "mnavNavmesh_FindPolygons");
    FillAll();
    CHECK(mnavNavmesh_FindNearest(nullptr, at, extents, nullptr, &s_out.nearest) ==
                  mnav_errorInvalid &&
              CLEARED(nearest),
          "mnavNavmesh_FindNearest");
    FillAll();
    CHECK(mnavQuery_FindPath(nullptr, navmesh, polygon, at, polygon, at, nullptr, &s_out.path) ==
                  mnav_errorInvalid &&
              CLEARED(path),
          "mnavQuery_FindPath");
    FillAll();
    CHECK(mnavQuery_FindShortestPath(nullptr, navmesh, polygon, at, polygon, at, nullptr,
                                     &s_out.path) == mnav_errorInvalid &&
              CLEARED(path),
          "mnavQuery_FindShortestPath");
    FillAll();
    CHECK(mnavQuery_ContinuePath(nullptr, navmesh, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavQuery_ContinuePath");
    FillAll();
    CHECK(mnavQuery_FinishPath(nullptr, navmesh, &s_out.path) == mnav_errorInvalid && CLEARED(path),
          "mnavQuery_FinishPath");
    FillAll();
    CHECK(mnavQuery_MoveAlongSurface(nullptr, navmesh, polygon, at, at, nullptr, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavQuery_MoveAlongSurface");
    FillAll();
    CHECK(mnavNavmesh_CheckCorridor(nullptr, &corridor, nullptr, &s_out.count) ==
                  mnav_errorInvalid &&
              CLEARED(count),
          "mnavNavmesh_CheckCorridor");
    FillAll();
    CHECK(mnavQuery_FindCorridorCorners(nullptr, &corridor, navmesh, &s_out.corners) ==
                  mnav_errorInvalid &&
              CLEARED(corners),
          "mnavQuery_FindCorridorCorners");
    FillAll();
    CHECK(mnavQuery_FindCorridorAreas(nullptr, &corridor, navmesh, &s_out.corners, runs, ROOM,
                                      &s_out.count) == mnav_errorInvalid &&
              CLEARED(corners) && CLEARED(count),
          "mnavQuery_FindCorridorAreas");
    FillAll();
    CHECK(mnavQuery_MoveCorridor(nullptr, &corridor, navmesh, at, nullptr, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavQuery_MoveCorridor");
    FillAll();
    CHECK(mnavQuery_MoveCorridorTarget(nullptr, &corridor, navmesh, at, nullptr, &s_out.move) ==
                  mnav_errorInvalid &&
              CLEARED(move),
          "mnavQuery_MoveCorridorTarget");
    FillAll();
    CHECK(mnavQuery_ShortcutCorridor(nullptr, &corridor, navmesh, at, nullptr, &s_out.flag) ==
                  mnav_errorInvalid &&
              CLEARED(flag),
          "mnavQuery_ShortcutCorridor");
    FillAll();
    CHECK(mnavQuery_ReplanCorridor(nullptr, &corridor, navmesh, extents, nullptr, &s_out.path) ==
                  mnav_errorInvalid &&
              CLEARED(path),
          "mnavQuery_ReplanCorridor");
    FillAll();
    CHECK(mnavQuery_FindGridPath(nullptr, &grid, (mnavCell){0, 0}, (mnavCell){1, 1}, nullptr,
                                 &s_out.gridPath) == mnav_errorInvalid &&
              CLEARED(gridPath),
          "mnavQuery_FindGridPath");
    FillAll();
    CHECK(mnavPolygon_GetHeight(nullptr, polygon, 1.0, 1.0, &s_out.height) == mnav_errorInvalid &&
              CLEARED(height),
          "mnavPolygon_GetHeight");
    FillAll();
    CHECK(mnavQuery_FindWallDistance(nullptr, navmesh, polygon, at, 1.0, nullptr, &s_out.wall) ==
                  mnav_errorInvalid &&
              CLEARED(wall),
          "mnavQuery_FindWallDistance");
    FillAll();
    CHECK(mnavQuery_FindWalls(nullptr, navmesh, polygon, at, 1.0, nullptr, walls, ROOM,
                              &s_out.walls) == mnav_errorInvalid &&
              CLEARED(walls),
          "mnavQuery_FindWalls");
    FillAll();
    CHECK(mnavNavmesh_FindRandomPoint(nullptr, 1, nullptr, &s_out.point) == mnav_errorInvalid &&
              CLEARED(point),
          "mnavNavmesh_FindRandomPoint");
    FillAll();
    CHECK(mnavQuery_FindRandomPointAround(nullptr, navmesh, polygon, at, 1.0, 1, nullptr,
                                          &s_out.point) == mnav_errorInvalid &&
              CLEARED(point),
          "mnavQuery_FindRandomPointAround");
    FillAll();
    CHECK(mnavQuery_CheckReachable(nullptr, navmesh, polygon, at, polygon, at, nullptr,
                                   &s_out.end) == mnav_errorInvalid &&
              CLEARED(end),
          "mnavQuery_CheckReachable");
    FillAll();
    CHECK(mnavQuery_Raycast(nullptr, navmesh, polygon, at, at, nullptr, &s_out.ray) ==
                  mnav_errorInvalid &&
              CLEARED(ray),
          "mnavQuery_Raycast");
    FillAll();
    CHECK(mnavQuery_GenerateLinks(nullptr, navmesh, &genDef, &(mnavTileRange){0, 0, 0, 0}, nullptr,
                                  links, ROOM, &s_out.count) == mnav_errorInvalid &&
              CLEARED(count),
          "mnavQuery_GenerateLinks");
    FillAll();
    CHECK(mnavQuery_FindHierarchicalPath(nullptr, nullptr, navmesh, polygon, at, polygon, at,
                                         &s_out.path) == mnav_errorInvalid &&
              CLEARED(path),
          "mnavQuery_FindHierarchicalPath");
    FillAll();
    CHECK(mnavHierarchy_Build(nullptr, query, navmesh, nullptr, &s_out.hierarchy) ==
                  mnav_errorInvalid &&
              CLEARED(hierarchy),
          "mnavHierarchy_Build");
    FillAll();
    CHECK(mnavHierarchy_Update(nullptr, query, navmesh, &s_out.hierarchy) == mnav_errorInvalid &&
              CLEARED(hierarchy),
          "mnavHierarchy_Update");
    FillAll();
    CHECK(mnavGridFlow_Continue(nullptr, &grid, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavGridFlow_Continue");
    FillAll();
    CHECK(mnavGridFlow_GetFlow(nullptr, (mnavCell){0, 0}, &s_out.flow) == mnav_errorInvalid &&
              CLEARED(flow),
          "mnavGridFlow_GetFlow");
    FillAll();
    CHECK(mnavNavFlow_Continue(nullptr, navmesh, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavNavFlow_Continue");
    FillAll();
    CHECK(mnavNavFlow_GetFlow(nullptr, navmesh, polygon, &s_out.polygonFlow) == mnav_errorInvalid &&
              CLEARED(polygonFlow),
          "mnavNavFlow_GetFlow");
    FillAll();
    CHECK(mnavBaker_CopyTile(nullptr, nullptr, 0, &s_out.size) == mnav_errorInvalid &&
              CLEARED(size),
          "mnavBaker_CopyTile");
    FillAll();
    CHECK(mnavFlightBaker_CopyTile(nullptr, nullptr, 0, &s_out.size) == mnav_errorInvalid &&
              CLEARED(size),
          "mnavFlightBaker_CopyTile");
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
    CHECK(mnavFlightVolume_GetTile(nullptr, 0, 0, &s_out.fingerprint) == mnav_errorInvalid &&
              CLEARED(fingerprint),
          "mnavFlightVolume_GetTile");
    FillAll();
    CHECK(mnavFlightVolume_IsOpen(nullptr, at, &s_out.flag) == mnav_errorInvalid && CLEARED(flag),
          "mnavFlightVolume_IsOpen");
    FillAll();
    CHECK(mnavQuery_FindFlightPath(nullptr, nullptr, at, at, &s_out.flightPath) ==
                  mnav_errorInvalid &&
              CLEARED(flightPath),
          "mnavQuery_FindFlightPath");
    FillAll();
    CHECK(mnavQuery_ContinueFlightPath(nullptr, nullptr, 1, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "mnavQuery_ContinueFlightPath");
    FillAll();
    CHECK(mnavQuery_FinishFlightPath(nullptr, nullptr, &s_out.flightPath) == mnav_errorInvalid &&
              CLEARED(flightPath),
          "mnavQuery_FinishFlightPath");
    FillAll();
    CHECK(mnavFlightVolume_Raycast(nullptr, at, at, &s_out.hit) == mnav_errorInvalid &&
              CLEARED(hit),
          "mnavFlightVolume_Raycast");
    FillAll();
    CHECK(mnavFlightVolume_CheckPath(nullptr, points, 2, &s_out.count, &s_out.hit) ==
                  mnav_errorInvalid &&
              CLEARED(count) && CLEARED(hit),
          "mnavFlightVolume_CheckPath");
    FillAll();
    CHECK(mnavFlightVolume_FindNearestPoint(nullptr, at, 1.0f, &s_out.position, &s_out.flag) ==
                  mnav_errorInvalid &&
              CLEARED(position) && CLEARED(flag),
          "mnavFlightVolume_FindNearestPoint");
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
    mnavVec2 velocities[2];
    Fill(velocities, sizeof(velocities));
    CHECK(mnavAvoidance_Avoid(avoidance, agents, 2, nullptr, 0, 0.1, velocities) ==
                  mnav_errorInvalid &&
              Zeroed(velocities, sizeof(velocities)),
          "an agent of no radius: every velocity cleared");
    mnavAgent3D fliers[2] = {
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.5, 2.0, 1.0, 1, 0, 0},
        {{5.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, 0.5, 2.0, 0.0, 2, 0, 0},
    };
    mnavVec3 velocities3[2];
    Fill(velocities3, sizeof(velocities3));
    CHECK(mnavAvoidance_Avoid3D(avoidance, fliers, 2, nullptr, 0, 0.1, velocities3) ==
                  mnav_errorInvalid &&
              Zeroed(velocities3, sizeof(velocities3)),
          "a flier of no priority: every velocity cleared");
    Fill(velocities, sizeof(velocities));
    CHECK(mnavAvoidance_Avoid(nullptr, agents, 2, nullptr, 0, 0.1, velocities) ==
                  mnav_errorInvalid &&
              (double)velocities[0].x != 0.0,
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
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 1, &gone) == mnav_success &&
              mnavNavmesh_StageTileRemoval(navmesh, 1, 1) == mnav_success &&
              mnavNavmesh_Commit(navmesh) == mnav_success,
          "a tile removed");
    mnavPolygonId stale = {gone.slot, gone.generation, 0};
    mnavPos3 at = {1.0, 0.0, 1.0};
    FillAll();
    CHECK(mnavPolygon_GetArea(navmesh, stale, &s_out.area) == mnav_errorStale && CLEARED(area),
          "a stale area");
    FillAll();
    CHECK(mnavPolygon_GetHeight(navmesh, polygon, 1000.0, 1000.0, &s_out.height) ==
                  mnav_errorOutside &&
              CLEARED(height),
          "a height off the polygon");
    FillAll();
    CHECK(mnavQuery_FindPath(query, navmesh, stale, at, polygon, at, nullptr, &s_out.path) ==
                  mnav_errorStale &&
              CLEARED(path),
          "a path from a stale polygon");
    FillAll();
    CHECK(mnavQuery_Raycast(query, navmesh, stale, at, at, nullptr, &s_out.ray) ==
                  mnav_errorStale &&
              CLEARED(ray),
          "a ray from a stale polygon");
    FillAll();
    CHECK(mnavQuery_ContinuePath(query, navmesh, 1, &s_out.flag) == mnav_errorState &&
              CLEARED(flag),
          "no search begun");
    FillAll();
    CHECK(mnavQuery_FinishPath(query, navmesh, &s_out.path) == mnav_errorState && CLEARED(path),
          "no search to finish");
    FillAll();
    CHECK(mnavNavmesh_FindNearest(navmesh, at, (mnavVec3){-1.0f, 1.0f, 1.0f}, nullptr,
                                  &s_out.nearest) == mnav_errorInvalid &&
              CLEARED(nearest),
          "a negative extent");
    FillAll();
    CHECK(mnavLink_GetState(navmesh, (mnavLinkId){7, 1}, &s_out.link) == mnav_errorInvalid &&
              CLEARED(link),
          "a link never handed out");
    FillAll();
    CHECK(mnavLink_IsEnabled(navmesh, (mnavLinkId){7, 1}, &s_out.flag) == mnav_errorInvalid &&
              CLEARED(flag),
          "a link never handed out, read as a switch");
    // A buffer too small defines its count: the total, past the room.
    mnavPolygonId one[1];
    FillAll();
    CHECK(mnavNavmesh_FindPolygons(navmesh, (mnavPos3){32.0, 0.0, 32.0},
                                   (mnavVec3){64.0f, 64.0f, 64.0f}, nullptr, one, 1,
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
    mnavGridFlowDef def = mnavDefaultGridFlowDef();
    mnavGridFlow* field = nullptr;
    CHECK(mnavCreateGridFlow(&def, &field) == mnav_success, "a field");
    mnavAreaType areas[16];
    memset(areas, 1, sizeof(areas));
    mnavGrid grid = {areas, 4, 4, 1.0f};
    mnavCellRange region = {0, 0, 1, 1};
    mnavCell goal = {0, 0};
    bool ended = false;
    CHECK(mnavGridFlow_Begin(field, &grid, &region, &goal, 1, nullptr) == mnav_success &&
              mnavGridFlow_Continue(field, &grid, 64, &ended) == mnav_success && ended,
          "built over a corner");
    FillAll();
    CHECK(mnavGridFlow_GetFlow(field, (mnavCell){3, 3}, &s_out.flow) == mnav_errorOutside &&
              CLEARED(flow),
          "a cell outside the region");
    mnavDestroyGridFlow(field);

    mnavNavmesh* navmesh = WorldNavmesh();
    mnavNavFlowDef navDef = mnavDefaultNavFlowDef();
    mnavNavFlow* navFlow = nullptr;
    CHECK(mnavCreateNavFlow(&navDef, &navFlow) == mnav_success, "a navmesh field");
    mnavPolygonId polygon = FirstPolygon(navmesh);
    mnavNavFlowGoal navGoal = {polygon, {1.0, 0.0, 1.0}};
    mnavTileRange tiles = {0, 0, 0, 0};
    CHECK(mnavNavFlow_Begin(navFlow, navmesh, &tiles, &navGoal, 1, nullptr) == mnav_success &&
              mnavNavFlow_Continue(navFlow, navmesh, 1 << 16, &ended) == mnav_success && ended,
          "built over one tile");
    mnavTileId far = {0};
    CHECK(mnavNavmesh_GetTile(navmesh, 1, 1, &far) == mnav_success, "a tile outside the region");
    FillAll();
    CHECK(mnavNavFlow_GetFlow(navFlow, navmesh, (mnavPolygonId){far.slot, far.generation, 0},
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
    def.enableGroundFill = true;
    return def;
}

// A flight volume asked about a point below it, and a baker with no tile.
static void TestFlightOutside(void)
{
    mnavFlightDef def = FlightDef();
    mnavFlightVolume* volume = nullptr;
    mnavFlightVolumeDef volumeDef = FlightVolumeDefFor(&def);
    CHECK(mnavCreateFlightVolume(&volumeDef, &volume).result == mnav_success, "a volume");
    const uint8_t* stored[4] = {s_flight0, s_flight1, s_flight2, s_flight3};
    const size_t sizes[4] = {sizeof(s_flight0), sizeof(s_flight1), sizeof(s_flight2),
                             sizeof(s_flight3)};
    for (int32_t t = 0; t < 4; ++t)
    {
        CHECK(mnavFlightVolume_StageTile(volume, stored[t], sizes[t]).result == mnav_success,
              "a flight tile");
    }
    CHECK(mnavFlightVolume_Commit(volume) == mnav_success, "committed");
    mnavQuery* query = Query();
    mnavPos3 below = {1.0, -100.0, 1.0};
    mnavPos3 inside = {1.0, 5.0, 1.0};
    FillAll();
    CHECK(mnavFlightVolume_IsOpen(volume, below, &s_out.flag) == mnav_errorOutside && CLEARED(flag),
          "open below the volume");
    FillAll();
    CHECK(mnavQuery_FindFlightPath(query, volume, below, inside, &s_out.flightPath) ==
                  mnav_errorOutside &&
              CLEARED(flightPath),
          "a path from below the volume");
    FillAll();
    CHECK(mnavFlightVolume_GetTile(volume, 9, 9, &s_out.fingerprint) == mnav_errorNotLoaded &&
              CLEARED(fingerprint),
          "a flight tile not loaded");
    // A "no" clears the outputs too: no open point within the radius of
    // one in the ground.
    FillAll();
    CHECK(mnavFlightVolume_FindNearestPoint(volume, (mnavPos3){1.0, -1.5, 1.0}, 0.25f,
                                            &s_out.position, &s_out.flag) == mnav_success &&
              CLEARED(position) && CLEARED(flag),
          "no open point near");
    mnavDestroyQuery(query);
    mnavDestroyFlightVolume(volume);

    mnavFlightBaker* baker = nullptr;
    CHECK(mnavCreateFlightBaker(&def, &baker).result == mnav_success, "a baker");
    FillAll();
    CHECK(mnavFlightBaker_CopyTile(baker, nullptr, 0, &s_out.size) != mnav_success && CLEARED(size),
          "no tile to copy");
    mnavDestroyFlightBaker(baker);
}

// A call that answers "no" without failing clears its outputs as a
// failure does: no polygon has its nearest point in a box far from every
// tile, and only whether the box was all loaded is an answer.
static void TestNoAnswer(void)
{
    mnavNavmesh* navmesh = WorldNavmesh();
    FillAll();
    CHECK(mnavNavmesh_FindNearest(navmesh, (mnavPos3){500.0, 0.0, 500.0},
                                  (mnavVec3){1.0f, 1.0f, 1.0f}, nullptr,
                                  &s_out.nearest) == mnav_success,
          "a nearest polygon far away");
    mnavNearest none;
    memcpy(&none, &s_out.nearest, sizeof(none));
    memset((unsigned char*)&none + offsetof(mnavNearest, incomplete), 0, sizeof(none.incomplete));
    CHECK(Zeroed(&none, sizeof(none)), "no polygon, and nothing else left");
    mnavDestroyNavmesh(navmesh);
}

int main(void)
{
    TestNoOwner();
    TestNoFlightOwner();
    TestAvoidanceOutputs();
    TestPastTheOwner();
    TestFlowOutside();
    TestFlightOutside();
    TestNoAnswer();
    return s_failures == 0 ? 0 : 1;
}
