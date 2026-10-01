// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The bake's settings and its input surface: the def a bake runs with,
// the agent profile it bakes for, the named limits that bound its work,
// and the triangle meshes a host hands it, each checked before any work
// (mnav-0002).

#ifndef MAUL_NAV_BAKE_H
#define MAUL_NAV_BAKE_H

#include "maul-nav/base.h"

#ifdef __cplusplus
extern "C"
{
#endif

// The number of area types: 0 is unwalkable, 1 to 63 are walkable kinds a
// query filter costs and includes or excludes.
#define MNAV_AREA_TYPES 64

// The range of a cell's size and height, in meters.
#define MNAV_MIN_CELL_SIZE 0.001f
#define MNAV_MAX_CELL_SIZE 10.0f

// The range of a tile's side, in cells.
#define MNAV_MIN_TILE_CELLS 16
#define MNAV_MAX_TILE_CELLS 1024

// How far input may lie from the bake's origin: horizontally, in cells of
// the bake's cell size; vertically, in cells of its cell height, up or down.
#define MNAV_MAX_EXTENT_CELLS 4194304
#define MNAV_MAX_HEIGHT_CELLS 32767

// The largest values the named limits may take.
#define MNAV_MAX_INPUT_TRIANGLES 268435456
#define MNAV_MAX_TILE_SPANS      67108864
#define MNAV_MAX_TILE_POLYGONS   65535
#define MNAV_MAX_TILE_VERTICES   65535
#define MNAV_MAX_TILE_LINKS      1048576
#define MNAV_MAX_TILES           1048576

    // An area type, 0 to MNAV_AREA_TYPES - 1.
    typedef uint8_t mnavAreaType;

    enum
    {
        // Not walkable: triangles of this area contribute nothing walkable
        // and block what lies under them.
        mnav_areaNone = 0,
        // The walkable area a triangle has when its mesh gives no areas.
        mnav_areaWalkable = 1,
    };

    // The agent a navmesh is baked for, in meters and degrees (N6: one
    // profile per navmesh).
    typedef struct mnavAgentProfile
    {
        // The agent's radius: walkable area keeps this far from walls. At
        // least 0.
        float radius;
        // The agent's height: walkable area has this much free space above
        // it. More than 0.
        float height;
        // The highest ledge the agent steps up or down. At least 0.
        float stepHeight;
        // The steepest walkable slope, in degrees, from 0 up to but not
        // including 90.
        float maxSlopeDegrees;
    } mnavAgentProfile;

    // The named limits that bound a bake's work. Each is at least 1 and at
    // most its MNAV_MAX_ value. Reaching one is mnav_errorLimit, never a
    // silent truncation. None is in seconds, so a bake's outcome does not
    // depend on the machine.
    typedef struct mnavBakeLimits
    {
        // Triangles in one bake's input.
        int32_t inputTriangles;
        // Input triangles that touch one tile.
        int32_t tileTriangles;
        // Span fragments rasterized into one tile: one per cell a triangle
        // covers, before they merge into spans (N12).
        int32_t tileSpans;
        // Polygons in one tile.
        int32_t tilePolygons;
        // Vertices in one tile.
        int32_t tileVertices;
        // Links in one tile.
        int32_t tileLinks;
        // Tiles in one navmesh.
        int32_t tiles;
        // Bytes the bake may hold allocated at once.
        uint64_t memoryBytes;
    } mnavBakeLimits;

    // How a bake runs. Build it with mnavDefaultBakeDef.
    typedef struct mnavBakeDef
    {
        uint32_t cookie;
        // The allocator the bake uses; zeroed for the C library's.
        mnavAllocator allocator;
        // The world position the tile grid starts at. Input vertices are
        // relative to it.
        mnavPos3 origin;
        // A cell's side on the ground, in meters, MNAV_MIN_CELL_SIZE to
        // MNAV_MAX_CELL_SIZE.
        float cellSize;
        // A cell's height, in meters, in the same range.
        float cellHeight;
        // A tile's side, in cells, MNAV_MIN_TILE_CELLS to
        // MNAV_MAX_TILE_CELLS.
        int32_t tileCells;
        // The agent the navmesh is for.
        mnavAgentProfile agent;
        // The smallest walkable region kept, in square meters, at least 0:
        // smaller islands away from a tile's edge are dropped.
        float minRegionArea;
        // How far a simplified wall may stray from the cells it follows, in
        // meters, at least 0.
        float maxEdgeError;
        // The longest wall edge, in meters, at least 0; 0 for no limit.
        float maxEdgeLength;
        // How far apart the detail mesh samples the floor's height along
        // polygon edges and inside polygons, in meters, at least 0; 0 for
        // none, and at least one cell otherwise.
        float detailSampleDistance;
        // How far the detail surface may stray from a sampled height, in
        // meters, at least 0.
        float detailMaxError;
        // The limits on the work.
        mnavBakeLimits limits;
    } mnavBakeDef;

    // The setting a def check refused.
    typedef uint8_t mnavBakeSetting;

    enum
    {
        mnav_settingNone = 0,
        mnav_settingCookie = 1,
        mnav_settingAllocator = 2,
        mnav_settingOrigin = 3,
        mnav_settingCellSize = 4,
        mnav_settingCellHeight = 5,
        mnav_settingTileCells = 6,
        mnav_settingAgentRadius = 7,
        mnav_settingAgentHeight = 8,
        mnav_settingAgentStepHeight = 9,
        mnav_settingAgentMaxSlope = 10,
        mnav_settingInputTriangles = 11,
        mnav_settingTileTriangles = 12,
        mnav_settingTileSpans = 13,
        mnav_settingTilePolygons = 14,
        mnav_settingTileVertices = 15,
        mnav_settingTileLinks = 16,
        mnav_settingTiles = 17,
        mnav_settingMemoryBytes = 18,
        mnav_settingMinRegionArea = 19,
        mnav_settingMaxEdgeError = 20,
        mnav_settingMaxEdgeLength = 21,
        mnav_settingDetailSampleDistance = 22,
        mnav_settingDetailMaxError = 23,
    };

    // A def check's outcome: the status, and the first setting it refused.
    typedef struct mnavBakeDefResult
    {
        mnavResult result;
        mnavBakeSetting setting;
    } mnavBakeDefResult;

    // What a def's meters become in cells (N8). A quotient within 2^-10 of
    // an integer counts as that integer; heights and the radius then round
    // up and the step height rounds down.
    typedef struct mnavBakeCells
    {
        // The agent's height, in cell heights.
        int32_t agentHeight;
        // The agent's step height, in cell heights.
        int32_t agentStep;
        // The agent's radius, in cells.
        int32_t agentRadius;
        // The border rasterized around each tile beyond its side, in
        // cells: the radius and 3 more.
        int32_t border;
        // The cosine of the maximum slope: a triangle is walkable when its
        // normal's Y is at least this times the normal's length.
        float cosMaxSlope;
        // A tile's side, in meters.
        float tileSize;
        // The minimum region area, in cells.
        int32_t minRegion;
        // The maximum edge error, in cells, unrounded.
        float edgeError;
        // The maximum wall edge length, in cells, rounded down; 0 for no
        // limit.
        int32_t edgeLength;
        // The detail sample distance, in sixteenths of a cell, rounded
        // down; 0 for none, at least 16 otherwise.
        int32_t detailSample;
        // The detail maximum error, in sixteenths of a cell height,
        // rounded down.
        int32_t detailError;
    } mnavBakeCells;

    // A triangle mesh in a bake's input.
    typedef struct mnavTriangleMesh
    {
        // The vertices, in meters relative to the bake's origin. Only read
        // during the call.
        const mnavVec3* vertices;
        // The number of vertices, at least 0.
        int32_t vertexCount;
        // Three vertex indices per triangle.
        const int32_t* indices;
        // The number of triangles, at least 0.
        int32_t triangleCount;
        // One area type per triangle, or NULL for mnav_areaWalkable on
        // every triangle.
        const mnavAreaType* areas;
    } mnavTriangleMesh;

    // The kind of input element a check refused.
    typedef uint8_t mnavInputElement;

    enum
    {
        // The input as a whole, or no element.
        mnav_elementNone = 0,
        mnav_elementVertex = 1,
        mnav_elementTriangle = 2,
    };

    // An input check's outcome: the status, and the first element it
    // refused, vertices before triangles, each in ascending index order.
    typedef struct mnavInputResult
    {
        mnavResult result;
        mnavInputElement element;
        // The element's index, or -1 for mnav_elementNone.
        int32_t index;
    } mnavInputResult;

    /// Returns the default bake def: cells of 0.25 m by 0.125 m, tiles of
    /// 128 cells, an agent 0.5 m in radius and 2 m tall that steps 0.75 m
    /// and walks slopes up to 45 degrees, regions of at least 2 square
    /// meters, walls within 0.3 m of the cells and at most 12 m long,
    /// detail samples every 1.5 m within 0.125 m, and limits sized for a
    /// large level.
    ///
    /// @return The def, with a valid cookie.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavBakeDef mnavDefaultBakeDef(void);

    /// Checks a bake def and converts its meters to cells.
    ///
    /// @param def       The def to check.
    /// @param cellsOut  Receives the settings in cells when the def is
    ///                  valid. May be NULL.
    /// @return `mnav_success`; `mnav_errorInvalid` with the setting for a
    /// NULL def, a def without its cookie, an allocator with one function,
    /// a value that is not finite or lies outside its range, an agent
    /// radius whose border is wider than a tile, or an agent height or
    /// step height past MNAV_MAX_HEIGHT_CELLS.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavBakeDefResult mnavValidateBakeDef(const mnavBakeDef* def,
                                                                  mnavBakeCells* cellsOut);

    /// Checks a triangle mesh as hostile input to a bake with a def.
    ///
    /// @param def   The bake def the mesh is for.
    /// @param mesh  The mesh.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, an
    /// invalid def, a negative count, a NULL array its count needs, a
    /// vertex coordinate that is not finite, a vertex index outside the
    /// vertices or an area type of MNAV_AREA_TYPES or more;
    /// `mnav_errorLimit` for more triangles than the def's inputTriangles
    /// limit; `mnav_errorRange` for a vertex more than MNAV_MAX_EXTENT_CELLS
    /// cells from the origin on the ground or MNAV_MAX_HEIGHT_CELLS cell
    /// heights above or below it. The element and index name the first
    /// offending vertex or triangle.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavInputResult mnavValidateTriangleMesh(const mnavBakeDef* def,
                                                                     const mnavTriangleMesh* mesh);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_BAKE_H
