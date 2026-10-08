// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flight volumes (mnav-0015): where agents that fly or swim may go, baked
// per tile from the navmesh bake's input into a compact sparse voxel
// octree, loaded from bytes, staged and committed together as navmesh
// tiles are.

#ifndef MAUL_NAV_FLIGHT_H
#define MAUL_NAV_FLIGHT_H

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/navmesh.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The flight tile format's version, written into every flight tile.
#define MNAV_FLIGHT_FORMAT 1

// The range of a flight tile's side, in voxels: 4 times a power of 2.
#define MNAV_MIN_FLIGHT_TILE_VOXELS 16
#define MNAV_MAX_FLIGHT_TILE_VOXELS 512

// The largest values a flight tile's node and leaf limits may take.
#define MNAV_MAX_FLIGHT_TILE_NODES  16777216
#define MNAV_MAX_FLIGHT_TILE_LEAVES 16777216

    // The named limits that bound a flight bake's work and a flight
    // volume's size. Each is at least 1 and at most its MNAV_MAX_ value.
    // Reaching one is mnav_errorLimit.
    typedef struct mnavFlightLimits
    {
        // Input triangles a bake reads, as mnavBakeLimits counts them.
        int32_t inputTriangles;
        // Solid spans in a tile's columns, at the voxel size.
        int32_t tileSpans;
        // Octree nodes and leaves in a tile.
        int32_t tileNodes;
        int32_t tileLeaves;
        // Tiles committed in a flight volume at once.
        int32_t tiles;
        // The most memory a baker or a flight volume holds at once; more
        // than 0.
        uint64_t memoryBytes;
    } mnavFlightLimits;

    // How flight tiles are baked and what a flight volume accepts. Made
    // by mnavDefaultFlightDef and then changed.
    typedef struct mnavFlightDef
    {
        // Set by mnavDefaultFlightDef; a def not made by it is refused.
        uint32_t cookie;
        // Where memory comes from; zeroed for the C library's functions.
        mnavAllocator allocator;
        // The world position of the input's (0, 0, 0), as in mnavBakeDef.
        mnavPos3 origin;
        // A voxel's side, in meters, from MNAV_MIN_CELL_SIZE to
        // MNAV_MAX_CELL_SIZE.
        float voxelSize;
        // A tile's side in voxels: 4 times a power of 2, from
        // MNAV_MIN_FLIGHT_TILE_VOXELS to MNAV_MAX_FLIGHT_TILE_VOXELS.
        int32_t tileVoxels;
        // The volume's bottom and top, in meters from the origin. The
        // volume runs from the floor rounded down to a voxel up in whole
        // cubes of the tile's side until it holds the ceiling, within
        // MNAV_MAX_HEIGHT_CELLS voxels of the origin.
        float floor;
        float ceiling;
        // The flier's radius, at least 0: the space it flies through
        // keeps this far from anything solid, rounded up to voxels, at
        // most the tile's side less 3 voxels.
        float radius;
        // Whether each column is solid from the floor up to its lowest
        // solid voxel, so that space beneath the ground is not open. Off
        // for open space such as a field of floating debris.
        bool groundBelow;
        mnavFlightLimits limits;
    } mnavFlightDef;

    // The setting a flight def check refused.
    typedef uint8_t mnavFlightSetting;
    enum
    {
        mnav_flightSettingNone = 0,
        mnav_flightSettingCookie = 1,
        mnav_flightSettingAllocator = 2,
        mnav_flightSettingOrigin = 3,
        mnav_flightSettingVoxelSize = 4,
        mnav_flightSettingTileVoxels = 5,
        mnav_flightSettingFloor = 6,
        mnav_flightSettingCeiling = 7,
        mnav_flightSettingRadius = 8,
        mnav_flightSettingInputTriangles = 9,
        mnav_flightSettingTileSpans = 10,
        mnav_flightSettingTileNodes = 11,
        mnav_flightSettingTileLeaves = 12,
        mnav_flightSettingTiles = 13,
        mnav_flightSettingMemoryBytes = 14,
    };

    // A flight def check's outcome: the status, and the first setting it
    // refused.
    typedef struct mnavFlightDefResult
    {
        mnavResult result;
        mnavFlightSetting setting;
    } mnavFlightDefResult;

    // What a flight bake did.
    typedef struct mnavFlightBakeReport
    {
        mnavResult result;
        // The first mesh refused, or -1, and what its check found, as in
        // mnavBakeReport.
        int32_t mesh;
        mnavInputResult input;
        // The hash of everything that shaped the tile: the generator's
        // version, the def's settings, the tile's place and the input that
        // reaches it. Written into the tile.
        uint64_t fingerprint;
        // Input triangles that reach the tile, solid spans, the tile's
        // cubes, nodes and leaves, its bytes, and the most memory the bake
        // held at once.
        int32_t triangles;
        int32_t spans;
        int32_t cubes;
        int32_t nodes;
        int32_t leaves;
        uint64_t tileBytes;
        uint64_t memoryPeak;
    } mnavFlightBakeReport;

    // A flight tile baker. Made by mnavCreateFlightBaker.
    typedef struct mnavFlightBaker mnavFlightBaker;

    // A flight volume: committed flight tiles queries read. Made by
    // mnavCreateFlightVolume.
    typedef struct mnavFlightVolume mnavFlightVolume;

    /// Makes a flight def with every setting at its default: 1 m voxels,
    /// tiles of 32 voxels, a volume from 0 to 64 m, a flier of 0.5 m, the
    /// ground below solid, and limits for a medium world.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavFlightDef mnavDefaultFlightDef(void);

    /// Checks a flight def.
    ///
    /// @param def  The def.
    /// @return `mnav_success`; `mnav_errorInvalid` with the first setting
    /// refused, or for a NULL def.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavFlightDefResult mnavValidateFlightDef(const mnavFlightDef* def);

    /// Makes a flight baker: checks a def and keeps a copy, its allocator
    /// and its memory limit.
    ///
    /// @param def       The def.
    /// @param bakerOut  Receives the baker, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` with the setting for an
    /// invalid def or a NULL argument; `mnav_errorLimit` when the baker
    /// does not fit the def's memory limit; `mnav_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavFlightDefResult mnavCreateFlightBaker(const mnavFlightDef* def,
                                                                      mnavFlightBaker** bakerOut);

    /// Destroys a flight baker and the tile it holds.
    ///
    /// @param baker  The baker, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the baker is used by one thread at a time.
    MNAV_API void mnavDestroyFlightBaker(mnavFlightBaker* baker);

    /// Bakes one flight tile from the input a navmesh bake reads
    /// (mnav-0015): every triangle is rasterized at the voxel size, and a
    /// voxel any triangle touches is solid; with groundBelow, so is each
    /// column below its lowest solid voxel; then every voxel within the
    /// flier's radius of a solid one, by whole voxels, is blocked too. A
    /// tile index in the input is not used: it lists triangles for a
    /// navmesh's tiles. The tile's bytes are the same on every platform.
    ///
    /// @param baker     The baker; the tile it held before is dropped.
    /// @param input     The meshes, terrains and volumes, as
    ///                  mnavBakeTileInput reads them.
    /// @param tileX     The tile's column.
    /// @param tileZ     The tile's row.
    /// @param reportOut Receives the report. May be NULL.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL baker or
    /// input or input mnavBakeTileInput refuses, the mesh named in the
    /// report; `mnav_errorRange` for input or a tile past the extent;
    /// `mnav_errorLimit` past a limit; `mnav_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the baker is used by one thread at a time.
    MNAV_NODISCARD MNAV_API mnavResult mnavBakeFlightTile(mnavFlightBaker* baker,
                                                          const mnavBakeInput* input, int32_t tileX,
                                                          int32_t tileZ,
                                                          mnavFlightBakeReport* reportOut);

    /// Copies the last baked flight tile's bytes into caller memory.
    ///
    /// @param baker     The baker.
    /// @param buffer    The memory, at least capacity bytes; may be NULL
    ///                  when capacity is 0.
    /// @param capacity  The buffer's size in bytes.
    /// @param sizeOut   Receives the tile's size in bytes. May be NULL.
    /// @return `mnav_success`; `mnav_errorCapacity` when the buffer is
    /// smaller than the tile, with the size still written;
    /// `mnav_errorInvalid` for a NULL baker, a NULL buffer with a
    /// positive capacity, or a baker holding no tile.
    /// @par Thread safety
    /// Safe from any thread; the baker is used by one thread at a time.
    MNAV_NODISCARD MNAV_API mnavResult mnavCopyFlightTile(const mnavFlightBaker* baker,
                                                          uint8_t* buffer, size_t capacity,
                                                          size_t* sizeOut);

    /// Makes an empty flight volume for tiles baked with a def: checks the
    /// def and keeps a copy, its allocator and its limits.
    ///
    /// @param def        The def the tiles are baked with.
    /// @param volumeOut  Receives the volume, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` with the setting for an
    /// invalid def or a NULL argument; `mnav_errorLimit` when the volume
    /// does not fit the def's memory limit; `mnav_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavFlightDefResult
    mnavCreateFlightVolume(const mnavFlightDef* def, mnavFlightVolume** volumeOut);

    /// Destroys a flight volume, its tiles and everything staged.
    ///
    /// @param volume  The volume, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the volume is used by one thread at a time.
    MNAV_API void mnavDestroyFlightVolume(mnavFlightVolume* volume);

    /// Loads flight tile bytes, checking every field as hostile input and
    /// the tile's settings against the volume's def, and stages the tile
    /// for the next commit, in place of anything staged at its place
    /// before. Queries do not see it until the commit.
    ///
    /// @param volume  The volume.
    /// @param bytes   The tile's bytes, as mnavCopyFlightTile gives them.
    /// @param size    Their size in bytes.
    /// @return `mnav_success`; `mnav_errorInvalid` naming the section and
    /// element for malformed bytes, a tile baked with other settings (the
    /// header) or a NULL argument; `mnav_errorVersion` for another flight
    /// tile format version; `mnav_errorLimit` for a tile past the def's
    /// node or leaf limit (the header) or past the memory limit;
    /// `mnav_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the volume is used by one thread at a time.
    /// Staging changes nothing queries read, but it may not run beside
    /// another call that stages or commits.
    MNAV_NODISCARD MNAV_API mnavTileResult mnavStageFlightTile(mnavFlightVolume* volume,
                                                               const uint8_t* bytes, size_t size);

    /// Stages the removal of the flight tile at a place for the next
    /// commit, in place of anything staged there before.
    ///
    /// @param volume  The volume.
    /// @param tileX   The tile's column.
    /// @param tileZ   The tile's row.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL volume;
    /// `mnav_errorLimit` past the memory limit; `mnav_errorCapacity` when
    /// the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the volume is used by one thread at a time.
    MNAV_NODISCARD MNAV_API mnavResult mnavStageFlightTileRemoval(mnavFlightVolume* volume,
                                                                  int32_t tileX, int32_t tileZ);

    /// Applies everything staged at once: installs and removes the tiles.
    /// Either all of it applies or, on failure, nothing does and the
    /// staged changes stay.
    ///
    /// @param volume  The volume.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL volume;
    /// `mnav_errorLimit` when the tiles would pass the tiles limit or
    /// memory its limit; `mnav_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the volume is used by one thread at a time,
    /// so no query may run during a commit.
    MNAV_NODISCARD MNAV_API mnavResult mnavCommitFlight(mnavFlightVolume* volume);

    /// Reads the fingerprint of the flight tile committed at a place, as
    /// its bake reported it, so that a host can tell whether its input
    /// has changed since.
    ///
    /// @param volume          The volume.
    /// @param tileX           The tile's column.
    /// @param tileZ           The tile's row.
    /// @param fingerprintOut  Receives the fingerprint, or 0.
    /// @return `mnav_success`; `mnav_errorNotLoaded` when no tile is
    /// committed there; `mnav_errorInvalid` for a NULL argument.
    /// @par Thread safety
    /// Safe from any thread. Any number of threads may read the volume at
    /// once between commits; none may while a stage or commit call runs.
    MNAV_NODISCARD MNAV_API mnavResult mnavGetFlightTile(const mnavFlightVolume* volume,
                                                         int32_t tileX, int32_t tileZ,
                                                         uint64_t* fingerprintOut);

    /// Tells whether the flier's center may be at a point: whether the
    /// voxel holding it is neither solid nor within the flier's radius of
    /// a solid voxel.
    ///
    /// @param volume   The volume.
    /// @param point    The point, in world coordinates.
    /// @param openOut  Receives whether the point is open; false on
    ///                 failure.
    /// @return `mnav_success`; `mnav_errorNotLoaded` when no tile is
    /// committed under the point; `mnav_errorRange` for a point not
    /// finite or below or above the volume; `mnav_errorInvalid` for a
    /// NULL argument.
    /// @par Thread safety
    /// Safe from any thread. Any number of threads may read the volume at
    /// once between commits; none may while a stage or commit call runs.
    MNAV_NODISCARD MNAV_API mnavResult mnavIsFlightOpen(const mnavFlightVolume* volume,
                                                        mnavPos3 point, bool* openOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_FLIGHT_H
