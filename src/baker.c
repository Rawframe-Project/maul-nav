// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The baker: one call runs every stage of the bake for a tile (mnav-0003).

#include "allocator.h"
#include "bake_def.h"
#include "border_vertices.h"
#include "compact.h"
#include "contour.h"
#include "detail.h"
#include "erode.h"
#include "filter.h"
#include "heightfield.h"
#include "holes.h"
#include "input.h"
#include "polymesh.h"
#include "raster.h"
#include "region.h"
#include "tile.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct mnavBaker
{
    mnavBakeDef def;
    mnavBakeCells cells;
    mnavMemory memory;
    uint8_t* tile;
    size_t tileSize;
};

// The stages' results for one tile, released together.
typedef struct Stages
{
    mnavHeightfield heightfield;
    mnavCompactField compact;
    mnavRegionMap regions;
    mnavContourSet set;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
} Stages;

mnavBakeDefResult mnavCreateBaker(const mnavBakeDef* def, mnavBaker** bakerOut)
{
    if (bakerOut == nullptr)
    {
        return (mnavBakeDefResult){mnav_errorInvalid, mnav_settingNone};
    }
    *bakerOut = nullptr;
    mnavBakeCells cells = {0};
    mnavBakeDefResult checked = mnavCheckBakeDef(def, &cells);
    if (checked.result != mnav_success)
    {
        return checked;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, def->limits.memoryBytes);
    mnavBaker* baker = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavBaker), alignof(mnavBaker), (void**)&baker);
    if (result != mnav_success)
    {
        return (mnavBakeDefResult){result, mnav_settingNone};
    }
    *baker = (mnavBaker){*def, cells, memory, nullptr, 0};
    *bakerOut = baker;
    return (mnavBakeDefResult){mnav_success, mnav_settingNone};
}

static void DropTile(mnavBaker* baker)
{
    mnavReleaseTileBytes(&baker->memory, baker->tile, baker->tileSize);
    baker->tile = nullptr;
    baker->tileSize = 0;
}

void mnavDestroyBaker(mnavBaker* baker)
{
    if (baker == nullptr)
    {
        return;
    }
    DropTile(baker);
    mnavMemory memory = baker->memory;
    mnavRelease(&memory, baker, 1, sizeof(mnavBaker), alignof(mnavBaker));
}

// Checks every mesh as hostile input and their total against the input
// limit; names the first refused mesh in the report.
static mnavResult CheckInput(const mnavBaker* baker, const mnavTriangleMesh* meshes,
                             int32_t meshCount, mnavBakeReport* report)
{
    if (meshCount < 0 || (meshCount > 0 && meshes == nullptr))
    {
        return mnav_errorInvalid;
    }
    int64_t total = 0;
    for (int32_t m = 0; m < meshCount; ++m)
    {
        mnavInputResult input = mnavCheckTriangleMesh(&baker->def, &meshes[m]);
        if (input.result != mnav_success)
        {
            report->mesh = m;
            report->input = input;
            return input.result;
        }
        total += meshes[m].triangleCount;
    }
    return total > baker->def.limits.inputTriangles ? mnav_errorLimit : mnav_success;
}

static uint64_t HashWords(uint64_t hash, const uint32_t* words, int32_t count)
{
    return mnavHash64(hash, words, count * (int32_t)sizeof(uint32_t));
}

static uint32_t Bits32(float f)
{
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

static uint64_t Bits64(double d)
{
    uint64_t bits = 0;
    memcpy(&bits, &d, sizeof(bits));
    return bits;
}

// The hash of the generator, the settings that shape the tile and its
// place.
static uint64_t HashSettings(const mnavBakeDef* def, int32_t tileX, int32_t tileZ)
{
    mnavVersion version = mnavGetVersion();
    uint64_t origin[3] = {Bits64(def->origin.x), Bits64(def->origin.y), Bits64(def->origin.z)};
    const uint32_t words[] = {
        version.major,
        version.minor,
        version.patch,
        (uint32_t)origin[0],
        (uint32_t)(origin[0] >> 32),
        (uint32_t)origin[1],
        (uint32_t)(origin[1] >> 32),
        (uint32_t)origin[2],
        (uint32_t)(origin[2] >> 32),
        Bits32(def->cellSize),
        Bits32(def->cellHeight),
        (uint32_t)def->tileCells,
        Bits32(def->agent.radius),
        Bits32(def->agent.height),
        Bits32(def->agent.stepHeight),
        Bits32(def->agent.maxSlopeDegrees),
        Bits32(def->minRegionArea),
        Bits32(def->maxEdgeError),
        Bits32(def->maxEdgeLength),
        Bits32(def->detailSampleDistance),
        Bits32(def->detailMaxError),
        (uint32_t)tileX,
        (uint32_t)tileZ,
    };
    return HashWords(MNAV_HASH_INIT, words, (int32_t)(sizeof(words) / sizeof(words[0])));
}

// Adds every input triangle that reaches the tile, as rasterization picks
// them, to the fingerprint, and counts them.
static uint64_t HashInput(const mnavBaker* baker, const mnavTileFrame* frame,
                          const mnavTriangleMesh* meshes, int32_t meshCount, uint64_t hash,
                          int32_t* count)
{
    *count = 0;
    for (int32_t m = 0; m < meshCount; ++m)
    {
        const mnavTriangleMesh* mesh = &meshes[m];
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            const int32_t* index = mesh->indices + (size_t)t * 3;
            const mnavVec3 corners[3] = {mesh->vertices[index[0]], mesh->vertices[index[1]],
                                         mesh->vertices[index[2]]};
            mnavAreaType given = mesh->areas != nullptr ? mesh->areas[t] : mnav_areaWalkable;
            int32_t area = mnavTriangleArea(corners, given, baker->cells.cosMaxSlope);
            if (area < 0 || !mnavTriangleTouchesTile(frame, corners))
            {
                continue;
            }
            uint32_t words[10];
            for (int32_t c = 0; c < 3; ++c)
            {
                words[c * 3 + 0] = Bits32(corners[c].x);
                words[c * 3 + 1] = Bits32(corners[c].y);
                words[c * 3 + 2] = Bits32(corners[c].z);
            }
            words[9] = (uint32_t)area;
            hash = HashWords(hash, words, 10);
            *count += 1;
        }
    }
    return hash;
}

// The detail lookup's search radius: the wall error rounded up, at least
// one cell.
static int32_t SearchRadius(float edgeError)
{
    int32_t radius = (int32_t)ceilf(edgeError);
    return radius < 1 ? 1 : radius;
}

// Runs the stages in order, recording each in the report before it runs.
static mnavResult RunStages(mnavBaker* baker, const mnavTriangleMesh* meshes, int32_t meshCount,
                            int32_t tileX, int32_t tileZ, Stages* s, mnavBakeReport* report)
{
    mnavMemory* memory = &baker->memory;
    const mnavBakeDef* def = &baker->def;
    const mnavBakeCells* cells = &baker->cells;
    report->stage = mnav_stageRasterize;
    mnavResult result =
        mnavBuildHeightfield(memory, def, cells, meshes, meshCount, tileX, tileZ, &s->heightfield);
    if (result != mnav_success)
    {
        return result;
    }
    mnavFilterWalkable(&s->heightfield, cells->agentHeight, cells->agentStep);
    report->stage = mnav_stageCompact;
    result = mnavBuildCompactField(memory, &s->heightfield, cells->agentHeight, cells->agentStep,
                                   &s->compact);
    if (result == mnav_success)
    {
        report->stage = mnav_stageErode;
        result = mnavErode(memory, &s->compact, cells->agentRadius);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageRegions;
        result =
            mnavBuildRegions(memory, &s->compact, cells->border, cells->minRegion, &s->regions);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageContours;
        result = mnavBuildContours(memory, &s->compact, &s->regions, cells->border,
                                   cells->edgeError, cells->edgeLength, &s->set);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageHoles;
        result = mnavMergeHoles(memory, &s->set, s->regions.count);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stagePolygons;
        result = mnavBuildPolyMesh(memory, &s->set, def->tileCells, def->limits.tileVertices,
                                   def->limits.tilePolygons, &s->mesh);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageBorderVertices;
        result = mnavRemoveBorderVertices(memory, &s->mesh, def->limits.tilePolygons);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageLinks;
        result = mnavLinkPolyMesh(memory, &s->mesh);
    }
    if (result == mnav_success)
    {
        report->stage = mnav_stageDetail;
        mnavDetailSettings settings = {cells->detailSample, cells->detailError,
                                       SearchRadius(cells->edgeError), cells->border};
        result =
            mnavBuildDetailMesh(memory, &s->compact, &s->regions, &s->mesh, settings, &s->detail);
    }
    return result;
}

static void ReleaseStages(mnavMemory* memory, Stages* s)
{
    mnavReleaseDetailMesh(memory, &s->detail);
    mnavReleasePolyMesh(memory, &s->mesh);
    mnavReleaseContours(memory, &s->set);
    mnavReleaseRegions(memory, &s->regions);
    mnavReleaseCompactField(memory, &s->compact);
    mnavReleaseHeightfield(memory, &s->heightfield);
}

// Writes the stages' counts into the report.
static void Count(const Stages* s, mnavBakeReport* report)
{
    report->spans = s->compact.spanCount;
    report->regions = (int32_t)s->regions.count;
    report->polygons = s->mesh.polygonCount;
    report->vertices = s->mesh.vertexCount;
    report->detailVertices = s->detail.vertexCount;
    report->detailTriangles = s->detail.triangleCount;
    report->droppedHoles = s->set.droppedHoles;
    report->partialRings = s->mesh.failedRings + s->detail.failedPolygons;
    report->fallbackHeights = s->detail.fallbackHeights;
    report->cappedDetail = s->detail.cappedPolygons;
}

// Encodes the tile from the stages.
static mnavResult Encode(mnavBaker* baker, int32_t tileX, int32_t tileZ, const Stages* s,
                         uint64_t fingerprint)
{
    const mnavBakeDef* def = &baker->def;
    const mnavBakeCells* cells = &baker->cells;
    mnavTileInfo info = {mnavGetVersion(),   fingerprint,      tileX,           tileZ,
                         def->tileCells,     def->cellSize,    def->cellHeight, cells->agentHeight,
                         cells->agentRadius, cells->agentStep, def->origin};
    return mnavEncodeTile(&baker->memory, &info, &s->mesh, &s->detail, &baker->tile,
                          &baker->tileSize);
}

mnavResult mnavBakeTile(mnavBaker* baker, const mnavTriangleMesh* meshes, int32_t meshCount,
                        int32_t tileX, int32_t tileZ, mnavBakeReport* reportOut)
{
    mnavBakeReport report = {0};
    report.mesh = -1;
    report.input = (mnavInputResult){mnav_success, mnav_elementNone, -1};
    if (baker == nullptr)
    {
        report.result = mnav_errorInvalid;
        if (reportOut != nullptr)
        {
            *reportOut = report;
        }
        return mnav_errorInvalid;
    }
    DropTile(baker);
    baker->memory.peak = baker->memory.used;
    report.stage = mnav_stageInput;
    mnavResult result = CheckInput(baker, meshes, meshCount, &report);
    mnavTileFrame frame = {0};
    if (result == mnav_success &&
        !mnavMakeTileFrame(&baker->def, &baker->cells, tileX, tileZ, &frame))
    {
        result = mnav_errorRange;
    }
    Stages stages = {0};
    if (result == mnav_success)
    {
        uint64_t settings = HashSettings(&baker->def, tileX, tileZ);
        report.fingerprint =
            HashInput(baker, &frame, meshes, meshCount, settings, &report.triangles);
        result = RunStages(baker, meshes, meshCount, tileX, tileZ, &stages, &report);
    }
    if (result == mnav_success)
    {
        Count(&stages, &report);
        report.stage = mnav_stageEncode;
        result = Encode(baker, tileX, tileZ, &stages, report.fingerprint);
    }
    ReleaseStages(&baker->memory, &stages);
    if (result == mnav_success)
    {
        report.stage = mnav_stageDone;
        report.tileBytes = baker->tileSize;
    }
    report.result = result;
    report.memoryPeak = baker->memory.peak;
    if (reportOut != nullptr)
    {
        *reportOut = report;
    }
    return result;
}

mnavResult mnavCopyBakedTile(const mnavBaker* baker, uint8_t* buffer, size_t capacity,
                             size_t* sizeOut)
{
    if (baker == nullptr || (buffer == nullptr && capacity > 0) || baker->tile == nullptr)
    {
        return mnav_errorInvalid;
    }
    if (sizeOut != nullptr)
    {
        *sizeOut = baker->tileSize;
    }
    if (capacity < baker->tileSize)
    {
        return mnav_errorCapacity;
    }
    if (buffer != nullptr)
    {
        memcpy(buffer, baker->tile, baker->tileSize);
    }
    return mnav_success;
}
