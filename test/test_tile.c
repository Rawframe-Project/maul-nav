// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tile format and its loader (mnav-0003). Run as test_tile --seed <file> it
// also writes the level's tile, the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates.
#define _CRT_SECURE_NO_WARNINGS

#include "allocator.h"
#include "bake_tile.h"
#include "detail.h"
#include "polymesh.h"
#include "soup.h"
#include "test_harness.h"
#include "tile.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The hash of the level tile's bytes, the same on every platform.
#define LEVEL_TILE_HASH 0xc889acd9ae9749c1ull

// Offsets into the header.
enum
{
    FORMAT_AT = 4,
    HEADER_BYTES_AT = 6,
    FLAGS_AT = 14,
    HASH_AT = 24,
    PAYLOAD_AT = 32,
    TILE_CELLS_AT = 44,
    CELL_SIZE_AT = 48,
    ORIGIN_AT = 64,
    VERTICES_AT = 88,
    DETAIL_TRIANGLES_AT = 96,
    // The offset cell height of a hand tile's floor.
    BASE = 32768
};

static void PutLittle(uint8_t* at, uint64_t value, int32_t bytes)
{
    for (int32_t k = 0; k < bytes; ++k)
    {
        at[k] = (uint8_t)(value >> (8 * k));
    }
}

// Sets the header's payload size and hash to match the bytes, so a change
// reaches the checks behind them.
static void Reseal(uint8_t* bytes, size_t size)
{
    size_t payload = size - MNAV_TILE_HEADER_BYTES;
    PutLittle(bytes + PAYLOAD_AT, payload, 4);
    PutLittle(bytes + HASH_AT,
              mnavHash64(MNAV_HASH_INIT, bytes + MNAV_TILE_HEADER_BYTES, (int32_t)payload), 8);
}

typedef struct Fixture
{
    mnavMemory memory;
    BakedTile baked;
    mnavTileInfo info;
    uint8_t* bytes;
    size_t size;
} Fixture;

static void Bake(Fixture* f)
{
    static mnavVec3 vertices[LEVEL_VERTICES];
    static int32_t indices[LEVEL_TRIANGLES * 3];
    MakeLevel(vertices, indices);
    mnavTriangleMesh input = {vertices, LEVEL_VERTICES, indices, LEVEL_TRIANGLES, nullptr};
    mnavBakeDef def = mnavDefaultBakeDef();
    def.origin = (mnavPos3){1000.5, -20.25, 4096.0};
    mnavBakeCells cells;
    CHECK(mnavValidateBakeDef(&def, &cells).result == mnav_success, "def");
    f->memory = mnavMakeMemory(def.allocator, def.limits.memoryBytes);
    CHECK(BakeTile(&f->memory, &def, &cells, &input, &f->baked) == mnav_success, "baked");
    f->info = (mnavTileInfo){(mnavVersion){1, 2, 3},
                             0x0123456789ABCDEFull,
                             0,
                             0,
                             def.tileCells,
                             def.cellSize,
                             def.cellHeight,
                             cells.agentHeight,
                             cells.agentRadius,
                             cells.agentStep,
                             def.origin};
    CHECK(mnavEncodeTile(&f->memory, &f->info, &f->baked.mesh, &f->baked.detail, &f->bytes,
                         &f->size) == mnav_success,
          "encoded");
}

static void Unbake(Fixture* f)
{
    mnavReleaseTileBytes(&f->memory, f->bytes, f->size);
    ReleaseBakedTile(&f->memory, &f->baked);
    CHECK(f->memory.used == 0, "everything released");
}

static bool SameInfo(const mnavTileInfo* a, const mnavTileInfo* b)
{
    return a->generator.major == b->generator.major && a->generator.minor == b->generator.minor &&
           a->generator.patch == b->generator.patch && a->fingerprint == b->fingerprint &&
           a->x == b->x && a->z == b->z && a->tileCells == b->tileCells &&
           a->cellSize == b->cellSize && a->cellHeight == b->cellHeight &&
           a->agentHeight == b->agentHeight && a->agentRadius == b->agentRadius &&
           a->agentStep == b->agentStep && a->origin.x == b->origin.x &&
           a->origin.y == b->origin.y && a->origin.z == b->origin.z;
}

static bool SameMesh(const mnavPolyMesh* a, const mnavPolyMesh* b)
{
    if (a->vertexCount != b->vertexCount || a->polygonCount != b->polygonCount ||
        a->tileCells != b->tileCells)
    {
        return false;
    }
    for (int32_t v = 0; v < a->vertexCount; ++v)
    {
        const mnavMeshVertex* p = &a->vertices[v];
        const mnavMeshVertex* q = &b->vertices[v];
        if (p->x != q->x || p->y != q->y || p->z != q->z)
        {
            return false;
        }
    }
    for (int32_t p = 0; p < a->polygonCount; ++p)
    {
        const mnavPolygon* s = &a->polygons[p];
        const mnavPolygon* t = &b->polygons[p];
        if (s->count != t->count || s->area != t->area ||
            memcmp(s->vertices, t->vertices, sizeof(s->vertices)) != 0 ||
            memcmp(s->neighbors, t->neighbors, sizeof(s->neighbors)) != 0 ||
            memcmp(s->sides, t->sides, sizeof(s->sides)) != 0)
        {
            return false;
        }
    }
    return true;
}

static bool SameDetail(const mnavDetailMesh* a, const mnavDetailMesh* b)
{
    if (a->partCount != b->partCount || a->vertexCount != b->vertexCount ||
        a->triangleCount != b->triangleCount)
    {
        return false;
    }
    for (int32_t p = 0; p < a->partCount; ++p)
    {
        const mnavDetailPart* s = &a->parts[p];
        const mnavDetailPart* t = &b->parts[p];
        if (s->firstVertex != t->firstVertex || s->firstTriangle != t->firstTriangle ||
            s->vertexCount != t->vertexCount || s->triangleCount != t->triangleCount)
        {
            return false;
        }
    }
    for (int32_t v = 0; v < a->vertexCount; ++v)
    {
        const mnavDetailVertex* s = &a->vertices[v];
        const mnavDetailVertex* t = &b->vertices[v];
        if (s->x != t->x || s->y != t->y || s->z != t->z)
        {
            return false;
        }
    }
    return memcmp(a->triangles, b->triangles,
                  (size_t)a->triangleCount * sizeof(mnavDetailTriangle)) == 0;
}

// Decodes bytes; when they load, encodes the result again and requires
// the same bytes back. Returns the load's result.
static mnavTileResult Load(mnavMemory* memory, const uint8_t* bytes, size_t size)
{
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    mnavTileResult result = mnavDecodeTile(memory, bytes, size, &info, &mesh, &detail);
    if (result.result == mnav_success)
    {
        uint8_t* again = nullptr;
        size_t againSize = 0;
        CHECK(mnavEncodeTile(memory, &info, &mesh, &detail, &again, &againSize) == mnav_success,
              "encoded again");
        CHECK(againSize == size && memcmp(again, bytes, size) == 0, "the same bytes again");
        mnavReleaseTileBytes(memory, again, againSize);
        mnavReleaseDetailMesh(memory, &detail);
        mnavReleasePolyMesh(memory, &mesh);
    }
    return result;
}

static void TestRoundTrip(void)
{
    static Fixture f;
    Bake(&f);
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    mnavTileResult result = mnavDecodeTile(&f.memory, f.bytes, f.size, &info, &mesh, &detail);
    CHECK(result.result == mnav_success, "decoded");
    CHECK(SameInfo(&info, &f.info), "the same header");
    CHECK(SameMesh(&mesh, &f.baked.mesh), "the same polygons");
    CHECK(SameDetail(&detail, &f.baked.detail), "the same detail");
    mnavReleaseDetailMesh(&f.memory, &detail);
    mnavReleasePolyMesh(&f.memory, &mesh);
    CHECK(Load(&f.memory, f.bytes, f.size).result == mnav_success, "loads and writes back");
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, f.bytes, (int32_t)f.size);
    printf("LEVEL_TILE_HASH=%016llx bytes=%zu\n", (unsigned long long)hash, f.size);
    CHECK(hash == LEVEL_TILE_HASH, "the pinned hash");
    Unbake(&f);
}

static void TestEveryTruncationIsRefused(void)
{
    static Fixture f;
    Bake(&f);
    static uint8_t copy[1 << 16];
    CHECK(f.size <= sizeof(copy), "room");
    uint64_t held = f.memory.used;
    for (size_t size = 0; size < f.size; ++size)
    {
        memcpy(copy, f.bytes, size);
        if (size >= MNAV_TILE_HEADER_BYTES)
        {
            Reseal(copy, size);
        }
        CHECK(Load(&f.memory, copy, size).result == mnav_errorInvalid, "refused");
    }
    CHECK(f.memory.used == held, "nothing held after a refusal");
    Unbake(&f);
}

static void TestTrailingBytesAreRefused(void)
{
    // A tile with bytes past its last section, sealed so that its size
    // and hash agree: the payload is longer than the sections it holds.
    static Fixture f;
    Bake(&f);
    static uint8_t copy[(1 << 16) + 8];
    CHECK(f.size + 8 <= sizeof(copy), "room");
    uint64_t held = f.memory.used;
    for (size_t extra = 1; extra <= 8; ++extra)
    {
        memcpy(copy, f.bytes, f.size);
        memset(copy + f.size, 0, extra);
        Reseal(copy, f.size + extra);
        mnavTileResult r = Load(&f.memory, copy, f.size + extra);
        CHECK(r.result == mnav_errorInvalid && r.section == mnav_tilePayload,
              "trailing bytes refused as the payload's");
    }
    CHECK(f.memory.used == held, "nothing held after a refusal");
    Unbake(&f);
}

static void TestEveryByteChangeIsRefusedOrCanonical(void)
{
    static Fixture f;
    Bake(&f);
    static uint8_t copy[1 << 16];
    uint64_t held = f.memory.used;
    int32_t loaded = 0;
    int32_t refused = 0;
    const uint8_t flips[3] = {0x01, 0x80, 0xFF};
    for (size_t at = 0; at < f.size; ++at)
    {
        for (int32_t k = 0; k < 3; ++k)
        {
            memcpy(copy, f.bytes, f.size);
            copy[at] ^= flips[k];
            bool sealField = at >= HASH_AT && at < PAYLOAD_AT + 4;
            if (!sealField)
            {
                Reseal(copy, f.size);
            }
            mnavTileResult result = Load(&f.memory, copy, f.size);
            loaded += result.result == mnav_success ? 1 : 0;
            refused += result.result != mnav_success ? 1 : 0;
            CHECK(!sealField || result.result != mnav_success, "a damaged seal is refused");
        }
    }
    printf("byte changes: %d load, %d refused\n", loaded, refused);
    CHECK(loaded > 0 && refused > 0, "some of each");
    CHECK(f.memory.used == held, "nothing held");
    Unbake(&f);
}

// The offset of polygon p's record in the tile's bytes.
static size_t PolygonAt(const Fixture* f, int32_t p)
{
    size_t at = MNAV_TILE_HEADER_BYTES + (size_t)f->baked.mesh.vertexCount * 6;
    for (int32_t q = 0; q < p; ++q)
    {
        at += 2 + (size_t)f->baked.mesh.polygons[q].count * 5;
    }
    return at;
}

// Applies a change to a copy, reseals it, and returns the load's result.
static mnavTileResult Changed(Fixture* f, size_t at, uint64_t value, int32_t bytes, bool reseal)
{
    static uint8_t copy[1 << 16];
    memcpy(copy, f->bytes, f->size);
    PutLittle(copy + at, value, bytes);
    if (reseal)
    {
        Reseal(copy, f->size);
    }
    return Load(&f->memory, copy, f->size);
}

static bool Is(mnavTileResult r, mnavResult result, mnavTileSection section)
{
    return r.result == result && r.section == section;
}

static void TestHeaderRefusals(void)
{
    static Fixture f;
    Bake(&f);
    CHECK(Is(Changed(&f, 0, 'X', 1, true), mnav_errorInvalid, mnav_tileHeader), "magic");
    CHECK(Is(Changed(&f, FORMAT_AT, 2, 2, true), mnav_errorVersion, mnav_tileHeader),
          "another format version");
    CHECK(Is(Changed(&f, HEADER_BYTES_AT, 108, 2, true), mnav_errorInvalid, mnav_tileHeader),
          "header size");
    CHECK(Is(Changed(&f, FLAGS_AT, 1, 2, true), mnav_errorInvalid, mnav_tileHeader), "flags");
    CHECK(Is(Changed(&f, TILE_CELLS_AT, 8, 2, true), mnav_errorInvalid, mnav_tileHeader),
          "tile cells");
    CHECK(Is(Changed(&f, CELL_SIZE_AT, 0x7FC00000u, 4, true), mnav_errorInvalid, mnav_tileHeader),
          "NaN cell size");
    CHECK(Is(Changed(&f, ORIGIN_AT, 0x7FF0000000000000ull, 8, true), mnav_errorInvalid,
             mnav_tileHeader),
          "infinite origin");
    CHECK(Is(Changed(&f, HASH_AT, 0, 8, false), mnav_errorInvalid, mnav_tilePayload), "hash");
    CHECK(Is(Changed(&f, PAYLOAD_AT, f.size, 4, false), mnav_errorInvalid, mnav_tilePayload),
          "payload size");
    CHECK(Is(Changed(&f, VERTICES_AT, 60000, 2, true), mnav_errorInvalid, mnav_tilePayload),
          "counts past the payload");
    CHECK(mnavDecodeTile(&f.memory, nullptr, 0, &(mnavTileInfo){0}, &(mnavPolyMesh){0},
                         &(mnavDetailMesh){0})
                  .result == mnav_errorInvalid,
          "no bytes");
    Unbake(&f);
}

static void TestMeshRefusals(void)
{
    static Fixture f;
    Bake(&f);
    const mnavPolyMesh* mesh = &f.baked.mesh;
    // A polygon with a neighbor on its first edge, and one with an
    // unlinked edge that lies on no tile side.
    int32_t linked = -1;
    int32_t loose = -1;
    int32_t looseEdge = 0;
    for (int32_t p = 0; p < mesh->polygonCount; ++p)
    {
        const mnavPolygon* polygon = &mesh->polygons[p];
        linked = linked < 0 && polygon->neighbors[0] != MNAV_NO_INDEX ? p : linked;
        for (int32_t k = 0; k < polygon->count && loose < 0; ++k)
        {
            if (polygon->neighbors[k] == MNAV_NO_INDEX && polygon->sides[k] == 0)
            {
                loose = p;
                looseEdge = k;
            }
        }
    }
    CHECK(linked >= 0 && loose >= 0, "the level has both");
    size_t at = PolygonAt(&f, linked);
    CHECK(Is(Changed(&f, at + 1, 0, 1, true), mnav_errorInvalid, mnav_tilePolygons), "area 0");
    CHECK(Is(Changed(&f, at + 1, MNAV_AREA_TYPES, 1, true), mnav_errorInvalid, mnav_tilePolygons),
          "area past the types");
    CHECK(Is(Changed(&f, at, 7, 1, true), mnav_errorInvalid, mnav_tilePolygons) ||
              Changed(&f, at, 7, 1, true).result == mnav_errorInvalid,
          "seven vertices");
    CHECK(Is(Changed(&f, at + 2, (uint64_t)mesh->vertexCount, 2, true), mnav_errorInvalid,
             mnav_tilePolygons),
          "a vertex past the vertices");
    CHECK(Is(Changed(&f, at + 4, MNAV_NO_INDEX, 2, true), mnav_errorInvalid, mnav_tilePolygons),
          "a neighbor that does not link back");
    CHECK(Is(Changed(&f, at + 4, (uint64_t)linked, 2, true), mnav_errorInvalid, mnav_tilePolygons),
          "its own neighbor");
    CHECK(Is(Changed(&f, at + 6, 1, 1, true), mnav_errorInvalid, mnav_tilePolygons),
          "a side on a linked edge");
    size_t edge = PolygonAt(&f, loose) + 2 + (size_t)looseEdge * 5;
    CHECK(Is(Changed(&f, edge + 4, 2, 1, true), mnav_errorInvalid, mnav_tilePolygons),
          "a side the edge does not lie on");
    // The first two vertices swapped: the polygon folds over itself.
    const mnavPolygon* polygon = &mesh->polygons[loose];
    size_t first = PolygonAt(&f, loose) + 2;
    static uint8_t copy[1 << 16];
    memcpy(copy, f.bytes, f.size);
    PutLittle(copy + first, polygon->vertices[1], 2);
    PutLittle(copy + first + 5, polygon->vertices[0], 2);
    Reseal(copy, f.size);
    CHECK(Is(Load(&f.memory, copy, f.size), mnav_errorInvalid, mnav_tilePolygons), "not convex");
    // The first detail part: one more vertex than the header counts, and a
    // corner past its part.
    size_t parts = PolygonAt(&f, mesh->polygonCount);
    CHECK(Is(Changed(&f, parts, f.baked.detail.parts[0].vertexCount - mesh->polygons[0].count + 1,
                     1, true),
             mnav_errorInvalid, mnav_tileDetailParts),
          "parts not summing to the header");
    size_t extra = (size_t)(f.baked.detail.vertexCount);
    for (int32_t p = 0; p < mesh->polygonCount; ++p)
    {
        extra -= (size_t)mesh->polygons[p].count;
    }
    size_t triangles = parts + (size_t)mesh->polygonCount * 2 + extra * 6;
    CHECK(Is(Changed(&f, triangles, 200, 1, true), mnav_errorInvalid, mnav_tileDetailTriangles),
          "a corner past its part");
    CHECK(Is(Changed(&f, triangles + 3, 8, 1, true), mnav_errorInvalid, mnav_tileDetailTriangles),
          "outline bits past three");
    Unbake(&f);
}

// A tile built by hand: up to 8 vertices and 2 polygons, unlinked, with
// detail triangles fanned from each polygon's first vertex and any extra
// detail vertices given.
typedef struct Hand
{
    mnavMeshVertex vertices[8];
    mnavPolygon polygons[2];
    mnavPolyMesh mesh;
    mnavDetailPart parts[2];
    mnavDetailVertex detailVertices[24];
    mnavDetailTriangle triangles[16];
    mnavDetailMesh detail;
    mnavTileInfo info;
} Hand;

static void HandBegin(Hand* h, const int32_t* points, int32_t count)
{
    *h = (Hand){0};
    for (int32_t v = 0; v < count; ++v)
    {
        h->vertices[v] =
            (mnavMeshVertex){(uint16_t)points[2 * v], BASE, (uint16_t)points[2 * v + 1]};
    }
    h->mesh = (mnavPolyMesh){h->vertices, nullptr, count, count, h->polygons, 0, 2, 32, 0};
    h->detail =
        (mnavDetailMesh){h->parts, 0, h->detailVertices, 0, 24, h->triangles, 0, 16, 0, 0, 0};
    h->info = (mnavTileInfo){(mnavVersion){1, 2, 3},   7, 0, 0, 32, 0.25f, 0.125f, 16, 2, 6,
                             (mnavPos3){0.0, 0.0, 0.0}};
}

// Adds a polygon over the given vertex indices, with extra detail vertices.
static void HandPolygon(Hand* h, const uint16_t* corners, int32_t count,
                        const mnavDetailVertex* extra, int32_t extraCount)
{
    mnavPolygon* polygon = &h->polygons[h->mesh.polygonCount];
    *polygon = (mnavPolygon){{0}, {0}, {0}, (uint8_t)count, 1, 1};
    memset(polygon->vertices, 0xFF, sizeof(polygon->vertices));
    memset(polygon->neighbors, 0xFF, sizeof(polygon->neighbors));
    mnavDetailPart* part = &h->parts[h->mesh.polygonCount];
    *part = (mnavDetailPart){h->detail.vertexCount, h->detail.triangleCount,
                             (uint8_t)(count + extraCount), (uint8_t)(count - 2)};
    for (int32_t k = 0; k < count; ++k)
    {
        polygon->vertices[k] = corners[k];
        const mnavMeshVertex* v = &h->vertices[corners[k]];
        h->detailVertices[h->detail.vertexCount++] = (mnavDetailVertex){v->x * 16, v->y, v->z * 16};
    }
    for (int32_t k = 0; k < extraCount; ++k)
    {
        h->detailVertices[h->detail.vertexCount++] = extra[k];
    }
    for (int32_t k = 1; k + 1 < count; ++k)
    {
        h->triangles[h->detail.triangleCount++] =
            (mnavDetailTriangle){{0, (uint8_t)k, (uint8_t)(k + 1)}, 0};
    }
    h->mesh.polygonCount += 1;
    h->detail.partCount += 1;
}

// Encodes the hand tile and loads it.
static mnavTileResult HandLoad(Hand* h)
{
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    uint8_t* bytes = nullptr;
    size_t size = 0;
    CHECK(mnavEncodeTile(&memory, &h->info, &h->mesh, &h->detail, &bytes, &size) == mnav_success,
          "encoded");
    mnavTileResult result = Load(&memory, bytes, size);
    mnavReleaseTileBytes(&memory, bytes, size);
    CHECK(memory.used == 0, "nothing held");
    return result;
}

static bool Names(mnavTileResult r, mnavTileSection section, int32_t index)
{
    return r.result == mnav_errorInvalid && r.section == section && r.index == index;
}

static void TestHandTilesRefuseOneFlawEach(void)
{
    // Vertices well inside a 32-cell tile.
    const int32_t points[10] = {4, 4, 4, 8, 8, 8, 8, 4, 6, 6};
    const uint16_t square[4] = {0, 1, 2, 3};
    static Hand h;
    HandBegin(&h, points, 5);
    HandPolygon(&h, square, 4, nullptr, 0);
    CHECK(HandLoad(&h).result == mnav_success, "a sound square loads");
    // A vertex past the tile's +Z side.
    h.vertices[4].z = 33;
    CHECK(Names(HandLoad(&h), mnav_tileVertices, 4), "a vertex past the tile");
    h.vertices[4].z = 6;
    // A reflex corner: (6, 6) between (8, 8) and (8, 4) pulls the
    // square's +X side in.
    HandBegin(&h, points, 5);
    const uint16_t reflex[5] = {0, 1, 2, 4, 3};
    HandPolygon(&h, reflex, 5, nullptr, 0);
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "not convex");
    // Two vertices at one place: an edge of zero length.
    HandBegin(&h, (const int32_t[]){4, 4, 4, 8, 8, 8, 8, 8}, 4);
    HandPolygon(&h, square, 4, nullptr, 0);
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "an edge of zero length");
    // Three vertices on a line: no area.
    HandBegin(&h, (const int32_t[]){4, 4, 4, 6, 4, 8}, 3);
    HandPolygon(&h, square, 3, nullptr, 0);
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "no area");
    // Wound the other way.
    const uint16_t backward[4] = {3, 2, 1, 0};
    HandBegin(&h, points, 5);
    HandPolygon(&h, backward, 4, nullptr, 0);
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "wound backward");
    // A polygon naming itself across an edge.
    HandBegin(&h, points, 5);
    HandPolygon(&h, square, 4, nullptr, 0);
    h.polygons[0].neighbors[1] = 0;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "its own neighbor");
    // A neighbor past the polygons.
    HandBegin(&h, points, 5);
    HandPolygon(&h, square, 4, nullptr, 0);
    h.polygons[0].neighbors[1] = 1;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "a neighbor past the polygons");
    // A vertex used twice.
    const uint16_t twice[4] = {0, 1, 0, 2};
    HandBegin(&h, points, 5);
    HandPolygon(&h, twice, 4, nullptr, 0);
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "a vertex used twice");
    // A detail vertex past the tile's +Z side.
    HandBegin(&h, points, 5);
    const mnavDetailVertex far = {96, BASE, 32 * 16 + 1};
    HandPolygon(&h, square, 4, &far, 1);
    CHECK(Names(HandLoad(&h), mnav_tileDetailVertices, 0), "a detail vertex past the tile");
}

static void TestBoundaryRefusals(void)
{
    // Each value one past what a check allows, refused by that check and
    // so named where it stands: a vertex a cell past the tile, a polygon
    // with no detail parts, fewer detail triangles than polygons in the
    // header, an edge side past the four, a polygon of two vertices.
    static Fixture f;
    Bake(&f);
    const mnavPolyMesh* mesh = &f.baked.mesh;
    CHECK(Names(Changed(&f, MNAV_TILE_HEADER_BYTES, (uint64_t)mesh->tileCells + 1, 2, true),
                mnav_tileVertices, 0),
          "a vertex past the tile");
    size_t parts = PolygonAt(&f, mesh->polygonCount);
    CHECK(Names(Changed(&f, parts + 1, 0, 1, true), mnav_tileDetailParts, 0),
          "a polygon with no detail parts");
    mnavTileResult few =
        Changed(&f, DETAIL_TRIANGLES_AT, (uint64_t)mesh->polygonCount - 1, 4, true);
    CHECK(few.result == mnav_errorInvalid && few.section == mnav_tilePayload,
          "fewer detail triangles than polygons");
    CHECK(Names(Changed(&f, PolygonAt(&f, 0) + 2 + 4, 5, 1, true), mnav_tilePolygons, 0),
          "a side past the four");
    CHECK(Names(Changed(&f, PolygonAt(&f, 0), 2, 1, true), mnav_tilePolygons, 0),
          "a polygon of two vertices");
    Unbake(&f);
}

static void TestHeaderSettingsAreChecked(void)
{
    const int32_t points[8] = {4, 4, 4, 8, 8, 8, 8, 4};
    const uint16_t square[4] = {0, 1, 2, 3};
    static Hand h;
    HandBegin(&h, points, 4);
    HandPolygon(&h, square, 4, nullptr, 0);
    h.info.agentRadius = 33;
    CHECK(Names(HandLoad(&h), mnav_tileHeader, -1), "a radius wider than the tile");
    h.info.agentRadius = 2;
    h.info.x = MNAV_MAX_EXTENT_CELLS / 32 + 1;
    CHECK(Names(HandLoad(&h), mnav_tileHeader, -1), "a tile past the extent");
    h.info.x = -(MNAV_MAX_EXTENT_CELLS / 32);
    CHECK(HandLoad(&h).result == mnav_success, "the last tile within it");
    h.info.x = 0;
    h.info.agentHeight = 0;
    CHECK(Names(HandLoad(&h), mnav_tileHeader, -1), "an agent of no height");
}

static void TestMemoryLimitIsTyped(void)
{
    static Fixture f;
    Bake(&f);
    mnavMemory small = mnavMakeMemory((mnavAllocator){0}, 4096);
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    CHECK(mnavDecodeTile(&small, f.bytes, f.size, &info, &mesh, &detail).result == mnav_errorLimit,
          "past the memory limit");
    CHECK(small.used == 0, "nothing held");
    Unbake(&f);
}

static void WriteSeed(const char* path)
{
    static Fixture f;
    Bake(&f);
    FILE* file = fopen(path, "wb");
    CHECK(file != nullptr && fwrite(f.bytes, 1, f.size, file) == f.size, "seed written");
    if (file != nullptr)
    {
        fclose(file);
    }
    Unbake(&f);
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        WriteSeed(argv[2]);
        return s_failures == 0 ? 0 : 1;
    }
    TestRoundTrip();
    TestEveryTruncationIsRefused();
    TestTrailingBytesAreRefused();
    TestEveryByteChangeIsRefusedOrCanonical();
    TestHeaderRefusals();
    TestMeshRefusals();
    TestHandTilesRefuseOneFlawEach();
    TestHeaderSettingsAreChecked();
    TestMemoryLimitIsTyped();
    TestBoundaryRefusals();
    return s_failures == 0 ? 0 : 1;
}
