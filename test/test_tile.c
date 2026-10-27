// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tile format and its loader. Run as test_tile --seed <file> it
// also writes the level's tile, the fuzz target's seed. The level's bytes
// hold under every host floating-point setting, baked inside the
// library's environment as a public bake runs.
//
// White-box: it encodes and decodes tiles with the format's own codec
// (mnavEncodeTile, mnavDecodeTile), and builds meshes with one flaw each
// that only that encoder can wrap in otherwise well-formed bytes. The
// loader's refusals are public (mnavStageTile); the inputs that reach
// each of them are not.

// The seed writer uses fopen, which the Microsoft C library deprecates.
#define _CRT_SECURE_NO_WARNINGS

#include "allocator.h"
#include "bake_tile.h"
#include "detail.h"
#include "fields.h"
#include "float_hosts.h"
#include "fpenv.h"
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
#define LEVEL_TILE_HASH 0x8983d326a3e9ac4eull

// Offsets into the header and into the TILE field's payload.
enum
{
    FORMAT_AT = 4,
    HASH_AT = 16,
    PAYLOAD_AT = 24,
    FIELDS_AT = 28,
    INFO_CELLS_AT = 8,
    INFO_CELL_SIZE_AT = 16,
    INFO_ORIGIN_AT = 24,
    // The offset cell height of a hand tile's floor.
    BASE = 32768,
    MOST_PIECES = 8
};

#define TAG_PARTS     MNAV_TAG('D', 'P', 'R', 'T')
#define TAG_TRIANGLES MNAV_TAG('D', 'T', 'R', 'I')
#define TAG_EXTRA     MNAV_TAG('D', 'V', 'R', 'T')
#define TAG_POLYGONS  MNAV_TAG('P', 'O', 'L', 'Y')
#define TAG_INFO      MNAV_TAG('T', 'I', 'L', 'E')
#define TAG_VERTICES  MNAV_TAG('V', 'E', 'R', 'T')

static void PutLittle(uint8_t* at, uint64_t value, int32_t bytes)
{
    for (int32_t k = 0; k < bytes; ++k)
    {
        at[k] = (uint8_t)(value >> (8 * k));
    }
}

static uint64_t GetLittle(const uint8_t* at, int32_t bytes)
{
    uint64_t value = 0;
    for (int32_t k = 0; k < bytes; ++k)
    {
        value |= (uint64_t)at[k] << (8 * k);
    }
    return value;
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

// A field of a well-formed tile: its tag and payload.
typedef struct Piece
{
    uint32_t tag;
    const uint8_t* bytes;
    size_t size;
} Piece;

// The fields of well-formed bytes, in order; returns their number.
static int32_t Pieces(const uint8_t* bytes, Piece pieces[MOST_PIECES])
{
    int32_t count = (int32_t)GetLittle(bytes + FIELDS_AT, 4);
    size_t at = MNAV_TILE_HEADER_BYTES;
    for (int32_t k = 0; k < count && k < MOST_PIECES; ++k)
    {
        size_t size = (size_t)GetLittle(bytes + at + 4, 4);
        pieces[k] = (Piece){(uint32_t)GetLittle(bytes + at, 4), bytes + at + 8, size};
        at += 8 + (size + 7) / 8 * 8;
    }
    return count;
}

// The offset of a field's payload in well-formed bytes, or 0.
static size_t FieldAt(const uint8_t* bytes, uint32_t tag)
{
    Piece pieces[MOST_PIECES];
    int32_t count = Pieces(bytes, pieces);
    for (int32_t k = 0; k < count; ++k)
    {
        if (pieces[k].tag == tag)
        {
            return (size_t)(pieces[k].bytes - bytes);
        }
    }
    return 0;
}

// Writes the header of header and the pieces as fields into out, each
// padded with pad bytes, and seals it; returns its size.
static size_t Assemble(const uint8_t* header, const Piece* pieces, int32_t count, uint8_t pad,
                       uint8_t* out)
{
    memcpy(out, header, MNAV_TILE_HEADER_BYTES);
    PutLittle(out + FIELDS_AT, (uint64_t)count, 4);
    size_t at = MNAV_TILE_HEADER_BYTES;
    for (int32_t k = 0; k < count; ++k)
    {
        PutLittle(out + at, pieces[k].tag, 4);
        PutLittle(out + at + 4, pieces[k].size, 4);
        memcpy(out + at + 8, pieces[k].bytes, pieces[k].size);
        at += 8 + pieces[k].size;
        while (at % 8 != 0)
        {
            out[at++] = pad;
        }
    }
    Reseal(out, at);
    return at;
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
    f->info = (mnavTileInfo){123,
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
    return a->generator == b->generator && a->fingerprint == b->fingerprint && a->x == b->x &&
           a->z == b->z && a->tileCells == b->tileCells && a->cellSize == b->cellSize &&
           a->cellHeight == b->cellHeight && a->agentHeight == b->agentHeight &&
           a->agentRadius == b->agentRadius && a->agentStep == b->agentStep &&
           a->origin.x == b->origin.x && a->origin.y == b->origin.y && a->origin.z == b->origin.z;
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

// The level baked, its scene made too, as a worker of its own: the
// environment's writes are calls, and no arithmetic moves across them.
static MNAV_NOINLINE uint64_t LevelHashWork(void)
{
    static Fixture f;
    Bake(&f);
    uint64_t hash = mnavHash64(MNAV_HASH_INIT, f.bytes, (int32_t)f.size);
    Unbake(&f);
    return hash;
}

// The level's bytes' hash, baked inside the library's environment.
static uint64_t LevelHash(void)
{
    uint64_t env = mnavFloatEnvEnter();
    uint64_t hash = LevelHashWork();
    mnavFloatEnvLeave(env);
    return hash;
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
    CHECK(EveryFloatHost(LevelHash, LEVEL_TILE_HASH), "the pinned hash under every host setting");
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
    size_t at = FieldAt(f->bytes, TAG_POLYGONS);
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
    CHECK(Is(Changed(&f, FORMAT_AT, MNAV_TILE_FORMAT + 1, 2, true), mnav_errorVersion,
             mnav_tileHeader),
          "another format version");
    size_t info = FieldAt(f.bytes, TAG_INFO);
    CHECK(Is(Changed(&f, info + INFO_CELLS_AT, 8, 2, true), mnav_errorInvalid, mnav_tileHeader),
          "tile cells");
    CHECK(Is(Changed(&f, info + INFO_CELL_SIZE_AT, 0x7FC00000u, 4, true), mnav_errorInvalid,
             mnav_tileHeader),
          "NaN cell size");
    CHECK(Is(Changed(&f, info + INFO_ORIGIN_AT, 0x7FF0000000000000ull, 8, true), mnav_errorInvalid,
             mnav_tileHeader),
          "infinite origin");
    CHECK(Is(Changed(&f, HASH_AT, 0, 8, false), mnav_errorInvalid, mnav_tilePayload), "hash");
    CHECK(Is(Changed(&f, PAYLOAD_AT, f.size, 4, false), mnav_errorInvalid, mnav_tilePayload),
          "payload size");
    CHECK(Is(Changed(&f, FIELDS_AT, 7, 4, true), mnav_errorInvalid, mnav_tilePayload),
          "more fields than the payload holds");
    CHECK(Is(Changed(&f, FIELDS_AT, 5, 4, true), mnav_errorInvalid, mnav_tilePayload),
          "bytes past the fields counted");
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
    // The first detail part: one more vertex than DVRT holds, and a
    // corner past its part.
    size_t parts = FieldAt(f.bytes, TAG_PARTS);
    CHECK(Is(Changed(&f, parts, f.baked.detail.parts[0].vertexCount - mesh->polygons[0].count + 1,
                     1, true),
             mnav_errorInvalid, mnav_tileDetailParts),
          "parts not summing to the detail fields");
    CHECK(Is(Changed(&f, FieldAt(f.bytes, TAG_TRIANGLES), 200, 1, true), mnav_errorInvalid,
             mnav_tileDetailTriangles),
          "a corner past its part");
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
    h->info = (mnavTileInfo){123, 7, 0, 0, 32, 0.25f, 0.125f, 16, 2, 6, (mnavPos3){0.0, 0.0, 0.0}};
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
            (mnavDetailTriangle){{0, (uint8_t)k, (uint8_t)(k + 1)}};
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
    // with no detail parts, an edge side past the four, a polygon of two
    // vertices.
    static Fixture f;
    Bake(&f);
    const mnavPolyMesh* mesh = &f.baked.mesh;
    CHECK(Names(Changed(&f, FieldAt(f.bytes, TAG_VERTICES), (uint64_t)mesh->tileCells + 1, 2, true),
                mnav_tileVertices, 0),
          "a vertex past the tile");
    size_t parts = FieldAt(f.bytes, TAG_PARTS);
    CHECK(Names(Changed(&f, parts + 1, 0, 1, true), mnav_tileDetailParts, 0),
          "a polygon with no detail parts");
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

// Loads the fixture's fields as given, each padded with pad bytes.
static mnavTileResult Fields(Fixture* f, const Piece* pieces, int32_t count, uint8_t pad)
{
    static uint8_t copy[(1 << 16) + 64];
    size_t size = Assemble(f->bytes, pieces, count, pad, copy);
    return Load(&f->memory, copy, size);
}

static void TestFieldRefusals(void)
{
    static Fixture f;
    Bake(&f);
    Piece pieces[MOST_PIECES + 1];
    int32_t count = Pieces(f.bytes, pieces);
    CHECK(count == 6 && pieces[0].tag == TAG_PARTS && pieces[1].tag == TAG_TRIANGLES &&
              pieces[2].tag == TAG_EXTRA && pieces[3].tag == TAG_POLYGONS &&
              pieces[4].tag == TAG_INFO && pieces[5].tag == TAG_VERTICES,
          "six fields in order of their names");
    CHECK(Fields(&f, pieces, count, 0).result == mnav_success, "reassembled, it loads");
    uint64_t held = f.memory.used;
    // A tag a later version may write, after the last or among them.
    const uint8_t later[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    pieces[count] = (Piece){MNAV_TAG('Z', 'O', 'N', 'E'), later, 8};
    mnavTileResult r = Fields(&f, pieces, count + 1, 0);
    CHECK(r.result == mnav_errorVersion && r.section == mnav_tilePayload && r.index == count,
          "an unknown tag after the last");
    Piece first = pieces[0];
    pieces[0].tag = MNAV_TAG('A', 'R', 'E', 'A');
    r = Fields(&f, pieces, count, 0);
    CHECK(r.result == mnav_errorVersion && r.section == mnav_tilePayload && r.index == 0,
          "an unknown tag first");
    pieces[0] = first;
    // The TILE field missing, repeated or out of order.
    Piece moved[MOST_PIECES];
    memcpy(moved, pieces, sizeof(moved));
    moved[4] = pieces[5];
    CHECK(Is(Fields(&f, moved, count - 1, 0), mnav_errorInvalid, mnav_tileHeader), "no TILE");
    moved[4] = pieces[5];
    moved[5] = pieces[4];
    CHECK(Is(Fields(&f, moved, count, 0), mnav_errorInvalid, mnav_tileHeader), "TILE after VERT");
    Piece twice[MOST_PIECES];
    memcpy(twice, pieces, sizeof(Piece) * 4);
    twice[4] = pieces[3];
    twice[5] = pieces[4];
    twice[6] = pieces[5];
    CHECK(Is(Fields(&f, twice, count + 1, 0), mnav_errorInvalid, mnav_tilePolygons), "POLY twice");
    // Padding that is not zero, on a field with padding.
    CHECK(pieces[5].size % 8 != 0 || pieces[3].size % 8 != 0, "a field with padding");
    mnavTileResult padded = Fields(&f, pieces, count, 1);
    CHECK(padded.result == mnav_errorInvalid && padded.section != mnav_tilePayload,
          "a padding byte not 0, named by its field");
    // Each array field one byte short of whole records, and one record
    // short; a field present with no bytes.
    const mnavTileSection sections[6] = {mnav_tileDetailParts,    mnav_tileDetailTriangles,
                                         mnav_tileDetailVertices, mnav_tilePolygons,
                                         mnav_tileHeader,         mnav_tileVertices};
    const size_t records[6] = {2, 3, 6, 0, 48, 6};
    for (int32_t k = 0; k < count; ++k)
    {
        size_t whole = pieces[k].size;
        pieces[k].size = whole - 1;
        CHECK(Is(Fields(&f, pieces, count, 0), mnav_errorInvalid, sections[k]),
              "a field a byte short of whole records");
        pieces[k].size = records[k] > 0 ? whole - records[k] : whole - 2 - 5 * 3;
        CHECK(Fields(&f, pieces, count, 0).result == mnav_errorInvalid, "a field a record short");
        pieces[k].size = 0;
        CHECK(Is(Fields(&f, pieces, count, 0), mnav_errorInvalid, sections[k]),
              "a field of no bytes");
        pieces[k].size = whole;
    }
    // TILE and DPRT, whose lengths are fixed, one record long.
    static uint8_t longer[1 << 12];
    for (int32_t k = 0; k <= 4; k += 4)
    {
        Piece whole = pieces[k];
        size_t record = k == 0 ? 2 : 48;
        memcpy(longer, whole.bytes, whole.size);
        memset(longer + whole.size, 0, record);
        pieces[k] = (Piece){whole.tag, longer, whole.size + record};
        CHECK(Is(Fields(&f, pieces, count, 0), mnav_errorInvalid, sections[k]),
              "a fixed field a record long");
        pieces[k] = whole;
    }
    CHECK(f.memory.used == held, "nothing held after a refusal");
    Unbake(&f);
}

static void TestEmptyFieldsAreLeftOut(void)
{
    // A square with no detail vertices past its own: no DVRT, which reads
    // as none.
    const int32_t points[8] = {4, 4, 4, 8, 8, 8, 8, 4};
    const uint16_t square[4] = {0, 1, 2, 3};
    static Hand h;
    HandBegin(&h, points, 4);
    HandPolygon(&h, square, 4, nullptr, 0);
    mnavMemory memory = mnavMakeMemory((mnavAllocator){0}, UINT64_MAX);
    uint8_t* bytes = nullptr;
    size_t size = 0;
    CHECK(mnavEncodeTile(&memory, &h.info, &h.mesh, &h.detail, &bytes, &size) == mnav_success,
          "encoded");
    Piece pieces[MOST_PIECES];
    int32_t count = Pieces(bytes, pieces);
    CHECK(count == 5 && FieldAt(bytes, TAG_EXTRA) == 0, "no DVRT");
    mnavTileInfo info;
    mnavPolyMesh mesh;
    mnavDetailMesh detail;
    CHECK(mnavDecodeTile(&memory, bytes, size, &info, &mesh, &detail).result == mnav_success &&
              detail.vertexCount == 4 && detail.parts[0].vertexCount == 4,
          "loads with the polygon's own vertices only");
    mnavReleaseDetailMesh(&memory, &detail);
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseTileBytes(&memory, bytes, size);
    // A tile with no polygons: only TILE, and nothing read.
    HandBegin(&h, points, 0);
    CHECK(mnavEncodeTile(&memory, &h.info, &h.mesh, &h.detail, &bytes, &size) == mnav_success,
          "encoded");
    CHECK(Pieces(bytes, pieces) == 1 && pieces[0].tag == TAG_INFO &&
              size == MNAV_TILE_HEADER_BYTES + 8 + 48,
          "only TILE");
    CHECK(mnavDecodeTile(&memory, bytes, size, &info, &mesh, &detail).result == mnav_success &&
              mesh.vertexCount == 0 && mesh.polygonCount == 0 && detail.partCount == 0 &&
              detail.triangleCount == 0,
          "loads empty");
    mnavReleaseDetailMesh(&memory, &detail);
    mnavReleasePolyMesh(&memory, &mesh);
    mnavReleaseTileBytes(&memory, bytes, size);
    CHECK(memory.used == 0, "nothing held");
}

static void TestLinksMatchTheGeometry(void)
{
    // Two squares side by side, (4, 4) to (8, 8) and (8, 4) to (12, 8),
    // sharing the edge from (8, 8) to (8, 4): the first's edge 2, the
    // second's edge 0, run the other way.
    const int32_t points[12] = {4, 4, 4, 8, 8, 8, 8, 4, 12, 8, 12, 4};
    const uint16_t left[4] = {0, 1, 2, 3};
    const uint16_t right[4] = {3, 2, 4, 5};
    static Hand h;
    HandBegin(&h, points, 6);
    HandPolygon(&h, left, 4, nullptr, 0);
    HandPolygon(&h, right, 4, nullptr, 0);
    h.polygons[0].neighbors[2] = 1;
    h.polygons[1].neighbors[0] = 0;
    CHECK(HandLoad(&h).result == mnav_success, "linked across the shared edge");
    // A link one way only.
    h.polygons[1].neighbors[0] = MNAV_NO_INDEX;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "a link not returned");
    // Links both ways, but across edges the squares do not share.
    h.polygons[0].neighbors[2] = MNAV_NO_INDEX;
    h.polygons[0].neighbors[1] = 1;
    h.polygons[1].neighbors[1] = 0;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "linked across edges not shared");
    h.polygons[0].neighbors[1] = MNAV_NO_INDEX;
    h.polygons[1].neighbors[1] = MNAV_NO_INDEX;
    h.polygons[0].neighbors[2] = 1;
    h.polygons[1].neighbors[0] = 0;
    // A side mark on a linked edge, and on an edge on no tile side.
    h.polygons[0].sides[2] = 3;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "a side on a linked edge");
    h.polygons[0].sides[2] = 0;
    h.polygons[0].sides[0] = 1;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "a side on an edge inside the tile");
    // A square on the tile's -X side: its edge there must carry side 1,
    // and no other.
    HandBegin(&h, (const int32_t[]){0, 4, 0, 8, 4, 8, 4, 4}, 4);
    HandPolygon(&h, left, 4, nullptr, 0);
    h.polygons[0].sides[0] = 1;
    CHECK(HandLoad(&h).result == mnav_success, "the -X side marked");
    h.polygons[0].sides[0] = 0;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "the side unmarked");
    h.polygons[0].sides[0] = 3;
    CHECK(Names(HandLoad(&h), mnav_tilePolygons, 0), "marked as another side");
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
    TestFieldRefusals();
    TestEmptyFieldsAreLeftOut();
    TestLinksMatchTheGeometry();
    return s_failures == 0 ? 0 : 1;
}
