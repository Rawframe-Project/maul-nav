// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the runtime navmesh (mnav-0004, mnav-0005): a sequence of
// changes read from the bytes, on a dynamic tier (tiles of the test world
// added, removed and replaced by a variant baked with another area, links
// added, removed and toggled, areas changed) with commits between, and
// queries, paths and a corridor that keep polygon and link ids from
// earlier commits, drawn now and then (fuzz_draw.h). Every call must end
// in a typed status; after each
// commit the corridor is checked and, where it breaks, replanned; the
// same sequence on a second navmesh gives the same results; everything
// is given back.

#include "counting_allocator.h"
#include "world.h"

#include "maul-nav/bake.h"
#include "maul-nav/base.h"
#include "maul-nav/debug.h"
#include "maul-nav/navmesh.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    OPS = 40,
    IDS = 8,
    CORRIDOR = 64,
    VARIANT_CAPACITY = 1 << 16
};

static void Expect(bool condition)
{
    if (!condition)
    {
        abort();
    }
}

#include "fuzz_draw.h"

typedef struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t at;
} Reader;

static uint8_t Byte(Reader* r)
{
    return r->at < r->size ? r->data[r->at++] : 0;
}

// A point on or near the world, a step of 0.28 m from -4 to 67.7 m.
static mnavPos3 Point(Reader* r)
{
    return (mnavPos3){(double)Byte(r) * 0.28125 - 4.0, 0.0, (double)Byte(r) * 0.28125 - 4.0};
}

static bool Typed(mnavResult result)
{
    return result <= mnav_success && result >= mnav_errorTier;
}

static bool Finite(mnavPos3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

// The world's tiles baked again with every triangle of area 3: the same
// places, other bytes, so that staging one replaces a tile.
static uint8_t s_variants[4][VARIANT_CAPACITY];
static size_t s_variantSizes[4];

static void BakeVariants(void)
{
    mnavTriangleMesh world = World();
    static mnavAreaType areas[WORLD_TRIANGLES];
    for (int32_t t = 0; t < world.triangleCount; ++t)
    {
        areas[t] = 3;
    }
    world.areas = areas;
    mnavBakeDef def = mnavDefaultBakeDef();
    mnavBaker* baker = nullptr;
    Expect(mnavCreateBaker(&def, &baker).result == mnav_success);
    for (int32_t t = 0; t < 4; ++t)
    {
        Expect(mnavBakeTile(baker, &world, 1, t % 2, t / 2, nullptr) == mnav_success &&
               mnavCopyBakedTile(baker, s_variants[t], VARIANT_CAPACITY, &s_variantSizes[t]) ==
                   mnav_success);
    }
    mnavDestroyBaker(baker);
}

// What one run of a sequence keeps, and a hash of every result.
typedef struct Run
{
    mnavNavmesh* navmesh;
    mnavQuery* query;
    mnavPolygonId polygons[IDS];
    mnavPos3 points[IDS];
    mnavLinkId links[IDS];
    mnavPolygonId buffer[CORRIDOR];
    mnavCorridor corridor;
    bool walking;
    uint64_t hash;
} Run;

static void Note(Run* run, const void* value, size_t size)
{
    run->hash = mnavHash64(run->hash, value, (int32_t)size);
}

static void NoteResult(Run* run, mnavResult result)
{
    Expect(Typed(result));
    int32_t code = (int32_t)result;
    Note(run, &code, sizeof(code));
}

static void NotePath(Run* run, mnavResult result, const mnavPath* path)
{
    NoteResult(run, result);
    if (result != mnav_success)
    {
        return;
    }
    Expect(path->pointCount >= 0 && path->polygonCount >= 0);
    Note(run, &path->pointCount, sizeof(path->pointCount));
    for (int32_t i = 0; i < path->pointCount; ++i)
    {
        Expect(Finite(path->points[i]));
        Note(run, &path->points[i].x, sizeof(double));
        Note(run, &path->points[i].z, sizeof(double));
    }
}

static mnavLinkDef Link(Reader* r)
{
    mnavLinkDef link = {Point(r),
                        Point(r),
                        (float)(Byte(r) % 8) * 0.5f,
                        (float)(Byte(r) % 8),
                        (mnavLinkKind)(Byte(r) % 8),
                        (Byte(r) & 1) != 0,
                        0.0f};
    link.width = Byte(r) % 4 == 0 ? (float)(Byte(r) % 16) * 0.5f : 0.0f;
    return link;
}

// After a commit: the ids kept read as typed, and the corridor is checked
// and replanned where it breaks.
static void AfterCommit(Run* run)
{
    for (int32_t i = 0; i < IDS; ++i)
    {
        mnavAreaType area = 0;
        mnavResult read = mnavGetArea(run->navmesh, run->polygons[i], &area);
        NoteResult(run, read);
        mnavLinkState state;
        NoteResult(run, mnavGetLink(run->navmesh, run->links[i], &state));
    }
    if (!run->walking)
    {
        return;
    }
    int32_t valid = -1;
    mnavResult checked = mnavCheckCorridor(run->navmesh, nullptr, &run->corridor, &valid);
    NoteResult(run, checked);
    Expect(checked != mnav_success || (valid >= 0 && valid <= run->corridor.count));
    Note(run, &valid, sizeof(valid));
    if (checked == mnav_success && valid < run->corridor.count)
    {
        mnavPath path;
        mnavResult replanned = mnavReplanCorridor(run->query, run->navmesh, nullptr, &run->corridor,
                                                  (mnavVec3){2, 2, 2}, &path);
        NotePath(run, replanned, &path);
        Expect(run->corridor.count >= 0 && run->corridor.count <= CORRIDOR);
    }
}

// What a draw call draws: tiles in a range, the links, the corridor or a
// path.
typedef struct Drawing
{
    const Run* run;
    const mnavPath* path;
    int32_t x0;
    int32_t z0;
    int32_t x1;
    int32_t z1;
} Drawing;

static mnavResult DrawTiles(void* context, mnavDebugBuffer* buffer)
{
    const Drawing* d = context;
    return mnavDebugNavmesh(d->run->navmesh, d->x0, d->z0, d->x1, d->z1, buffer);
}

static mnavResult DrawLinks(void* context, mnavDebugBuffer* buffer)
{
    const Drawing* d = context;
    return mnavDebugLinks(d->run->navmesh, buffer);
}

static mnavResult DrawCorridor(void* context, mnavDebugBuffer* buffer)
{
    const Drawing* d = context;
    return mnavDebugCorridor(d->run->navmesh, d->run->corridor.polygons, d->run->corridor.count,
                             buffer);
}

static mnavResult DrawPath(void* context, mnavDebugBuffer* buffer)
{
    const Drawing* d = context;
    return mnavDebugPath(d->path, buffer);
}

static void Drawn(const Run* run, Reader* r)
{
    uint8_t what = Byte(r) % 3;
    Drawing d = {run, nullptr, Byte(r) % 4 - 1, Byte(r) % 4 - 1, Byte(r) % 4 - 1, Byte(r) % 4 - 1};
    DrawFn draw = what == 0 ? DrawTiles : (what == 1 ? DrawLinks : DrawCorridor);
    if (what == 2 && !run->walking)
    {
        return;
    }
    Draw(draw, &d, Byte(r), Byte(r), Byte(r));
}

static void Step(Run* run, Reader* r)
{
    uint8_t op = Byte(r) % 13;
    uint8_t pick = Byte(r);
    int32_t k = pick % IDS;
    if (op == 0)
    {
        int32_t t = Byte(r) % 4;
        bool variant = (pick & 1) != 0;
        mnavTileResult staged = mnavStageTile(run->navmesh, variant ? s_variants[t] : s_tiles[t],
                                              variant ? s_variantSizes[t] : s_sizes[t]);
        NoteResult(run, staged.result);
    }
    else if (op == 1)
    {
        NoteResult(run, mnavStageTileRemoval(run->navmesh, Byte(r) % 4 - 1, Byte(r) % 4 - 1));
    }
    else if (op == 2)
    {
        mnavLinkDef link = Link(r);
        mnavLinkId id = {0, 0};
        mnavResult staged = mnavStageLink(run->navmesh, &link, &id);
        NoteResult(run, staged);
        run->links[k] = staged == mnav_success ? id : run->links[k];
    }
    else if (op == 3)
    {
        NoteResult(run, mnavStageLinkRemoval(run->navmesh, run->links[k]));
    }
    else if (op == 4)
    {
        NoteResult(run, mnavStageLinkEnabled(run->navmesh, run->links[k], (pick & 8) != 0));
    }
    else if (op == 5)
    {
        NoteResult(run, mnavStageArea(run->navmesh, run->polygons[k], (mnavAreaType)(pick % 4)));
    }
    else if (op <= 7)
    {
        mnavResult committed = mnavCommit(run->navmesh);
        NoteResult(run, committed);
        AfterCommit(run);
    }
    else if (op == 8)
    {
        mnavNearest nearest = {0};
        mnavResult found = mnavFindNearest(run->navmesh, nullptr, Point(r),
                                           (mnavVec3){2.0f, 2.0f, 2.0f}, &nearest);
        NoteResult(run, found);
        if (found == mnav_success && nearest.polygon.slot != 0)
        {
            run->polygons[k] = nearest.polygon;
            run->points[k] = nearest.point;
        }
    }
    else if (op == 9)
    {
        int32_t j = Byte(r) % IDS;
        mnavPath path;
        mnavResult found = mnavFindPath(run->query, run->navmesh, nullptr, run->polygons[k],
                                        run->points[k], run->polygons[j], run->points[j], &path);
        NotePath(run, found, &path);
        if (found == mnav_success && (pick & 16) != 0)
        {
            Drawing d = {run, &path, 0, 0, 0, 0};
            Draw(DrawPath, &d, Byte(r), Byte(r), Byte(r));
        }
        if (found == mnav_success && path.polygonCount > 0 && path.polygonCount <= CORRIDOR)
        {
            Expect(mnavResetCorridor(&run->corridor, run->buffer, CORRIDOR, path.polygons[0],
                                     run->points[k]) == mnav_success);
            NoteResult(run, mnavSetCorridor(&run->corridor, &path));
            run->walking = true;
        }
    }
    else if (op == 12)
    {
        Drawn(run, r);
    }
    else if (run->walking)
    {
        mnavMove move;
        mnavResult moved =
            mnavMoveCorridor(run->query, run->navmesh, nullptr, &run->corridor, Point(r), &move);
        NoteResult(run, moved);
        Expect(moved != mnav_success || Finite(move.point));
        Expect(run->corridor.count >= 0 && run->corridor.count <= CORRIDOR);
    }
}

static uint64_t Play(const uint8_t* data, size_t size)
{
    Reader r = {data, size, 0};
    Run run = {0};
    run.hash = MNAV_HASH_INIT;
    mnavBakeDef def = mnavDefaultBakeDef();
    def.allocator = CountingAllocator();
    def.tier = mnav_tierDynamic;
    def.limits.links = 1 + Byte(&r) % 96;
    Expect(mnavCreateNavmesh(&def, &run.navmesh).result == mnav_success);
    mnavQueryDef queryDef = mnavDefaultQueryDef();
    queryDef.allocator = CountingAllocator();
    queryDef.limits.nodes = 64 + Byte(&r) * 4;
    Expect(mnavCreateQuery(&queryDef, &run.query) == mnav_success);
    for (int32_t n = 0; n < OPS && r.at < r.size; ++n)
    {
        Step(&run, &r);
    }
    mnavDestroyQuery(run.query);
    mnavDestroyNavmesh(run.navmesh);
    Expect(s_held == 0);
    return run.hash;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    static bool baked = false;
    if (!baked)
    {
        BakeWorld();
        BakeVariants();
        baked = true;
    }
    Expect(Play(data, size) == Play(data, size));
    return 0;
}
