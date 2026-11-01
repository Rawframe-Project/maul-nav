// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug output under fuzzing: a draw call into a buffer of
// capacities from the bytes must end in a typed status; on success its
// counts lie within the capacities, its indices name vertices drawn and
// its positions are finite; on a capacity error its counts say what the
// whole needs, and drawing again into a buffer that large succeeds with
// those counts. The including file defines Expect.

#ifndef MAUL_NAV_TEST_FUZZ_DRAW_H
#define MAUL_NAV_TEST_FUZZ_DRAW_H

#include "maul-nav/base.h"
#include "maul-nav/draw.h"

#include <math.h>
#include <stdint.h>

enum
{
    DRAW_VERTICES = 1 << 16,
    DRAW_TRIANGLES = 1 << 15,
    DRAW_LINES = 1 << 16
};

typedef mnavResult (*DrawFn)(void* context, mnavDebugBuffer* buffer);

static mnavDebugVertex s_drawVertices[DRAW_VERTICES];
static uint32_t s_drawTriangles[3 * DRAW_TRIANGLES];
static uint32_t s_drawLines[2 * DRAW_LINES];

static inline mnavDebugBuffer DrawBuffer(int32_t vertices, int32_t triangles, int32_t lines)
{
    return (mnavDebugBuffer){
        {0.0, 0.0, 0.0}, s_drawVertices, vertices, 0, s_drawTriangles, triangles, 0,
        s_drawLines,     lines,          0};
}

static inline void CheckDrawn(const mnavDebugBuffer* b)
{
    Expect(b->vertexCount >= 0 && b->vertexCount <= b->vertexCapacity && b->triangleCount >= 0 &&
           b->triangleCount <= b->triangleCapacity && b->lineCount >= 0 &&
           b->lineCount <= b->lineCapacity);
    for (int32_t i = 0; i < b->vertexCount; ++i)
    {
        const mnavDebugVertex* v = &b->vertices[i];
        Expect(isfinite(v->x) && isfinite(v->y) && isfinite(v->z));
    }
    for (int32_t i = 0; i < 3 * b->triangleCount; ++i)
    {
        Expect(b->triangles[i] < (uint32_t)b->vertexCount);
    }
    for (int32_t i = 0; i < 2 * b->lineCount; ++i)
    {
        Expect(b->lines[i] < (uint32_t)b->vertexCount);
    }
}

// Draws with capacities picked by three bytes: small ones, or the most.
static inline void Draw(DrawFn draw, void* context, uint8_t a, uint8_t b, uint8_t c)
{
    int32_t vertices = a % 4 == 0 ? DRAW_VERTICES : a * 4;
    int32_t triangles = b % 4 == 0 ? DRAW_TRIANGLES : b * 2;
    int32_t lines = c % 4 == 0 ? DRAW_LINES : c * 2;
    mnavDebugBuffer buffer = DrawBuffer(vertices, triangles, lines);
    mnavResult result = draw(context, &buffer);
    Expect(result == mnav_success || result == mnav_errorCapacity || result == mnav_errorInvalid ||
           result == mnav_errorStale);
    if (result == mnav_success)
    {
        CheckDrawn(&buffer);
        return;
    }
    if (result != mnav_errorCapacity)
    {
        return;
    }
    Expect(buffer.vertexCount > vertices || buffer.triangleCount > triangles ||
           buffer.lineCount > lines);
    if (buffer.vertexCount > DRAW_VERTICES || buffer.triangleCount > DRAW_TRIANGLES ||
        buffer.lineCount > DRAW_LINES)
    {
        return;
    }
    mnavDebugBuffer whole = DrawBuffer(buffer.vertexCount, buffer.triangleCount, buffer.lineCount);
    Expect(draw(context, &whole) == mnav_success && whole.vertexCount == buffer.vertexCount &&
           whole.triangleCount == buffer.triangleCount && whole.lineCount == buffer.lineCount);
    CheckDrawn(&whole);
}

#endif // MAUL_NAV_TEST_FUZZ_DRAW_H
