// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Small heightfields built by hand for the white-box tests of the bake's
// stages.

#ifndef MAUL_NAV_TEST_HAND_FIELD_H
#define MAUL_NAV_TEST_HAND_FIELD_H

#include "heightfield.h"
#include "raster.h"

#include "maul-nav/bake.h"

#include <stdint.h>

enum
{
    WIDTH = 7,
    MAX_SPANS = 4,
    HEIGHT = 16,
    STEP = 6,
    BASE = MNAV_HEIGHT_OFFSET
};

// A field of WIDTH by WIDTH columns, each with up to MAX_SPANS spans.
typedef struct Field
{
    mnavSpan input[WIDTH * WIDTH][MAX_SPANS];
    int32_t counts[WIDTH * WIDTH];
    uint32_t columns[WIDTH * WIDTH + 1];
    mnavSpan spans[WIDTH * WIDTH * MAX_SPANS];
    mnavHeightfield heightfield;
} Field;

static inline void AddSpan(Field* field, int32_t x, int32_t z, int32_t bottom, int32_t top,
                           mnavAreaType area)
{
    int32_t column = x + z * WIDTH;
    field->input[column][field->counts[column]++] =
        (mnavSpan){(uint16_t)(BASE + bottom), (uint16_t)(BASE + top), area};
}

// Every column a floor whose top is at height top, walkable.
static inline void Floor(Field* field, int32_t top)
{
    for (int32_t z = 0; z < WIDTH; ++z)
    {
        for (int32_t x = 0; x < WIDTH; ++x)
        {
            AddSpan(field, x, z, top - 1, top, mnav_areaWalkable);
        }
    }
}

static inline mnavHeightfield* Pack(Field* field)
{
    uint32_t written = 0;
    for (int32_t c = 0; c < WIDTH * WIDTH; ++c)
    {
        field->columns[c] = written;
        for (int32_t k = 0; k < field->counts[c]; ++k)
        {
            field->spans[written++] = field->input[c][k];
        }
    }
    field->columns[WIDTH * WIDTH] = written;
    field->heightfield = (mnavHeightfield){{WIDTH, 0.0f, 0.0f, 0.25f, 0.125f},
                                           field->columns,
                                           field->spans,
                                           (int32_t)written,
                                           (int32_t)written};
    return &field->heightfield;
}

static inline mnavAreaType Area(const Field* field, int32_t x, int32_t z, int32_t k)
{
    return field->spans[field->columns[x + z * WIDTH] + (uint32_t)k].area;
}

#endif // MAUL_NAV_TEST_HAND_FIELD_H
