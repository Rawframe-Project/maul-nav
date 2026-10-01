// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Flow fields (mnav-0007): for every cell of a grid, the cost of its
// cheapest way to the nearest of a set of goals and the next cell on it,
// so that any number of agents sharing the goals find their way at the
// cost of one search.

#ifndef MAUL_NAV_FLOW_H
#define MAUL_NAV_FLOW_H

#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The most cells a flow field may hold.
#define MNAV_MAX_FLOW_CELLS 16777216

    // How a flow field is made. Build it with mnavDefaultFlowFieldDef.
    typedef struct mnavFlowFieldDef
    {
        uint32_t cookie;
        // The allocator the field uses; zeroed for the C library's.
        mnavAllocator allocator;
        // The most cells a grid may have, 1 to MNAV_MAX_FLOW_CELLS.
        int32_t cells;
    } mnavFlowFieldDef;

    // A flow field: the memory for its cells, and the field last built.
    typedef struct mnavFlowField mnavFlowField;

    // A cell's way to the goals: its cost, infinite where no goal is
    // reached, and the next cell, the cell itself at a goal and where no
    // goal is reached.
    typedef struct mnavFlow
    {
        double cost;
        mnavCell next;
    } mnavFlow;

    /// Returns the default flow field def: up to 65536 cells.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavFlowFieldDef mnavDefaultFlowFieldDef(void);

    /// Makes a flow field with the memory its cell limit needs.
    ///
    /// @param def      The def, from mnavDefaultFlowFieldDef.
    /// @param fieldOut Receives the field, or NULL on failure.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument or a
    /// def not from mnavDefaultFlowFieldDef; `mnav_errorRange` for a cell
    /// limit out of its range; `mnav_errorCapacity` when the allocator
    /// fails.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_NODISCARD MNAV_API mnavResult mnavCreateFlowField(const mnavFlowFieldDef* def,
                                                           mnavFlowField** fieldOut);

    /// Destroys a flow field.
    ///
    /// @param field The field, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the field is used by one thread at a time.
    MNAV_API void mnavDestroyFlowField(mnavFlowField* field);

    /// Builds the field for a grid and a set of goal cells by one search
    /// from all the goals, with the steps of mnavFindGridPath: to the 8
    /// neighbours, never across a blocked corner, a step costing its length
    /// times the mean of its two cells' area costs. A cell's way to the
    /// goals is a cheapest one, ties going to the cell found first by
    /// cost, then by index; the same grid and goals give the same field on
    /// every platform. Blocked goals are left out.
    ///
    /// @param field     The field; its last field is replaced.
    /// @param grid      The grid, read only during the call.
    /// @param filter    The areas usable and their costs, or NULL.
    /// @param goals     The goal cells.
    /// @param goalCount How many, at least 0.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, a
    /// negative count, a grid with no areas, a side out of range or a cell
    /// size not more than 0 or not finite, a goal outside the grid, or a
    /// filter not built from mnavDefaultQueryFilter; `mnav_errorRange` for
    /// a filter cost out of its range; `mnav_errorLimit` for a grid with
    /// more cells than the field's limit. On an error the field holds no
    /// grid.
    /// @par Thread safety
    /// Safe from any thread; the field is used by one thread at a time.
    MNAV_NODISCARD MNAV_API mnavResult mnavBuildFlowField(mnavFlowField* field,
                                                          const mnavGrid* grid,
                                                          const mnavQueryFilter* filter,
                                                          const mnavCell* goals, int32_t goalCount);

    /// Reads a cell's way to the goals from the field last built.
    ///
    /// @param field   The field.
    /// @param cell    The cell.
    /// @param flowOut Receives the cell's way.
    /// @return `mnav_success`; `mnav_errorInvalid` for a NULL argument, or
    /// a cell outside the grid last built or no grid built.
    /// @par Thread safety
    /// Safe from any thread. Any number of threads may read a field at once
    /// while no build runs on it.
    MNAV_NODISCARD MNAV_API mnavResult mnavFlowAt(const mnavFlowField* field, mnavCell cell,
                                                  mnavFlow* flowOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_FLOW_H
