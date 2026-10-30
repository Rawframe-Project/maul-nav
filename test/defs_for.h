// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The defs of a navmesh and a flight volume for the tiles a suite bakes
// with a bake or flight def.

#ifndef MAUL_NAV_TEST_DEFS_FOR_H
#define MAUL_NAV_TEST_DEFS_FOR_H

#include "maul-nav/bake.h"
#include "maul-nav/flight.h"
#include "maul-nav/navmesh.h"

// The default navmesh def for tiles baked with a bake def, taking the bake
// def's allocator and memory limit, as a navmesh made from a bake def did.
static inline mnavNavmeshDef NavmeshDefFor(const mnavBakeDef* bake)
{
    mnavNavmeshDef def = mnavDefaultNavmeshDef();
    def.bake = *bake;
    def.allocator = bake->allocator;
    def.limits.memoryBytes = bake->limits.memoryBytes;
    return def;
}

// The default flight volume def for tiles baked with a flight def, taking
// the flight def's allocator and memory limit, as a volume made from a
// flight def did.
static inline mnavFlightVolumeDef FlightVolumeDefFor(const mnavFlightDef* flight)
{
    mnavFlightVolumeDef def = mnavDefaultFlightVolumeDef();
    def.flight = *flight;
    def.allocator = flight->allocator;
    def.limits.memoryBytes = flight->limits.memoryBytes;
    return def;
}

#endif // MAUL_NAV_TEST_DEFS_FOR_H
