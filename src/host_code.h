// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The host's code an owner object runs (its allocator, link
// generation's clearance test), marked while it runs: a call of the
// object made back in from there is refused with mnav_errorBusy and one
// misuse counted.

#ifndef MAUL_NAV_SRC_HOST_CODE_H
#define MAUL_NAV_SRC_HOST_CODE_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct mnavHostCode
{
    // How many of the host's calls run on the object now: atomic, since
    // link generation may run the clearance test on several threads at
    // once over one navmesh.
    _Atomic int32_t running;
    // Calls refused as misuse, cumulative.
    _Atomic uint64_t misuse;
} mnavHostCode;

// Marks host code running on the object, and its end; NULL marks nothing.
static inline void mnavHostEnter(mnavHostCode* host)
{
    if (host != nullptr)
    {
        atomic_fetch_add_explicit(&host->running, 1, memory_order_acq_rel);
    }
}

static inline void mnavHostLeave(mnavHostCode* host)
{
    if (host != nullptr)
    {
        atomic_fetch_sub_explicit(&host->running, 1, memory_order_acq_rel);
    }
}

// Whether a call of the object comes while host code runs on it: then one
// misuse is counted and the call refuses.
static inline bool mnavHostCalledBack(mnavHostCode* host)
{
    if (atomic_load_explicit(&host->running, memory_order_acquire) == 0)
    {
        return false;
    }
    atomic_fetch_add_explicit(&host->misuse, 1, memory_order_relaxed);
    return true;
}

static inline uint64_t mnavHostMisuse(const mnavHostCode* host)
{
    return atomic_load_explicit(&host->misuse, memory_order_relaxed);
}

#endif // MAUL_NAV_SRC_HOST_CODE_H
