// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The host's code an owner object runs (its allocator, link
// generation's clearance test), marked while it runs: a call of the
// object made back in from there is refused with mnav_errorBusy and one
// misuse counted.
//
// Two counts, so that no call needs to know its thread. The allocator
// runs inside a call that changes the object, which no other call may
// run beside: every call that changes the object refuses while it runs.
// A clearance test runs inside link generation, which only reads the
// navmesh: a call that changes nothing a reader reads (staging a tile or
// its removal) stays allowed, from the test itself or from another
// thread, and every other call that changes the object refuses.

#ifndef MAUL_NAV_SRC_HOST_CODE_H
#define MAUL_NAV_SRC_HOST_CODE_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct mnavHostCode
{
    // How many allocator calls, and how many clearance tests, run on the
    // object now: atomic, since link generation may run the clearance
    // test on several threads at once over one navmesh.
    _Atomic int32_t writing;
    _Atomic int32_t reading;
    // Calls refused as misuse, cumulative.
    _Atomic uint64_t misuse;
} mnavHostCode;

// Marks the object's allocator running, and its end; NULL marks nothing.
static inline void mnavHostEnter(mnavHostCode* host)
{
    if (host != nullptr)
    {
        atomic_fetch_add_explicit(&host->writing, 1, memory_order_acq_rel);
    }
}

static inline void mnavHostLeave(mnavHostCode* host)
{
    if (host != nullptr)
    {
        atomic_fetch_sub_explicit(&host->writing, 1, memory_order_acq_rel);
    }
}

// Marks a clearance test running over the object, and its end.
static inline void mnavHostEnterReader(mnavHostCode* host)
{
    atomic_fetch_add_explicit(&host->reading, 1, memory_order_acq_rel);
}

static inline void mnavHostLeaveReader(mnavHostCode* host)
{
    atomic_fetch_sub_explicit(&host->reading, 1, memory_order_acq_rel);
}

static inline bool mnavHostCountMisuse(mnavHostCode* host)
{
    atomic_fetch_add_explicit(&host->misuse, 1, memory_order_relaxed);
    return true;
}

// Whether a call that changes the object comes while host code of either
// kind runs on it: then one misuse is counted and the call refuses.
static inline bool mnavHostCalledBack(mnavHostCode* host)
{
    if (atomic_load_explicit(&host->writing, memory_order_acquire) == 0 &&
        atomic_load_explicit(&host->reading, memory_order_acquire) == 0)
    {
        return false;
    }
    return mnavHostCountMisuse(host);
}

// The same for a call that changes nothing a reader reads: refused only
// while the object's allocator runs.
static inline bool mnavHostCalledBackWriting(mnavHostCode* host)
{
    if (atomic_load_explicit(&host->writing, memory_order_acquire) == 0)
    {
        return false;
    }
    return mnavHostCountMisuse(host);
}

static inline uint64_t mnavHostMisuse(const mnavHostCode* host)
{
    return atomic_load_explicit(&host->misuse, memory_order_relaxed);
}

#endif // MAUL_NAV_SRC_HOST_CODE_H
