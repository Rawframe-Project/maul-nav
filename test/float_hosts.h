// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A suite's pinned results under every host floating-point setting
// test_fenv.h plays: each rounding mode, with and without flush-to-zero
// and denormals-are-zero where the platform has them. Only the library's
// calls run under the host's setting: a scene's own arithmetic is done
// beforehand, in the default one, or is exact in any.

#ifndef MAUL_NAV_TEST_FLOAT_HOSTS_H
#define MAUL_NAV_TEST_FLOAT_HOSTS_H

#include "test_fenv.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Whether compute gives want under every step-th host setting from the
// first, each run leaving the setting as it found it; the thread's own
// setting is put back after.
static inline bool FloatHosts(uint64_t (*compute)(void), uint64_t want, int step)
{
    static bool said = false;
    if (!TEST_HAS_FLUSH && !said)
    {
        printf("float hosts: no flush-to-zero here, the rounding modes alone\n");
        said = true;
    }
    TestFloatSetting saved = TestFloatGet();
    bool same = true;
    for (int i = 0; i < TestFloatHostileCount(); i += step)
    {
        TestFloatSetting host = TestFloatHostileAt(i);
        TestFloatSet(host);
        uint64_t got = compute();
        bool kept = TestFloatSame(TestFloatGet(), host);
        TestFloatSet(saved);
        if (got != want || !kept)
        {
            printf("float host %d: %016llx%s\n", i, (unsigned long long)got,
                   kept ? "" : ", the setting changed");
            same = false;
        }
    }
    return same;
}

// Whether compute gives want under every host setting.
static inline bool EveryFloatHost(uint64_t (*compute)(void), uint64_t want)
{
    return FloatHosts(compute, want, 1);
}

#endif // MAUL_NAV_TEST_FLOAT_HOSTS_H
