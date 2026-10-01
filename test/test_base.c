// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The version and result names.

#include "test_harness.h"

#include "maul-nav/base.h"

#include <stdint.h>
#include <string.h>

static void TestVersionMatchesHeader(void)
{
    mnavVersion version = mnavGetVersion();
    CHECK(version.major == MNAV_VERSION_MAJOR, "major version");
    CHECK(version.minor == MNAV_VERSION_MINOR, "minor version");
    CHECK(version.patch == MNAV_VERSION_PATCH, "patch version");
}

static void TestResultNames(void)
{
    CHECK(strcmp(mnavResultName(mnav_success), "mnav_success") == 0, "success name");
    CHECK(strcmp(mnavResultName(mnav_errorInvalid), "mnav_errorInvalid") == 0, "invalid name");
    CHECK(strcmp(mnavResultName(mnav_errorCapacity), "mnav_errorCapacity") == 0, "capacity name");
    CHECK(strcmp(mnavResultName(mnav_errorLimit), "mnav_errorLimit") == 0, "limit name");
    CHECK(strcmp(mnavResultName(mnav_errorRange), "mnav_errorRange") == 0, "range name");
    CHECK(strcmp(mnavResultName(12345), "unknown result") == 0, "unknown name");
}

static void TestHashIsTheFamilyHash(void)
{
    // The family hash's frozen values: FNV-1a's offset for no bytes, and
    // one full round and one leftover byte for nine.
    CHECK(mnavHash64(MNAV_HASH_INIT, nullptr, 0) == MNAV_HASH_INIT, "no bytes");
    const uint8_t bytes[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    uint64_t word = 0;
    for (int32_t i = 7; i >= 0; --i)
    {
        word = (word << 8) | bytes[i];
    }
    uint64_t expected = (MNAV_HASH_INIT ^ word) * 0x9E3779B97F4A7C15ull;
    expected ^= expected >> 32;
    expected = (expected ^ 9u) * 0x100000001B3ull;
    CHECK(mnavHash64(MNAV_HASH_INIT, bytes, 9) == expected, "nine bytes");
}

int main(void)
{
    TestVersionMatchesHeader();
    TestResultNames();
    TestHashIsTheFamilyHash();
    return s_failures == 0 ? 0 : 1;
}
