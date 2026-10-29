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
    // The family's shared errors keep its values; the library's own are
    // numbered from -32 down.
    const struct
    {
        mnavResult result;
        int32_t value;
        const char* name;
    } results[] = {
        {mnav_success, 0, "mnav_success"},
        {mnav_errorInvalid, -1, "mnav_errorInvalid"},
        {mnav_errorCapacity, -2, "mnav_errorCapacity"},
        {mnav_errorStale, -3, "mnav_errorStale"},
        {mnav_errorUnsupported, -4, "mnav_errorUnsupported"},
        {mnav_errorState, -6, "mnav_errorState"},
        {mnav_errorVersion, -7, "mnav_errorVersion"},
        {mnav_errorFormat, -8, "mnav_errorFormat"},
        {mnav_errorLimit, -9, "mnav_errorLimit"},
        {mnav_errorBusy, -10, "mnav_errorBusy"},
        {mnav_errorNotLoaded, -32, "mnav_errorNotLoaded"},
        {mnav_errorOutside, -33, "mnav_errorOutside"},
    };
    for (size_t i = 0; i < sizeof(results) / sizeof(results[0]); ++i)
    {
        CHECK(results[i].result == results[i].value, "the family's value");
        CHECK(strcmp(mnavGetResultName(results[i].result), results[i].name) == 0, "its name");
    }
    CHECK(strcmp(mnavGetResultName(-5), "unknown result") == 0 &&
              strcmp(mnavGetResultName(-34), "unknown result") == 0,
          "values no error has");
    CHECK(strcmp(mnavGetResultName(12345), "unknown result") == 0, "unknown name");
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
