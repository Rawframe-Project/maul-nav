// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The version and result names.

#include "test_harness.h"

#include "maul-nav/base.h"

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
    CHECK(strcmp(mnavResultName(12345), "unknown result") == 0, "unknown name");
}

int main(void)
{
    TestVersionMatchesHeader();
    TestResultNames();
    return s_failures == 0 ? 0 : 1;
}
