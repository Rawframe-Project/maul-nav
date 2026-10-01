// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The version and result names.

#include "maul-nav/base.h"

mnavVersion mnavGetVersion(void)
{
    return (mnavVersion){MNAV_VERSION_MAJOR, MNAV_VERSION_MINOR, MNAV_VERSION_PATCH};
}

const char* mnavResultName(mnavResult result)
{
    switch (result)
    {
    case mnav_success:
        return "mnav_success";
    case mnav_errorInvalid:
        return "mnav_errorInvalid";
    case mnav_errorCapacity:
        return "mnav_errorCapacity";
    default:
        return "unknown result";
    }
}
