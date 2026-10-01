// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The base of the Maul Nav API: the library version, the export and
// attribute macros, and the result codes every fallible function
// returns.

#ifndef MAUL_NAV_BASE_H
#define MAUL_NAV_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The library version. CMake reads it from here.
#define MNAV_VERSION_MAJOR 0
#define MNAV_VERSION_MINOR 0
#define MNAV_VERSION_PATCH 1

// MNAV_API marks the public functions: dllexport or dllimport in a
// shared Windows build (maul_nav_EXPORTS is defined while building
// the library), default visibility in a shared build elsewhere.
#if defined(MAUL_NAV_SHARED) && defined(_WIN32)
#if defined(maul_nav_EXPORTS)
#define MNAV_API __declspec(dllexport) extern
#else
#define MNAV_API __declspec(dllimport) extern
#endif
#elif defined(MAUL_NAV_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define MNAV_API __attribute__((visibility("default"))) extern
#else
#define MNAV_API extern
#endif

// MNAV_NODISCARD marks a function whose result must be read: every
// function that returns a status. The attribute is standard in C23 and
// C++17 and left out for older dialects.
#if defined(__cplusplus) && __cplusplus >= 201703L
#define MNAV_NODISCARD [[nodiscard]]
#elif !defined(__cplusplus) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L
#define MNAV_NODISCARD [[nodiscard]]
#else
#define MNAV_NODISCARD
#endif

    // The status a fallible function returns. Zero is success, positive
    // values are outcomes that are not errors, negative values are errors.
    // The type has a fixed width so that result structs have one layout in
    // C and C++.
    typedef int32_t mnavResult;

    enum
    {
        // The call did what was asked.
        mnav_success = 0,
        // An argument is invalid: a null pointer where one is required, a
        // value out of range.
        mnav_errorInvalid = -1,
        // A caller buffer or a named limit is too small for the result.
        mnav_errorCapacity = -2,
    };

    // A library version: major, minor and patch.
    typedef struct mnavVersion
    {
        uint16_t major;
        uint16_t minor;
        uint16_t patch;
    } mnavVersion;

    /// Returns the version of the library that was linked, which may differ
    /// from the MNAV_VERSION macros a program was compiled with.
    ///
    /// @return The library version.
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API mnavVersion mnavGetVersion(void);

    /// Returns the name of a result code, for diagnostics.
    ///
    /// @param result  Any value; an unknown one is named as such.
    /// @return A static, NUL-terminated string such as "mnav_errorCapacity".
    /// @par Thread safety
    /// Safe from any thread.
    MNAV_API const char* mnavResultName(mnavResult result);

#ifdef __cplusplus
}
#endif

#endif // MAUL_NAV_BASE_H
