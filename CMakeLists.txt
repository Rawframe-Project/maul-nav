cmake_minimum_required(VERSION 3.25)

# The library's identity. The family's CMake modules and tools read these.
set(MAUL_LIBRARY maul-nav)
set(MAUL_API_PREFIX mnav)
set(MAUL_MACRO_PREFIX MNAV)
set(MAUL_TITLE "Maul Nav")
set(MAUL_PREFIX MAUL_NAV)

include(cmake/ReadVersion.cmake)
maul_read_version(${CMAKE_CURRENT_SOURCE_DIR}/include/maul-nav/base.h MNAV MAUL_VERSION)
project(maul-nav
    VERSION ${MAUL_VERSION}
    DESCRIPTION "Navigation: navmesh generation, path and spatial queries, flow fields and avoidance"
    LANGUAGES C)

option(MAUL_NAV_BUILD_TESTS "Build the test suites" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_NAV_BUILD_BENCH "Build the benchmarks" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_NAV_BUILD_SHARED "Build a shared library instead of a static one" OFF)
option(MAUL_NAV_INSTALL "Generate install rules" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_NAV_WERROR "Treat compiler warnings as errors" OFF)
option(MAUL_NAV_SANITIZE "Instrument with AddressSanitizer and UBSan" OFF)
option(MAUL_NAV_TSAN "Instrument with ThreadSanitizer" OFF)
option(MAUL_NAV_COVERAGE "Instrument for gcov coverage" OFF)

# Every executable (tests, benchmarks, tools) lands in the top of the
# build tree, so scripts and CI find them in one place.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

include(GNUInstallDirs)
include(cmake/CompilerFlags.cmake)
include(cmake/Library.cmake)
include(cmake/Testing.cmake)

if(MAUL_NAV_BUILD_SHARED)
    set(MAUL_LIBRARY_KIND SHARED)
else()
    set(MAUL_LIBRARY_KIND STATIC)
endif()
add_library(maul-nav ${MAUL_LIBRARY_KIND}
    src/allocator.c
    src/bake_def.c
    src/base.c
    src/border_vertices.c
    src/compact.c
    src/contour.c
    src/detail.c
    src/erode.c
    src/filter.c
    src/hash.c
    src/height_patch.c
    src/heightfield.c
    src/holes.c
    src/input.c
    src/planar.c
    src/polymesh.c
    src/raster.c
    src/region.c
    src/scalar.c
    src/sort.c
    src/triangulate.c)
maul_configure_library(maul-nav)
# floor, ceil, fabs and sqrt, the exactly rounded functions the library
# takes from libm (mnav-0001).
if(NOT WIN32 AND NOT APPLE)
    target_link_libraries(maul-nav PUBLIC m)
endif()

if(MAUL_NAV_BUILD_TESTS)
    enable_testing()
    maul_add_test(base)
    maul_add_test(bake)
    maul_add_test(border_vertices WHITEBOX)
    maul_add_test(compact WHITEBOX)
    maul_add_test(contour WHITEBOX)
    maul_add_test(detail WHITEBOX)
    maul_add_test(filter WHITEBOX)
    maul_add_test(height_patch WHITEBOX)
    maul_add_test(heightfield WHITEBOX)
    maul_add_test(holes WHITEBOX)
    maul_add_test(polymesh WHITEBOX)
    maul_add_test(region WHITEBOX)
    maul_add_test(scalar WHITEBOX)
    maul_add_header_tests()
endif()

if(MAUL_NAV_BUILD_BENCH AND EXISTS ${PROJECT_SOURCE_DIR}/bench/bench_main.c)
    add_subdirectory(bench)
endif()

if(MAUL_NAV_INSTALL)
    if(NOT WIN32 AND NOT APPLE)
        set(MAUL_PKG_LIBS_PRIVATE "-lm")
    endif()
    include(cmake/Install.cmake)
endif()
