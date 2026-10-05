# The differential harness (mnav-0011): Recast and Detour at the commit
# the research cites, fetched for this harness alone and never linked
# into the library. A local copy stands in with
# -DFETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION=<dir>.
enable_language(CXX)
include(FetchContent)
FetchContent_Declare(recastnavigation
    GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation.git
    GIT_TAG 9f4ce64458dfae86e1239c525ddc219c4e9e06f1)
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(recastnavigation)

add_executable(differential differential.cpp scenes.c maul_side.c)
target_compile_features(differential PRIVATE cxx_std_17)
set_target_properties(differential PROPERTIES C_STANDARD 23 C_STANDARD_REQUIRED ON)
# Recast builds without run-time type information; its context class is
# derived from here.
if(NOT MSVC)
    target_compile_options(differential PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
endif()
target_include_directories(differential PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(differential PRIVATE ${PROJECT_NAME} RecastNavigation::Recast
                                           RecastNavigation::Detour)
add_test(NAME differential COMMAND differential)
