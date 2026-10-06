# A separate project that knows nothing about the maul-nav source tree
# and consumes only the installed package. CI installs the library and
# builds this; if it builds, links and runs, the package works.
#   cmake -S samples/minimal -B build-minimal -DCMAKE_PREFIX_PATH=<prefix>
cmake_minimum_required(VERSION 3.25)
project(maul-nav_minimal LANGUAGES C)

find_package(maul-nav REQUIRED CONFIG)
add_executable(minimal main.c)
target_link_libraries(minimal PRIVATE maul-nav::maul-nav)
set_target_properties(minimal PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
