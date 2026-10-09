# In-tree samples: small programs built with the library. The minimal/
# subdirectory is a separate project that consumes the installed
# package instead; CI builds it against a fresh install.

foreach(sample walk crowd flight obstacles bake_world)
    add_executable(sample_${sample} ${sample}.c)
    target_link_libraries(sample_${sample} PRIVATE maul-nav)
    maul_apply_flags(sample_${sample})
    # Each sample returns 0 when its agents arrive: run as a test, so a
    # sample that stops working fails the suite.
    if(MAUL_NAV_BUILD_TESTS)
        add_test(NAME sample_${sample} COMMAND sample_${sample})
    endif()
endforeach()
