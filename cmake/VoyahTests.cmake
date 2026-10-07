# Test binaries and the `check` / `check_all` targets.
#
# voyah_add_test_binary(<name> LABEL <unit|integration> SOURCES ... LIBS ...)
#   Adds a GoogleTest binary linking LIBS and GTest::gtest_main; its tests are
#   discovered with the ctest label LABEL. `check` and `check_all` depend on
#   every binary added this way.
#
# voyah_optimize_target(<target>)  (see below; applied to every test binary)
#
# voyah_add_check_targets()
#   Call once, after every test binary has been added.

include(GoogleTest)

# voyah_optimize_target(<target>)
#   With VOYAHSLAM_OPTIMIZE_TESTS ON and a Debug build, compiles <target> with
#   -O2 (keeping -g; NDEBUG stays undefined, so asserts remain on).
function(voyah_optimize_target target)
    if(VOYAHSLAM_OPTIMIZE_TESTS)
        target_compile_options(${target} PRIVATE
            $<$<CONFIG:Debug>:-O2>)
    endif()
endfunction()

function(voyah_add_test_binary name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "LABEL" "SOURCES;LIBS")
    if(NOT ARG_LABEL MATCHES "^(unit|integration)$")
        message(FATAL_ERROR "voyah_add_test_binary(${name}): LABEL must be unit or integration")
    endif()
    add_executable(${name} ${ARG_SOURCES})
    voyah_optimize_target(${name})
    target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
    gtest_discover_tests(${name} PROPERTIES LABELS ${ARG_LABEL} DISCOVERY_TIMEOUT 60)
    set_property(GLOBAL APPEND PROPERTY VOYAH_TEST_BINARIES ${name})
endfunction()

function(voyah_add_check_targets)
    get_property(binaries GLOBAL PROPERTY VOYAH_TEST_BINARIES)
    # Fast loop: unit tests only.
    add_custom_target(check
        COMMAND ${CMAKE_CTEST_COMMAND} -L unit -j8 --timeout 120 --output-on-failure
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        USES_TERMINAL
    )
    # Every test: end of each task and before every merge.
    add_custom_target(check_all
        COMMAND ${CMAKE_CTEST_COMMAND} -j8 --timeout 300 --output-on-failure
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        USES_TERMINAL
    )
    add_dependencies(check ${binaries})
    add_dependencies(check_all ${binaries})
endfunction()
