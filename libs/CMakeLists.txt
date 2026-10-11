add_subdirectory(common)
add_subdirectory(events)
add_subdirectory(algs)
add_subdirectory(slam)

# Shared test helpers (voyah_test_support), used by the libraries' tests.
if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(test_support)
endif()

# voyahslam: umbrella target linking all four libraries, so apps and outside
# users keep linking a single name.
add_library(voyahslam INTERFACE)
target_link_libraries(voyahslam
    INTERFACE voyah_common
    INTERFACE voyah_events
    INTERFACE voyah_algs
    INTERFACE voyah_slam
)

# Install and export (DESIGN §7 "Build options and packaging"): the four
# libraries and the umbrella as voyahslam::<target>, their headers (each
# include root merges into one include/voyahslam/ tree) and a CMake package
# config. Conan consumers use the recipe's components instead (conanfile.py).
include(CMakePackageConfigHelpers)

set(VOYAHSLAM_LIBRARIES voyah_common voyah_events voyah_algs voyah_slam)
install(TARGETS ${VOYAHSLAM_LIBRARIES} voyahslam
    EXPORT voyahslamTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
foreach(lib common events algs slam)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${lib}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endforeach()

set(VOYAHSLAM_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/voyahslam)
install(EXPORT voyahslamTargets
    NAMESPACE voyahslam::
    FILE voyahslamTargets.cmake
    DESTINATION ${VOYAHSLAM_CMAKE_DIR}
)

# A static voyah_algs exports g2o as $<LINK_ONLY:...>, so the config must find
# g2o; a shared one has g2o linked in and needs nothing.
get_target_property(VOYAHSLAM_ALGS_TYPE voyah_algs TYPE)
if(VOYAHSLAM_ALGS_TYPE STREQUAL "STATIC_LIBRARY")
    set(VOYAHSLAM_STATIC ON)
else()
    set(VOYAHSLAM_STATIC OFF)
endif()
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/voyahslamConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/voyahslamConfig.cmake
    INSTALL_DESTINATION ${VOYAHSLAM_CMAKE_DIR}
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/voyahslamConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/voyahslamConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/voyahslamConfigVersion.cmake
    DESTINATION ${VOYAHSLAM_CMAKE_DIR}
)
