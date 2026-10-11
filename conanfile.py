import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, rmdir
from conan.tools.scm import Version


class VoyahSlamConan(ConanFile):
    """voyahslam: four libraries (voyah_common, voyah_events, voyah_algs,
    voyah_slam) plus the umbrella target voyahslam::voyahslam.

    Package: `conan create . --build=missing` (g2o first: third_party/README.md).
    Development trees: `conan install .` with `-o with_tests=True
    -o with_examples=True` (README "Build and test").
    """

    name = "voyahslam"
    version = "0.1.0"
    package_type = "library"
    license = "proprietary"
    description = "Lock-free, event-driven, ORB-SLAM3-inspired SLAM library"
    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_tests": [True, False],
        "with_examples": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_tests": False,
        "with_examples": False,
    }
    exports_sources = (
        "CMakeLists.txt",
        "cmake/*",
        "libs/*",
        "vizualization/*",
        "examples/*",
        "tools/convert_vocabulary/*",
    )

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")
            # spdlog and fmt are PUBLIC: static copies would be in both
            # libvoyah_common.so and the consumer (two default loggers).
            self.options["spdlog/*"].shared = True
            self.options["fmt/*"].shared = True

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        # PUBLIC: their headers are included by ours, and their libraries are
        # needed by whoever links us.
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("spdlog/1.17.0", transitive_headers=True, transitive_libs=True)
        self.requires("concurrentqueue/1.0.4", transitive_headers=True)
        # PRIVATE: never in our headers. g2o's libraries still reach a static
        # consumer's link line (Conan propagates libs of a static dependency of
        # a static library), but not its include paths. stb is header-only.
        self.requires("g2o/20241228.1@voyah/stable", transitive_headers=False)
        self.requires("stb/cci.20240531", transitive_headers=False, transitive_libs=False)
        if self.options.with_examples:
            # Only the dataset reader of the apps; never part of the package.
            self.requires("yaml-cpp/0.8.0", visible=False)

    def build_requirements(self):
        if self.options.with_tests:
            self.test_requires("gtest/1.15.0")

    def validate(self):
        # Public headers use C++20, so the consumer must compile with it too.
        check_min_cppstd(self, 20)
        minimum = {"gcc": "11", "clang": "14", "apple-clang": "15"}
        compiler = str(self.settings.compiler)
        if compiler in minimum and Version(self.settings.compiler.version) < minimum[compiler]:
            raise ConanInvalidConfiguration(
                f"voyahslam needs C++20: {compiler} >= {minimum[compiler]}")

    def package_id(self):
        # Tests and examples are not packaged.
        del self.info.options.with_tests
        del self.info.options.with_examples

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.cache_variables["VOYAHSLAM_BUILD_TESTS"] = bool(self.options.with_tests)
        tc.cache_variables["VOYAHSLAM_BUILD_EXAMPLES"] = bool(self.options.with_examples)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        # Conan consumers get the components below (CMakeDeps); the exported
        # CMake config is for plain `cmake --install` users.
        rmdir(self, os.path.join(self.package_folder, "lib", "cmake"))

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "voyahslam")
        # The package-level target aggregates every component: it is the
        # umbrella voyahslam::voyahslam of the CMake export.
        self.cpp_info.set_property("cmake_target_name", "voyahslam::voyahslam")

        def component(name, requires):
            c = self.cpp_info.components[name]
            c.set_property("cmake_target_name", f"voyahslam::{name}")
            c.libs = [name]
            c.requires = requires
            return c

        common = component("voyah_common", ["eigen::eigen", "spdlog::spdlog"])
        if self.settings.os in ("Linux", "FreeBSD"):
            common.system_libs = ["m", "pthread"]
        component("voyah_events", ["voyah_common", "concurrentqueue::concurrentqueue"])
        component("voyah_algs", [
            "voyah_common",
            "g2o::core", "g2o::stuff", "g2o::types_sba", "g2o::types_sim3",
            "g2o::types_slam3d", "g2o::solver_eigen",
            "stb::stb",
        ])
        component("voyah_slam", ["voyah_common", "voyah_events", "voyah_algs"])
