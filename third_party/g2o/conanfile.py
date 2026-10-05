import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy


class G2oConan(ConanFile):
    name = "g2o"
    version = "20241228"
    user = "voyah"
    channel = "stable"
    package_type = "library"
    license = "BSD-2-Clause"
    url = "https://github.com/RainerKuemmerle/g2o"
    description = "g2o: general framework for graph optimization (vendored, tag 20241228_git)"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    exports_sources = "src/*"

    def requirements(self):
        # g2o's public headers include Eigen, so consumers must see it.
        self.requires("eigen/3.4.0", transitive_headers=True)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        # Upstream names this BUILD_UNITTESTS (there is no BUILD_TESTING switch).
        tc.variables["BUILD_UNITTESTS"] = False
        tc.variables["G2O_USE_CHOLMOD"] = False
        tc.variables["G2O_USE_CSPARSE"] = False
        tc.variables["G2O_USE_OPENGL"] = False
        tc.variables["G2O_USE_LOGGING"] = False
        tc.variables["G2O_BUILD_APPS"] = False
        tc.variables["G2O_BUILD_EXAMPLES"] = False
        tc.variables["G2O_BUILD_BENCHMARKS"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "LICENSE", self.source_folder,
             os.path.join(self.package_folder, "licenses"))
        CMake(self).install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "g2o")

        # Components mirror upstream's target link order:
        #   stuff <- core <- types_slam3d <- types_sba ; core <- solver_eigen
        def comp(name, lib, requires):
            c = self.cpp_info.components[name]
            c.set_property("cmake_target_name", f"g2o::{name}")
            c.libs = [lib]
            c.requires = requires
            c.includedirs = ["include"]
            return c

        comp("stuff", "g2o_stuff", ["eigen::eigen"])
        comp("core", "g2o_core", ["stuff", "eigen::eigen"])
        comp("types_slam3d", "g2o_types_slam3d", ["core"])
        comp("types_sba", "g2o_types_sba", ["core", "types_slam3d"])
        comp("solver_eigen", "g2o_solver_eigen", ["core"])
        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.components["stuff"].system_libs.append("rt")
