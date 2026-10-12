---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# Technology Stack

**Analysis Date:** 2026-10-12

## Languages

**Primary:**
- C++20 - All libraries and applications; required by public headers in `libs/common`, `libs/events`, `libs/algs`, `libs/slam`

## Runtime

**Environment:**
- Linux x86_64 (primary development platform, tested on Ubuntu 24.04)
- Android arm64-v8a (cross-compile support with NDK r30, API level 24+)

**Build System:**
- CMake >= 3.22 (3.23 for Conan presets; version 4.0 used in development)
- Conan 2 (package manager: `conan install .` for development, `conan create .` for packaging)

**Compilers:**
- GCC 13+ (verified; C++20 required)
- Clang 14+ (verified; C++20 required)
- Apple Clang 15+ (macOS support)
- NDK r30 Clang 21 (Android arm64 builds)

## Frameworks & Build Tools

**Build & Test Infrastructure:**
- GTest 1.15.0 - Unit and integration testing framework
  - Test binaries: `voyah_common_tests`, `voyah_events_tests`, `voyah_algs_tests`, `voyah_slam_tests`, `voyah_integration_tests`, `voyahviz_tests`, `dataset_reader_tests`
  - Run: `cmake --build . --target check` (unit tests, ~10 s) or `check_all` (all tests, ~30 s)
  - Custom cmake module: `cmake/VoyahTests.cmake`

**Optimization & Debugging:**
- VOYAHSLAM_OPTIMIZE_TESTS=ON (default) - Compiles libraries/tests with `-O2 -g` in Debug builds for performance
- TSAN/ASAN build trees available: `cmake-build-tsan/`, `cmake-build-asan/` for thread/memory sanitizing

## Key Dependencies

### Core Libraries (Conan)

**Critical, publicly exposed:**
- `Eigen/3.4.0` - Linear algebra (vectors, matrices, geometric computations)
  - Declared: `transitive_headers=True`, `transitive_libs=True`
  - Used in: `libs/common`, `libs/algs` (SLAM geometry)
  - File: `conanfile.py:65`

- `spdlog/1.17.0` - Structured logging
  - Declared: `transitive_headers=True`, `transitive_libs=True`
  - Public logger in: `libs/common/include/voyahslam/logging/logger.h`
  - File: `conanfile.py:66`

- `concurrentqueue/1.0.4` - Lock-free concurrent queue (moodycamel)
  - Declared: `transitive_headers=True`
  - Used in: `libs/events` for event pub/sub
  - Target name resolution: `conanfile.py:26-31` (handles Conan Center name variations)
  - File: `conanfile.py:67`

- `fmt/<auto>` - Text formatting (transitive via spdlog)
  - Shared library support: conanfile.py forces `shared=True` when library is shared (`conanfile.py:56-57`)

**Private, not exposed in headers:**
- `g2o/20241228.1@voyah/stable` - Pose graph optimization and structure-from-motion algorithms
  - Vendored in: `third_party/g2o/src/` (upstream tag 20241228_git, pristine, never modified)
  - Conan recipe: `third_party/g2o/conanfile.py`
  - Components packaged: `g2o::core`, `g2o::stuff`, `g2o::types_slam3d`, `g2o::types_sba`, `g2o::types_sim3`, `g2o::solver_eigen`
  - License: BSD-2-Clause (LGPL parts disabled via `G2O_USE_CHOLMOD=OFF`, `G2O_USE_CSPARSE=OFF`, `G2O_USE_OPENGL=OFF`)
  - Used in: `libs/algs/` for optimization, bundle adjustment
  - Must be pre-built: `conan create third_party/g2o --build=missing -s build_type={Debug,Release}`

- `stb/cci.20240531` - Single-header image loading library (header-only)
  - Declared: `transitive_headers=False`, `transitive_libs=False`
  - Used in: `libs/algs` image I/O (raw image data)
  - File: `conanfile.py:72`

### Development Tools (Conan, optional with `with_examples=True`)

- `yaml-cpp/0.8.0` - YAML config parsing
  - Used in: `examples/dataset_reader/` (EuRoC dataset format)
  - Not part of core library; examples only

- `OpenCV 5` - Image processing (system build from source, not Conan)
  - Source build: `/usr/local` with `WITH_GTK=ON`
  - Components: `opencv_core`, `opencv_imgproc`, `opencv_features`, `opencv_highgui`
  - Used in: Tests (reference implementations), visualization tool, examples
  - Not linked by `voyah_*` libraries; private to tests/tools
  - Build instructions: README.md provides exact cmake flags

- `Pangolin 0.9.3` - OpenGL window and visualization (system build from source, not Conan)
  - Source build: `/usr/local`
  - Component: `pango_display`
  - Used in: `vizualization/` library (map viewer, implementation detail)
  - Not exposed in public API; private to visualization tool

## Configuration

**Build Options** (via Conan options, passed to CMake):

```
-o shared=True|False              # Shared or static libraries (default: False)
-o fPIC=True|False               # Position-independent code for static (default: True)
-o with_tests=True|False         # Build unit & integration tests (default: False)
-o with_examples=True|False      # Build apps, tools, visualization (default: False)
```

**CMake Cache Variables** (set via Conan):
- `VOYAHSLAM_BUILD_TESTS` - Build GTest-based test suite
- `VOYAHSLAM_BUILD_EXAMPLES` - Build apps, visualization, tools
- `VOYAHSLAM_OPTIMIZE_TESTS=ON` (default) - Compile Debug builds with `-O2 -g` for test performance

**Build Profiles** (in `profiles/`):
- `linux-gcc13` - GCC 13, x86_64, C++20, Release by default
  - Include: `default` Conan profile
  - Settings: `compiler.cppstd=gnu20`, `build_type=Release`

- `android-arm64` - NDK r30 Clang 21, arm64-v8a, C++20, Android API 24
  - `compiler.libcxx=c++_static` (single JNI .so rule)
  - Template: `os.getenv("ANDROID_NDK_HOME")` or `~/Android/Sdk/ndk/30.0.16248370`
  - Tools: `tools.android:ndk_path={{ ndk }}`

**Environment Configuration:**
- `CMAKE_CXX_STANDARD=20` (default, enforced: `CMAKE_CXX_STANDARD_REQUIRED=ON`)
- Development trees: `cmake-build-debug/build/Debug`, `build/build/Release`
- Conan layout: `cmake_layout()` (standard layout)

**Version:**
- Package version: `0.3.0` (conanfile.py, CMakeLists.txt)
- g2o version: `20241228.1@voyah/stable` (upstream tag 20241228, recipe revision `.1`)

## Platform Requirements

**Development:**
- OS: Ubuntu 24.04 (tested); Linux/BSD with glibc
- Build: CMake 3.23+, Conan 2, gcc 13 or clang 18+
- System packages (apt):
  - Build tools: `build-essential`, `git`, `pkg-config`
  - GUI/GL (for visualization): `libgtk2.0-dev`, `libgl1-mesa-dev`, `libglew-dev`, `libx11-dev`, `libwayland-dev`, `libxkbcommon-dev`, `wayland-protocols`
  - Python: `pip` (for `pip install conan`)
- One-time: `conan profile detect`

**Production:**
- Deployment: Static linking (default `shared=False`)
  - Linux: Links `libm`, `libpthread` automatically for Linux/FreeBSD (`conanfile.py:139-140`)
  - Android: Links via NDK, `c++_static` runtime
  - macOS: Standard system libraries
- No runtime dependencies (vendored g2o, Eigen, spdlog/fmt headers only)
- ABI: C++20 (consumers must compile with C++20)

**Validation:**
- Compiler minimum: GCC 11, Clang 14, Apple Clang 15 (enforced in `conanfile.py:95-99`)
- Windows: Shared builds not supported (no symbol export macros)
- NDK: `c++_static` incompatible with `shared=True` (single shared library rule)

## Build & Test Workflow

**Quick cycle (unit tests, ~10 s):**

```bash
cmake --build cmake-build-debug/build/Debug --target check
```

**Full validation (all tests, ~30 s):**

```bash
cmake --build cmake-build-debug/build/Debug --target check_all
```

**Release build:**

```bash
cmake --build build/build/Release -j8
```

**Package creation (Conan):**

```bash
conan create . --build=missing -pr profiles/linux-gcc13
```

---

*Stack analysis: 2026-10-12*
