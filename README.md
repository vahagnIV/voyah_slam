# voyah_slam

Lock-free, event-driven, ORB-SLAM3-inspired SLAM library (C++20, Eigen, Conan 2).
Architecture and conventions: `docs/DESIGN.md`; pending work: `docs/TODO.md`.

## Libraries

| Target | Directory | Contents | Depends on |
|---|---|---|---|
| `voyah_common` | `libs/common` | ids, geometry types, cameras and distortion models, grow-only column buffers, logging, image and feature/measurement data types, map storage | Eigen, spdlog |
| `voyah_events` | `libs/events` | event types, pub/sub (`Observable`, `AsyncObserver`, `WorkerThread`) | common, concurrentqueue |
| `voyah_algs` | `libs/algs` | geometry algorithms, estimators, feature extractors and matchers, triangulation, optimization | common (stb, g2o private; no OpenCV) |
| `voyah_slam` | `libs/slam` | Tracker, Mapper, LoopCloser, `Slam` | common, events, algs |
| `voyahslam` | `libs` | umbrella (INTERFACE) linking all four; what apps link | all of the above |
| `voyahviz` | `vizualization` | map viewer and image windows (dev tool) | common, events (OpenCV, Pangolin private) |

Each library has its own include root (`libs/<lib>/include/voyahslam/...`); headers are
still included as `#include <voyahslam/...>`, and a library sees only its own root and
its dependencies' roots, so a lower layer cannot include an upper one.

## Prerequisites

- gcc 13 or clang 18, CMake >= 3.23 (Conan presets; 4.0 used), Conan 2 (`pip install conan`, then
  `conan profile detect` once), Linux (Ubuntu 24.04 tested).
- System packages for the GUI/GL parts: GTK2 dev files (`libgtk2.0-dev`), OpenGL/GLEW/X11
  dev files (`libgl1-mesa-dev libglew-dev libx11-dev libwayland-dev libxkbcommon-dev
  wayland-protocols`), `pkg-config`, `git`.

### OpenCV 5.1 and Pangolin 0.9.3 (not from Conan; built from source into `/usr/local`)

The `voyah_*` libraries do not use OpenCV (since 2026-10-11); it is needed only by the
tests and benchmarks (as a reference) and by `voyahviz`, which the `full_slam` apps use.

The exact configure flags used on the development machine were not recorded; only
`WITH_GTK=ON` is (DESIGN.md section 7). The following is the minimal set this project
needs (`core imgproc features highgui` for OpenCV, `pango_display` for Pangolin):

```
git clone --branch 5.1.0 --depth 1 https://github.com/opencv/opencv.git
cmake -S opencv -B opencv/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DWITH_GTK=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build opencv/build -j"$(nproc)" && sudo cmake --install opencv/build

git clone --branch v0.9.3 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build Pangolin/build -j"$(nproc)" && sudo cmake --install Pangolin/build
sudo ldconfig
```

CMake finds them as `OpenCV 5` (`/usr/local/lib/cmake/opencv5`) and `Pangolin 0.9`.

## One-time: package g2o with Conan

g2o 20241228 is vendored in `third_party/g2o/src/` and packaged by a local recipe as
`g2o/20241228.1@voyah/stable` (upstream tag 20241228, recipe revision `.1`). It must be in your local Conan cache before the project's
`conan install`, otherwise Conan fails with `ERROR: Package 'g2o/20241228.1@voyah/stable' not resolved: Unable to find 'g2o/20241228.1@voyah/stable' in remotes.`
Create both build types (about two minutes each):

```
conan create third_party/g2o --build=missing -s build_type=Debug
conan create third_party/g2o --build=missing -s build_type=Release
```

See `third_party/README.md` for details (license, options, updating).

**Loop closing (2026-10-09) added the `types_sim3` component** to the recipe,
whose version became `20241228.1` (2026-10-10). A machine with only the old
`g2o/20241228` package gets the "not resolved" error above: run both
`conan create` commands, then the project's `conan install` for each build type.

## Build and test

Debug (tree `cmake-build-debug/build/Debug`):

```
conan install . --output-folder=cmake-build-debug --build=missing -pr profiles/linux-gcc13 \
      -s build_type=Debug -o with_tests=True -o with_examples=True
cmake --preset conan-debug
cmake --build cmake-build-debug/build/Debug -j"$(nproc)"
cmake --build cmake-build-debug/build/Debug --target check       # unit tests (792, ~10 s)
cmake --build cmake-build-debug/build/Debug --target check_all   # every test (832, ~30 s)
```

`check` runs `ctest -L unit`, `check_all` runs every label (`unit`, `integration`, `vocabulary`); both build the
test binaries first. With `VOYAHSLAM_OPTIMIZE_TESTS=ON` (the default) the Debug tree
compiles the libraries and tests with `-O2 -g` and keeps asserts on; configure with
`-DVOYAHSLAM_OPTIMIZE_TESTS=OFF` to step through the code in a debugger. Each library
has its own test binary (`voyah_<lib>_tests`, plus `voyah_integration_tests`,
`voyahviz_tests` and `dataset_reader_tests`).

Release (tree `build/build/Release`):

```
conan install . --output-folder=build --build=missing -pr profiles/linux-gcc13 \
      -o with_tests=True -o with_examples=True
cmake --preset conan-release
cmake --build build/build/Release -j"$(nproc)"
ctest --test-dir build/build/Release -j8 --timeout 120 --output-on-failure
```

The library, tests, examples and the visualization library build in one CMake tree.
The presets come from the Conan-generated `CMakePresets.json`, which `CMakeUserPresets.json`
includes (so the `conan install` must run first).

**Build options.** `conanfile.py` (the package recipe, also used for these dev installs) has
the options `with_tests` and `with_examples` (both False by default). They set the CMake
options `VOYAHSLAM_BUILD_TESTS` and `VOYAHSLAM_BUILD_EXAMPLES` (cache variables in the
generated preset) and add the matching Conan dependencies (gtest; yaml-cpp). With both OFF
only the four libraries are configured, needing just Eigen, spdlog/fmt, concurrentqueue, g2o
and stb:

- `VOYAHSLAM_BUILD_TESTS`: the tests, `voyah_test_support`, the benchmarks, the `check` /
  `check_all` targets; needs GTest and OpenCV.
- `VOYAHSLAM_BUILD_EXAMPLES`: the apps, `voyahviz`, `tools/convert_vocabulary`; needs OpenCV,
  Pangolin and yaml-cpp.

**C++20.** The recipe's `validate()` requires `compiler.cppstd` >= 20 (the public headers
are C++20); the default profile's `gnu17` is rejected. `profiles/linux-gcc13` is the default
profile plus `compiler.cppstd=gnu20` and `build_type=Release` (override with `-s
build_type=Debug`); passing `-s compiler.cppstd=gnu20` with the default profile is
equivalent. Dependencies built for `gnu17` are reused (Conan's cppstd compatibility).

## Install locally (Conan package)

The package is `voyahslam/0.9.2` (options `shared`, default False, and `fPIC`). g2o first
(once per profile and build type; see "One-time: package g2o with Conan"):

```
conan create third_party/g2o --build=missing -pr profiles/linux-gcc13
conan create . --build=missing -pr profiles/linux-gcc13
```

`conan create` builds the four libraries (no tests, no apps) and runs `test_package/`, which
links `voyahslam::voyahslam`, projects through a Brown-Conrady camera and runs a pose
optimization. With `-o shared=True` the libraries are shared, and spdlog and fmt are made
shared too (both are PUBLIC: static copies would end up in `libvoyah_common.so` and in the
consumer). `shared=True` is rejected on Windows (no export macros) and together with
`compiler.libcxx=c++_static` (see "Android"); shared libraries get versioned sonames
(`libvoyah_slam.so.0.9`) and an `$ORIGIN` runpath.

A consumer (`conanfile.txt` with `[requires] voyahslam/0.9.2`, generators `CMakeDeps` and
`CMakeToolchain`) must also build with C++20: install its dependencies with
`-pr profiles/linux-gcc13` (or `-s compiler.cppstd=gnu20`, or a profile of its own that sets
`compiler.cppstd` to 20 / gnu20), otherwise `validate()` rejects the package:

```cmake
find_package(voyahslam REQUIRED CONFIG)
target_link_libraries(app PRIVATE voyahslam::voyahslam)   # or voyahslam::voyah_slam, ...
target_compile_features(app PRIVATE cxx_std_20)
```

Targets: `voyahslam::voyah_common`, `voyahslam::voyah_events`, `voyahslam::voyah_algs`,
`voyahslam::voyah_slam` and the umbrella `voyahslam::voyahslam`. Only the PUBLIC
dependencies' include paths reach the consumer (Eigen, spdlog/fmt, concurrentqueue); g2o and
stb are private (a static consumer links g2o's libraries, without its headers).

Without Conan, `cmake --install` of a build tree installs the same targets with a CMake
package config (`lib/cmake/voyahslam`), which finds Eigen3, spdlog, concurrentqueue and, for
a static build, g2o with `find_dependency`. (The Conan package drops these files: Conan's
CMakeDeps generates the config from the recipe's components.)

## Android (arm64-v8a)

Cross-build with the NDK (r30 tested, at `~/Android/Sdk/ndk/30.0.16248370`; set
`ANDROID_NDK_HOME` to use another) using `profiles/android-arm64` as the host profile. Conan
2.18 does not know clang 21 (NDK r30's), so install `profiles/settings_user.yml` once.
`conan config install` **replaces** an existing `~/.conan2/settings_user.yml` (it does not
merge); if you already have one, add `"21"` to its `compiler.clang.version` list by hand
instead:

```
conan config install profiles/settings_user.yml
conan create third_party/g2o --build=missing -pr:h profiles/android-arm64 -pr:b default
conan create .               --build=missing -pr:h profiles/android-arm64 -pr:b default
```

The profile: `os=Android`, `os.api_level=24`, `arch=armv8`, `compiler=clang` 21,
`compiler.libcxx=c++_static`, `compiler.cppstd=20`, `build_type=Release`, and
`tools.android:ndk_path`. All dependencies (Conan Center's eigen, spdlog, fmt,
concurrentqueue, stb; our g2o) build unpatched. `test_package` is cross-built but not run
(`can_run` is false). `voyah_algs` compiles the NEON FAST kernel
(`CMAKE_SYSTEM_PROCESSOR` is `aarch64`); no x86 code is built.

- **API level 24** (Android 7.0): the default minSdk of new Android Studio projects and above
  NDK r30's minimum (21); the library needs nothing newer. The app's `minSdkVersion` must be
  >= the profile's `os.api_level`; change both together.
- **`c++_static` (user decision, 2026-10-11).** The app has a single JNI `.so`, which links
  the static voyahslam libraries and is built with `ANDROID_STL=c++_static`. NDK rule:
  `c++_static` is safe only while the app has **exactly one** shared library containing C++.
  If a second one is ever added (another JNI library, a prebuilt C++ `.so`), every library
  and the package must switch to `c++_shared` (`compiler.libcxx=c++_shared`, and package
  `libc++_shared.so` in the APK); otherwise each `.so` has its own copy of libc++ (separate
  heaps for `new`/`delete` across the boundary, duplicated globals, broken exceptions and
  RTTI between them).

### Using the package from an Android app (Gradle + CMake)

1. Install the Android dependencies into a folder the app's CMake will read (once, and again
   after `conan create` of a new voyahslam version). From this repository:

   ```
   conan install --requires=voyahslam/0.9.2 -g CMakeDeps \
         -pr:h profiles/android-arm64 -pr:b default \
         --output-folder=<app>/app/src/main/cpp/voyahslam-deps
   ```

2. The app's `app/src/main/cpp/CMakeLists.txt`:

   ```cmake
   cmake_minimum_required(VERSION 3.22.1)
   project(app_jni CXX)
   find_package(voyahslam REQUIRED CONFIG)
   add_library(app_jni SHARED jni.cpp)
   target_compile_features(app_jni PRIVATE cxx_std_20)
   target_link_libraries(app_jni PRIVATE voyahslam::voyahslam android log)
   ```

3. `app/build.gradle.kts`:

   ```kotlin
   android {
       ndkVersion = "30.0.16248370"
       defaultConfig {
           minSdk = 24                       // >= the profile's os.api_level
           ndk { abiFilters += "arm64-v8a" }
           externalNativeBuild {
               cmake {
                   val deps = file("src/main/cpp/voyahslam-deps").absolutePath
                   arguments += listOf(
                       "-DANDROID_STL=c++_static",
                       "-DCMAKE_FIND_ROOT_PATH=$deps",
                       "-DCMAKE_MAP_IMPORTED_CONFIG_DEBUG=Release",
                   )
               }
           }
       }
       externalNativeBuild {
           cmake { path = file("src/main/cpp/CMakeLists.txt") }
       }
   }
   ```

- **`CMAKE_FIND_ROOT_PATH`, not `CMAKE_PREFIX_PATH`:** the NDK toolchain sets
  `CMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY`, which re-roots every search path under the
  sysroot, so a deps folder on `CMAKE_PREFIX_PATH` is not found ("Could not find a package
  configuration file provided by voyahslam").
- **`CMAKE_MAP_IMPORTED_CONFIG_DEBUG=Release`:** the package and its dependencies are built
  Release only; Gradle's debug variant configures with `CMAKE_BUILD_TYPE=Debug`, and
  CMakeDeps' targets then carry no include directories or libraries for that configuration
  (`'voyahslam/geometry/camera.h' file not found`). The map makes Debug use the Release
  binaries.

Checked on 2026-10-11 with the NDK's `build/cmake/android.toolchain.cmake` (CMake 4.0.3,
Ninja, `ANDROID_ABI=arm64-v8a`, `ANDROID_PLATFORM=24`): a JNI `.so` calling `OptimizePose`
links for Debug and Release, with no `libc++_shared.so` dependency. Without either flag it
fails as described. Gradle's own CMake (Android SDK `cmake;3.22.1`) and an APK build were not
run.

## Running the apps

Datasets are EuRoC-layout directories (TUM-VI, e.g. `dataset-corridor1_1024_16`, or EuRoC MAV).
Supported camera distortion models: `equidistant` (Kannala-Brandt fisheye), Brown-Conrady
(`radial-tangential` / `radtan` / `plumb_bob`, 4 or 5 coefficients) and `none`. EuRoC MAV has
not yet been run end to end (see `docs/TODO.md`).

```
build/build/Release/examples/full_slam/monocular_full_slam <dataset_root> [options]
build/build/Release/examples/full_slam/binocular_full_slam <dataset_root> [options]
```

- `--rate=R`: playback speed (1 = real time, 0 = as fast as possible)
- `--max-frames=N`: stop after N frames
- `--show`: show frames with keypoints (q / Esc quits)
- `--viewer`: open the Pangolin 3D map viewer
- `--trajectory_out <file>`: write the tracked poses (TUM format, camera-to-world); `tools/ate.py
  <file> <dataset_root>/mav0/mocap0/data.csv` reports the ATE against the mocap (the largest map;
  `--map ID` picks one, e.g. the map that survives a merge)
- `--vocabulary <file.bin>`: bag-of-words vocabulary in our binary format (see below); without it
  the Mapper matches by brute force when triangulating, which is much slower
- `--loop_closing`: enable loop closing (needs `--vocabulary`). Off by default: it works on
  synthetic loops and nearby revisits, but on TUM-VI corridor1 same-heading look-alikes make the
  ATE worse (see `docs/DESIGN.md` "Loop Closing"; tuning is roadmap step 6). With it the maps are
  also merged: a lost map of at least 10 Frames is kept inactive and merged with the active map
  when the camera revisits it (ORB-SLAM3's Atlas); without it every lost map is discarded; the
  apps print the maps left and the merges at the end
- `--no_global_ba`: with `--loop_closing`, correct loops by the essential graph only. By default a
  global BA (ORB-SLAM3's, in its own thread) follows each loop correction; on a full corridor1 map
  it takes up to about 85 s (stereo) and keeps one more copy of the map (peak memory about 4 GB
  instead of 2.3 GB). The app prints "Global BA: on" only when it runs (it needs `--loop_closing`
  and `--vocabulary`)
- `--drop_frames a:b`: feed frames a..b (inclusive, 0-based) without features (a simulated blackout)

### Vocabulary (ORB-SLAM3's `ORBvoc.txt`)

The vocabulary is not in this repository. Download `Vocabulary/ORBvoc.txt.tar.gz` from
[github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), extract it, and convert
it once to the binary format (about 0.8 s; the 145 MB text becomes a 48 MB binary that loads in
about 0.13 s):

```
tar -xzf ORBvoc.txt.tar.gz
build/build/Release/tools/convert_vocabulary/convert_vocabulary ORBvoc.txt ORBvoc.bin
build/build/Release/examples/full_slam/monocular_full_slam <dataset_root> --vocabulary ORBvoc.bin
```

`convert_vocabulary` reloads the binary and verifies it against the text. Its licence terms are
not settled (`docs/TODO.md`), so do not commit or redistribute it. The DBoW2 equivalence test
(ctest label `vocabulary`) runs only when `VOYAHSLAM_ORBVOC` points to `ORBvoc.txt`:

```
VOYAHSLAM_ORBVOC=/path/to/ORBvoc.txt ctest --test-dir build/build/Release -L vocabulary
```
