# voyah_slam

Lock-free, event-driven, ORB-SLAM3-inspired SLAM library (C++20, Eigen, Conan 2).
Architecture and conventions: `docs/DESIGN.md`; pending work: `docs/TODO.md`.

## Prerequisites

- gcc 13 or clang 18, CMake >= 3.23 (Conan presets; 4.0 used), Conan 2 (`pip install conan`, then
  `conan profile detect` once), Linux (Ubuntu 24.04 tested).
- System packages for the GUI/GL parts: GTK2 dev files (`libgtk2.0-dev`), OpenGL/GLEW/X11
  dev files (`libgl1-mesa-dev libglew-dev libx11-dev libwayland-dev libxkbcommon-dev
  wayland-protocols`), `pkg-config`, `git`.

### OpenCV 5.1 and Pangolin 0.9.3 (not from Conan; built from source into `/usr/local`)

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
`g2o/20241228@voyah/stable`. It must be in your local Conan cache before the project's
`conan install`, otherwise Conan fails with `ERROR: Package 'g2o/20241228@voyah/stable' not resolved: Unable to find 'g2o/20241228@voyah/stable' in remotes.`
Create both build types (about two minutes each):

```
conan create third_party/g2o --build=missing -s build_type=Debug
conan create third_party/g2o --build=missing -s build_type=Release
```

See `third_party/README.md` for details (license, options, updating).

## Build and test

Debug (tree `cmake-build-debug/build/Debug`):

```
conan install . --output-folder=cmake-build-debug --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build cmake-build-debug/build/Debug -j"$(nproc)"
ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120 --output-on-failure
```

Release (tree `build/build/Release`):

```
conan install . --output-folder=build --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build build/build/Release -j"$(nproc)"
ctest --test-dir build/build/Release -j8 --timeout 120 --output-on-failure
```

The library, tests, examples and the visualization library build in one CMake tree.
The presets come from the Conan-generated `CMakePresets.json`, which `CMakeUserPresets.json`
includes (so the `conan install` must run first).

## Running the apps

Datasets are TUM-VI style directories (e.g. `dataset-corridor1_1024_16`).

```
build/build/Release/examples/full_slam/monocular_full_slam <dataset_root> [--rate=R] [--max-frames=N] [--show] [--viewer]
build/build/Release/examples/full_slam/binocular_full_slam <dataset_root> [--rate=R] [--max-frames=N] [--show] [--viewer]
```

- `--rate=R`: playback speed (1 = real time, 0 = as fast as possible)
- `--max-frames=N`: stop after N frames
- `--show`: show frames with keypoints (q / Esc quits)
- `--viewer`: open the Pangolin 3D map viewer
