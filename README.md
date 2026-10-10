# voyah_slam

Lock-free, event-driven, ORB-SLAM3-inspired SLAM library (C++20, Eigen, Conan 2).
Architecture and conventions: `docs/DESIGN.md`; pending work: `docs/TODO.md`.

## Libraries

| Target | Directory | Contents | Depends on |
|---|---|---|---|
| `voyah_common` | `libs/common` | ids, geometry types, cameras and distortion models, memory pools, logging, image and feature/measurement data types, map storage | Eigen, spdlog |
| `voyah_events` | `libs/events` | event types, pub/sub (`Observable`, `AsyncObserver`, `WorkerThread`) | common, concurrentqueue |
| `voyah_algs` | `libs/algs` | geometry algorithms, estimators, feature extractors and matchers, triangulation, optimization | common (OpenCV, stb, g2o private) |
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
conan install . --output-folder=cmake-build-debug --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build cmake-build-debug/build/Debug -j"$(nproc)"
cmake --build cmake-build-debug/build/Debug --target check       # unit tests (633, ~5.5 s)
cmake --build cmake-build-debug/build/Debug --target check_all   # every test (649, 17-25 s)
```

`check` runs `ctest -L unit`, `check_all` runs every label (`unit`, `integration`, `vocabulary`); both build the
test binaries first. With `VOYAHSLAM_OPTIMIZE_TESTS=ON` (the default) the Debug tree
compiles the libraries and tests with `-O2 -g` and keeps asserts on; configure with
`-DVOYAHSLAM_OPTIMIZE_TESTS=OFF` to step through the code in a debugger. Each library
has its own test binary (`voyah_<lib>_tests`, plus `voyah_integration_tests`,
`voyahviz_tests` and `dataset_reader_tests`).

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
build/build/Release/examples/full_slam/monocular_full_slam <dataset_root> [options]
build/build/Release/examples/full_slam/binocular_full_slam <dataset_root> [options]
```

- `--rate=R`: playback speed (1 = real time, 0 = as fast as possible)
- `--max-frames=N`: stop after N frames
- `--show`: show frames with keypoints (q / Esc quits)
- `--viewer`: open the Pangolin 3D map viewer
- `--trajectory_out <file>`: write the tracked poses (TUM format, camera-to-world); `tools/ate.py
  <file> <dataset_root>/mav0/mocap0/data.csv` reports the ATE against the mocap
- `--vocabulary <file.bin>`: bag-of-words vocabulary in our binary format (see below); without it
  the Mapper matches by brute force when triangulating, which is much slower
- `--loop_closing`: enable loop closing (needs `--vocabulary`). Off by default: it works on
  synthetic loops and nearby revisits, but on TUM-VI corridor1 same-heading look-alikes make the
  ATE worse (see `docs/DESIGN.md` "Loop Closing"; tuning is roadmap step 6)
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
