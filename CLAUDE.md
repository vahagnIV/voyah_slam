# voyahslam

Lock-free, event-driven, ORB-SLAM3-inspired SLAM library (C++20, Eigen, Conan 2).

The ordered plan of next steps is in `docs/ROADMAP.md`.
Pending work is tracked in `docs/TODO.md` — check it, and add items there
whenever something is deferred.

**Read `docs/DESIGN.md` first** — it holds the architecture, principles, coding
conventions, decisions log, open questions and next steps. Keep it up to date:
whenever a design decision is made or changed, record it there (decisions log +
relevant section) in the same change.

Key rules (details in DESIGN.md):
- No locks on map data. Threads communicate only via events; each thread owns a map replica.
- Mapper is the only map writer. Tracker and LoopCloser never mutate map objects.
- Frame IDs are never reused within a run. Map point IDs are never reused within a
  map (append-only); they restart per map, and `(map_id, id)` identifies a point.
- Google style naming: `PascalCase` types/functions, `snake_case` variables,
  `snake_case_` members, `out_` prefix for non-const reference parameters.

Layout: four libraries under `libs/<lib>/{include,src,tests}`: `voyah_common`;
`voyah_events` and `voyah_algs` (each on common only); `voyah_slam` (on all
three); `voyahslam` is the umbrella target. Each has its own include root, so a
lower layer cannot include an upper one (DESIGN §7 "Library layout").

Build and test (`cmake` from the CLion bundle or any CMake >= 3.23):
- Debug: `cmake --build cmake-build-debug/build/Debug -j8`
- Fast loop (798 unit tests, about 10 s plus the build): `cmake --build cmake-build-debug/build/Debug --target check`
- Every test (end of a task, before a merge; about 30 s): `cmake --build cmake-build-debug/build/Debug --target check_all`
  (838 tests, labels `unit` + `integration` + `vocabulary`). Both run in the Debug tree with `VOYAHSLAM_OPTIMIZE_TESTS=ON`
  (default: `-O2 -g`, asserts on); turn it off to step through code in a debugger.
- Release: `cmake --build build/build/Release -j8` (apps in `examples/full_slam/`).
- Tests, apps and tools are built only with `VOYAHSLAM_BUILD_TESTS` / `VOYAHSLAM_BUILD_EXAMPLES`
  (OFF by default; the package build needs neither). The dev trees get them from
  `conan install … -pr profiles/linux-gcc13 -o with_tests=True -o with_examples=True` (README
  "Build"). If `check` / `check_all` are missing, re-run that install and `cmake --preset`.
