---
phase: 01-api-ownership-and-lifetime-contracts
plan: 11
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, map-follower, regression]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-08..01-10: MapFollower/MapSnapshot, thread-affine accessors private behind *TestAccess, DESIGN and version 0.5.0"
provides:
  - "R2 gates for voyahslam 0.5.0 all green on the second full run (check_all, apps, TSan, ASan, thread-affinity checks, three conan create)"
  - "Fix of a performance regression the first gate run missed: MapFollower defined in the library so an app built at -O0 does not slow the Tracker"
  - "refactor/thread-affinity merged into master (no-ff) and pushed; origin/master == master"
  - "voyahslam/0.5.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.5.0"
affects: [voyah_jni, phase-01-R3]

actuals:
  tokens: 0       # the merge only joins already-reviewed commits; the four fix commits are counted in the commit total
  tasks: 3
  commits: 5      # 4 fix commits on the branch (a9def71..5cd2077) + the merge commit; the docs commit that adds this SUMMARY is not counted
plan_head_before: a9def71
plan_head_after: f1ecb289141305851adc2c82cdbf0d9134ab21e4
commits: 5

tech-stack:
  added: []
  patterns:
    - "Release gates run on the branch, the merge happens only on the user's literal 'merge and push'"
    - "A class template that an -O0 app and the -O2 library would both instantiate is defined in the library (INSTANTIATE_CLASS), with a build check that the app object holds no library instantiations"

key-files:
  created: []
  modified: []

key-decisions:
  - "A gate that passes is not proof of behavior: the binocular 40-frame app run is compared with the R1 numbers (39 FrameTracked, 3 Frames), not only with rc 0"
  - "MapFollower is defined in the library (src/), not header-only, so a consumer built at -O0 (the example apps, a Debug voyah_jni) does not replace library functions with its own -O0 copies"
  - "The apps' remaining -O0 overrides of about 75 library functions are older than R2 and deferred in TODO, not fixed in this release"

requirements-completed: [OWN-03]

duration: "two full gate runs (TSan integration 16m31s, slam 6m56s) plus a bisect and fix; merge and push under 1 min"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 11: Release R2 (0.5.0) Summary

**R2 passed every D-21 gate on its second full run (after a regression the first run missed was bisected and fixed), was merged into master on the user's "merge and push" as f1ecb28, and pushed; OWN-03 ships as voyahslam 0.5.0.**

## Task 1: Release gates (two runs)

### Run 1 (branch HEAD a9def71): all gates passed, one regression missed

Every gate listed in the plan returned its pass signal. The orchestrator then compared the binocular app with R1 and found what the gates did not measure:

| App, corridor1, `--rate=0 --max-frames=40` | R1 (0.4.0) | R2 run 1 |
|--------------------------------------------|-----------|----------|
| `binocular_full_slam` | 39 FrameTracked, 3 Frames, about 750-777 points | **10-11 FrameTracked, 1 Frame, 375 points** |

The gates only required rc 0, no "terminate" and a "tracker state:" line, all of which held for the degraded run.

**How it was found, bisected and fixed.** The orchestrator bisected the regression to `fd43a13` (the header-only `MapFollower`, compiled into the `-O0` example apps). The apps build at `-O0`, the libraries at `-O2`; the linker takes every inline or template function both instantiate from the app object (first on the link line), so the app's `-O0` copies of `AtlasReplica` code and of Eigen descriptor copies (used by the Tracker's `MatchStereo`) replaced the optimized library copies. The Tracker then ran slowly enough to lose frames. Fix, in four commits on `refactor/thread-affinity`:

| Commit | Change |
|--------|--------|
| `dac50cf` | `fix(01-10)`: `MapFollower` defined in the library (`INSTANTIATE_CLASS`), so apps instantiate nothing of it |
| `202fe43` | `test(01-10)`: build check `BuildCheck.MapFollowerAddsNoLibraryInstantiations` (the consumer object `follower_consumer.cpp` must hold no library instantiations; `check_consumer_instantiations.cmake`) |
| `5b2bb46` | `docs(01-10)`: DESIGN and TODO record the cause; the apps' remaining `-O0` overrides are deferred in TODO |
| `5cd2077` | `docs(01-11)`: CLAUDE.md test counts after the build check (772 unit, 803 total) |

Binocular went back to 39 FrameTracked and 3 Frames.

### Run 2 (branch HEAD 5cd2077): full re-run, all gates green

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 803 tests, 100% pass: 772 unit, 30 integration, 1 skipped (`VocabularyEquivalenceTest`, ORBvoc absent). No flaky failure (`SlamTest.DuplicatesFusedInLongSequence`, which failed once in 01-10, did not fail) |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames (same as R1) |
| `binocular_full_slam`, same arguments | rc 0, **39 FrameTracked, 3 Frames, 777 points**. New bar for later releases: at least 36 FrameTracked and at least 3 Frames |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 67 pass; `voyah_slam_tests` (Tracker*, Mapper*, Slam*, GlobalBundle*, LoopCloser*, MapFollower*) 194 pass, 6m56s; `voyah_integration_tests` 28 pass, 16m31s; 0 TSan warnings. The first integration attempt was cut off by a harness timeout (not a test failure) and was re-run alone to completion |
| ASan `voyah_events_tests` | 68 pass, no AddressSanitizer or LeakSanitizer error |
| `ThreadAffinityTest.*:MapFollowerTest.*` | 9 pass |
| Restricted-accessor grep (`mapper.map_ids()`, `mapper.active_map()`, `tracker().map_replica()`) over `examples`, `vizualization` | empty |
| `conan create` voyahslam/0.5.0 | linux-gcc13 static rc 0 (1m5s) and shared rc 0; test_package printed "voyahslam 0.5: unproject error 2.48e-14, pose error 8.21e-16, inliers 30" on both; android-arm64 rc 0 |

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r2b_*.log`.

## Task 2: User review

Two replies were recorded.

- After run 1 (the orchestrator reported the binocular regression and asked for direction), the user's reply, verbatim: "debug it"
- After run 2, the user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- Preconditions checked before merging: branch `refactor/thread-affinity` HEAD 5cd2077, tree clean apart from the orchestrator's untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `master` == `origin/master` == 3b82497.
- `git switch master`, `git merge --no-ff refactor/thread-affinity`. Merge commit: **f1ecb289141305851adc2c82cdbf0d9134ab21e4**, subject "Merge branch 'refactor/thread-affinity': MapFollower read path, thread-affine accessors test-only; version 0.5.0". It brings in 16 branch commits (37 files, +2102/-421).
- `git push origin master`: `3b82497..f1ecb28  master -> master` succeeded; `git rev-parse origin/master master` gave identical SHAs at push time.
- The branch `refactor/thread-affinity` was not deleted. No force push.

## Deviations from Plan

### Auto-fixed / user-directed issues

**1. [Rule 1 - Bug] Binocular tracking regression from the header-only MapFollower**
- **Found during:** Task 1 (orchestrator comparison of run 1 with R1; the plan's gates did not catch it)
- **Issue:** see "Run 1" above: 10-11 of 40 frames tracked instead of 39.
- **Fix:** `MapFollower` defined in the library, plus a build check; user chose "debug it" at the checkpoint. Task 1 and Task 2 were repeated in full, as the plan requires for a fix on the branch.
- **Files modified:** `libs/slam/include/voyahslam/mapping/map_follower.h`, `libs/slam/src/mapping/map_follower.cpp`, `libs/slam/tests/build_checks/*`, `docs/DESIGN.md`, `docs/TODO.md`, `CLAUDE.md`
- **Commits:** `dac50cf`, `202fe43`, `5b2bb46`, `5cd2077`

**Gate gap (not fixed here):** the plan's app gate checks only rc 0 and a "tracker state:" line. This run established a quantitative bar for the binocular run (at least 36 FrameTracked, at least 3 Frames); later release plans (R3 and after) should state it.

## Open items (recorded in docs/TODO.md)

1. **Apps' remaining `-O0` overrides** of about 75 library functions (older than R2; after the fix the set is a subset of 5079f94's). Possible effect on monocular, 200 frames: 13 and 13 Frames with the app at `-O0` against 16, 16 and 17 at `-O2`. Few and timing-dependent runs; unconfirmed. Options in TODO: `voyah_optimize_target()` on the apps, keeping library hot loops off out-of-line template helpers.
2. **`MapFollower::atlas()` public or not.** Its replica accessors are inline, so a caller instantiates them in its own translation unit; apps and a future Session should read `Snapshot()`. Decision deferred with the `voyahslam::Session` facade and the viewer on a follower.

## Known Stubs

None.

## Threat Flags

None. T-01-19 mitigated: blocking-human checkpoint, literal replies recorded above, minor bump 0.5.0, and the voyah_jni message below.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.5.0** (`voyahslam/0.5.0`, SOVERSION 0.5), merged and pushed to master as f1ecb28. R2 (ownership audit #5) is an API break for the binding, on top of 0.4.0 (R1: `EventPtr`, `Subscription`, etc.). Every break:

1. **Thread-affine accessors are no longer public** (they are private behind test-only `*TestAccess` structs; calling them from the binding no longer compiles):
   - `Tracker::atlas()`, `Tracker::map_replica()`, `Tracker::last_matches()`
   - `LoopCloser::atlas()`, `map_replica()`, `num_coincidences()`, `num_merge_coincidences()`, `last_stats()`, `last_merge_stats()`, `query_stats()`
   - `GlobalBundleAdjuster::atlas()`, `map_replica()`
   - the Mapper's `covisibility()`, `Reference()`, `NumVisible()`, `NumFound()`, `last_local_ba()`, `BowVectorOf()`, `Parent()`, `LoopEdges()`, `KeyFrame()`, `map_ids()`, `active_map()`, `num_keyframes(MapId)`
2. **To read the map from the binding's own thread, use `mapping::MapFollower<OrbFeatureExtractor>`:**
   - subscribe `mapper.AddObserver(follower.inbox())` **before** `Start()` and hold the returned `Subscription` (declare it before the `Slam`, as in 0.4.0);
   - call `Update()` on one thread (the follower owns an `AtlasReplica` and applies the Mapper's transactions on the caller's thread);
   - hand `Snapshot()` to Java or other threads: it returns `std::shared_ptr<const MapSnapshot>` holding, per map id, the epoch, the alive point ids and positions, the Frame ids and `T_cw`, plus the active map. The snapshot is immutable and safe to share.
3. **Still public:** `num_keyframes()` with no argument and the other atomic counters (including GlobalBA's run counters, which are now atomic).
4. **`MapFollower` is defined in the library**, so a JNI layer built at `-O0` does not slow tracking. (A header-only `MapFollower` compiled into an `-O0` consumer caused the binocular regression found during these gates: the linker took the consumer's `-O0` copies of library template code. The library build check `MapFollowerAddsNoLibraryInstantiations` guards this.) Caveat: any consumer built with less optimization than the package can still replace library inline/template functions with its own copies; this is an open item in docs/TODO.md (about 75 functions in the example apps), so build the binding with optimization or against a package built the same way.
5. **Not in this release:** the `voyahslam::Session` facade and the viewer on a follower are deferred (docs/TODO.md); until then the binding keeps its own session code and uses `MapFollower` directly.

## Self-Check

Merge commit f1ecb28 is an ancestor of master HEAD and equals origin/master at push time; the fix commits `dac50cf`, `202fe43`, `5b2bb46`, `5cd2077` are ancestors of master; gate numbers above are copied from the run 2 report. Result: PASSED.
