---
phase: 01-api-ownership-and-lifetime-contracts
plan: 04
subsystem: slam-tests
tags: [cpp20, gtest, shared_ptr, subscription, event-ownership, migration, voyahviz]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-01: EventPtr, rvalue Notify, Subscription; plan 01-02: production code on make_shared events, LoopCloser/GlobalBA publish unbounded; plan 01-03: Tracker and Mapper tests on the same pattern"
provides:
  - "loop_closer_test, global_ba_test, global_bundle_adjuster_test, projection_matcher_test, atlas_replica_test, map_replica_test compile against the new event API"
  - "LoopCloserTest.DeliversLoopWhileEarlierLoopIsHeld and GlobalBundleAdjusterTest.DeliversResultWhileEarlierResultIsHeld replace the two drop-limit tests (D-25)"
  - "voyahviz_tests (SceneState, MapViewer, 46 tests) green on the new event layer, including viewer restart-after-Stop (D-26)"
affects: [01-05]

actuals:
  tokens: 12066   # chars/4 over the realized non-.planning diff (48263 chars)
  tasks: 2
  commits: 2
plan_head_before: d9dc1dc15f80a43cf782234db8213449db0b6a7b
plan_head_after: 5d0c0bbd7b46083f637f94f51527f1b865a163a0
commits: 2

tech-stack:
  added: []
  patterns:
    - "Harness in-flight limit without a pool: pool_size kept as a constructor parameter, every sent transaction remembered by weak_ptr, Send() refuses and counts in num_exhausted() while pool_size of them are alive"
    - "Harness 'hold': Run() moves published events into a std::vector<EventPtr> (the consumer keeps them, unacknowledged); Take() drains held events first"
    - "Release check on a published event through std::weak_ptr (Released(weak) == expired()) instead of a free pool slot"

key-files:
  created: []
  modified:
    - libs/slam/tests/loop_closing/loop_closer_test.cpp
    - libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp
    - libs/slam/tests/global_ba/global_ba_test.cpp
    - libs/slam/tests/features/projection_matcher_test.cpp
    - libs/slam/tests/tracking/atlas_replica_test.cpp
    - libs/slam/tests/tracking/map_replica_test.cpp
    - vizualization/tests/scene_state_test.cpp
    - vizualization/tests/map_viewer_test.cpp

key-decisions:
  - "Held events in the LoopCloser and GlobalBA harnesses are kept as EventPtr in the harness (held_), not left in the observer's queue: holding by the consumer is now legal and is what the delivery tests exercise"
  - "All transactions the LoopCloser harness creates (not only Send()) are tracked for the in-flight count, as the pool counted every Create(); only Send()/the GlobalBA Send refuse"

patterns-established:
  - "Test helper Publish(producer) returns a weak_ptr to the event it published; Released(weak) replaces SlotReturned(pool)"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "A LoopDetected is still published, and all of them reach the consumer, while the consumer holds an earlier one (D-25); no test expects a drop"
    requirement: OWN-01
    verification:
      - kind: unit
        ref: "libs/slam/tests/loop_closing/loop_closer_test.cpp#LoopCloserTest.DeliversLoopWhileEarlierLoopIsHeld (run in a throwaway binary linked from the six migrated objects: 89/89 pass)"
        status: pass
    human_judgment: false
  - id: D2
    description: "Two corrections while the first GlobalBAFinished is held yield two published results, epochs 1 and 2 in order, none dropped (D-25)"
    requirement: OWN-01
    verification:
      - kind: unit
        ref: "libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp#GlobalBundleAdjusterTest.DeliversResultWhileEarlierResultIsHeld (same throwaway binary)"
        status: pass
    human_judgment: false
  - id: D3
    description: "The six slam test sources compile against the new API with every other assertion unchanged; pool-semantics tests keep their assertions (RunReleasesTransactionsAtEachPoll, SlowDetectorHoldsNoTransaction)"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "per-object builds of the six objects (no diagnostics); plan assertion-multiset check empty for all six; git grep for ObjectPool/Recycle/max_events_in_flight/num_dropped_loops/num_results_dropped/Event* empty"
        status: pass
    human_judgment: false
  - id: D4
    description: "voyahviz_tests builds and passes in the Debug tree, including MapViewer restart-after-Stop tests (D-26)"
    requirement: OWN-02
    verification:
      - kind: unit
        ref: "cmake-build-debug/build/Debug/vizualization/tests/voyahviz_tests: 46/46 pass; MapViewerTest.* repeated 30 times with no failure"
        status: pass
    human_judgment: false
  - id: D5
    description: "The full voyah_slam_tests binary passes"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "not executed: voyah_slam_tests links only after plan 01-05 migrates slam_test.cpp; the six objects of this plan were linked into a scratch binary instead (89 tests pass)"
        status: unknown
    human_judgment: true
    rationale: "The scratch binary excludes the rest of voyah_slam_tests; first full run is plan 01-05 (check_all)."

duration: 5min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 04: LoopCloser, GlobalBA, replica, matcher and voyahviz tests on self-owning events Summary

**Six slam test sources and both `voyahviz_tests` files are on `std::make_shared` events and held Subscriptions; the two drop-limit tests became `DeliversLoopWhileEarlierLoopIsHeld` and `DeliversResultWhileEarlierResultIsHeld`, and `voyahviz_tests` (46 tests) is the first fully green consumer binary of the new layer.**

## Performance

- **Duration:** 5 min
- **Started:** 2026-10-13T14:09:30Z
- **Completed:** 2026-10-13T14:15:00Z
- **Tasks:** 2
- **Files modified:** 8 (0 created)

## Accomplishments

- `loop_closer_test.cpp` and `global_bundle_adjuster_test.cpp`: harnesses keep `pool_size` as an in-flight limit without a pool (each sent transaction remembered by `weak_ptr`; `Send()` refuses and counts in `num_exhausted()` while `pool_size` of them are alive), keep their Subscriptions in a `std::vector<events::Subscription>` declared first, hold published events as `EventPtr`, and read received events through `const events::Event&`. Option fields and drop counters are gone from the tests.
- The two drop tests were replaced as specified; their header comments say why (never dropped since 2026-10-13, D-25). This makes the Phase 3 criterion "a dropped GlobalBAFinished is retried" moot.
- `global_ba_test.cpp`, `projection_matcher_test.cpp` (including `BuildSyntheticMap`, which now holds its two Subscriptions and uses `Notify(std::move(...))` and an `EventPtr` listener loop), `atlas_replica_test.cpp`, `map_replica_test.cpp`: pool-per-call becomes `std::make_shared`, `Recycle()` removed.
- `scene_state_test.cpp`: 16 pool declarations and 10 creates removed (`ApplyOne<TEvent>(scene)`); `map_viewer_test.cpp`: `Publish` returns a `weak_ptr`, `Released(weak)` replaces `SlotReturned(pool)` in all eight checks, every `ConnectAsync` result held in a vector declared before viewer and producer.
- Verified at runtime beyond the plan: the six slam objects linked into a scratch binary run 89/89, including both delivery tests and the two pool-semantics tests.

## Task Commits

1. **Task 1: LoopCloser, GlobalBA, replica and matcher tests; drop tests become delivery tests** - `6734eed` (test)
2. **Task 2: voyahviz tests on the new API, voyahviz_tests green** - `5d0c0bb` (test)

**Plan metadata:** docs commit follows this summary (`docs(01-04): complete ...`).

## Files Created/Modified

- `libs/slam/tests/loop_closing/loop_closer_test.cpp` - harness on `weak_ptr` in-flight tracking, `held_` events, Subscriptions; `DeliversLoopWhileEarlierLoopIsHeld`
- `libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp` - same harness changes; `DeliversResultWhileEarlierResultIsHeld`
- `libs/slam/tests/global_ba/global_ba_test.cpp`, `features/projection_matcher_test.cpp`, `tracking/atlas_replica_test.cpp`, `tracking/map_replica_test.cpp` - `make_shared` transactions
- `vizualization/tests/scene_state_test.cpp`, `map_viewer_test.cpp` - pool-free, `weak_ptr` release checks, held Subscriptions

## Decisions Made

- Harness hold keeps `EventPtr`s in the harness rather than leaving events in the observer's queue, so the consumer genuinely holds the first event while later ones are published (the behaviour D-25 allows).
- In the LoopCloser harness every created transaction (cull, epoch, removal, empty) counts as alive against `pool_size`, as every pool `Create()` did; only `Send()` refuses.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] The plan's per-object verify command names targets that do not exist**
- **Found during:** Task 1 verification
- **Issue:** `make -C .../libs/slam/tests loop_closing/loop_closer_test.cpp.o ...` is not a valid target path; as in plans 01-02 and 01-03 the objects belong to `voyah_slam_tests`' own `build.make`.
- **Fix:** built the six objects with `make -f libs/slam/tests/CMakeFiles/voyah_slam_tests.dir/build.make libs/slam/tests/CMakeFiles/voyah_slam_tests.dir/<path>.o` from `cmake-build-debug/build/Debug`: no diagnostic.
- **Files modified:** none (command only)

**2. [Rule 3 - Blocking] The two `ASSERT_NE(taken.back(), nullptr)` slot loops cannot keep their expression**
- **Found during:** Task 2 (`ReceivesFromTwoProducers`, `StopEndsRunOnCurrentThread`)
- **Issue:** the plan names only the eight `SlotReturned` checks as re-expressed; these two loops asserted that every pool slot came back, and the assertion-multiset check requires `ASSERT_NE` to stay `ASSERT_NE`.
- **Fix:** each loop now runs over the `weak_ptr`s of the published events with `ASSERT_NE(Released(published[i]), false) << "event " << i << " not released"`, the same macro and meaning (every event released).
- **Files modified:** `vizualization/tests/map_viewer_test.cpp`
- **Verification:** assertion-multiset check empty for both viz files.
- **Committed in:** `5d0c0bb`

---

**Total deviations:** 2 auto-fixed (both Rule 3: one verify command, one assertion re-expression keeping its macro)
**Impact on plan:** none on scope; no assertion removed or weakened.

## Issues Encountered

None. Note for plan 01-05: a scratch link of the six objects (recipe: `link.txt` of `voyah_slam_tests` with the object list replaced) is a cheap way to run the migrated files before `slam_test.cpp` compiles.

## Known Stubs

None.

## Threat Flags

None. T-01-09 (held LoopDetected / GlobalBAFinished accumulate) is accepted as planned; the new delivery tests show results arrive while an earlier one is held.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Every test source except `slam_test.cpp` (plan 01-05) is on the new API; `voyah_slam_tests` and `voyah_integration_tests` link once `slam_test.cpp` is migrated, then `check_all` is the first full run.
- Docs owed to later plans unchanged (DESIGN, TODO, version bump at the end of R1).

## Self-Check: PASSED

Modified files exist; commits `6734eed` and `5d0c0bb` are ancestors of HEAD; acceptance criteria re-run: no `ObjectPool`/`Recycle`/`max_events_in_flight`/`num_dropped_loops`/`num_results_dropped`/`Event*` under the six slam test paths or `vizualization/tests`, both delivery tests listed, `num_exhausted` counters and assertions still present, assertion-multiset check empty for all eight files (with the two named replacements excluded where the plan allows), six objects compile, `voyahviz_tests` 46/46.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
