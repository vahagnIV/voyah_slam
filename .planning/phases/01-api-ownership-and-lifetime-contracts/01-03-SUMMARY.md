---
phase: 01-api-ownership-and-lifetime-contracts
plan: 03
subsystem: slam-tests
tags: [cpp20, gtest, shared_ptr, subscription, event-ownership, migration]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-01: EventPtr, rvalue Notify, Subscription; plan 01-02: production code on make_shared events, Mapper::CreateTransaction() returning shared_ptr"
provides:
  - "Tracker fixture and tracking tests (tracker_test, local_map_test, relocalization_test, integration/tracker_test) on make_shared events, EventPtr and held Subscriptions"
  - "mapper_test.cpp (MapperTestAccess and every Mapper fixture) on the same API"
affects: [01-04, 01-05]

actuals:
  tokens: 10900   # chars/4 over the realized non-.planning diff (43675 chars)
  tasks: 2
  commits: 2
plan_head_before: bf5a1dd26d32f32ac23552c78bf1b9341289781d
plan_head_after: 038f71b73d6dde8366c2c5ea37d71b7e49db305d
commits: 2

tech-stack:
  added: []
  patterns:
    - "Fixture wiring: std::vector<events::Subscription> subscriptions_ declared before the components it wires; every AddObserver/ConnectAsync/AddSyncObserver result is pushed into it"
    - "Sync observer whose ctx is a stack object (Probe): the Subscription is a local declared after the Probe, so it disconnects before the Probe dies"
    - "Test-created events: auto e = std::make_shared<T>(); fill; observable.Notify(std::move(e));"

key-files:
  created: []
  modified:
    - libs/slam/tests/tracking/tracker_test_fixture.h
    - libs/slam/tests/tracking/tracker_test.cpp
    - libs/slam/tests/tracking/local_map_test.cpp
    - libs/slam/tests/tracking/relocalization_test.cpp
    - libs/slam/tests/mapping/mapper_test.cpp

key-decisions:
  - "Probe-style sync observers (ctx on the test's stack) keep their Subscription as a local declared after the Probe, not in the fixture vector, so the connection never outlives its ctx"
  - "MapperTest.PooledMapTransactionIsReset deleted: it tested only the removed ObjectPool slot reuse and reset"

patterns-established:
  - "TrackerTest::Connect(events::Subscription) for tests that wire extra observers"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "Tracker and Mapper test sources create events with std::make_shared, publish with Notify(std::move(...)), read received events through const references and hold every Subscription (D-01, D-06)"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "git grep -n -i -e ObjectPool -e Recycle -e 'Event\\*' -e '.Create()' -e pool over the five migrated files (empty); per-object builds of tracker_test, local_map_test, relocalization_test, mapper_test and integration/tracker_test compile with no error"
        status: pass
    human_judgment: false
  - id: D2
    description: "Fixtures declare their std::vector<events::Subscription> before the components they wire (D-07, D-23) and TearDown still stops the Tracker"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "grep -n 'subscriptions_\\|TMapper mapper_' tracker_test_fixture.h: member at line 316 above mapper_; MapperTest::subscriptions_ declared before tracker_ and mapper_"
        status: pass
    human_judgment: false
  - id: D3
    description: "No assertion removed, added or changed in kind by the migration (T-01-08)"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "plan assertion-multiset check against the branch base: empty for the fixture, tracker_test, local_map_test, relocalization_test, integration/tracker_test; for mapper_test.cpp empty only with the one test PooledMapTransactionIsReset excluded (see Deviations)"
        status: pass
    human_judgment: false
  - id: D4
    description: "The migrated tests run green"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "not executed: voyah_slam_tests links only after plans 01-04 and 01-05 migrate the remaining test sources; first executed run is plan 01-05 (check_all)"
        status: unknown
    human_judgment: true
    rationale: "Per-object compilation proves the API fit only; runtime behaviour (ordering, Subscription lifetime in the fixtures) is exercised once the binary links in plan 01-05."

duration: 7min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 03: Tracker and Mapper tests on self-owning events Summary

**The Tracker fixture, the three tracking tests, and the 5000-line `mapper_test.cpp` now build events with `std::make_shared`, publish with `Notify(std::move(...))`, read received events through `const` references and keep every connection in a `std::vector<events::Subscription>` declared before the components, with the EXPECT_/ASSERT_ multiset equal to the branch base apart from one pool-only test.**

## Performance

- **Duration:** 7 min
- **Started:** 2026-10-13T14:01:30Z
- **Completed:** 2026-10-13T14:09:00Z
- **Tasks:** 2
- **Files modified:** 5 (0 created)

## Accomplishments

- `TrackerTest` fixture: `transaction_pool_` and the `object_pool.h` include gone; `subscriptions_` is declared above `mapper_`, the four constructor wires are pushed into it, a `Connect(events::Subscription)` helper is added, the capture loop reads `events::EventPtr` with `static_cast<const T&>(*event)`, and the pool-return comments and justification are removed. `TearDown` keeps `tracker_.Stop()` and `Drain()` (tests call `Drain()` for their own assertions).
- `tracker_test.cpp` (7 transaction sites, 2 sync observers, 1 cast), `local_map_test.cpp` (2 pools) and `relocalization_test.cpp` (1 site): `make_shared` transactions, `Notify(std::move(...))`, `const events::Event&` handlers. The two `Probe` sync observers hold their `Subscription` as a local declared after the `Probe`.
- `mapper_test.cpp`: `MapperTestAccess::PublishLoopEdge` / `CullFrame` publish the `shared_ptr` transaction, `LoopConnections` keeps a local scratch `shared_ptr`; the five pools (`initialization_`, `tracking_lost_`, `frame_tracked_`, `loop_`, `global_ba_`) and their 22 create-fill-notify sites become `make_shared` + `Notify(std::move(...))`; `MapperTest::subscriptions_` is the first fixture member and holds the shared wires plus the per-test `urgent_` and `global_ba_` connections; the three sync handlers take `const events::Event&`.
- Per-object build (Debug, `-O2 -g`): `tracker_test`, `local_map_test`, `relocalization_test`, `mapper_test` (`voyah_slam_tests`) and `integration/tracker_test` (`voyah_integration_tests`) compile without a diagnostic.

## Task Commits

1. **Task 1: Tracker fixture and tracking tests on EventPtr and held Subscriptions** - `fa359de` (test)
2. **Task 2: Mapper tests and MapperTestAccess on shared_ptr transactions** - `038f71b` (test)

**Plan metadata:** docs commit follows this summary (`docs(01-03): complete ...`).

## Files Created/Modified

- `libs/slam/tests/tracking/tracker_test_fixture.h` - `subscriptions_` before `mapper_`, `Connect()` helper, `EventPtr` capture loop, pool removed
- `libs/slam/tests/tracking/tracker_test.cpp`, `local_map_test.cpp`, `relocalization_test.cpp` - `make_shared` transactions, rvalue `Notify`, `const Event&` sync handlers, pool includes removed
- `libs/slam/tests/mapping/mapper_test.cpp` - `MapperTestAccess`, `MapperTest` and its subclasses on the new API; `PooledMapTransactionIsReset` removed

## Decisions Made

- A sync observer whose `ctx` lives on the test's stack gets a local `Subscription` declared after that stack object, so disconnecting happens before the `ctx` is destroyed (the `ctx` rule of D-22); handlers whose `ctx` is the fixture (`this`) are kept in `subscriptions_`.
- `static_cast<const T&>(*event)` is used in the fixture and tracker tests; `TakeCaptured` in `mapper_test.cpp` keeps a `const T*` pointer from `event.get()` where the code already read through `transaction->` (a type-only change, no logic change).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] `MapperTest.PooledMapTransactionIsReset` could not be migrated and was deleted**
- **Found during:** Task 2
- **Issue:** the plan states the assertion multiset must equal the branch base with `X=NONE` and that pool-behaviour tests "were handled in plan 01-01", but this test was not among the ones 01-01 removed. It creates a `MapTransaction` from an `ObjectPool`, `Recycle()`s it, and asserts that the same slot comes back reset (3 `EXPECT_*` + 1 `ASSERT_EQ`). It tests only the removed pool mechanics: `MapTransaction` is no longer recyclable, and `ObjectPool` has its own tests in `libs/common/tests/memory/object_pool_test.cpp`. It cannot compile and has no equivalent on `make_shared` events (a fresh object is default-constructed).
- **Fix:** deleted the test. The assertion check was run with `X=PooledMapTransactionIsReset` (the mechanism the plan provides for a named allowed difference): empty output. With `X=NONE` it reports exactly this file as changed, which is this test.
- **Files modified:** `libs/slam/tests/mapping/mapper_test.cpp`
- **Verification:** assertion-multiset command empty with the one allowed name; `git grep -i pool` over the file is empty.
- **Committed in:** `038f71b`

**2. [Rule 3 - Blocking] The plan's per-object verify does not rebuild `integration/tracker_test.cpp`**
- **Found during:** Task 1 verification
- **Issue:** `make -C .../libs/slam/tests ... integration/tracker_test.o` is a silent no-op: that file belongs to the `voyah_integration_tests` target, whose objects are built through its own `build.make`, and the object's dependency list did not trigger a rebuild after the fixture header changed.
- **Fix:** touched `integration/tracker_test.cpp` (mtime only, no content change) and built `libs/slam/tests/CMakeFiles/voyah_integration_tests.dir/integration/tracker_test.cpp.o` through `make -f libs/slam/tests/CMakeFiles/voyah_integration_tests.dir/build.make` from `cmake-build-debug/build/Debug`; the object was rebuilt with no diagnostic.
- **Files modified:** none (command only)
- **Verification:** object timestamp newer than the fixture edit.

---

**Total deviations:** 2 auto-fixed (both Rule 3: one pool-only test, one verify command)
**Impact on plan:** none on scope; one fewer test (it checked a mechanism that no longer exists).

## Issues Encountered

None.

## Known Stubs

None.

## Threat Flags

None. No new network, auth or file surface. T-01-08 (silently weakened assertions) is mitigated by the multiset check against the branch base on all five files, with the single named exception above.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plans 01-04 and 01-05 migrate the remaining `voyah_slam_tests` and `voyah_integration_tests` sources (`loop_closer_test`, `global_ba_test`, `slam_test`, `map_viewer_test`, `map_replica_test`, `atlas_replica_test`, ...); until then neither binary links and nothing in this plan has run.
- `libs/slam/tests/tracking/map_replica_test.cpp` and `atlas_replica_test.cpp` still use `ObjectPool` and `Recycle()` and are not in this plan's file list.
- The runtime checks that matter for this plan (fixture destruction order, `Probe` subscription lifetime, urgent and global-BA sync handlers) run first at plan 01-05's `check_all`.

## Self-Check: PASSED

Commits `fa359de` and `038f71b` are ancestors of HEAD; the five modified files exist; the pool/recycle/`Event*`/`.Create()` greps are empty on the five migrated files; `subscriptions_` is declared above `TMapper mapper_;` in the fixture; the per-object builds compile; the assertion check is empty (with the one allowed name for `mapper_test.cpp`).

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
