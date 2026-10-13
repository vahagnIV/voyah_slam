---
phase: 01-api-ownership-and-lifetime-contracts
plan: 01
subsystem: events
tags: [cpp20, shared_ptr, raii, subscription, lock-free, moodycamel, asan, tsan, gtest]

requires:
  - phase: none
    provides: first plan of Release R1 (audit #3 + #2)
provides:
  - "events own their memory: std::make_shared events, rvalue-only Observable::Notify, delivered as EventPtr = shared_ptr<const Event>"
  - "InboxCore: lock-free queue of EventPtr, priority counter, closed flag, owned WakeSignal, backlog warning (D-24)"
  - "Wiring + move-only [[nodiscard]] Subscription; UsageError on connect/reset while running, terminate on destroy while running"
  - "voyahslam::UsageError (first library exception policy, D-09)"
  - "destruction-order proof: 24 orders of {producer, Subscription, AsyncObserver, Listener} green under Debug, TSan and ASan"
affects: [01-02, 01-03, 01-04, 01-05, 01-06, voyah_slam, voyahviz, voyah_jni]

actuals:
  tokens: 23000   # chars/4 over the realized libs/ diff (93409 chars, additions + deletions)
  tasks: 2
  commits: 3
plan_head_before: f7b83cb7919e298feb08ebadb260ba377c424354
plan_head_after: 705ba37b254de92d3eca8876c1f2f47315dc05cd
commits: 3

tech-stack:
  added: []
  patterns:
    - "Self-owning events: make_shared + EventPtr (shared_ptr<const Event>), destroyed on the thread of the last drop"
    - "Shared blocks instead of raw pointers: InboxCore (observer side) and Wiring (producer side) are held by every party, so any destruction order is safe"
    - "RAII Subscription with a loud failure for the one unsafe order (destroy while running: stderr + log + terminate)"
    - "Concepts, not bare requires, for negative compile-time checks outside templates"

key-files:
  created:
    - libs/common/include/voyahslam/errors.h
    - libs/events/include/voyahslam/events/inbox_core.h
    - libs/events/include/voyahslam/events/subscription.h
    - libs/events/tests/events/probe_event.h
    - libs/events/tests/events/event_lifetime_test.cpp
    - libs/events/tests/events/subscription_test.cpp
  modified:
    - libs/events/include/voyahslam/events/event_types.h
    - libs/events/include/voyahslam/events/frame_tracked.h
    - libs/events/include/voyahslam/events/global_ba_finished.h
    - libs/events/include/voyahslam/events/initialization_measurement.h
    - libs/events/include/voyahslam/events/loop_detected.h
    - libs/events/include/voyahslam/events/map_transaction.h
    - libs/events/include/voyahslam/events/tracking_lost.h
    - libs/events/include/voyahslam/events/async_observer.h
    - libs/events/include/voyahslam/events/listener.h
    - libs/events/include/voyahslam/events/observable.h
    - libs/events/include/voyahslam/events/event_node.h
    - libs/events/tests/CMakeLists.txt
    - libs/events/tests/events/event_system_test.cpp
    - libs/events/tests/events/wake_signal_test.cpp

key-decisions:
  - "~Observable stores RunState::Stopped, so a Subscription that outlives a producer destroyed while still marked running (a TestNode never Stop()ped) is a no-op instead of a terminate"
  - "InboxCore warns through Logger::CreateLogger(\"AsyncObserver\") lazily, only on the warning path; the threshold (std::atomic<size_t> warn_at_) only grows"
  - "Negative compile-time checks use concepts (CanAssignPriority, CanNotify): a bare requires with an invalid requirement outside a template is a hard error, not false"
  - "AsyncObserver has no Push: tests push through Observable::Notify, so the delivery path under test is the production one"

patterns-established:
  - "Test helper TestNode : EventNode with Start()/Stop() = BeginRun()/EndRun() to drive D-05/D-07 without a real worker thread"
  - "ProbeEvent::alive live counter: every lifetime test asserts it returns to 0"
  - "Backlog-warning tests capture the log with a ringbuffer_sink_mt appended to the shared logger and removed at the end"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "A make_shared event published via Observable::Notify(shared_ptr<T>&&) is delivered as shared_ptr<const Event>; a received event is immutable by type and an lvalue Notify does not compile (D-01)"
    requirement: OWN-01
    verification:
      - kind: unit
        ref: "libs/events/tests/events/event_lifetime_test.cpp (static_asserts CanAssignPriority, CanNotify) + EventSystemTest.ObservableAsyncFanOut"
        status: pass
    human_judgment: false
  - id: D2
    description: "An event outlives its producer, Subscription, AsyncObserver and Listener, is destroyed exactly once and on the thread of the last drop; zero observers free it before Notify returns (D-01, D-03)"
    requirement: OWN-01
    verification:
      - kind: unit
        ref: "libs/events/tests/events/event_lifetime_test.cpp#EventLifetimeTest.*"
        status: pass
    human_judgment: false
  - id: D3
    description: "All 24 destruction orders of {producer, Subscription, AsyncObserver, Listener} are memory-safe and free every event once, under Debug, TSan (setarch -R, death tests excluded) and ASan (D-22)"
    requirement: OWN-02
    verification:
      - kind: unit
        ref: "libs/events/tests/events/subscription_test.cpp#AllOrders/DestructionOrderTest.AnyOrderIsSafe/*"
        status: pass
    human_judgment: false
  - id: D4
    description: "AddObserver/AddSyncObserver on a running node and Subscription::reset() while running throw UsageError; destroying a Subscription while the producer runs prints the reason and terminates; no-op cases (outlives producer, moved-from, second reset) hold (D-05, D-07, D-09)"
    requirement: OWN-02
    verification:
      - kind: unit
        ref: "libs/events/tests/events/subscription_test.cpp#SubscriptionTest.*, SubscriptionDeathTest.DestroyedWhileProducerRunsTerminates"
        status: pass
    human_judgment: false
  - id: D5
    description: "AsyncObserver destruction releases queued events; Subscription disconnect never drains an inbox; later Notify drops at once and a surviving Listener returns an empty EventPtr (D-08, D-22)"
    requirement: OWN-02
    verification:
      - kind: unit
        ref: "AsyncObserverTest.DestructorReleasesQueuedEvents, SubscriptionTest.DisconnectKeepsOtherProducersQueuedEvents, SubscriptionTest.ObserverDestroyedWhileProducerRunsIsSafe"
        status: pass
    human_judgment: false
  - id: D6
    description: "An inbox logs one 'consumer is behind' warning naming the observer at 1024 queued events and at each doubling, never per event (D-24)"
    requirement: OWN-02
    verification:
      - kind: unit
        ref: "libs/events/tests/events/subscription_test.cpp#AsyncObserverTest.BacklogWarningOncePerDoubling"
        status: pass
    human_judgment: false
  - id: D7
    description: "The inbox core, Wiring and Subscription contain no mutex or other lock on the event path (principle 1)"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "git grep -n 'std::mutex' -- libs/events/include (empty)"
        status: pass
    human_judgment: true
    rationale: "'No other lock' is a design judgment (plan marks it verification: backstop): the grep proves no std::mutex, a reviewer confirms by reading inbox_core.h, subscription.h and observable.h."

duration: 6min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 01: Self-owning events and Subscription tracer slice Summary

**voyah_events now delivers `std::make_shared` events as `shared_ptr<const Event>` through a lock-free shared `InboxCore` and RAII `Subscription`/`Wiring` blocks, with all 24 destruction orders proven green under Debug, ThreadSanitizer and AddressSanitizer.**

## Performance

- **Duration:** 6 min
- **Started:** 2026-10-13T13:49:44Z
- **Completed:** 2026-10-13T13:55:38Z (code; docs close-out follows)
- **Tasks:** 2 (1 tracer, 1 TDD)
- **Files modified:** 20 (6 created, 14 modified)

## Accomplishments

- Events own their memory: pools, `Recyclable`, `Reset()`, `recycle_`/`set_ref_count_` and `SetWakeSignal` are gone from `voyah_events`; a producer fills a `shared_ptr<T>` and hands it over with `Notify(std::move(p))`, an lvalue is a compile error, a received `EventPtr` cannot be written to.
- `InboxCore` (queue, priority counter, closed flag, owned `WakeSignal`) is shared by `AsyncObserver`, `Listener` and the producer's entries; `~AsyncObserver` closes it and releases the queue, later pushes drop. `Wiring` plus move-only `[[nodiscard]]` `Subscription` make a connection an RAII value; misuse while running throws `voyahslam::UsageError` or, from a destructor, logs and terminates.
- Sanitizer gate: `voyah_events_tests` (68 tests incl. 24 destruction orders and 1 death test) passes in the Debug tree, under TSan (`setarch -R`, `-*DeathTest*`, 67 tests, zero reports) and under ASan (68 tests, zero reports).
- D-24 backlog warning (once at 1024 queued events, then each doubling) implemented test-first.

## Task Commits

1. **Task 1: Tracer, one make_shared event from an EventNode through a Subscription, an AsyncObserver and a Listener, outliving its producer** - `afe4b70` (feat)
2. **Task 2: Destruction-order matrix, misuse errors, backlog warning, sanitizer gate** (TDD)
   - RED `225a73c` (test): `BacklogWarningOncePerDoubling` fails (`lines.size()` 0, expected 3)
   - GREEN `705ba37` (feat): `InboxCore::Push` backlog warning

**Plan metadata:** docs commit follows this summary (`docs(01-01): complete ...`).

## TDD Evidence (Task 2)

- **RED:** `voyah_events_tests --gtest_filter=...` (gtest XML), target `AsyncObserverTest.BacklogWarningOncePerDoubling` in `libs/events/tests/events/subscription_test.cpp`. `gsd_run check tdd-red-evidence` returned `RED_EVIDENCE_OK` (`target_test_failed`, 21 run, 20 pass, 1 fail). Semantic assessment: the target executed and failed on the planned assertion `ASSERT_EQ(lines.size(), 3u)` (observed 0), because `Push` had only the marked placeholder, not a build or fixture fault. The other 20 selected tests (misuse errors, drain-on-destroy, 23 further cases) passed already because Task 1 had delivered that behaviour; only the warning was new.
- **GREEN:** the same run, 68/68 pass.
- **REFACTOR:** none needed.

## Files Created/Modified

- `libs/common/include/voyahslam/errors.h` - `voyahslam::UsageError : std::logic_error`
- `libs/events/include/voyahslam/events/event_types.h` - `Event` without function pointers (protected non-virtual dtor), `EventPtr`
- `libs/events/include/voyahslam/events/{frame_tracked,global_ba_finished,initialization_measurement,loop_detected,map_transaction,tracking_lost}.h` - `Recyclable` base, `Reset()` and constructor lambdas removed
- `libs/events/include/voyahslam/events/inbox_core.h` - `InboxCore`, `kBacklogWarningFloor`
- `libs/events/include/voyahslam/events/{async_observer,listener}.h` - owner of / handle on the shared core
- `libs/events/include/voyahslam/events/subscription.h` - `RunState`, `IsRunning`, `Wiring`, `Subscription`
- `libs/events/include/voyahslam/events/{observable,event_node}.h` - Subscription-returning connect API, rvalue `Notify`, `BeginRun`/`EndRun`, `run_state()`
- `libs/events/tests/events/probe_event.h` - `ProbeEvent` (live counter), `TestNode`
- `libs/events/tests/events/event_lifetime_test.cpp`, `subscription_test.cpp` - new; `event_system_test.cpp`, `wake_signal_test.cpp` ported; `libs/events/tests/CMakeLists.txt` registers both

## Decisions Made

- `~Observable` stores `RunState::Stopped` so a Subscription outliving a producer that was destroyed without `Stop()` is a no-op, not a terminate (D-06 "outlives its producer is a no-op" made unconditional).
- The backlog logger is fetched lazily on the warning path only, so the hot `Push` path pays one atomic load and one `size_approx()`.
- Tests push through `Observable::Notify` because `AsyncObserver` deliberately has no `Push` (the contract), so the tested path is the production one.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] Plan's gtest filter does not select the 24 destruction-order instances**
- **Found during:** Task 2 (first RED run: "21 tests", not 45)
- **Issue:** the verify command filters `DestructionOrderTest/*`, but a parameterized instance is named `AllOrders/DestructionOrderTest.AnyOrderIsSafe/<order>`, so the 24 cases silently did not run (the `fails_when` of "fewer than 24 DestructionOrderTest result lines" would have fired).
- **Fix:** use `--gtest_filter='*DestructionOrderTest*:SubscriptionTest.*:SubscriptionDeathTest.*:AsyncObserverTest.*'`; the full-binary run (no filter) executes all 68 tests including the 24.
- **Files modified:** none (command only)
- **Verification:** `--gtest_filter='*DestructionOrderTest*'` lists 24 passing tests in Debug, TSan and ASan.

**2. [Rule 2 - Missing critical] `~Observable` marks the wiring Stopped**
- **Found during:** Task 1 (writing `Subscription` outliving-producer semantics)
- **Issue:** a producer destroyed while its state was still `Running` (e.g. a node never `Stop()`ped) would make every surviving Subscription terminate in its destructor, contradicting "a Subscription that outlives its producer is a no-op".
- **Fix:** `~Observable()` stores `Stopped` into the shared wiring.
- **Files modified:** `libs/events/include/voyahslam/events/observable.h`
- **Verification:** `SubscriptionTest.OutlivesProducer`, `OutlivesProducerThatWasStartedAndStopped`, the 24 destruction orders.
- **Committed in:** `afe4b70`

**3. [Rule 1 - Bug in plan sketch] Immutability check written as concepts**
- **Found during:** Task 1
- **Issue:** the RESEARCH sketch `static_assert(!requires(EventPtr e) { e->priority = true; })` is ill-formed outside a template (an invalid requirement is a hard error, not `false`).
- **Fix:** `template <typename TPtr> concept CanAssignPriority = requires(TPtr p) { p->priority = true; }; static_assert(!CanAssignPriority<EventPtr>);` and the same for `CanNotify` with lvalue/const-lvalue/raw-pointer arguments.
- **Files modified:** `libs/events/tests/events/event_lifetime_test.cpp`
- **Committed in:** `afe4b70`

---

**Total deviations:** 3 auto-fixed (1 blocking, 1 missing-critical, 1 bug in plan sketch)
**Impact on plan:** none on scope; all three keep the plan's stated behaviour and make its verification real.

## Issues Encountered

None beyond the deviations above.

## Known Stubs

None.

## Threat Flags

None. No new network, auth or file surface; the plan's threats T-01-01..T-01-03 are mitigated by `EventLifetimeTest`, the 24-order matrix and the rvalue-only `Notify`; T-01-04 is accepted (visibility only, D-24).

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- The `<interfaces>` contract exists exactly as named; plans 01-02..01-05 can migrate `voyah_slam`, `voyahviz`, the apps and the JNI-facing wiring against it.
- Expected and accepted: `voyah_slam`, `voyahviz` and the apps do not compile until plan 01-02/01-05 (they still use the pool, `Event*` and `SetWakeSignal`); only `--target voyah_events_tests` verifies until then.
- Docs owed to later plans: DESIGN §6/§6.1 and decisions log (events pools to `shared_ptr`, `Subscription`, `UsageError` policy), TODO items closed (`AsyncObserver::queue()` private), version bump 0.4.0 at the end of R1. Nothing was merged or pushed; work is on `refactor/event-ownership` (D-21).

## Self-Check: PASSED

Created files exist (`errors.h`, `inbox_core.h`, `subscription.h`, `probe_event.h`, `event_lifetime_test.cpp`, `subscription_test.cpp`); commits `afe4b70`, `225a73c`, `705ba37` are ancestors of HEAD; `check evaluation-scope --plan 01-01 --commits-only` resolves with the plan's commits; all acceptance criteria re-run (no `std::mutex`, no `recycle_`/`set_ref_count_`/`Recyclable`/`SetWakeSignal` under `libs/events`, `kBacklogWarningFloor = 1024`, branch `refactor/event-ownership`); `voyah_events_tests` 68/68 in Debug, 67/67 under TSan (death test excluded), 68/68 under ASan.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
