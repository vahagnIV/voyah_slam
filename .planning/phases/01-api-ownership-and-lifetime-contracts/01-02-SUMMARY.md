---
phase: 01-api-ownership-and-lifetime-contracts
plan: 02
subsystem: slam
tags: [cpp20, shared_ptr, subscription, raii, event-ownership, migration]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-01: EventPtr, rvalue Notify, Subscription, EventNode::BeginRun/EndRun, named AsyncObserver"
provides:
  - "voyah_slam, voyahviz and both example apps on make_shared publishing, EventPtr consumption and Subscription wiring"
  - "Tracker, Mapper, LoopCloser, GlobalBA drive RunState (BeginRun before the worker starts, EndRun after it joins)"
  - "event pools and drop limits deleted (D-02, D-25); Slam global_ba_ ordering invariant and the apps' StopAll guard deleted (D-04, D-23)"
  - "first executed check of the migrated production event flow: both apps run 40 corridor1 frames and exit 0"
affects: [01-03, 01-04, 01-05, 01-06, voyah_jni]

actuals:
  tokens: 12800   # chars/4 over the realized non-.planning diff (51333 chars)
  tasks: 3
  commits: 3
plan_head_before: c2f0ae5b1e99cceb74c178d8ed14cf0359a5b5b4
plan_head_after: e144baecbf7d853827a799d78f50fdbeeeea3753
commits: 3

tech-stack:
  added: []
  patterns:
    - "Create-fill-Notify: auto event = std::make_shared<T>(); fill; observable_.Notify(std::move(event)); nothing touches the event afterwards"
    - "Consume loops: while (events::EventPtr event = listener.GetNextEvent()) so each event is released at the end of its iteration"
    - "Subscription holder declared before the Slam (destroyed after it), Slam's own subscriptions_ declared last"

key-files:
  created: []
  modified:
    - libs/slam/include/voyahslam/tracking/tracker.h
    - libs/slam/src/tracking/tracker.cpp
    - libs/slam/include/voyahslam/mapping/mapper.h
    - libs/slam/src/mapping/mapper.cpp
    - libs/slam/src/mapping/mapper_global_ba.cpp
    - libs/slam/src/mapping/mapper_quality.cpp
    - libs/slam/src/mapping/mapper_loop.cpp
    - libs/slam/src/mapping/mapper_merge.cpp
    - libs/slam/include/voyahslam/loop_closing/loop_closer.h
    - libs/slam/include/voyahslam/loop_closing/common_region.h
    - libs/slam/src/loop_closing/loop_closer.cpp
    - libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h
    - libs/slam/include/voyahslam/global_ba/global_ba.h
    - libs/slam/src/global_ba/global_bundle_adjuster.cpp
    - libs/slam/include/voyahslam/slam/slam.h
    - libs/slam/src/slam/slam.cpp
    - vizualization/include/voyahviz/map_viewer.h
    - vizualization/src/map_viewer.cpp
    - vizualization/src/pangolin_renderer.cpp
    - examples/full_slam/full_slam_common.h
    - examples/full_slam/monocular_full_slam.cpp
    - examples/full_slam/binocular_full_slam.cpp

key-decisions:
  - "Mapper::CreateTransaction() returns shared_ptr<MapTransaction>; helpers keep taking MapTransaction&, so the seven publish sites changed only at creation and at Notify"
  - "Slam::subscriptions_ is the last member, so it is destroyed first, after ~Slam() has stopped every producer; the six internal edges are pushed in the constructor"
  - "The apps keep slam.Stop(); viewer.Stop(); trajectory->Poll() on the normal path because they produce output, not as a safety requirement"

patterns-established:
  - "BeginRun() is the first statement of Start() and EndRun() follows worker_.Stop() in both Stop() and the destructor"
  - "Component inboxes are named at EventNode construction (\"Tracker\", \"Mapper\", \"LoopCloser\", \"GlobalBA\") so a D-24 backlog warning names the stalled consumer"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "All production publish sites (Tracker 3, Mapper 7, LoopCloser 1, GlobalBA 1) create with make_shared and publish with Notify(std::move(...)); consume sites take an owning EventPtr and the Mapper batch releases each entry after applying it (D-01)"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "git grep -n 'Notify(std::move(' -- libs/slam/src: 12 sites (Tracker 3, Mapper 7, LoopCloser 1, GlobalBA 1); both apps run 40 corridor1 frames, exit 0"
        status: pass
    human_judgment: false
  - id: D2
    description: "Pools and drop limits are gone: event_pool_size, transaction_pool_size, the two pool-overflow accessors, max_events_in_flight (LoopCloser, GlobalBA), num_dropped_loops(), num_results_dropped() (D-02, D-25)"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "git grep -n -e max_events_in_flight -e num_dropped_loops -e num_results_dropped -e ObjectPool -e event_pool -e transaction_pool -e recycle_ -- libs/slam/include libs/slam/src (empty)"
        status: pass
    human_judgment: false
  - id: D3
    description: "The four components drive RunState (BeginRun before the worker starts, EndRun after it joins), Slam holds its six internal wires as a trailing std::vector<events::Subscription>, the global_ba_ ordering comment is gone, the apps hold their Subscriptions in a vector declared before slam and have no StopAll (D-04, D-05, D-07, D-23)"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "grep BeginRun() in tracker.cpp/mapper.cpp/loop_closer.cpp/global_bundle_adjuster.cpp; grep subscriptions_ in slam.h after global_ba_; git grep StopAll -- examples (empty); targets voyah_slam voyahviz monocular_full_slam binocular_full_slam build"
        status: pass
    human_judgment: false
  - id: D4
    description: "Unwinding edge: when an exception leaves the app's try block after Start(), ~Slam stops every producer before the subscription vector is destroyed (T-01-05)"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "structural: subscriptions declared immediately before slam, destroyed after it; the SlamTest unwind tests that assert this are scheduled in plan 01-05"
        status: pass
    human_judgment: true
    rationale: "No test of the exception path exists yet in this plan; the check is the declaration order read by a reviewer, executed coverage arrives with the SlamTest unwind tests in plan 01-05."

duration: 5min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 02: Production migration to self-owning events Summary

**voyah_slam, voyahviz and both example apps now publish `std::make_shared` events via `Notify(std::move(...))`, consume owning `EventPtr`, wire through `Subscription`s, and run 40 corridor1 frames end to end with all pools, drop limits and the `StopAll` guard deleted.**

## Performance

- **Duration:** 5 min
- **Started:** 2026-10-13T13:57:38Z
- **Completed:** 2026-10-13T14:02:30Z
- **Tasks:** 3
- **Files modified:** 22 (0 created)

## Accomplishments

- Tracker (3 sites), Mapper (7 sites), LoopCloser (1) and GlobalBA (1) create events with `std::make_shared` and hand them over with an rvalue `Notify`; no site touches an event after `Notify`. `Mapper::CreateTransaction()` returns a `shared_ptr`, `Mapper::batch_` is a `std::vector<events::EventPtr>` and each entry is `reset()` right after it is applied.
- `BeginRun()`/`EndRun()` bracket every worker thread, so `AddObserver` and `Subscription::reset()` throw while a component runs; every inbox is named.
- Pools and bounds removed: `TrackerOptions::event_pool_size`, `MapperOptions::transaction_pool_size`, both overflow accessors, `max_events_in_flight` in `LoopCloserOptions` and `GlobalBaOptions`, `num_dropped_loops()`, `num_results_dropped()`. `LoopDetected` and `GlobalBAFinished` are published unbounded; `max_query_backlog` stays.
- `Slam` keeps its six internal edges in `std::vector<events::Subscription> subscriptions_`, its last member; the `global_ba_` ordering comment is gone. Both apps hold their external Subscriptions in a vector declared immediately before `slam` and have no `StopAll`.
- Executed check: `monocular_full_slam` and `binocular_full_slam` on corridor1, `--rate=0 --max-frames=40`, both exit 0, print `tracker state:` and no `terminate called` (binocular reaches `Tracking`/`Mapping` with 3 Frames and 777 points; monocular is still `WaitingForMap` after 40 frames and reaches `Tracking`/`Mapping` with 16 Frames and 1728 points at `--max-frames=250`).

## Task Commits

1. **Task 1: Tracker and Mapper publish make_shared events, consume EventPtr, drive RunState, lose their pools** - `d768212` (feat)
2. **Task 2: LoopCloser and GlobalBA publish unbounded, Slam holds its wiring as Subscriptions** - `4834ff0` (feat)
3. **Task 3: MapViewer and the example apps use Subscriptions; StopAll and the declaration-order rules are gone** - `e144bae` (feat)

**Plan metadata:** docs commit follows this summary (`docs(01-02): complete ...`).

## Files Created/Modified

- `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp` - pool/option/accessor removed, `EventNode("Tracker")`, make_shared publishing, `BeginRun`/`EndRun`, `EventPtr` consumption
- `libs/slam/include/voyahslam/mapping/mapper.h`, `libs/slam/src/mapping/mapper*.cpp` - `CreateTransaction()` returns `shared_ptr`, `batch_` of `EventPtr`, seven `Notify(std::move(...))` sites, `RunState` driving
- `libs/slam/{include,src}/.../loop_closer.*`, `common_region.h` - unbounded `LoopDetected`, no drop counter, `max_events_in_flight` deleted
- `libs/slam/{include,src}/.../global_bundle_adjuster.*`, `global_ba.h` - unbounded `GlobalBAFinished`, no drop counter, `max_events_in_flight` deleted
- `libs/slam/include/voyahslam/slam/slam.h`, `libs/slam/src/slam/slam.cpp` - trailing `subscriptions_`, class comment rewritten
- `vizualization/include/voyahviz/map_viewer.h`, `vizualization/src/map_viewer.cpp`, `vizualization/src/pangolin_renderer.cpp` - named inbox, `EventPtr` drain, comments without pool language
- `examples/full_slam/{full_slam_common.h,monocular_full_slam.cpp,binocular_full_slam.cpp}` - `TrajectoryWriter` on `EventPtr`, `subscriptions` vector before `slam`, `StopAll` deleted

## Decisions Made

- `CreateTransaction()` returns `std::shared_ptr` and the fill code keeps working through `*created` / `created->`, so only the creation line and the `Notify` line changed at each site.
- The apps keep `slam.Stop(); viewer.Stop(); trajectory->Poll();` on the normal path, commented as the step that produces output; the exception path is covered by `~Slam` stopping first.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] Per-object Make targets are named `.o`, not `.cpp.o`**
- **Found during:** Task 1 verification
- **Issue:** the plan's `make -C .../libs/slam src/tracking/tracker.cpp.o ...` is silently a no-op here (`make help` lists `src/tracking/tracker.o`), so the verify would pass without compiling anything.
- **Fix:** used `src/<path>.o` targets; confirmed from the object timestamps that all six objects were compiled.
- **Files modified:** none (command only)
- **Verification:** all six `*.cpp.o` files under `CMakeFiles/voyah_slam.dir` rebuilt, no `error:`.

**2. [Rule 1 - Stale wording] Pool language in comments outside the listed files**
- **Found during:** Tasks 2 and 3 (acceptance greps)
- **Issue:** `common_region.h` ("Mapper's pooled events"), `loop_closer.h`, `global_bundle_adjuster.h` ("transaction pool aborts when exhausted") and `pangolin_renderer.cpp` ("producers' event pools") described the deleted pools.
- **Fix:** rewrote the comments; `pangolin_renderer.cpp` is outside the plan's `files_modified` list but is a comment-only change in `voyahviz`.
- **Files modified:** `common_region.h`, `loop_closer.h`, `global_bundle_adjuster.h`, `vizualization/src/pangolin_renderer.cpp`
- **Verification:** `git grep -n pool -- libs/slam/include libs/slam/src vizualization/src vizualization/include examples` shows nothing production-side.
- **Committed in:** `4834ff0`, `e144bae`

---

**Total deviations:** 2 auto-fixed (1 blocking verify-command name, 1 stale comments)
**Impact on plan:** none on scope or behaviour.

## Issues Encountered

None. The monocular 40-frame smoke ends in `WaitingForMap` (the initializer has not triggered yet, 40 `InitializationMeasurement` events processed by the viewer); this is expected for the mono pipeline and a 250-frame run reaches `Tracking`/`Mapping`.

## Known Stubs

None.

## Threat Flags

None. No new network, auth or file surface. T-01-05 is mitigated structurally (`subscriptions_` last in `Slam`, apps' vector before `slam`, `StopAll` removed because `~Slam` stops first) with executed coverage arriving in plan 01-05; T-01-07 is mitigated by the rvalue-only `Notify` at all 12 sites; T-01-06 is accepted (D-25).

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plans 01-03..01-05 can migrate the `voyah_slam` and `voyahviz` test sources (`tracker_test`, `mapper_test`, `loop_closer_test`, `global_ba_test`, `slam_test`, `map_viewer_test`, which still use the pool API, `Event*` and the dropped accessors); until then those test targets stay red, and `voyah_events_tests` is the only test target that builds.
- Docs owed to later plans: DESIGN §6/§6.1 and the decisions log (pools to `shared_ptr`, `Subscription`, unbounded `LoopDetected`/`GlobalBAFinished`), the TODO entries for the removed options, and the version bump at the end of R1. Nothing was merged or pushed; work stays on `refactor/event-ownership`.

## Self-Check: PASSED

Commits `d768212`, `4834ff0`, `e144bae` are ancestors of HEAD; the 22 changed files exist; acceptance criteria re-run (pool/drop-limit/`recycle_`/`StopAll` greps empty, 12 `Notify(std::move(` sites, one `BeginRun()` per component file, `subscriptions_` declared after `global_ba_`, no `recycles` in `slam.h`); `voyah_slam`, `voyahviz`, `monocular_full_slam` and `binocular_full_slam` build; both apps run 40 corridor1 frames, exit 0, `tracker state:` printed, no `terminate called`.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
