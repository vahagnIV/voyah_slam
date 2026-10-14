---
gsd_state_version: "1.0"
current_phase: 01
current_phase_name: API Ownership and Lifetime Contracts
status: executing
stopped_at: Completed 01-18-PLAN.md
last_updated: "2026-10-14T21:24:37.404Z"
last_activity: 2026-10-13
last_activity_desc: Phase 01 execution started
state_head: 93c00ce44ce5fdf7b56ad57e3bee8bd25f334c66
progress:
  total_phases: 7
  completed_phases: 0
  total_plans: 20
  completed_plans: 18
  percent: 0
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-10-12)

**Core value:** ORB-SLAM3-class accuracy on real sequences (TUM-VI corridor, mono and stereo) delivered through a lock-free, replica-per-thread, event-only architecture.
**Current focus:** Phase 01 — API Ownership and Lifetime Contracts

## Current Position

Phase: 01 (API Ownership and Lifetime Contracts) — EXECUTING
Plan: 19 of 20
Status: Ready to execute
Last activity: 2026-10-13 — Phase 01 execution started

Progress: [░░░░░░░░░░] 0%

## Performance Metrics

**Velocity:**
- Total plans completed: 0
- Average duration: -
- Total execution time: 0.0 hours

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

**Recent Trend:**
- Last 5 plans: -
- Trend: -

*Updated after each plan completion*
**Per-Plan Metrics:**

| Plan | Duration | Tasks | Files |
|------|----------|-------|-------|
| Phase 01 P01 | 6 min | 2 tasks | 20 files |
| Phase 01 P02 | 5 min | 3 tasks | 22 files |
| Phase 01 P03 | 7 min | 2 tasks | 5 files |
| Phase 01 P04 | 5 min | 2 tasks | 8 files |
| Phase 01 P05 | 13 min | 2 tasks | 10 files |
| Phase 01 P06 | 10 min | 2 tasks | 7 files |
| Phase 01 P07 | ~30min | 3 tasks | 0 files |
| Phase 01 P08 | 40 min | 2 tasks | 8 files |
| Phase 01 P09 | 15 min | 2 tasks | 13 files |
| Phase 01 P10 | 14 min | 2 tasks | 12 files |
| Phase 01 P11 | gates x2 + regression fix | 3 tasks | 0 files |
| Phase 01 P12 | 80 min | 2 tasks | 20 files |
| Phase 01 P13 | 25 min | 2 tasks | 8 files |
| Phase 01 P14 | gates x2 + option (a) change + merge | 3 tasks | 14 files |
| Phase 01 P15 | 35 min | 3 tasks | 15 files |
| Phase 01 P16 | gates + merge | 3 tasks | 0 files |
| Phase 01 P17 | 15 min | 3 tasks | 11 files |
| Phase 01 P18 | gates + merge | 3 tasks | 0 files |

## Accumulated Context

### Decisions

Locked decisions are in PROJECT.md `<decisions>`; the full set is `docs/DESIGN.md` section 8 (214 entries in `.planning/intel/decisions.md`).
Recent decisions affecting current work:

- [2026-10-10]: Loop closing is opt-in and off by default (user decision); do not flip without explicit approval.
- [2026-10-11]: Drift-consistency check for loops requested by the user; whether it replaces or complements a drift-bound gate, and whether verify-before-publish is added, is OPEN.
- [2026-10-12]: Object pools grow on overflow (0.3.0); `MapTransaction` must never be dropped.
- [2026-10-12]: The voyah_jni ownership audit is Phase 1 (user decision); loop-closing look-alike rejection moved to Phase 2, later phases renumbered.
- [Roadmap]: Step 6 split: look-alike rejection is Phase 2, remaining TUM-VI tuning is Phase 7 (last).
- [Phase 01]: 01-01: ~Observable stores RunState::Stopped so a Subscription outliving a producer is always a no-op
- [Phase 01]: 01-01: tests push through Observable::Notify (AsyncObserver has no Push); negative compile checks use concepts, not bare requires
- [Phase 01]: Slam::subscriptions_ is the last member and the apps' Subscription vector is declared before slam; ~Slam stops every producer first, so StopAll is not needed (D-04, D-23)
- [Phase 01]: Mapper::CreateTransaction() returns shared_ptr<MapTransaction>; LoopDetected and GlobalBAFinished are published unbounded, max_events_in_flight removed (D-02, D-25)
- [Phase 01]: 01-03: Probe-style sync observers (ctx on the test stack) keep their Subscription as a local declared after the Probe; fixture-ctx handlers go into the fixture subscriptions_ vector
- [Phase 01]: 01-03: MapperTest.PooledMapTransactionIsReset deleted; it tested only the removed ObjectPool slot reuse and reset (ObjectPool keeps its own tests)
- [Phase 01]: Plan 01-04: LoopCloser/GlobalBA test harnesses keep pool_size as a weak_ptr-based in-flight limit and hold published events as EventPtr; MapViewer tests check release via weak_ptr
- [Phase 01]: ProtocolRecorder owns its Subscriptions; Slam-level tests declare the Subscription holder before the Slam (D-23) — Unwinding destroys the holder after ~Slam has stopped every producer; executed by SlamTest.UnwindPastSubscriptionHolderDoesNotTerminate
- [Phase 01]: -Werror=unused-result on every test binary and both example apps (GNU/Clang) — A discarded [[nodiscard]] Subscription disconnects at once; error in tests and apps, package build unaffected
- [Phase 01]: Package version 0.4.0 (SOVERSION 0.4); DESIGN 6.1 rewritten as the shared-ownership lifecycle, decisions-log rows R1 implemented and Package version 0.4.0 — Every audit item is an API break for the voyah_jni binding; DESIGN must describe the code as merged (CLAUDE.md)
- [Phase 01]: R1 (0.4.0) merged to master as 33a270f on user's 'merge and push'; origin/master == master
- [Phase 01]: MapFollower: Update()/atlas()/Snapshot() thread-affine (debug assert from first Update()); Snapshot is immutable shared_ptr<const MapSnapshot>; Update counts only admitted transactions
- [Phase 01]: 01-09: Tracker/LoopCloser/GlobalBA replicas and LoopCloser stats private behind *TestAccess in tests/test_access.h; GlobalBA run counters atomic — D-14/D-26: same hazard class as the Mapper accessors; ThreadAffinityTest concepts fail the build if one becomes public
- [Phase 01]: 01-10: Mapper's twelve Mapper-thread accessors private behind MapperTestAccess in tests/test_access.h; ThreadAffinityTest concept checks; version 0.5.0 — OWN-03 enforced by types for all four components; D-14, D-26 recorded in DESIGN decisions log
- [Phase 01]: [01-11] MapFollower defined in the library (INSTANTIATE_CLASS) so -O0 consumers do not override optimized library code; binocular app gate bar: >=36 FrameTracked, >=3 Frames on corridor1 40 frames
- [Phase 01]: 01-12: GlobalBA stop_requested_ is not reset in Start() (Stopped is terminal); it is raised inside the stop lambda and restored if a self-stop is refused
- [Phase 01]: 01-12: a Slam::Stop() refused by a component's own thread restores the Slam state and rethrows; a later Stop() from another thread completes it
- [Phase 01]: Stop() then Start() is not supported: Stopped is terminal for Slam and its components, a new session is a new Slam; WorkerThread and MapViewer stay restartable (01-13)
- [Phase 01]: Slam::Stop() from a component's own thread is a documented gap (DESIGN 4, TODO), options left for the user; package version 0.6.0 (01-13)
- [Phase 01]: R3 (0.6.0) merged to master as 124a668 on user's 'merge and push'; option (a) chosen by user: Slam::Stop() from a component thread throws UsageError and changes nothing, ~Slam there terminates, Slam::run_state() added
- [Phase 01]: 01-15: OrbFeatureExtractor pyramid moved into a caller-owned Workspace (chosen over non-const method and thread_local); Workspace re-initialises on a new image size, level count or scale factor (ImagePyramid::Fits); version 0.7.0 on branch refactor/extractor-workspace (merge pending)
- [Phase 01]: R4 (0.7.0) merged to master as 823470d on user's 'merge and push'; one shared OrbFeatureExtractor with one Workspace per extraction thread; branch kept
- [Phase 01]: R5 (0.8.0): all loggers share one function-local spdlog dist_sink_mt; Logger::Initialize re-targets via set_sinks, level is atomic, CreateLogger serialized; logcat default stays in PLAT-01
- [Phase 01]: R5 (0.8.0) merged to master as 93c00ce on user's 'merge and push'; Logger::Initialize re-targets existing loggers, branch kept

### Pending Todos

None in `.planning/todos/`. Pending work is tracked in `docs/TODO.md` (about 193 open items), which stays the tracker.

### Blockers/Concerns

- [Phase 2] OPEN DECISION: drift-bound vs verify-before-publish vs both for loops (LOOP-02/03/04). Decide with the user in `/gsd-discuss-phase 2`. Merges need verify-before-publish regardless (no shared world).
- [Phase 2] Merge integration margins are thin (map 1 holds 10-11 Frames vs keep threshold 10; stereo ratio 1.33-1.40 vs bar 1.5).
- [Phase 2] The false-loop diagnosis file cited in TODO (`.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`) is git-excluded and absent from the working tree.
- [Phase 6] LATE-01 contradicts the locked "no late joining" decision (DEC-2026-10-10-8) and needs explicit user approval.
- [Phases 3-6] Their order and grouping are a planning proposal built from TODO items, not a user decision. Not yet in `docs/ROADMAP.md`; keep the two roadmaps in sync once agreed.
- [Phase 1] Every audit item is an API break for voyah_jni: bump the minor version, recreate the Conan packages (linux-gcc13 static + shared, android-arm64) and tell the voyah_jni session.
- [All] Acceptance bars are "to be specced" for most pending requirements; none were invented.
- [All] Flaky tests on record: `GeometryTest.SquaredSampsonErrorScaleIndependent` (unseeded random matrix), `GlobalBundleAdjusterTest.StopInterruptsRunningBa` (test-only hang, now has 5 s deadlines). ASan runs abort in g2o (PLAT-01).
- [All] No `.planning/config.json` existed: standard granularity and sequential phase ids were assumed.

## Deferred Items

Items acknowledged and deferred at milestone close, most recent first:

| Category | Item | Status | Deferred At | Milestone |
|----------|------|--------|-------------|-----------|
| *(none)* | | | | |

## Session Continuity

Last session: 2026-10-14T21:24:37.377Z
Stopped at: Completed 01-18-PLAN.md
Resume file: None
