---
gsd_state_version: "1.0"
current_phase: 1
current_phase_name: API Ownership and Lifetime Contracts
status: planning
stopped_at: Phase 1 context gathered
last_updated: "2026-10-12T13:20:02.003Z"
last_activity: 2026-10-12
last_activity_desc: Roadmap created from the brownfield ingest (41 documents, 20 pending requirements mapped to 6 phases)
state_head: 242cce5dc3303b122c119b4631a2706cef2c6404
progress:
  total_phases: 7
  completed_phases: 0
  total_plans: 0
  completed_plans: 0
  percent: 0
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-10-12)

**Core value:** ORB-SLAM3-class accuracy on real sequences (TUM-VI corridor, mono and stereo) delivered through a lock-free, replica-per-thread, event-only architecture.
**Current focus:** Phase 1 - API Ownership and Lifetime Contracts (voyah_jni ownership audit; next items #3 with #2)

## Current Position

Phase: 1 of 7 (API Ownership and Lifetime Contracts)
Plan: 0 of TBD in current phase
Status: Ready to discuss (audit #1 and #4 merged before GSD as 0.2.0 / 0.3.0; #3 with #2 next)
Last activity: 2026-10-12 - Roadmap created from the brownfield ingest (41 documents, 20 pending requirements mapped to 6 phases)

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

## Accumulated Context

### Decisions

Locked decisions are in PROJECT.md `<decisions>`; the full set is `docs/DESIGN.md` section 8 (214 entries in `.planning/intel/decisions.md`).
Recent decisions affecting current work:

- [2026-10-10]: Loop closing is opt-in and off by default (user decision); do not flip without explicit approval.
- [2026-10-11]: Drift-consistency check for loops requested by the user; whether it replaces or complements a drift-bound gate, and whether verify-before-publish is added, is OPEN.
- [2026-10-12]: Object pools grow on overflow (0.3.0); `MapTransaction` must never be dropped.
- [2026-10-12]: The voyah_jni ownership audit is Phase 1 (user decision); loop-closing look-alike rejection moved to Phase 2, later phases renumbered.
- [Roadmap]: Step 6 split: look-alike rejection is Phase 2, remaining TUM-VI tuning is Phase 7 (last).

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

Last session: 2026-10-12T13:20:01.991Z
Stopped at: Phase 1 context gathered
Resume file: .planning/phases/01-api-ownership-and-lifetime-contracts/01-CONTEXT.md
