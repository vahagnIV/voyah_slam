---
phase: 01-api-ownership-and-lifetime-contracts
plan: 06
subsystem: docs
tags: [design-doc, decisions-log, versioning, conan, usage-error, subscription]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-01..01-05: the implemented shared-event, inbox-core, Wiring/Subscription design, migrated production code and tests, pools deleted"
provides:
  - "DESIGN 6.1 rewritten as the shared-ownership lifecycle, described from the code as implemented"
  - "DESIGN exception policy (UsageError), death-test and -Werror=unused-result conventions in section 7"
  - "decisions log rows 'Ownership audit #3+#2 implemented' and 'Package version 0.4.0'; OQ 25 closed, OQ 14 obsolete"
  - "TODO, ROADMAP, README and CLAUDE.md consistent with R1; measured test counts (762 unit, 792 total)"
  - "package version 0.4.0 in project(), conanfile.py, README and DESIGN (SOVERSION 0.4)"
affects: [01-07, voyah_jni]

actuals:
  tokens: 23800   # chars/4 over the realized diff (95372 chars, additions + deletions)
  tasks: 2
  commits: 3
plan_head_before: 619503e270755ba11061dceb54f387def0d528ad
plan_head_after: a8853d3b0f20084319f44d5430ebdd802a04abc6
commits: 3

tech-stack:
  added: []
  patterns:
    - "Document the code as merged, not the plan: DESIGN 6.1 names RunState as driven by BeginRun/EndRun until audit #6, ~Observable storing Stopped, and the closed-core one-event race"
    - "Superseded decision-log rows keep their text and gain an 'Superseded <date>' note instead of being deleted"

key-files:
  created: []
  modified:
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "Superseded decisions-log rows are annotated in place (three 2026-09-30 rows and the 2026-10-12 pool-growth row) rather than removed; the 2026-10-02 pool rows are named historical in the new row"
  - "OQ 14 and OQ 25 are struck through with a pointer, keeping their original text for history"
  - "TODO 'Subscriber lifetime' is ticked but records the consumer-side rule that remains open as documentation: a sync handler's ctx must outlive its connection"
  - "CLAUDE.md names the vocabulary label in the check_all count, since the 792 includes the one ORBvoc test (skipped without the vocabulary)"

patterns-established:
  - "A version bump touches project(), conanfile.py, README (package, SOVERSION, install lines) and DESIGN (layout and SOVERSION lines) together"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "DESIGN 6.1 is the shared-ownership lifecycle (make_shared, rvalue Notify, EventPtr fan-out, InboxCore, Wiring/Subscription/RunState, destruction-order table, ~AsyncObserver release, backlog warning, unbounded LoopDetected/GlobalBAFinished) and the 'Superseded' note is gone"
    requirement: OWN-01
    verification:
      - kind: other
        ref: "grep -n -e '### 6.1 Event Lifecycle: Shared' -e 'UsageError' -e 'Package version 0.4.0' -e 'must outlive its connection' docs/DESIGN.md (all four found); ! grep -e 'Superseded (decision 2026-10-13' -e 'Pooled (256 in the Mapper)' -e 'Pool of 16, published with' docs/DESIGN.md (empty)"
        status: pass
    human_judgment: false
  - id: D2
    description: "DESIGN section 7 records the exception policy (UsageError in every build, destructors never throw), the death-test convention (DeathTest suffix, threadsafe style, excluded from TSan) and -Werror=unused-result on tests and apps"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "grep -n 'Exception policy' -A8 docs/DESIGN.md names voyahslam/errors.h; 'Test conventions for misuse' paragraph present"
        status: pass
    human_judgment: false
  - id: D3
    description: "Decisions log has the two 2026-10-13 rows (R1 implemented, naming the superseded rows; Package version 0.4.0); OQ 25 closed, OQ 14 obsolete; TODO ticks #3, #2 and the four related items"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "docs/DESIGN.md decisions log tail and section 9 OQ 14/25; docs/TODO.md 'Ownership audit' and 'Event system' sections"
        status: pass
    human_judgment: false
  - id: D4
    description: "Package version is 0.4.0 in project(), conanfile.py, README and DESIGN; CLAUDE.md test counts equal the measured ctest -N counts; check is green with the new version"
    requirement: OWN-02
    verification:
      - kind: other
        ref: "git grep -n '0\\.3\\.0' -- CMakeLists.txt conanfile.py README.md (empty); ctest -N -L unit = 762, ctest -N = 792; cmake --build cmake-build-debug/build/Debug --target check: 100% passed, 762 tests"
        status: pass
    human_judgment: false
  - id: D5
    description: "A reader of DESIGN 2, 4, 5, 6, 6.1, the viewer, Loop Closing and Global BA sections finds no statement that events are pooled, recycled, reference-counted by Recyclable or dropped by LoopCloser/GlobalBA bounds"
    requirement: OWN-01
    verification: []
    human_judgment: true
    rationale: "Plan marks this truth 'verification: backstop': a case-insensitive grep for pool/recycl/Recyclable/SyncPolicy/SetWakeSignal/TryCreate/ref count over sections 1-8 leaves only explicit 'no pool' / 'since 2026-10-13, was a pool' statements, but whether the prose reads correctly is a reviewer's judgment."

duration: 10min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 06: Document R1 and bump to 0.4.0 Summary

**DESIGN 6.1 is rewritten around `make_shared` events, `EventPtr`, `InboxCore` and `Subscription` as merged, with the exception policy, two decisions-log rows and the closed/obsolete open questions, and the package is 0.4.0 with measured test counts (762 unit, 792 total) and `check` green.**

## Performance

- **Duration:** about 10 min
- **Started:** 2026-10-13T14:31Z (approximate, after the 01-05 close-out)
- **Completed:** 2026-10-13T14:40Z
- **Tasks:** 2
- **Files modified:** 7 (0 created)

## Accomplishments

- DESIGN 6.1 now reads "Event Lifecycle: Shared, Immutable, Released by the Last Holder": the building blocks, the one-event lifecycle diagram, producer and consumer rules, the wiring rules (D-05..D-07), the destruction-order table, `~AsyncObserver` release (D-08), the sync-handler `ctx` rule in the words of the `ConnectSync` comment (D-22), the subscriptions-before-Slam rule (D-23), the backlog warning at 1024 and each doubling (D-24), the unbounded `LoopDetected` / `GlobalBAFinished` (D-25) and "Adding a new event type". The old pool, Treiber-stack, `Recyclable` and reuse text is gone.
- Sections 2, 4, 5, 6, the viewer, Loop Closing, Global BA and the layout/status parts match the code: no pool rows in the data-layer table, `InboxCore` / `Subscription` / `Wiring` / `RunState` / `UsageError` rows added, `num_dropped_loops()` / `num_results_dropped()` out of the component table, `SetWakeSignal` out of the waking paragraph, pool wording gone from the viewer section.
- Section 7 gains the exception policy (`voyahslam::UsageError`, every build, destructors never throw) and the test conventions (death tests, `-Werror=unused-result`).
- Decisions log: "Ownership audit #3+#2 implemented" (superseding the three 2026-09-30 delivery rows and the pool-growth mechanism) and "Package version 0.4.0" (the API breaks for voyah_jni); OQ 25 closed, OQ 14 obsolete; section 10 status and test-suite line updated.
- TODO ticks #3, #2, "AsyncObserver destruction", "Event pool lifetime", "Subscriber lifetime", "Size event pools", the `queue()`-private note and "A dropped result is not retried"; lossy delivery stays open as EVT-01 with the new inbox-growth wording. ROADMAP records #3+#2 done with #5 next. README drops "memory pools" and moves to 0.4.0; CLAUDE.md carries the measured counts.

## Task Commits

1. **Task 1: DESIGN.md rewrite (6.1, dependent sections, exception policy, decisions log)** - `d0c26df` (docs)
2. **Task 2: TODO, ROADMAP, README, CLAUDE.md and the 0.4.0 bump** - `a44eb3f` (chore)
3. **Follow-up: exact sanitizer claim in the DESIGN test-suite line** - `a8853d3` (docs)

**Plan metadata:** docs commit follows this summary (`docs(01-06): complete ...`).

## Files Created/Modified

- `docs/DESIGN.md` - 6.1 rewritten; sections 2, 4, 5, 6, viewer, Loop Closing, Global BA, 7, decisions log, OQ 14/25 and section 10 updated
- `docs/TODO.md` - ownership audit #3 and #2 and six related items ticked, lossy-delivery item reworded
- `docs/ROADMAP.md` - API hardening: #3+#2 done (0.4.0), next #5
- `README.md` - "memory pools" removed; `voyahslam/0.4.0`, `libvoyah_slam.so.0.4`
- `CLAUDE.md` - 762 unit / 792 total test counts
- `CMakeLists.txt`, `conanfile.py` - version 0.4.0

## Decisions Made

- Superseded rows in the decisions log are annotated, not deleted; the new "implemented" row names exactly which rows it supersedes and calls the 2026-10-02 pool rows historical.
- 6.1 describes `RunState` as implemented: driven by `EventNode::BeginRun()` / `EndRun()` and gating wiring only, with the throwing `Start`/`Stop` transitions left to audit #6, and notes that `~Observable` stores `Stopped`.
- The "Subscriber lifetime" TODO item is closed but keeps the one rule that remains documentation-only: a sync handler's `ctx` must outlive its connection.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Stale text] Further pool wording beyond the plan's site list**
- **Found during:** Task 1 (case-insensitive grep over DESIGN)
- **Issue:** besides the sites the plan lists, DESIGN had pool/drop wording in the Tracker/Mapper component text ("options, vocabulary, pools"), the waking paragraph (`SetWakeSignal()`), the viewer's drain/ordering bullets, the Loop Closing apply-on-receipt paragraph, the Global BA apply-on-poll rationale and its test list ("a full result pool", "a pool of 4"), the map-id and measurement paragraphs ("pooled events are reused without reset", "Reset()").
- **Fix:** rewrote each to the shared-event design, keeping the historical fact where the text was a dated result (for example "the 256-slot pool aborted until 0.3.0").
- **Files modified:** `docs/DESIGN.md`
- **Committed in:** `d0c26df`

**2. [Rule 1 - Accuracy] TODO review item and sanitizer claim**
- **Found during:** Task 2 and the self-check
- **Issue:** a TODO review follow-up ("event pool size (256) is not documented") was moot; my first test-suite line claimed ASan on the Slam tests, which only the events binary had (01-05 ran TSan on the Slam tests).
- **Fix:** struck the TODO item as moot; corrected the DESIGN sentence.
- **Files modified:** `docs/TODO.md`, `docs/DESIGN.md`
- **Committed in:** `a44eb3f`, `a8853d3`

---

**Total deviations:** 2 auto-fixed (both Rule 1, documentation accuracy)
**Impact on plan:** none on scope; the requested doc sites are all done, plus the leftover pool wording the plan's negative grep alone would not have caught.

## Issues Encountered

- The plan's phrase check `must outlive its connection` initially matched only the decisions-log row, because the 6.1 sentence wrapped across two lines; the sentence was reflowed so the phrase is on one line in 6.1 too.

## Known Stubs

None.

## Threat Flags

None. T-01-12 (stale text leading a consumer to the old teardown order) is mitigated by the 6.1 rewrite, the stale-text negative grep and the 0.4.0 row listing every API break; T-01-SC accepted (no package added; only the package's own version string changed).

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plan 01-07 (release R1: sanitizer runs, `conan create` for linux-gcc13 static and shared and android-arm64, review, and the "merge and push" gate, then telling the voyah_jni session) can package 0.4.0; the DESIGN row "Package version 0.4.0" is the API-break list to hand to the binding.
- Not done here by design: `conan create` and the voyah_jni notification (plan 01-07). No merge, no push.

## Self-Check: PASSED

`docs/DESIGN.md`, `docs/TODO.md`, `docs/ROADMAP.md`, `README.md`, `CLAUDE.md`, `CMakeLists.txt` and `conanfile.py` exist; commits `d0c26df`, `a44eb3f` and `a8853d3` are ancestors of HEAD; both Task 1 verify commands pass (four patterns found, negative grep empty); `git grep '0\.3\.0' -- CMakeLists.txt conanfile.py README.md` is empty; TODO shows #3 and #2 ticked; CLAUDE.md counts equal `ctest -N` (762 unit, 792 total); `check` passes at 762 tests with the 0.4.0 version.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
