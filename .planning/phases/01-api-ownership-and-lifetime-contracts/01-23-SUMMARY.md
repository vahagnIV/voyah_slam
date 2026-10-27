---
phase: 01-api-ownership-and-lifetime-contracts
plan: 23
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, gap-closure, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-21 and 01-22: own-thread Stop in every state, private ProcessPending, drained example followers, checked Image views and CopyTo, validated ORB extractor, version 0.9.2 on fix/phase1-gaps"
provides:
  - "Gap-closure gates for voyahslam 0.9.2 all green on the first run (check_all, apps, TSan, ASan, golden test, greps, three conan create)"
  - "fix/phase1-gaps merged into master (no-ff, a975ee7) and pushed together with f0d02d3 and 643cc0e; origin/master == master"
  - "voyahslam/0.9.2 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every behaviour change of 0.9.2"
  - "voyah_jni/docs/voyahslam-0.9-api.md brought to 0.9.2 (left uncommitted in voyah_jni)"
affects: [voyah_jni, phase-01-verification]

actuals:
  tokens: 0          # no source change in this plan (gates and merge); the docs edits are outside the measured diff
  tasks: 3
  commits: 1         # MEASURED: git rev-list --count 6231158..a975ee7 (the no-ff merge); the docs commit with this SUMMARY follows it
plan_head_before: 6231158
plan_head_after: a975ee72cdb868008b48b4a452d19a72126708df
commits: 1

tech-stack:
  added: []
  patterns:
    - "Release plan with no code fixes: gates run on the branch head, the merge is the only code-bearing commit"

key-files:
  created: []
  modified:
    - docs/DESIGN.md

key-decisions:
  - "Merged on the user's literal 'merge and push' (second reply); the first reply 'list of changes' was a request to see the changes, so nothing was merged then"
  - "Branch fix/phase1-gaps kept, no force push"

requirements-completed: [OWN-01, OWN-02, OWN-03, OWN-04, OWN-05, OWN-06, OWN-07]

duration: "gates about 1 h (TSan slam 6m47s, integration 12m29s), merge and push under 1 min"
completed: 2026-10-27
status: complete
---

# Phase 1 Plan 23: Release 0.9.2 (gap closure) Summary

**The gap-closure release passed every D-21 and G1-G5 gate on the first run, was merged into master on the user's "merge and push" as a975ee7 and pushed; voyahslam 0.9.2 closes the two Phase 1 verification gaps (own-thread `Stop()` deadlock, public `ProcessPending()`) and the review findings WR-01, WR-06, IN-07, CR-01 and CR-02.**

## Task 1: Release gates (branch head 6231158, no source changes)

| Gate | Result 0.9.2 | R6 (0.9.0, plan 01-20) |
|------|--------------|------------------------|
| `check_all` (Debug tree) | 866/866 pass (825 unit, 40 integration, 1 skipped: `VocabularyEquivalenceTest`, ORBvoc absent); no flake; 27.9 s | 852/852 (812 unit, 39 integration, 1 skipped); 25.8 s |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, `map: 0 Frames 0 points`, `map follower: 0 of 0 transactions applied after Stop()`, mean extract 111.5 ms | rc 0, WaitingForMap, 0 Frames |
| `binocular_full_slam`, same arguments | rc 0, 39 `[Mapper] FrameTracked` (bar >= 36), Tracking, `map: 3 Frames 777 points` (bar >= 3), `map follower: 0 of 5 transactions applied after Stop()`, mean load 21.7 ms, mean extract 222.7 ms | rc 0, 39 FrameTracked, 3 Frames, 777 points |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 82; `LoggerTest.*` 6; `OrbExtractorWorkspaceTest.*` + `OrbInputTest.*` 9; `voyah_slam_tests` component filter 199 (6m47s); `voyah_integration_tests` 37 (12m29s, run to completion); 0 "WARNING: ThreadSanitizer" | events 80; slam 196 (6m11s); integration 36 (11m56s); 0 warnings |
| ASan | `voyah_events_tests` 83 and `ImageTest.*` 18 pass; no AddressSanitizer or LeakSanitizer error | events 81; `ImageTest.*` 14 |
| `OrbExtractorGoldenTest.Corridor1OutputUnchanged` | pass (573 ms), not skipped, digests unchanged | pass (568 ms) |
| G1 removed-symbol grep (`ObjectPool`, `Recyclable`, `StopAll`, `event_pool_size`, ...) | empty | empty |
| IN-07 grep (`.ProcessPending()` outside `test_access.h`) | only the `requires` concept in `thread_affinity_test.cpp:78` (a compile-time negative check, not a call) | not applicable |
| Version | `CMakeLists.txt:2` VERSION 0.9.2, `conanfile.py:21` version "0.9.2" | 0.9.0 |
| `conan create` voyahslam/0.9.2 | linux-gcc13 static 47 s and shared 51 s rc 0, both print "voyahslam 0.9: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 46 s rc 0 | static 46.9 s, shared 48.4 s, android 44.8 s |
| G5 lock-freedom read | `inbox_core.h`, `subscription.h`, `observable.h`: lock grep empty and the files read, no lock (principle 1) | same |
| DESIGN / TODO | DESIGN has "Package version 0.9.2" (line 3590), no "fix planned for 0.9.2" under `docs/`; `docs/TODO.md` ticks the closed items and lists the deferred findings | |

Notes:
- The binocular extract time was checked on an idle machine by the orchestrator (A/B, new `image.h` against the old one): 203-228 ms against 219-226 ms, no regression from the checked views.
- No fixes were needed, so no gate was re-run and none was weakened or filtered further.
- The gate logs were lost when the scratchpad was wiped by a session restart. The numbers above are from the gate executor's report, verified by the orchestrator at the time.

## Task 2: User review

The user's replies, verbatim:

1. "list of changes" - the orchestrator treated this as a request to see the change list (it showed the 16 branch commits and 36 files), not as a change request. No code changed and nothing was merged.
2. "merge and push" - the release word; Task 3 ran after it.

## Task 3: Merge and push

- Preconditions checked before merging: branch `fix/phase1-gaps` HEAD 6231158; tree clean apart from the untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `git fetch`, then `origin/master` (68ce66b) is an ancestor of `master` (643cc0e), so nobody else pushed; master was 2 commits ahead (f0d02d3 CLAUDE.md counts, 643cc0e gap plans).
- `git switch master`, `git merge --no-ff fix/phase1-gaps`. Merge commit: **a975ee72cdb868008b48b4a452d19a72126708df**, subject "Merge branch 'fix/phase1-gaps': own-thread Stop throws in every state, private ProcessPending, checked Image views and ORB input; version 0.9.2". It brings in 41 files, +1246/-117.
- `git push origin master`: `68ce66b..a975ee7  master -> master` (publishes f0d02d3 and 643cc0e as well). The branch `fix/phase1-gaps` was not deleted. No force push.
- The final docs commit (this SUMMARY, STATE, ROADMAP, DESIGN "merge pending" wording) is pushed after the merge, so origin/master == master at the end.

## voyah_jni handover file

`/home/vahagn/git/voyah_jni/docs/voyahslam-0.9-api.md` now describes 0.9.2 (untracked in voyah_jni, edited, not staged or committed; voyah_jni has no new commit, HEAD bed2012): title, intro, section 1 package reference and `self.requires`; section 2 rules 6, 10 and 11; section 4 (`ProcessPending()` private since 0.9.2, hook setters still test-only, added to the not-public list); section 5 (the "Known bug in 0.9.0/0.9.1 (fix planned for 0.9.2)" paragraph replaced by the fixed behaviour); section 6 (ORB input checks); section 8 (checked views and `CopyTo`); section 9 (`std::out_of_range` row, widened `std::invalid_argument` row, `Guarded` paragraph); section 10 (error mapping, follower updated every frame); "Notes on the sources".

## Deviations from Plan

None - plan executed as written, with one documentation addition: `docs/DESIGN.md` still said "merge pending" for the 0.9.2 branch in the status list; it now says merged and pushed (a975ee7). [Rule 1 - stale doc, project rule: keep DESIGN up to date]

## Phase 1 release history

| Release | Version | Audit item / scope | Merge commit on master |
|---------|---------|--------------------|------------------------|
| R1 | 0.4.0 | events are `shared_ptr<const T>`, RAII subscriptions (OWN-01/02) | 33a270f |
| R2 | 0.5.0 | MapFollower read path, thread-affine accessors test-only (OWN-03) | f1ecb28 |
| R3 | 0.6.0 | component lifecycle state machine, Stopped is terminal (OWN-04) | 124a668 |
| R4 | 0.7.0 | caller-owned `OrbFeatureExtractor::Workspace` (OWN-05) | 823470d |
| R5 | 0.8.0 | shared logger dist sink, `Initialize` re-targets (OWN-06) | 93c00ce |
| R6 | 0.9.0 | `images::Image` views, overflow guard, empty default (OWN-07) | 9e57bfa |
| - | 0.9.1 | `RelocalizationOptions::enabled` | 68ce66b |
| R7 | 0.9.2 | gap closure: own-thread `Stop()` in every state, private `ProcessPending()`, checked `Image` views, validated ORB extractor, drained example followers | a975ee7 |

(0.2.0 and 0.3.0, ownership audit items #1 and #4, were merged before this phase's plans.)

## Open items

- Phase 1 verification is re-run by the orchestrator next; the phase is not marked complete here, and the seven OWN requirements are marked complete (ready-ids reported all ready) but the phase itself is left for verification.
- Deferred review findings are listed in `docs/TODO.md`.

## Known Stubs

None.

## Threat Flags

None. T-01-43 mitigated: blocking-human checkpoint, the user's literal "merge and push" above, gates run on the merged head (6231158), the voyah_jni message below and the updated handover file.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.9.2** (`voyahslam/0.9.2`, SOVERSION 0.9), merged and pushed to master as **a975ee7**, on top of 0.9.1 (relocalization switch, `TrackerOptions::relocalization.enabled`, default true, which you may already use for the app toggle). It closes the Phase 1 verification gaps. No class layout changed, but rebuild and re-run `conan install` against 0.9.2 (packages for linux-gcc13 static and shared and android-arm64 are in the local Conan cache of the build machine). What changed and what the binding must do:

1. **A component's own `Stop()` on its worker throws `UsageError` in every state**, also while another thread is stopping or starting that component, and it changes nothing. In 0.9.0 and 0.9.1 that case deadlocked (a JNI callback stopping its component while `close()` stopped the `Slam`); the hang is gone. Callbacks should still not stop components, and must not stop each other's components either (a component thread stopping a different component is not refused, see DESIGN section 4). `Slam::Stop()` from a non-component thread (your dispatch thread, a Java thread) stays the way to stop.
2. **`ProcessPending()` of Tracker, Mapper, LoopCloser and GlobalBundleAdjuster is no longer public.** The binding never called it. The `set_*_hook()` test setters are unchanged.
3. **`OrbFeatureExtractor::DetectAndExtract` (both overloads) throws `std::invalid_argument`** when any pyramid level would be below 66x66 px, which with the defaults (scale 1.2, 8 levels) means a width or height below 235 px (a 224x224 image used to divide by zero in release builds), and for an empty image. The check runs first, so the `Workspace` stays usable. **The constructor throws `std::invalid_argument`** for `levels == 0` or a scale factor that is not a finite number above 1; this matches the binding's own `ExtractorPeer` checks, which may stay. `OrbFeatureExtractor::MinLevelSize()` (66) is public for pre-checks.
4. **`images::Image(parent, x, y, w, h)` throws `std::out_of_range`** when the rectangle leaves the parent, and **`Image::CopyTo` throws `std::invalid_argument`** on a width or height mismatch (before copying), both in every build. `At`, `Set` and `operator[]` stay unchecked.
5. **Error mapping:** add a `std::out_of_range` clause (for example to `IndexOutOfBoundsException`) before `std::logic_error` in `Guarded`, next to the `std::length_error` clause from 0.9.0. `std::invalid_argument` now also comes from the library (extractor, `CopyTo`); the existing clause maps it. All new exceptions are `std::logic_error`s, so the `std::logic_error` clause stays the catch-all.
6. **The example apps now call `MapFollower::Update()` every frame** (an undrained follower holds every `MapTransaction` until `Stop()`). A binding holding a follower should update it regularly on its owning thread.
7. The handover file `voyah_jni/docs/voyahslam-0.9-api.md` is updated to 0.9.2 and left uncommitted in voyah_jni. Lifecycle, subscription, ownership, `Workspace`, logger and `Image` rules of 0.4.0 to 0.9.1 stand.

## Self-Check: PASSED

Merge commit a975ee7 is an ancestor of master and equals origin/master after the push; branch HEAD 6231158 is an ancestor of master; `fix/phase1-gaps` still exists; the handover file contains 0.9.2, `out_of_range`, "too small"/235 and no "planned for 0.9.2", and voyah_jni's `docs/` is still untracked. Gate numbers in Task 1 are from the gate executor's report (logs lost with the scratchpad).
