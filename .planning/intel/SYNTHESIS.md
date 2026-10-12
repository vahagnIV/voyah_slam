# Synthesis Summary

Entry point for `gsd-roadmapper`. Mode: new (brownfield, already implemented library, about 773 tests). Generated from `.planning/intel/classifications/*.json` and the source documents.

## Inputs

- 41 classified documents: 22 SPEC, 19 DOC, 0 ADR, 0 PRD, 0 UNKNOWN.
  - SPEC: 19 files in `docs/superpowers/specs/` plus 3 plans the classifier tagged SPEC (`plans/2026-10-05-tracking.md`, `plans/2026-10-10-global-ba.md`, `plans/2026-10-10-map-merging.md`), which were treated as history.
  - DOC: `docs/DESIGN.md` (precedence override 0), `docs/ROADMAP.md`, `docs/TODO.md`, `.superpowers/spikes/2026-10-07-stereo-drift.md`, 15 implementation plans.
- Precedence applied: DESIGN.md (0) > SPEC > PRD > DOC. A later-dated spec supersedes an earlier one on the same topic. Plans are history. ROADMAP.md governs the order of upcoming work. TODO.md governs pending and deferred items.
- There are no formal ADRs. The DESIGN.md decisions log (section 8) plus its principles (section 2) form the effective locked set.

## Decisions

- File: `.planning/intel/decisions.md`
- 202 decisions-log entries (`DEC-...`), 8 principles (`PRINCIPLE-1..8`), 4 repo rules from CLAUDE.md (`RULE-...`): 214 entries, all in force unless marked.
- 6 log entries are marked `locked (superseded ...)` or carry a narrowing note and must not be read as current. Examples: pool `Create()` aborting on exhaustion (now grows), "newest map id wins" (now the replica rule), stereo `ObservationStorage` in the Mapper (now `CovisibilityIndex` only).
- 10 entries carry explicit `[USER DECISION]` markers. Key ones: loop closing opt-in and off by default (2026-10-10), GlobalBA owns a replica from the start with no late joining, components stay pluggable through gRPC, map merging scope A with ORB-SLAM3 parity, SIFT removed, shared immutable `TDistortionModelPtr`, Android `c++_static`.
- Sources: all `docs/DESIGN.md` section 8 rows, section 2, CLAUDE.md.

## Requirements

- File: `.planning/intel/requirements.md`
- 38 requirements. No PRDs exist, so they were derived from spec goals and success criteria and from ROADMAP and TODO.
- Part A, shipped (19, described as existing behaviour): `REQ-event-memory-infrastructure`, `REQ-data-layer`, `REQ-component-threads-and-state-machines`, `REQ-two-view-geometry`, `REQ-mono-initialization`, `REQ-stereo-initialization`, `REQ-camera-model-raw-pixel`, `REQ-tracking-against-map`, `REQ-keyframes-and-map-growth`, `REQ-map-quality`, `REQ-bow-vocabulary`, `REQ-relocalization`, `REQ-library-restructure-and-fast-tests`, `REQ-loop-closing` (acceptance gap), `REQ-global-ba`, `REQ-map-merging-atlas`, `REQ-map-viewer`, `REQ-opencv-free-libraries`, `REQ-packaging-and-android`.
- Part B, pending (19): `REQ-loop-lookalike-rejection-drift-bound`, `REQ-loop-lookalike-rejection-verify-before-publish` (competing variants), `REQ-loop-drift-consistency-check` (user decision 2026-10-11), `REQ-merge-false-match-protection`, `REQ-tumvi-tuning`, `REQ-orb-extractor-rework`, `REQ-relocalization-into-inactive-maps`, `REQ-remote-visualization-and-serialization`, `REQ-component-late-joining`, `REQ-global-ba-hardening`, `REQ-lossy-observer-delivery`, `REQ-mixed-mono-stereo-maps`, `REQ-storage-growth-and-pool-sizing`, `REQ-tracker-measurement-policy`, `REQ-geometry-robustness`, `REQ-dataset-validation`, `REQ-platform-validation`, `REQ-vocabulary-training-and-licence`, `REQ-descriptor-traits-refactor`.
- Next work per ROADMAP: step 6 (TUM-VI tuning), whose first item is loop-closing look-alike rejection. Many pending entries have no stated acceptance bar (see WARNINGS).

## Constraints

- File: `.planning/intel/constraints.md`
- 22 constraints: 7 protocol, 6 api-contract, 5 nfr, 4 schema.
- Main areas: pooled event lifecycle, `MapTransaction` contract, replica rule, id rules, pointer rule, threading, library layering, coding conventions, build and packaging, event payload schemas, geometry conventions, camera and optimization APIs, Tracker thresholds, Mapper pipeline, BoW formats, loop closing options, global BA, Atlas and merging, viewer lifecycle, app CLI, and recorded performance budgets.

## Context

- File: `.planning/intel/context.md`
- 11 topics: project goal, roadmap status, TODO index, corridor1 smoke history, stereo-drift spike, open questions (OQ 1, 2, 9, 10, 14, 17, 19, 21, 22, 25), ownership and Android notes, test suite shape, implementation plans (history), spec out-of-scope lists, codebase map pointer.

## Conflicts

Detail: `.planning/INGEST-CONFLICTS.md`

- Blockers: 0
- Competing variants / warnings: 3
  1. Competing candidate fixes for loop-closing look-alike rejection (needs a user choice before step 6).
  2. Cross-reference cycles (one component of 22 documents around DESIGN, ROADMAP and TODO). Treated as benign mutual citations and synthesized in full, a deliberate deviation from the strict rule, recorded for acknowledgement.
  3. Pending requirements without stated acceptance criteria.
- Auto-resolved / info: 14. Mostly older specs superseded by DESIGN.md (core data structures, tracking states, camera model, stereo init, keyframes, loop closing defaults, global BA bar, SIFT and OpenCV), stale TODO items, stale counts and headers in DESIGN.md, plans classified SPEC, and accepted acceptance gaps.

## Pointers

- Decisions: `.planning/intel/decisions.md`
- Requirements: `.planning/intel/requirements.md`
- Constraints: `.planning/intel/constraints.md`
- Context: `.planning/intel/context.md`
- Conflicts: `.planning/INGEST-CONFLICTS.md`
- Existing codebase map (not ingested): `.planning/codebase/`

Status: no blockers. The three warnings need user acknowledgement or a choice before routing.
