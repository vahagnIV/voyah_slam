# Map Merging (Multi-Map Atlas) — Design Spec

Date: 2026-10-10
Status: approved in conversation, pending spec review
Roadmap: step 5 in `docs/ROADMAP.md`. Builds on loop closing (4a+4b, opt-in)
and global BA (4c).

## Context and Scope

**Today.**
- When tracking is finally lost, the Mapper throws its map away and starts
  map n+1.
- Every replica adopts the newer id and clears the old map ("newest map id
  wins").
- Loop detection searches only the current map, and the Mapper rejects a loop
  across maps.
- `LoopDetected` already carries both map ids, and the 4b correction steps
  take a window and a Sim3.

**Goal.** Follow ORB-SLAM3's Atlas, with three parts:
1. Keep lost maps as inactive maps.
2. Detect when the active map revisits an inactive map.
3. Merge the active map into it (`LoopClosing::MergeLocal`, the visual path).

**Decisions made in conversation:**
- **Scope A, ORB-SLAM3 parity.**
  - Relocalization stays active-map-only, as in ORB-SLAM3.
  - Relocalization into inactive maps goes to TODO.
- **Every replica mirrors the whole Atlas, with shared measurements.**
  - Each replica (Tracker, LoopCloser, GlobalBA, viewer scene) holds every
    map.
  - A Frame's measurement is created once and shared read-only, to keep
    memory down.
- **The target map survives, as in ORB-SLAM3.**
  - The inactive (old) map keeps its id and world frame, and absorbs the
    active map.
  - The absorbed map's points get new ids appended to the target, and the
    absorbed map is retired.
  - The active map is named explicitly. It is no longer "the newest id".

**Constraints.**
- No locks on map data, and the Mapper is the only writer.
- Frame ids are never reused in a run, and map ids are never reused.
- Point ids are append-only per map; `(map_id, id)` identifies a point.
- Merging is part of loop closing: it is opt-in under `--loop_closing`, off by
  default.
- A wrong merge corrupts two maps, so merges never get looser gates than
  loops.

**Out of scope (TODO):**
- relocalization into inactive maps;
- showing inactive maps in the viewer;
- merge edges kept separate from loop edges;
- the corridor1 false-match problem (step 6).

## 1. Atlas, map lifecycle and replicas

**The Mapper's Atlas.**
- The Mapper holds maps by id. At most one map is active.
- On a final `TrackingLost` of the active map, following ORB-SLAM3
  `Tracking`:
  - with fewer than `min_frames_to_keep` (10) Frames, the map is discarded,
    as today (`ResetActiveMap`);
  - otherwise it is kept as an inactive map (`CreateMapInAtlas`).

  The next initialization creates a new active map with the next id.
- Inactive maps are frozen. There is no local mapping, culling, BA or global
  BA on them. Only a merge changes them.

**Transactions.** `MapTransaction` (its `Event::map_id` names the map it
changes) gains two fields:
- `active_map`, an optional `MapId`: the active map after this transaction.
  It is set on every transaction.
- `removed_maps`: a discarded small map, or a map retired by a merge.

**The replica rule** replaces "newest id wins / older ignored". A replica:
- applies a transaction to the map it names;
- ignores transactions naming an unknown or removed map;
- takes the active map only from `active_map`.

A final `TrackingLost` no longer clears replicas. The Mapper answers it with
one transaction that deactivates or removes the map, and every replica
follows.

**Replicas.** Every replica mirrors the Mapper's Atlas.
- **Tracker:** tracks and relocalizes only in the active map.
- **LoopCloser:** its `FrameDatabase` covers every live map.
- **GlobalBA:** runs only on the map of the correction that triggered it.
  That is always the active map.
- **Viewer scene:** shows the active map.

**Shared measurements.**
- A Frame's measurement is created once by the Tracker. From then on it is
  held as `std::shared_ptr<const TMeasurement<FE>>`:
  - in `FrameTracked` / `NewFrame`;
  - in the Mapper's Frames;
  - in every replica.
- It is immutable after creation, so it can be shared without locks. The plan
  verifies that nothing writes to a measurement after creation.

## 2. Merge detection (LoopCloser, ORB-SLAM3 `NewDetectCommonRegions`)

**Candidates.** `DetectNBestCandidates` returns two lists:
- **loop candidates:** hits in the query's own map, which is the active map;
- **merge candidates:** hits in inactive maps.

Hits in removed maps are never returned. Covisible Frames are excluded only
within the query's own map. Each list keeps its best `num_candidates` (3).
Detection is skipped while the active map has fewer than `min_map_frames`
(12) Frames.

**Verification.**
- Both kinds go through the same `DetectCommonRegion`:
  - BoW matching;
  - Sim3 RANSAC;
  - the two projection stages;
  - `OptimizeSim3`;
  - the normal-consistency gate.

  The ORB-SLAM3 thresholds are the same for loops and merges.
- Mono merges solve a Sim3 with free scale, since two mono maps have
  unrelated scales. Stereo keeps scale 1.

**Confirmation.**
- A merge region has its own pending state, separate from the loop region
  (ORB-SLAM3 `mnMergeNumCoincidences`, `mnMergeNumNotFound`).
- It needs `required_coincidences` (3) confirmations, from the query's
  covisible Frames or from later queries.
- It is dropped after `max_unconfirmed_queries` (2) failed queries.

**Publishing.**
- A confirmed merge reuses `LoopDetected`, with `match_map_id ≠
  query_map_id`. The payload is unchanged.
- If a loop and a merge are confirmed at the same query, the merge is
  published and the loop is dropped (ORB-SLAM3 handles `mbMergeDetected`
  first).

## 3. The merge (Mapper, ORB-SLAM3 `MergeLocal`, visual)

Notation: M is the active map, N the inactive target, and S the Sim3 taking
M's world coordinates into N's.

**Rejection.** The merge is rejected when any of these holds:
- the query is not in the active map;
- the match map is not an existing inactive map;
- the query or match Frame is culled;
- a correction or merge is running.

A running local BA is interrupted first.

**Steps.**
1. **Windows.** The current window is the query plus its covisible Frames,
   filled up to `merge_window` (25, ORB-SLAM3 `numTemporalKFs`) through
   covisibles of covisibles. The merge window is built the same way around
   the match Frame in N.
2. **Move the current window.** Its Frames take S. Its points follow their
   reference Frame. For mono, the scale is folded into translations and
   positions, as in the 4b correction (`CorrectWindow`).
3. **Re-root M's spanning tree.** The query becomes a child of the match
   Frame. The parent chain from the query to M's root is inverted (ORB-SLAM3
   `ChangeParent` walk). The result is one tree with N's root.
4. **Fuse.** The `LoopDetected` pairs, plus `SearchAndFuse` (Sim3, radius 4),
   fuse duplicates across the seam through the replace machinery (4b
   `FuseLoopPoints`).
5. **Weld.** A local BA with the current window free and the merge window
   fixed (ORB-SLAM3 `LocalBundleAdjustment(current, current window, merge
   window)`), using `optimization::BundleAdjust`.
6. **The rest of M.** Frames of M outside the window take S. Then the
   essential graph runs over M's part, with both windows fixed (4b
   `OptimizeGraph`). Points follow their reference Frame.
7. **Absorb.**
   - All of M's Frames and points now belong to N.
   - M's points get new ids `N.next_id + k`, where k is the rank of their old
     id in ascending order.
   - A merge edge links query ↔ match. It is stored as a loop edge, a
     simplification of ORB-SLAM3's separate merge edges.
8. **Publish one `MapTransaction` of map N.** It carries:
   - M's Frames with their corrected poses and new parents;
   - M's points under their new ids, with positions;
   - fusions and replacements;
   - the merge edge;
   - `removed_maps = {M}`, `active_map = N`, and N's `map_epoch + 1`.

   Every replica already holds M. It moves M's Frames and points into N by
   the id rule and drops M.

**After the merge.**
- Global BA runs on N, as ORB-SLAM3 relaunches it after `MergeLocal`. It is
  triggered by the merge edge, which is carried as a loop edge.
- **Reuse:** steps 2, 4 and 6 are the 4b functions. Step 5 is the existing
  BA.

## 4. Tracker and other state across a merge

- **While Tracking.** A transaction moving `active_map` from M to N switches
  the Tracker to N. Its last pose shifts by its reference Frame's correction
  (the 4b epoch path). `FrameTracked` then carries map N and N's epoch, so the
  Mapper's stale-epoch rule keeps applying.
- **Point references.** The Tracker's last frame and local map refer to M's
  point ids. They are translated by the merge's id rule and by its fusions. A
  reference that cannot be translated becomes unmatched.
- **Mapper state.** The Mapper's per-map state for M is remapped or moved the
  same way: recent points, culling queues, the `FrameTracked` pipeline.
- **While RecentlyLost or Lost.** No shift. Relocalization searches the
  active map, now N.
- **No active map.** After a final loss with no active map, the Tracker waits
  for the next initialization, as today.
- **Viewer.** It follows `active_map`.

## 5. Options

- `MapperOptions::atlas`:
  - `min_frames_to_keep` 10;
  - `merge_window` 25.
- Merge detection reuses `LoopCloserOptions`. Merging is on whenever loop
  closing is on.

## 6. Testing

**`voyah_slam`, Atlas:**
- A map with fewer than 10 Frames is discarded; a larger one is kept
  inactive.
- New maps get new ids.
- `removed_maps` and `active_map` reach every replica.
- A final `TrackingLost` no longer clears replicas.
- Inactive maps are never local-mapped, culled or optimized.

**Shared measurements:** replicas share one measurement object per Frame (the
same address), and it is `const` by type.

**Detection.** A synthetic two-map scene: map 1, a loss, map 2, then a revisit
of map 1's area.
- A merge candidate is found only in the inactive map, and is confirmed 3
  times.
- The loop and merge pending states do not reset each other.
- A merge wins over a loop at the same query.
- A removed map is never matched.

**Mapper merge:**
- The windows are built as specified.
- The re-rooted tree is a single acyclic tree.
- M's points get contiguous new ids after N's highest id.
- Duplicates are fused.
- The weld moves only the current window, and the rest of M is moved.
- Exactly one transaction is published, with `removed_maps = {M}` and
  `active_map = N`. Every replica passes its consistency check and equals the
  Mapper.
- The merge is rejected when the match map is active, the match map is
  removed, a Frame is culled, or a correction is running.

**Tracker:** keeps tracking across a merge, with its point references
translated.

**Integration: `SlamTest.{Mono,Stereo}MergesMaps`.**
- **Scene:** a synthetic loop with a blackout long enough to lose the map
  before the return, over 3 fixed seeds.
- **Success:**
  - one map at the end, holding the Frames of both parts;
  - no tracking loss across the merge;
  - the whole-run trajectory error, under one alignment, at most 1.5× that of
    the same scene without the blackout (one continuous map closing its
    loop).
- A missed bar is diagnosed, not tuned.

**ThreadSanitizer:** the slam and integration test binaries.

**Smoke (corridor1, Release):**

| Run | Must | Report |
|---|---|---|
| Merge: `--vocabulary --loop_closing --drop_frames` over a stretch longer than 5 s (e.g. 1500:1700), mono and stereo, 2 runs each | No crash; no tracking loss caused by a merge | merges found and applied, false merges, maps over time, ATE of the final map |
| Memory: default flags (no loop closing), stereo | Peak RSS not above master's | RSS reduction from shared measurements (master: about 1.7 GB peak) |

There is no ATE Must: corridor1's false matches still dominate (step 6).

## 7. Docs

- **`DESIGN.md`:**
  - the Atlas and the map-id rules (replacing "newest id wins");
  - shared measurements;
  - merge detection and the merge;
  - decisions-log entries (scope A, all replicas mirror the Atlas, the target
    map survives).
- **`TODO.md`:**
  - relocalization into inactive maps;
  - inactive maps in the viewer;
  - separate merge edges.
- **`ROADMAP.md`:** step 5.
