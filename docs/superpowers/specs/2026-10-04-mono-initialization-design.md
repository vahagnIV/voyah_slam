# Monocular Map Initialization — Design Spec (Monocular Initialization, Parts B + C)

Date: 2026-10-04
Status: approved in conversation, pending spec review

## Context and Scope

Part A (merged, `docs/superpowers/specs/2026-10-04-two-view-geometry-design.md`)
built the two-view geometry toolkit (`TwoViewReconstructor`). This spec makes
the Mapper build a **real initial map from monocular frames**: match features
between a reference frame and the current frame (ORB-SLAM3
`SearchForInitialization`), reconstruct, scale, store and publish keyframes
plus map points with their observations.

**In scope**

1. Prerequisites: keypoint pyramid level in `ImageFeatures` (with a per-extractor
   level → scale mapping), a `FeatureGrid` for area queries, and the
   `TPose` → `RigidTransformation` migration (keyframes store `T_cw`).
2. `InitializationMatcher` — ORB-SLAM3's modified second-nearest-neighbour
   initialization matcher.
3. Real monocular `MapInitializer`, `InitializationResult` with map points.
4. Mapper storage and publication of the initial map, including a new
   observations field on `KeyFrameCreated`.
5. Synthetic multi-frame sequence generator for tests; updated Mapper and
   end-to-end tests.

**Out of scope (→ TODO.md):** Pangolin viewer (next — needed to check
initialization visually before tuning); tuning on TUM-VI 512 including the
per-frame "dropped by the 85° rule" statistic; bundle adjustment after
initialization; real stereo initialization (needs stereo matching — stereo
keeps the stub); real tracking (`PoseEstimator` stays a stub); robust-loss LO.

**Success criteria**

1. On a synthetic monocular sequence the Mapper initializes once parallax is
   sufficient (not before), publishes `MapPointsAdded` → `KeyFrameCreated` ×2
   (with observations) → `MapCreated`, and the map's median depth is 1.
2. The matcher reproduces ORB-SLAM3's rules (window, ratio, one-to-one with
   replacement, orientation histogram, level restriction).
3. All existing behaviour is preserved by the prerequisite refactors.
4. The full_slam apps run on the 1024 dataset without crashing (initialization
   is logged, not asserted).

## Decisions (from the conversation)

- Prerequisites 1–3 are tasks of this spec; observations travel on
  `KeyFrameCreated`; no bundle adjustment here.
- SIFT has natural levels (DoG octave/layer); store a level index per keypoint
  and let each extractor define what a level means as a scale.
- Matcher = ORB-SLAM3 `SearchForInitialization`, with a configurable maximum
  reference level (ORB default 0; SIFT users widen it).
- Keyframe poses are `T_cw` (world → camera); the world frame is camera 1.
- Map point ids restart at 0 per map; (`map_id`, `MapPointId`) identifies a
  point.
- If matching yields too few matches the **current** frame becomes the
  reference (ORB-SLAM3 waits for the next frame).
- The stub's `min_mono_init_interval` is removed.

## 1. Prerequisites

### 1.1 Keypoint level — `features/image_features.h`

- `ImageFeatures` gains `Eigen::Matrix<uint8_t, 1, Eigen::Dynamic> levels_`
  with accessor `[[nodiscard]] const TLevels& levels() const` (alias
  `TLevels`), passed as a fifth constructor argument after `descriptors`.
- `KeepFeatures(indices)` compacts `levels_` together with points, strengths,
  angles and descriptors.
- Every extractor exposes `[[nodiscard]] double ScaleFactor(uint8_t level) const`:
  - `OrbFeatureExtractor`: `scale_factor^level` (default 1.2), from its
    existing scale table; `DetectAndExtract` fills the level of every keypoint.
  - `SIFTFeatureExtractor`: decode OpenCV's packed `KeyPoint::octave`
    (octave = low byte as signed int8, layer = next byte); level =
    (octave + 1) × kOctaveLayers + layer with `kOctaveLayers = 3`;
    `ScaleFactor(level) = 2^(level / 3) / 2` (relative to the original image;
    the base layer is the ×2 upsampled image).
- All `ImageFeatures` construction sites (extractors, tests, test helpers)
  pass levels.

### 1.2 `FeatureGrid` — `features/feature_grid.h`

```cpp
class FeatureGrid {
public:
    // Builds a uniform grid over the bounding box of `points` (2×N, undistorted
    // pixel coordinates — may extend beyond the image rectangle for fisheye).
    FeatureGrid(const geometry::TPoints2DF& points, const Eigen::Matrix<uint8_t, 1, Eigen::Dynamic>& levels,
                int cols = 64, int rows = 48);
    // Indices of features with ‖p − center‖∞ <= radius (square window, as
    // ORB-SLAM3 GetFeaturesInArea) and min_level <= level <= max_level.
    [[nodiscard]] std::vector<Eigen::Index> FeaturesInArea(const geometry::TPoint2DF& center, double radius,
                                                           int min_level, int max_level) const;
};
```

Non-template, `src/features/feature_grid.cpp`. Empty input → every query
returns nothing.

### 1.3 `TPose` → `RigidTransformation`

All uses of `geometry::TPose` move to `geometry::RigidTransformation`:
`map/frame.h` (`pose()`, `mutable_pose()`), events `KeyFrameCreated`,
`KeyFramePoseUpdated`, `PositionUpdated`, `NewKeyFrameCandidate`,
`MapCreated`, `tracking::PoseEstimator`/`PoseEstimate`, `Tracker`
(`last_pose_`), `mapping::MapInitializer`/`InitialKeyFrame`, and their tests.
`defs.h` keeps the `TPose` typedef only if something non-pose still uses it;
otherwise it is removed. `Frame`'s documentation states: **pose = `T_cw`**,
world → camera. No behaviour change; all tests stay green.

## 2. `InitializationMatcher` — `features/initialization_matcher.h`

```cpp
struct InitializationMatcherOptions {
    double window_px = 100.0;     // square half-size around the last matched position
    uint32_t max_distance = 50;   // descriptor distance threshold (ORB TH_LOW)
    double nn_ratio = 0.9;        // best < nn_ratio × second best
    bool check_orientation = true;
    size_t histogram_bins = 30;
    int max_reference_level = 0;  // only reference features with level <= this
};

template <typename TFeatureExtractor>
[[nodiscard]] std::vector<Eigen::Index> MatchForInitialization(
    const features::TMonocularMeasurement<TFeatureExtractor>& reference,
    const features::TMonocularMeasurement<TFeatureExtractor>& current,
    const FeatureGrid& current_grid,
    std::vector<geometry::TPoint2DF>& out_last_matched,
    const InitializationMatcherOptions& options);   // size = #reference features; −1 = unmatched
```

Algorithm (ORB-SLAM3 `ORBmatcher::SearchForInitialization`), with distances
from `TFeatureExtractor::Distance`:

1. For each reference feature i with `levels(i) <= max_reference_level`:
   candidates = `current_grid.FeaturesInArea(out_last_matched[i], window_px,
   levels(i), levels(i))`.
2. Over the candidates, skip any j already matched with distance ≤ the
   current distance (`matched_distance[j] <= d`); track best and second-best
   distances.
3. Accept if `best <= max_distance` and `best < nn_ratio × second_best`; if
   current feature j was already claimed by another reference feature, that
   claim is undone; record `matched_distance[j] = best`.
4. If `check_orientation`: rotation difference
   `angle_ref − angle_cur` in radians, wrapped to [0, 2π), binned into
   `histogram_bins`; keep only matches in the three largest bins, where the
   2nd/3rd are dropped if smaller than 0.1 × the largest (ORB-SLAM3
   `ComputeThreeMaxima`).
5. For every surviving match, `out_last_matched[i]` = the matched current
   point.

Points are the measurements' `points()` (undistorted pixel coordinates).
The matcher is a header-only function template (no explicit instantiation).

## 3. `MapInitializer` and the result — `mapping/map_initializer.h`

```cpp
struct MapInitializerOptions {
    size_t min_features = 100;
    size_t min_matches = 100;
    double sigma_px = 1.0;
    features::InitializationMatcherOptions matcher;
    geometry::TwoViewOptions reconstruction;
};

template <typename FE> struct InitialKeyFrame {
    geometry::RigidTransformation T_cw;
    features::TMeasurement<FE> measurement;
};
struct InitialMapPoint {
    geometry::TPoint3DF position;      // world (= camera 1) frame, scaled
    Eigen::Index feature_in_first;
    Eigen::Index feature_in_second;
};
template <typename FE> struct InitializationResult {
    std::vector<InitialKeyFrame<FE>> keyframes;
    std::vector<InitialMapPoint> map_points;   // empty for the stereo stub
};

// MapInitializer<FE>(MapInitializerOptions options = {});
// std::optional<InitializationResult<FE>> TryInitialize(features::TMeasurement<FE> measurement);
// void Reset();
```

Monocular `TryInitialize` (binocular keeps the existing stub: 1 keyframe at
identity, no points; a binocular measurement also drops any mono reference):

1. If the current frame has ≤ `min_features` features: drop the reference;
   return nullopt.
2. No reference: store the measurement as the reference,
   `last_matched_ = reference points`; return nullopt.
3. Build `FeatureGrid` over the current frame; `MatchForInitialization`.
   Fewer than `min_matches` matches → the current frame becomes the
   reference (`last_matched_` re-initialised); return nullopt.
4. `TwoViewCorrespondences` from the matched pairs' normalized points (first
   two rows of `normalized_points()`), σ = `sigma_px` / K(0,0) of the
   reference. `TwoViewReconstructor(reconstruction).Reconstruct`. Failure →
   keep the reference and `last_matched_`; return nullopt.
5. Success: map points = triangulated pairs; median depth z̃ of their
   positions in camera 1; if z̃ is not finite or ≤ 0 → treat as failure
   (step 4). Scale positions and the translation of `T_21` by 1/z̃.
   Keyframes: reference with `T_cw = Identity`, current with
   `T_cw = scaled T_21`. Clear the reference; return the result.

`MapperOptions` replaces `min_mono_init_interval` with
`MapInitializerOptions initializer`.

## 4. Mapper publication — `mapping/mapper.h/.cpp`, events

- `KeyFrameCreated<FE>` gains `std::vector<uint32_t> observed_features` and
  `std::vector<map::MapPointId> observed_points` (same length; pairs) and a
  `Reset()` clearing both (capacity kept).
- On a successful initialization the Mapper (map `m = next_map_id_++`):
  1. Creates a fresh `MapPointStorage` and `ObservationStorage<Monocular>` for
     the map (point ids restart at 0).
  2. Adds each map point: position; descriptor = the first keyframe's
     descriptor of `feature_in_first`; normal = normalized mean of the unit
     viewing directions from the two camera centres (centre of keyframe k =
     translation of `T_cw.Inverse()`).
  3. Creates the keyframes (frame ids from `next_frame_id_`, never reused),
     records observations (point, frame, feature index) in the observation
     storage.
  4. Publishes, in order: one `MapPointsAdded` (`start_id` = 0, positions,
     descriptors, normals; `map_id = m`), then `KeyFrameCreated` per keyframe
     with its observations, then `MapCreated(m, second keyframe id, its T_cw)`.
- The stereo stub publishes no `MapPointsAdded` (no points) — unchanged.
- On `TrackingLost` for the current map the map point and observation storage
  are discarded with the keyframes.
- The info log line includes the point count:
  `Initializing -> Mapping (map 1, 2 keyframes, 312 points)`.

## 5. Testing

Test helper `tests/support/synthetic_sequence.h`: one 3D scene (points in
front of the first camera), N camera poses along a trajectory (translation and
rotation per frame configurable), each point projected into each frame through
a pinhole K (fx = fy = 190, c = (256, 256), 512×512) — producing a
`TMonocularMeasurement<OrbFeatureExtractor>` per frame with: undistorted
points (+ Gaussian pixel noise), level 0, a per-point base descriptor with a
few random bit flips per frame, per-point angles with small noise, and a few
random extra (unmatched) features. Seeded.

| Area | Tests |
|---|---|
| Levels | ORB levels in [0, n_levels), `ScaleFactor(l) = 1.2^l`; SIFT level decoding from packed octave/layer (unit test on the decode helper); `KeepFeatures` keeps levels aligned |
| `FeatureGrid` | `FeaturesInArea` equals brute force on random points (several radii/centres); level filter; points outside the image rectangle; empty input |
| TPose migration | all existing tests green; `Frame` doc states `T_cw` |
| `InitializationMatcher` | hand-built: window excludes far candidates; ratio rejection; one-to-one replacement; orientation filter removes an inconsistent match; `max_reference_level`; synthetic two frames: precision ≥ 95 % (recall reported) and `out_last_matched` updated |
| `MapInitializer` | synthetic sequence: no result while parallax is insufficient, then success; median depth of returned points == 1 (1e-9); ≤ `min_features` resets; < `min_matches` makes the current frame the reference; binocular stub unchanged |
| Mapper | publication order `MapPointsAdded` → `KeyFrameCreated` ×2 → `MapCreated`; every observation references an existing point id and a valid feature index; second map's point ids restart at 0 after `TrackingLost` |
| End to end (`Slam`, threads) | rewritten on the synthetic sequence: `MapCreated(1)` with points → `PositionUpdated` → forced loss (0-feature measurement) → further sequence frames → `MapCreated(2)`; also under ThreadSanitizer |
| Apps | smoke run of both apps on the 1024 dataset (no crash; log whether/when initialization happened) |

## 6. Documentation

- `DESIGN.md`: monocular initialization pipeline (matcher, MapInitializer,
  scaling, publication order), `T_cw` convention, observations on
  `KeyFrameCreated`, per-map point ids, keypoint levels and per-extractor
  scales; decisions log.
- `TODO.md`: Pangolin viewer (next); tuning on TUM-VI 512 (+ dropped-feature
  statistic); bundle adjustment after initialization; real stereo
  initialization; mark the done items (keypoint level, TPose migration).
