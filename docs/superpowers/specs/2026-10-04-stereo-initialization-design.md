# Stereo (Binocular) Map Initialization — Design Spec

Date: 2026-10-04
Status: approved in conversation, pending spec review

## Context and Scope

Monocular initialization is real (spec
`2026-10-04-mono-initialization-design.md`). Binocular initialization is
still a stub: one identity keyframe and no points. `BinocularMeasurement`
carries placeholder depths and right indices.

This spec adds left↔right stereo matching and a real stereo initializer
that builds a metric-scale map from one stereo frame, published like the
mono map.

**In scope**

1. Fix the `StereoRig` extrinsics convention: replace `R_rl`/`t_rl` with
   `RigidTransformation T_rl`, and fix the dataset reader and the test
   fixtures.
2. A stereo matcher (ORB-SLAM3 fisheye method): descriptor matching, then
   triangulation checks.
3. `BinocularMeasurement` stores the matches and a copy of its rig. The
   Tracker runs stereo matching on its own thread.
4. A stereo `MapInitializer` and Mapper storage/publication of stereo maps,
   with stereo observation storage.
5. Synthetic stereo test support and tests, an app smoke run, and docs.

**Out of scope (→ TODO.md):** epipolar-guided stereo search (an
optimization); stereo-aware tracking (using depths and right observations);
tuning on TUM-VI 512 (deferred to the very end).

**Success criteria**

1. On synthetic stereo, the matcher's precision is ≥ 98 %, and noise-free
   depths are exact (1e-6).
2. The stereo initializer builds one identity keyframe whose points lie
   near the true metric 3D points.
3. The Mapper publishes `MapPointsAdded` → `KeyFrameCreated` → `MapCreated`
   for stereo maps.
4. `binocular_full_slam <corridor1_1024> --viewer --max-frames=20`
   initializes within the first frames with hundreds of points and a
   plausible (corridor-scale) median depth.

## Decisions (from the conversation)

- Stereo matching runs in the Tracker thread, on dequeue, before the
  measurement is used or forwarded.
- Matcher: ORB-SLAM3 `ComputeStereoFishEyeMatches` style. It uses
  brute-force descriptor matching with a 0.7 ratio test, then triangulation
  checks. Rectified row search is unsuitable for fisheye. Epipolar-guided
  search is deferred.
- Stereo initialization follows ORB-SLAM3: one keyframe, and every valid
  stereo depth becomes a map point, at metric scale.
- Mixed mono/stereo keyframes stay possible later, so the Mapper keeps both
  a mono and a stereo observation storage.

## 1. Rig convention, stereo matcher, measurement storage

### 1.1 `StereoRig` (`features/stereo_rig.h`)

```cpp
struct StereoRig {
    geometry::TMatrix33F K_left;
    geometry::TMatrix33F K_right;
    geometry::RigidTransformation T_rl;  // left camera -> right camera: p_r = T_rl * p_l
    precision_t baseline;                // ‖T_rl.translation()‖
    const distortion_models::DistortionModel* distortion_left = nullptr;
    const distortion_models::DistortionModel* distortion_right = nullptr;
};
```

- The dataset reader (`examples/dataset_reader`) computes
  `T_rl = T_right_imu · T_left_imu⁻¹`, where `T_cam_imu` maps IMU to camera.
- Test fixtures put the right camera at +x, so
  `T_rl.translation() = (−b, 0, 0)`.

### 1.2 Stereo matcher (`features/stereo_matcher.h`, header-only template)

```cpp
struct StereoMatcherOptions {
    double nn_ratio = 0.7;
    double sigma_px = 1.0;               // level-0 keypoint noise (px)
    double level_scale = 1.2;            // σ_level = sigma_px · level_scale^level (ORB; SIFT: 2^(1/3))
    double max_chi2 = 5.991;             // per-image reprojection test, 2 dof
    double max_parallax_cos = 0.99998;   // rays more parallel than this → rejected
};
struct StereoMatches {
    Eigen::VectorXi right_indices;       // per left feature; −1 = none
    geometry::TRowVectorF depths;        // z in the left camera; −1 = none
    size_t num_matches = 0;
};
template <typename TFeatureExtractor>
[[nodiscard]] StereoMatches MatchStereo(const TBinocularMeasurement<TFeatureExtractor>& measurement,
                                        const StereoMatcherOptions& options);   // uses measurement.rig()
```

Algorithm:

1. **Descriptor match.** For each left feature, do a brute-force search
   over all right features for the best and second-best
   `TFeatureExtractor::Distance`. Keep the pair if best < `nn_ratio` ×
   second-best.
2. **One-to-one.** If a right feature is claimed twice, keep the claim with
   the smaller distance and drop the other.
3. **Triangulate.**
   - Left camera: `P_l = [I | 0]`. Right camera: `P_r = T_rl` as 3×4. The
     inputs are the undistorted homogeneous normalized points of both
     images.
   - Use `geometry::Triangulate` (DLT, `geometry/geometry.h`).
   - If triangulation fails, the pair is rejected.
4. **Accept** the pair if all of the following hold:
   - The point's z in the left camera and in the right camera
     (`T_rl.Transform(X)`) are both > 0.
   - For each image, the squared reprojection error in normalized
     coordinates, divided by `(σ_level / f)²`, is ≤ `max_chi2`. Here
     `f = K(0,0)` of that camera, and `σ_level` uses that image's keypoint
     level.
   - The cosine of the angle at X between the rays to the two camera
     centres is < `max_parallax_cos`.
5. **Record** `right_indices[i]` and `depths[i] = X.z()`. Because the left
   normalized point has z = 1, the 3D point is `depths[i] ·
   normalized_point(i)`.

### 1.3 `BinocularMeasurement`

- It stores a copy of the `StereoRig` it was built with: `const StereoRig&
  rig() const`. The distortion pointers still refer to app-owned models,
  as today.
- `void SetStereoMatches(StereoMatches&& matches)` replaces `depths_` and
  `right_indices_` and sets `stereo_matched_ = true`. The sizes must equal
  the number of left features (asserted).
- `bool stereo_matched() const`.
- The constructor still fills the placeholders (−1), with
  `stereo_matched() == false`.

### 1.4 Tracker

- `struct TrackerOptions { features::StereoMatcherOptions stereo_matcher; };`
- `Tracker(TrackerOptions options = {})`.
- `Slam(mapping::MapperOptions mapper_options = {},
  tracking::TrackerOptions tracker_options = {})`.
- When the Tracker dequeues a binocular measurement, it calls
  `MatchStereo(measurement, options.stereo_matcher)` and then
  `SetStereoMatches`. This happens before the measurement is tracked or
  forwarded as an `InitializationMeasurement`.
- Monocular measurements are untouched.

## 2. Stereo initialization and Mapper publication

### 2.1 `MapInitializer`, binocular path (replaces the stub)

`MapInitializerOptions` gains:

```cpp
size_t stereo_min_features = 500;   // left features (ORB-SLAM3: N > 500)
size_t stereo_min_points = 100;     // valid stereo depths required
```

1. A binocular measurement always drops any monocular reference (as
   today).
2. If the left image has ≤ `stereo_min_features` features, return nullopt.
3. Otherwise, collect the left features with a depth > 0. A measurement
   with `stereo_matched() == false` counts as having none (debug assert).
   If fewer than `stereo_min_points` remain, return nullopt.
4. Otherwise, return `InitializationResult` with:
   - one keyframe, `T_cw = Identity`;
   - one `InitialMapPoint` per valid depth i:
     `{position = depths[i] · normalized_point(i), feature_in_first = i,
     feature_in_second = −1}`.
   - The scale is metric (no median normalization).

`InitialMapPoint::feature_in_second == −1` means "no second keyframe" (a
documented sentinel).

### 2.2 Mapper

- **Point storage** is unchanged (per map).
  - Descriptor: the first keyframe's descriptor of `feature_in_first`.
    For a binocular keyframe this is the left image's descriptor.
  - Normal: the normalized mean of `normalize(X − C_k)` over the keyframes
    that observe the point. For a single stereo keyframe, that is its one
    viewing direction.
- **Observations.** Each map has two storages:
  - Mono keyframes go into `ObservationStorage<Monocular>`.
  - Stereo keyframes go into `ObservationStorage<Stereo>`, with
    `depth = depths[i]` and `right_feature_index = right_indices[i]`
    (`int16_t`; assert it fits).
  - Both storages are cleared on a new map and on `TrackingLost`.
- **`KeyFrameCreated`** observation pairs are unchanged: (feature index,
  point id) for the keyframe's left image. Depth and the right index are
  in the binocular measurement the event already carries.
- **Publication order** for every map: `MapPointsAdded` → `KeyFrameCreated`
  per keyframe → `MapCreated`. The stereo stub's "no points" case
  disappears. The "skip `MapPointsAdded` when there are 0 points" guard
  stays.
- The log line is unchanged in format (it already reports keyframes and
  points).

Unchanged: Tracker tracking (still the stub), the viewer, and the mono
path.

## 3. Testing, apps, docs

**Synthetic stereo support** (`tests/support/synthetic_sequence.h`):

- `SyntheticStereoOptions` and `MakeSyntheticStereoFrame(...)` (or a stereo
  sequence) use the same scene machinery as mono, with:
  - Intrinsics: fx = fy = 380, c = (512, 512), on a 1024² image, for both
    cameras.
  - Rig: `T_rl = (I, (−0.11, 0, 0))`.
  - Depths in [2, 6] m.
  - Independent 0.5 px noise per image.
  - Left and right descriptors that differ by a few bit flips.
  - Separate extra features per image.
- Ground truth: per left feature, the true right index (or −1) and the
  true depth.

| Area | Tests |
|---|---|
| `StereoRig` | `T_rl` maps a left-camera point into the right camera (right at +x → x decreases by b). Fixtures updated. |
| `MatchStereo`, hand-built | Ratio rejection; one-to-one (closer claim wins); a point behind the right camera is rejected; near-parallel rays are rejected; a geometrically inconsistent pair with a perfect descriptor is rejected by the reprojection test. |
| `MatchStereo`, synthetic | Noise-free: every true pair is found and its depth equals ground truth (1e-6). With 0.5 px noise: precision ≥ 98 %, median relative depth error < 10 %, recall reported. |
| `BinocularMeasurement` | `rig()` round-trips; `SetStereoMatches` stores the matches; `stereo_matched()` flips; a size mismatch asserts (death test optional). |
| Tracker | A binocular measurement is stereo-matched before the forwarded `InitializationMeasurement` (`stereo_matched()` true, depths match a direct `MatchStereo`). |
| `MapInitializer`, stereo | A synthetic frame gives 1 keyframe at identity. Points equal `depth · ray` and are within a few % of the true metric points. `feature_in_second == −1`. ≤ `stereo_min_features` → nullopt. < `stereo_min_points` → nullopt. An existing mono reference is dropped. |
| Mapper | Stereo publishes `MapPointsAdded` → `KeyFrameCreated` (observations valid, count = points) → `MapCreated`. Ids restart after `TrackingLost`. |
| `Slam` end to end | Binocular synthetic frames: `MapCreated` with points → `PositionUpdated` → forced loss → map 2. Also under ThreadSanitizer. |
| Apps (manual) | `binocular_full_slam <corridor1_1024> --viewer --max-frames=20`. Record the init frame, the point count, the median depth, and a viewer screenshot. |

Existing tests that build binocular measurements change as follows:

- Mapper, Slam and Tracker tests use random features with no geometry. A
  stereo-matched random measurement yields almost no depths, so these
  tests switch to the synthetic stereo frame wherever they expect a map.
- The stereo-stub expectations ("1 keyframe, 0 points") become "1 keyframe,
  ≥ `stereo_min_points` points".

**Docs.**

- `DESIGN.md`:
  - the stereo pipeline (matching in the Tracker → stereo `MapInitializer`
    → publication);
  - the `T_rl` convention;
  - the stereo observation storage;
  - decisions-log rows for the fisheye matcher, matching in the Tracker
    thread, and the rig convention fix.
- `TODO.md`:
  - remove "Stereo matching in `BinocularMeasurement`" and "Real stereo
    initialization";
  - add epipolar-guided stereo search and stereo-aware tracking.
