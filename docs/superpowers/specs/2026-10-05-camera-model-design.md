# Camera Model: Residuals and Gates in Raw-Pixel Space — Design Spec

Date: 2026-10-05
Status: approved in conversation, pending spec review

## Context and Scope

Two diagnostic spikes on corridor1 (TUM-VI fisheye, Kannala-Brandt) found
why mono tracking loses its map after 1–11 frames (2026-10-05, branch
`spike/mono-tracking-diag`, not merged).

**Cause.** Search windows and noise models live in *undistorted* pixel or
normalized space. The fisheye periphery is stretched there: median stretch
5.8× at 60–85°.

- Only 29 % of findable points land in the 2.5 px window.
- At 60–85°, `OptimizePose` rejects about half of the correct matches.
- Widening windows alone changes nothing. Measuring the error in distorted
  pixels as well brings every diagnosed frame above the 30-inlier gate.

| Variant | Mono tracked frames / re-initializations (400 frames) |
|---|---|
| Baseline | 32 / 7 |
| Window and σ scaled by the Kannala-Brandt stretch | 53 / 5 |
| …plus local-map gate 20 | 108 / 5 |

ORB-SLAM3 projects, searches and measures residuals in the raw (distorted)
image through its camera model. This spec does the same, for every residual
and gate in the library.

**In scope**

1. Move `DistortionModel` (and the Dummy and Kannala-Brandt models) into
   `geometry/`, with forwarding headers at the old path. Add the analytic
   `DistortJacobian`.
2. A `geometry::Camera` value type: K plus a distortion pointer, with
   `Project`, `ProjectJacobian` and `Unproject`.
3. Measurements keep the raw keypoints (`distorted_points()`) and expose
   `camera()`.
4. Five consumers move to raw-pixel space, with σ = `sigma_px · 1.2^level`
   in pixels:
   - the projection matcher;
   - `OptimizePose`;
   - `BundleAdjust` (initialization);
   - the stereo matcher's reprojection gate;
   - the initialization matcher's windows.

   Rays stay rays: two-view RANSAC, triangulation and the 85° ray test are
   unchanged.
5. Tests, smoke runs and docs.

**Out of scope (→ TODO.md):**

- Full camera-model classes (ORB-SLAM3 `GeometricCamera`). This was approach
  B in the conversation.
- Keeping features beyond 85°, using bearing vectors.
- Tuning: the local-map inlier gate, and a larger mono initial map.
- Keyframes and map growth (tracking sub-project 2), which is the next step.

**Success criteria**

1. `DistortJacobian` and `Camera::ProjectJacobian` match finite
   differences. The `Project` / `Unproject` round trip holds up to 85°.
2. **Pinhole invariance.** With a null distortion, every consumer's χ²,
   inlier decisions and results equal today's (1e-9). The existing pinhole
   synthetic tests pass with only call-site updates.
3. On fisheye synthetic scenes, points up to 80° off-axis are recovered:
   - `OptimizePose` (mono and stereo) and `BundleAdjust` recover ground truth;
   - the projection matcher finds peripheral points;
   - the stereo gate accepts true peripheral pairs.
4. **corridor1.**
   - Mono tracks at least as many frames as the spike's Kannala-Brandt
     variant with default gates: ≥ 53 tracked frames per 400, ≤ 5
     re-initializations.
   - Stereo still tracks all 200 frames.
   - The 20-inlier gate is reported, as a measurement only.
5. ThreadSanitizer is clean, and the clang-18/libc++ check is clean.

## Decisions (from the conversation)

- **Camera fix first,** before keyframes (sub-project 2).
- **Measurements keep both** the distortion model pointer and the raw
  distorted keypoints.
- **All five consumers move** to raw-pixel space, so the library has one
  noise model.
- **Approach A:** a small `geometry::Camera` value type (K + distortion
  pointer); not full camera classes, and not free functions.
- **`DistortionModel` moves to `geometry/`,** so `geometry` stays at the
  bottom of the dependency stack.
- **The calibration-free angle allowance** (σ · 1.5 · (1 + r²), window ·
  (1 + r²); spike 2 measured it within 0.6 % of the Kannala-Brandt stretch
  up to 60°) is recorded as a fallback for lenses without a model. It is not
  implemented.

## 1. `Camera`, the distortion Jacobian, the measurement

### 1.1 `DistortionModel` in `geometry/`

**Moves:**

| From | To |
|---|---|
| `include/voyahslam/features/distortion_models/*.h` | `include/voyahslam/geometry/distortion_models/` |
| `src/features/distortion_models/*.cpp` | `src/geometry/distortion_models/` |

- Namespace: `voyahslam::geometry::distortion_models`.
- The old headers become forwarding headers. They include the new one and
  add `namespace voyahslam::features { namespace distortion_models =
  geometry::distortion_models; }`, so existing includes and qualified names
  keep compiling.
- In-repo users are updated to the new path.
- Tests move to `tests/geometry/distortion_models/`.

**New pure virtual:**

```cpp
// d(distorted normalized) / d(undistorted normalized) at an undistorted
// normalized point (z = 1 plane).
[[nodiscard]] virtual geometry::TMatrix22F DistortJacobian(const geometry::TPoint2DF& undistorted) const = 0;
```

- `DummyDistortionModel` returns the identity.
- `KannalaBrandtDistortionModel` uses the analytic form:
  - r = ‖p‖, θ = atan r, d(θ) = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸), and the
    distorted point is (d/r)·p.
  - J = (d/r)·I + (d′(θ)·θ′(r) − d/r)·p̂p̂ᵀ, with p̂ = p/r (corrected during implementation, Ruling U),
    θ′(r) = 1/(1 + r²), and
    d′(θ) = 1 + 3k1θ² + 5k2θ⁴ + 7k3θ⁶ + 9k4θ⁸.
  - For r < 1e-8, J = I (the limit).

### 1.2 `geometry::Camera`

Header `include/voyahslam/geometry/camera.h`, source
`src/geometry/camera.cpp`.

```cpp
struct Camera {
    TMatrix33F K;
    const distortion_models::DistortionModel* distortion = nullptr;  // null = pinhole; must outlive every Camera copy

    // Camera-frame point → raw (distorted) pixel. Precondition: Xc.z() > 0.
    [[nodiscard]] TPoint2DF Project(const TPoint3DF& Xc) const;
    // d(raw pixel)/d(Xc), 2×3: K₂ₓ₂ · DistortJacobian(Xc.xy/Xc.z) · [[1/z, 0, −x/z²], [0, 1/z, −y/z²]].
    [[nodiscard]] Eigen::Matrix<precision_t, 2, 3> ProjectJacobian(const TPoint3DF& Xc) const;
    // Raw pixel → undistorted normalized ray (z = 1), via K⁻¹ and Undistort.
    [[nodiscard]] THomogenousPointF Unproject(const TPoint2DF& pixel) const;
};
```

`K₂ₓ₂` is K's upper-left 2×2 block, which includes the skew. The pixel is
`K · [distorted normalized; 1]`.

### 1.3 Measurements

`MonocularMeasurement` additions:

- `const geometry::TPoints2DF& distorted_points() const`: the raw keypoints
  as passed to the constructor. They are compacted together with all other
  arrays when features beyond 85° are dropped.
- `const geometry::Camera& camera() const`: `{K, distortion_model}`, stored
  at construction. A null model means pinhole.

`points()` (undistorted pixels) and `normalized_points()` stay as they are.
`BinocularMeasurement` needs no API change: it already has `left().camera()`,
`right().camera()` and `rig().T_rl`.

**Pointer rule.** The distortion model must outlive every measurement and
event copy. App calibration objects already live for the whole run. This is
added to DESIGN's pointer rule.

## 2. Consumers in raw-pixel space

All residuals and gates compare raw pixels with σ = `sigma_px · 1.2^level`
(pixels). The χ² thresholds are unchanged: 5.991 (2 dof) and 9.488 (4 dof).

1. **Projection matcher** (`features/projection_matcher.h`).
   - Projects with `current.camera().Project(Xc)`.
   - The bounding box and `FeatureGrid` use `distorted_points()`.
   - Windows are in raw pixels; the values are unchanged.
   - The ray-angle, normal, level and descriptor logic is unchanged.
2. **`OptimizePose`** (`optimization/pose_optimization.h`).

   ```cpp
   struct PoseObservation { TPoint3DF point; TPoint2DF left; std::optional<TPoint2DF> right; double sigma; }; // raw px, px
   struct StereoCameras { geometry::Camera right; geometry::RigidTransformation T_rl; };
   PoseOptimizationResult OptimizePose(const RigidTransformation& initial_T_cw, const geometry::Camera& left,
                                       const std::optional<StereoCameras>& stereo,
                                       const std::vector<PoseObservation>& observations,
                                       const PoseOptimizationOptions& options);
   ```

   - Mono: e = `left.Project(T_cw·X) − z`, and
     J = `left.ProjectJacobian(Xc)·[−[Xc]× | I]`.
   - Stereo rows: `right.Project(T_rl·Xc)`, and
     J = `right.ProjectJacobian(Xr)·R_rl·[−[Xc]× | I]`.
   - Positive depth is checked in both cameras, as today.
   - Invalid input: a `right` observation without `stereo` → `success =
     false`.
   - The helpers in `src/optimization/projection.h` take a `Camera`; the
     normalized-plane helpers are removed. The FD tests are updated.
3. **`BundleAdjust`** (`optimization/bundle_adjustment.h`).
   - `BaProblem` gains `std::vector<geometry::Camera> keyframe_cameras`, one
     per keyframe, validated.
   - `BaObservation::measurement` is in raw pixels and `sigma` in pixels.
   - Point block: `ProjectJacobian · R`.
   - The two-stage logic and the rulings (N, O, O′) are unchanged.
   - `MapInitializer` builds the observations from `distorted_points()` and
     each view's `camera()`.
4. **Stereo matcher gate** (`features/stereo_matcher.h`).
   - Triangulation stays on rays.
   - The gate projects the triangulated point with each image's `camera()`
     and compares it with `distorted_points()`, using σ = `sigma_px ·
     level_scale^level`.
5. **Initialization matcher** (`features/initialization_matcher.h`).
   - The grid is built on the current frame's `distorted_points()`.
   - `out_last_matched` starts as the reference's `distorted_points()` and
     is updated in raw pixels.
   - The window (100 px) is ORB-SLAM3's raw-pixel value.
6. **Tracker.**
   - Builds `PoseObservation`s from `distorted_points()`, with sigma in
     pixels.
   - Passes `left().camera()`, and for stereo
     `StereoCameras{right().camera(), rig().T_rl}`.
   - `TrackerOptions::sigma_px` keeps its meaning, now in raw pixels.

## 3. Testing, verification, docs

| Area | Tests |
|---|---|
| `DistortJacobian` | KB with TUM-VI coefficients (from the dataset's calibration, or the existing KB test's values): equals central FD (1e-7) at r = 0, 1e-6, 0.1, 1, tan 80°, and 10 random points. Dummy model: identity. |
| `Camera` | `Unproject(Project(Xc))` ∥ Xc (angle < 1e-6 rad) for angles 0–85° with KB; exact for pinhole. `ProjectJacobian` = FD (1e-6). Pinhole `Project` = (K·Xc).hnormalized(). |
| Measurement | `distorted_points()` equals the input. A fisheye case where features beyond 85° are dropped keeps everything aligned. `camera()` round-trips. |
| Pinhole invariance | `OptimizePose` and `BundleAdjust` on a pinhole problem: the new pixel formulation equals the old normalized one (pose, χ², inliers; 1e-9). The existing synthetic tests for all five consumers pass with call-site updates only. |
| Fisheye | Synthetic scenes through KB (TUM-VI coefficients), points up to 80°: `OptimizePose` mono/stereo and `BundleAdjust` recover ground truth (1e-6 noise-free). The projection matcher matches peripheral points that the undistorted-window version misses. The stereo gate accepts true peripheral pairs and rejects pairs shifted by 5 raw px. |
| ThreadSanitizer | `Tracker*:Slam*:Mapper*`. |
| Smoke | corridor1, Release. Mono, 400 frames: tracked frames per map, total and re-initializations, against baseline 32/7 and the spike's KB variants. Also report the local-map gate 20 as a measurement only. Stereo, 200 frames: all tracked, median inliers. Init frame and point counts, mono and stereo. |

**Docs**

- `DESIGN.md`:
  - `Camera` and the residual-space rule: raw pixels for residuals and gates,
    rays for geometry.
  - `DistortionModel` in `geometry/`.
  - The measurement fields.
  - The pointer rule.
  - Decisions-log rows with the spikes' numbers.
- `TODO.md`:
  - Remove the pixel-space residual item.
  - Add: the angle-allowance fallback note; keeping features beyond 85°
    (bearings); local-map gate / init map size tuning; camera-model classes
    (approach B) if ever needed.
