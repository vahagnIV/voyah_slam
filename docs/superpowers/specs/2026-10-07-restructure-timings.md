# Restructure baseline test timings

Date: 2026-10-07. Measured with `ctest -j1 --timeout 300`, per-test times from the JUnit output.

- (a) today's Debug build (`cmake-build-debug/build/Debug`, no optimisation).
- (b) the same Debug configuration with `CMAKE_CXX_FLAGS_DEBUG="-O2 -g"` (scratch tree, deleted).

## Totals

| Suite | Tests | (a) Debug | (b) Debug `-O2 -g` |
|---|---|---|---|
| top-level ctest (`voyahslam_tests` 447 + `dataset_reader_tests` 11) | 458 | 381.1 s | 14.8 s (447 tests; dataset_reader_tests not built in the scratch tree, 0.24 s in (a)) |
| `voyahviz_tests` | 43 | 0.41 s | 0.40 s |

Note: `voyahviz_tests` is not part of the top-level `ctest` run; it is run from `vizualization/tests`.

## Tests above 0.5 s in either build (seconds)

| Test | (a) | (b) |
|---|---|---|
| SlamTest.StereoMapGrowsWhileLeavingInitialView | 41.36 | 1.11 |
| TrackerTest.TracksSyntheticStereo | 38.67 | 1.26 |
| MapInitializerTest.BundleAdjustmentReachesOptimum | 25.28 | 0.37 |
| SlamTest.MonoMapGrowsWhileLeavingInitialView | 24.13 | 0.63 |
| TrackerTest.NewMapResetsTrackingState | 17.07 | 0.51 |
| SlamTest.EndToEndInitializeTrackLoseReinitialize | 16.00 | 0.31 |
| MapInitializerTest.PureRotationNeverInitializes | 15.98 | 0.24 |
| TrackerMonoAccuracyTest.TracksSyntheticMono | 12.55 | 0.26 |
| MapInitializerTest.TooFewPointsAfterBaFails | 11.63 | 0.18 |
| TrackerTest.TracksSyntheticMonoDefaultInit | 10.97 | 0.27 |
| MapperInboxTest.IgnoresFrameTrackedOfOldMap | 7.56 | 0.12 |
| TrackerTest.TeleportedFrameIsLost | 6.18 | 0.25 |
| MapperFrameCreationTest.NoFrameWhenStrong | 5.30 | 0.09 |
| MapperFrameCreationTest.SkipsInvalidAndDuplicatePoints | 5.11 | 0.09 |
| MapperFrameCreationTest.RefPointsUsesMinObs | 5.09 | 0.09 |
| MapperTest.PointIdsRestartAfterTrackingLost | 5.04 | 0.09 |
| MapperFrameCreationTest.CreatesFrameWhenWeak | 4.99 | 0.09 |
| MapperInboxTest.DrainsToLatest | 4.96 | 0.09 |
| MapperInboxTest.TrackingLostAfterFrameTrackedCreatesNoFrame | 4.95 | 0.09 |
| ReduceMatrixRank.Rect | 4.93 | 0.12 |
| MapperTest.MonoInitializesWithParallax | 4.93 | 0.08 |
| MapperTest.MonoPublishesKeyFramesWithPointsThenMap | 4.89 | 0.09 |
| MapperFrameCreationTest.MonoTriangulatesAgainstNeighbours | 4.82 | 0.09 |
| MapperFrameCreationTest.ReplicaStaysConsistent | 4.78 | 0.09 |
| RansacTest.SameSeedSameResult | 4.53 | 0.10 |
| RansacTest.RefinementImprovesRecoveryWith30PercentOutliers | 4.48 | 0.11 |
| TrackerTest.ThreadProcessesTrackedMeasurements | 4.46 | 0.16 |
| RansacTest.RefinementImprovesRecoveryWith50PercentOutliers | 4.43 | 0.10 |
| TwoViewReconstructionTest.Deterministic | 3.93 | 0.09 |
| ReduceMatrixRank.Square | 3.83 | 0.10 |
| SlamTest.EndToEndStereoInitializeTrackLoseReinitialize | 2.98 | 0.16 |
| TrackerTest.FrameTrackedCarriesLocalMap | 2.84 | 0.06 |
| MapperTest.StereoMapAfterMonoMap | 2.73 | 0.07 |
| MapInitializerTest.FewMatchesReplacesReferenceEvenWithEnoughParallax | 2.58 | 0.05 |
| MapInitializerTest.MedianDepthIsOne | 2.58 | 0.05 |
| MapInitializerTest.MonoInitializesOnceParallaxSuffices | 2.57 | 0.04 |
| ProjectionMatcherTest.SyntheticPrecision | 2.52 | 0.05 |
| MapInitializerTest.FewFeaturesResetsReference | 2.51 | 0.05 |
| MapInitializerTest.ResetDropsReference | 2.49 | 0.05 |
| MapperTest.MapCreatedCarriesSecondKeyFramePose | 2.48 | 0.04 |
| MapperTest.FewFeaturesReplacesReference | 2.46 | 0.05 |
| TwoViewReconstructionTest.GeneralSceneSelectsEssential | 2.36 | 0.05 |
| RansacTest.RecoversEssentialWith50PercentOutliers | 2.27 | 0.05 |
| RansacTest.RecoversEssentialWith30PercentOutliers | 2.27 | 0.06 |
| TrackerTest.IgnoresStaleMapCreated | 1.82 | 0.11 |
| TrackerTest.TracksAndPublishesFrameTracked | 1.74 | 0.08 |
| TrackerTest.MergesOwnFrameObservationsIntoLastFrame | 1.71 | 0.08 |
| TwoViewReconstructionTest.PlanarSceneSelectsHomography | 1.23 | 0.03 |
| ObjectPoolDeathTest.PoolExhaustion | 1.09 | 1.09 |
| ObjectPoolTest.ConcurrentCreateRecycleNeverHandsOutSameSlotTwice | 0.98 | 0.92 |
| SolveHomogeneousSystem.RectRankDeficient | 0.90 | 0.06 |
| BundleAdjustTest.FisheyeRecoversPerturbedNoiseFree | 0.79 | 0.02 |
| MapperFrameCreationTest.StereoNeighbourObservationCarriesDepth | 0.78 | 0.09 |
| SolveHomogeneousSystem.SquareRankDeficient | 0.70 | 0.06 |
| BundleAdjustTest.RobustFlagChangesCost | 0.68 | 0.02 |
| BundleAdjustTest.PinholeInvariance | 0.67 | 0.02 |
| TwoViewReconstructionTest.NarrowFrontalPlanarSceneRejected | 0.65 | 0.02 |
| OrbLevelsTest.LevelsInRangeAndScaleFactor | 0.62 | 0.05 |
| SolveHomogeneousSystem.RectRankFull | 0.61 | 0.04 |
| BundleAdjustTest.FlagsOutliersAndStaysAccurate | 0.61 | 0.02 |
| TwoViewReconstructionTest.TinyBaselineRejected | 0.58 | 0.01 |
| SlamTest.ForwardsTrackerOptions | 0.56 | 0.34 |
| BundleAdjustTest.SingleObservationPointStaysFinite | 0.52 | 0.01 |
| BundleAdjustTest.RecoversPerturbedNoiseFree | 0.52 | 0.01 |
| TwoViewReconstructionTest.PureRotationRejected | 0.51 | 0.01 |
| TrackerTest.NewMapAfterLoss | 0.51 | 0.06 |

## Integration list (`voyah_integration_tests`)

Rule: every `SlamTest.*` test, plus every test above 1 s in (b), except
`ObjectPoolDeathTest.PoolExhaustion` (Ruling E: it stays `unit`). 10 tests.

| Test | (b) s | Selected by |
|---|---|---|
| SlamTest.EndToEndInitializeTrackLoseReinitialize | 0.31 | SlamTest.* |
| SlamTest.EndToEndStereoInitializeTrackLoseReinitialize | 0.16 | SlamTest.* |
| SlamTest.ExternalObserversCanBeAddedToEveryComponent | 0.01 | SlamTest.* |
| SlamTest.ForwardsTrackerOptions | 0.34 | SlamTest.* |
| SlamTest.MonoMapGrowsWhileLeavingInitialView | 0.63 | SlamTest.* |
| SlamTest.StartStopAndDestroy | 0.01 | SlamTest.* |
| SlamTest.StereoMapGrowsWhileLeavingInitialView | 1.11 | SlamTest.* and >1 s |
| SlamTest.TrackEnqueuesMonocularAndBinocularMeasurements | 0.01 | SlamTest.* |
| SlamTest.WiresComponentsAsynchronously | 0.01 | SlamTest.* |
| TrackerTest.TracksSyntheticStereo | 1.26 | >1 s in (b) |

Notes:
- `ObjectPoolDeathTest.PoolExhaustion` (1.09 s in both builds) is above 1 s in (b) but stays
  `unit` (Ruling E): it is a fixed-cost death test, not compute-bound and not a pipeline test.
- `TrackerTest.TracksSyntheticStereo` is the only Tracker/Mapper test above 1 s in (b). Other threaded Tracker/Mapper sequences (spec §3) are for the Task owner to pick by name from the 0.5 s table above (all are under 0.5 s in (b) except `TrackerTest.NewMapResetsTrackingState`, 0.51 s).
