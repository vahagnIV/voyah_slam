// Compile-time checks that the components' thread-affine accessors are not
// public (DESIGN section 7, D-14, D-26): the check is the static_assert, so an
// accessor made public again fails the build, and the TEST only gives the file
// a place in the gtest report.

#include <gtest/gtest.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TTracker = tracking::Tracker<Extractor>;
using TLoopCloser = loop_closing::LoopCloser<Extractor>;
using TGlobalBa = global_ba::GlobalBundleAdjuster<Extractor>;
using TMapper = mapping::Mapper<Extractor>;

// Access is checked during constraint satisfaction, so a private member makes
// the requirement false instead of an error.
template <typename T>
concept HasAtlas = requires(const T& t) { t.atlas(); };
template <typename T>
concept HasMapReplica = requires(const T& t) { t.map_replica(); };
template <typename T>
concept HasLastMatches = requires(const T& t) { t.last_matches(); };
template <typename T>
concept HasNumCoincidences = requires(const T& t) { t.num_coincidences(); };
template <typename T>
concept HasNumMergeCoincidences = requires(const T& t) { t.num_merge_coincidences(); };
template <typename T>
concept HasLastStats = requires(const T& t) { t.last_stats(); };
template <typename T>
concept HasLastMergeStats = requires(const T& t) { t.last_merge_stats(); };
template <typename T>
concept HasQueryStats = requires(const T& t) { t.query_stats(); };
template <typename T>
concept HasNumRunsStarted = requires(const T& t) { t.num_runs_started(); };
template <typename T>
concept HasNumRunsInterrupted = requires(const T& t) { t.num_runs_interrupted(); };
template <typename T>
concept HasState = requires(const T& t) { t.state(); };
template <typename T>
concept HasCovisibility = requires(const T& t) { t.covisibility(); };
template <typename T>
concept HasReference = requires(const T& t, map::MapPointId point) { t.Reference(point); };
template <typename T>
concept HasNumVisible = requires(const T& t, map::MapPointId point) { t.NumVisible(point); };
template <typename T>
concept HasNumFound = requires(const T& t, map::MapPointId point) { t.NumFound(point); };
template <typename T>
concept HasLastLocalBa = requires(const T& t) { t.last_local_ba(); };
template <typename T>
concept HasBowVectorOf = requires(const T& t, map::FrameId frame) { t.BowVectorOf(frame); };
template <typename T>
concept HasParent = requires(const T& t, map::FrameId frame) { t.Parent(frame); };
template <typename T>
concept HasLoopEdges = requires(const T& t, map::FrameId frame) { t.LoopEdges(frame); };
template <typename T>
concept HasKeyFrame = requires(const T& t, map::FrameId frame) { t.KeyFrame(frame); };
template <typename T>
concept HasMapIds = requires(const T& t) { t.map_ids(); };
template <typename T>
concept HasActiveMap = requires(const T& t) { t.active_map(); };
template <typename T>
concept HasNumKeyframesOfMap = requires(const T& t, map::MapId id) { t.num_keyframes(id); };
template <typename T>
concept HasNumKeyframes = requires(const T& t) { t.num_keyframes(); };

static_assert(!HasAtlas<TTracker>, "Tracker::atlas() is Tracker-thread state");
static_assert(!HasMapReplica<TTracker>, "Tracker::map_replica() is Tracker-thread state");
static_assert(!HasLastMatches<TTracker>, "Tracker::last_matches() is Tracker-thread state");
static_assert(HasState<TTracker>, "Tracker::state() is atomic and stays public");

static_assert(!HasAtlas<TLoopCloser>, "LoopCloser::atlas() is LoopCloser-thread state");
static_assert(!HasMapReplica<TLoopCloser>, "LoopCloser::map_replica() is LoopCloser-thread state");
static_assert(!HasNumCoincidences<TLoopCloser>, "LoopCloser::num_coincidences() reads the pending region");
static_assert(!HasNumMergeCoincidences<TLoopCloser>,
              "LoopCloser::num_merge_coincidences() reads the pending merge region");
static_assert(!HasLastStats<TLoopCloser>, "LoopCloser::last_stats() returns worker-written vectors");
static_assert(!HasLastMergeStats<TLoopCloser>, "LoopCloser::last_merge_stats() returns worker-written vectors");
static_assert(!HasQueryStats<TLoopCloser>, "LoopCloser::query_stats() returns worker-written counters");

static_assert(!HasAtlas<TGlobalBa>, "GlobalBundleAdjuster::atlas() is GlobalBA-thread state");
static_assert(!HasMapReplica<TGlobalBa>, "GlobalBundleAdjuster::map_replica() is GlobalBA-thread state");
static_assert(HasNumRunsStarted<TGlobalBa>, "num_runs_started() is atomic and stays public");
static_assert(HasNumRunsInterrupted<TGlobalBa>, "num_runs_interrupted() is atomic and stays public");

static_assert(!HasCovisibility<TMapper>, "Mapper::covisibility() is Mapper-thread state");
static_assert(!HasReference<TMapper>, "Mapper::Reference() is Mapper-thread state");
static_assert(!HasNumVisible<TMapper>, "Mapper::NumVisible() is Mapper-thread state");
static_assert(!HasNumFound<TMapper>, "Mapper::NumFound() is Mapper-thread state");
static_assert(!HasLastLocalBa<TMapper>, "Mapper::last_local_ba() is Mapper-thread state");
static_assert(!HasBowVectorOf<TMapper>, "Mapper::BowVectorOf() is Mapper-thread state");
static_assert(!HasParent<TMapper>, "Mapper::Parent() is Mapper-thread state");
static_assert(!HasLoopEdges<TMapper>, "Mapper::LoopEdges() is Mapper-thread state");
static_assert(!HasKeyFrame<TMapper>, "Mapper::KeyFrame() is Mapper-thread state");
static_assert(!HasMapIds<TMapper>, "Mapper::map_ids() is Mapper-thread state");
static_assert(!HasActiveMap<TMapper>, "Mapper::active_map() is Mapper-thread state");
static_assert(!HasNumKeyframesOfMap<TMapper>, "Mapper::num_keyframes(MapId) is Mapper-thread state");
static_assert(HasNumKeyframes<TMapper>, "Mapper::num_keyframes() is atomic and stays public");

TEST(ThreadAffinityTest, RestrictedAccessorsAreNotPublic) {
    // The checks above are compile-time; reaching here means they hold.
    SUCCEED();
}

}  // namespace
}  // namespace voyahslam
