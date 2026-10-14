// Compile-time checks that the components' thread-affine accessors are not
// public (DESIGN section 7, D-14, D-26): the check is the static_assert, so an
// accessor made public again fails the build, and the TEST only gives the file
// a place in the gtest report.

#include <gtest/gtest.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TTracker = tracking::Tracker<Extractor>;
using TLoopCloser = loop_closing::LoopCloser<Extractor>;
using TGlobalBa = global_ba::GlobalBundleAdjuster<Extractor>;

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

TEST(ThreadAffinityTest, RestrictedAccessorsAreNotPublic) {
    // The checks above are compile-time; reaching here means they hold.
    SUCCEED();
}

}  // namespace
}  // namespace voyahslam
