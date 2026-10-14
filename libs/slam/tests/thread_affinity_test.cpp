// Compile-time checks that the components' thread-affine accessors are not
// public (DESIGN section 7, D-14, D-26): the check is the static_assert, so an
// accessor made public again fails the build, and the TEST only gives the file
// a place in the gtest report.

#include <gtest/gtest.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TTracker = tracking::Tracker<Extractor>;

// Access is checked during constraint satisfaction, so a private member makes
// the requirement false instead of an error.
template <typename T>
concept HasAtlas = requires(const T& t) { t.atlas(); };
template <typename T>
concept HasMapReplica = requires(const T& t) { t.map_replica(); };
template <typename T>
concept HasLastMatches = requires(const T& t) { t.last_matches(); };
template <typename T>
concept HasState = requires(const T& t) { t.state(); };

static_assert(!HasAtlas<TTracker>, "Tracker::atlas() is Tracker-thread state");
static_assert(!HasMapReplica<TTracker>, "Tracker::map_replica() is Tracker-thread state");
static_assert(!HasLastMatches<TTracker>, "Tracker::last_matches() is Tracker-thread state");
static_assert(HasState<TTracker>, "Tracker::state() is atomic and stays public");

TEST(ThreadAffinityTest, RestrictedAccessorsAreNotPublic) {
    // The checks above are compile-time; reaching here means they hold.
    SUCCEED();
}

}  // namespace
}  // namespace voyahslam
