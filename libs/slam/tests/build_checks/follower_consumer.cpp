// A translation unit shaped like an app that reads the map through a
// MapFollower (examples/full_slam), built at -O0 like an app in the Debug tree.
// With VOYAH_CONSUMER_USES_FOLLOWER=0 it is the same app without the follower:
// a plain inbox on the Mapper and a snapshot of its own. The function is never
// called; check_consumer_instantiations.cmake only inspects the two objects.
//
// Why (DESIGN "Map Follower", decisions log 2026-10-14): a function the -O0 app
// object instantiates and the -O2 library also instantiates exists twice, and
// the linker keeps the app's copy (static libraries, the app object first), so
// the library's own calls run it unoptimized. A header-only MapFollower
// instantiated the replica code, and with it the Eigen descriptor copy that
// MatchStereo calls about two million times per stereo frame: the Tracker
// became 3.5 times slower in the Debug-tree apps.

#include <cstddef>
#include <memory>
#include <ostream>
#include <vector>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/mapping/map_follower.h>
#include <voyahslam/mapping/map_snapshot.h>
#include <voyahslam/slam/slam.h>

namespace voyahslam::build_checks {

std::size_t RunConsumer(std::ostream& out) {
    using Extractor = features::OrbFeatureExtractor;
#if VOYAH_CONSUMER_USES_FOLLOWER
    mapping::MapFollower<Extractor> follower;
#else
    events::AsyncObserver inbox("Consumer");
#endif
    std::vector<events::Subscription> subscriptions;
    slam::Slam<Extractor> slam;
#if VOYAH_CONSUMER_USES_FOLLOWER
    subscriptions.push_back(slam.mapper().AddObserver(follower.inbox()));
#else
    subscriptions.push_back(slam.mapper().AddObserver(inbox));
#endif
    slam.Start();
    slam.Stop();
#if VOYAH_CONSUMER_USES_FOLLOWER
    follower.Update();
    const std::shared_ptr<const mapping::MapSnapshot> snapshot = follower.Snapshot();
#else
    const std::shared_ptr<const mapping::MapSnapshot> snapshot = std::make_shared<const mapping::MapSnapshot>();
#endif
    // Read the snapshot the way the apps print the Atlas.
    std::size_t num_frames = 0;
    for (const mapping::MapSnapshot::Map& map : snapshot->maps) {
        out << map.id.value << " " << map.frame_ids.size() << (snapshot->active_map == map.id ? " active" : "")
            << "\n";
        num_frames += map.frame_ids.size();
    }
    return num_frames + snapshot->num_transactions_applied;
}

}  // namespace voyahslam::build_checks
