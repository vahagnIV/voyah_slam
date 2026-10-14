#ifndef VOYAHSLAM_MAPPING_MAP_FOLLOWER_H
#define VOYAHSLAM_MAPPING_MAP_FOLLOWER_H

#include <cstddef>
#include <memory>
#include <thread>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/mapping/map_snapshot.h>
#include <voyahslam/tracking/atlas_replica.h>

namespace voyahslam::mapping {

// The supported way for an app to read the map from its own thread (D-13): an
// inbox plus a private AtlasReplica that the app brings up to date by calling
// Update() on its own thread. Nothing here writes the Mapper's map or shares
// mutable storage with another replica; the Mapper stays the only writer.
//
// Wiring: subscribe the inbox to the Mapper before the Mapper starts,
//   events::Subscription s = slam.mapper().AddObserver(follower.inbox());
// Because connections change only while the producer is stopped (D-05), a
// follower sees every transaction from the first one on; it cannot join a
// running Slam. Declare the Subscription before the Slam it connects to, and
// the follower before the Subscription (the inbox must outlive the wire).
//
// Threads: Update(), atlas() and Snapshot() belong to one thread, the one that
// makes the first Update() (checked by assert in debug builds of the library).
// Snapshot() returns immutable plain data that any thread may hold and read.
//
// Cost: a full AtlasReplica per follower (descriptors, BoW, covisibility) and
// an O(map) copy per Snapshot(). A follower that is never updated lets its
// inbox grow; the "consumer is behind" warning names it "MapFollower" (D-24).
//
// Defined in the library (map_follower.cpp, INSTANTIATE_CLASS) like the
// components, not in this header: a header-only follower made every app that
// uses it instantiate the replica code, and with it Eigen helpers the
// library's hot loops call. Built at -O0 and linked before the library, the
// app's copies replaced the library's optimized ones and the Debug-tree
// Tracker ran 3.5 times slower (decisions log 2026-10-14). Every member,
// trivial ones included, is therefore out of line; the build check
// BuildCheck.MapFollowerAddsNoLibraryInstantiations guards it.
template <typename TFeatureExtractor>
class MapFollower {
public:
    MapFollower();
    ~MapFollower();
    MapFollower(const MapFollower&) = delete;
    MapFollower& operator=(const MapFollower&) = delete;

    // Subscribe before the Mapper starts: Subscription s = mapper.AddObserver(follower.inbox());
    [[nodiscard]] events::AsyncObserver& inbox();

    // Applies every pending MapTransaction on the calling thread, in order;
    // other events are released. Returns the number of transactions applied by
    // this call (those the replica ignores, such as a removed map's, are not
    // counted).
    size_t Update();

    // The follower's own Atlas; same thread as Update(). Reading it calls the
    // replica's inline accessors from the caller's translation unit: apps read
    // a Snapshot() instead.
    [[nodiscard]] const tracking::AtlasReplica<TFeatureExtractor>& atlas() const;

    // A copy of the Atlas as it is now; same thread as Update(), but the result
    // is immutable and any thread may hold it. O(map).
    [[nodiscard]] std::shared_ptr<const MapSnapshot> Snapshot() const;

    // Transactions applied so far; same thread as Update().
    [[nodiscard]] size_t num_transactions_applied() const;

private:
    void ClaimThread();
    void AssertOwner() const;

    // The inbox first: the listener (and the events it holds) is destroyed before it.
    events::AsyncObserver inbox_;
    events::Listener listener_;
    tracking::AtlasReplica<TFeatureExtractor> atlas_;
    size_t num_applied_ = 0;
    std::thread::id owner_;  // the thread of the first Update(); empty before
};

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_MAP_FOLLOWER_H
