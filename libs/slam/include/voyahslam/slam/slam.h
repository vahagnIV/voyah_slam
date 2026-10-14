#ifndef VOYAHSLAM_SLAM_SLAM_H
#define VOYAHSLAM_SLAM_SLAM_H

#include <atomic>
#include <memory>
#include <optional>
#include <vector>
#include <voyahslam/errors.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::slam {

// Owns the Tracker, Mapper, LoopCloser and (with loop closing) GlobalBA and
// wires them asynchronously:
//   Tracker    → Mapper
//   Mapper     → Tracker, LoopCloser, GlobalBA
//   LoopCloser → Mapper
//   GlobalBA   → Mapper
// The six internal edges are held as Subscriptions in subscriptions_, which is
// destroyed before the components and after ~Slam() has stopped every one of
// them. Additional observers (e.g. a visualizer) are attached through
// tracker(), mapper(), loop_closer() and global_ba() before Start(); the caller
// keeps each returned Subscription, best in a container declared before the
// Slam, so it is destroyed after it (a no-op on a stopped producer). Not
// movable: subscriptions_ refer to the members' inboxes.
//
// Lifecycle (D-10, D-11): Created -> Running -> Stopped, Stopped is terminal; a
// new session is a new Slam. Start() twice or after Stop() throws UsageError;
// Stop() before Start() stops every component (so none can start later); Stop()
// is idempotent and every concurrent call returns only after all threads are
// joined. Both are safe from any thread (the destructor calls Stop()). A Start()
// that fails midway stops the components it started and rethrows, leaving the
// Slam Stopped. Track() after Stop() throws UsageError.
template <typename TFeatureExtractor>
class Slam {
public:
    using TTracker = tracking::Tracker<TFeatureExtractor>;
    using TMapper = mapping::Mapper<TFeatureExtractor>;
    using TLoopCloser = loop_closing::LoopCloser<TFeatureExtractor>;
    using TGlobalBundleAdjuster = global_ba::GlobalBundleAdjuster<TFeatureExtractor>;

    // `vocabulary` (may be null) goes to the Mapper, the Tracker and the LoopCloser; it is
    // immutable, so the threads share it without locks. Loop closing is opt-in: it needs the
    // vocabulary and `loop_closer_options.enabled` (off by default). Global BA after each loop
    // correction exists only with loop closing in effect (a vocabulary and
    // `loop_closer_options.enabled`) and `global_ba_options.enabled` (on by default): otherwise
    // no GlobalBundleAdjuster is created.
    explicit Slam(mapping::MapperOptions mapper_options = {},
                  tracking::TrackerOptions tracker_options = {},
                  std::shared_ptr<const typename TMapper::TVocabulary> vocabulary = nullptr,
                  loop_closing::LoopCloserOptions loop_closer_options = {},
                  global_ba::GlobalBaOptions global_ba_options = {});
    // Stops every component thread before any component is destroyed.
    ~Slam();
    Slam(const Slam&) = delete;
    Slam& operator=(const Slam&) = delete;

    // Starts consumers before producers: Mapper, GlobalBA, LoopCloser, Tracker.
    // Throws UsageError unless the Slam is Created.
    void Start();
    // Stops producers before consumers: Tracker, LoopCloser, GlobalBA, Mapper.
    // Idempotent; see the class comment.
    void Stop();
    // The Slam's own lifecycle state; any thread.
    [[nodiscard]] events::RunState run_state() const { return state_.load(std::memory_order_acquire); }

    [[nodiscard]] TTracker& tracker() { return tracker_; }
    [[nodiscard]] TMapper& mapper() { return mapper_; }
    [[nodiscard]] TLoopCloser& loop_closer() { return loop_closer_; }
    // Null without global BA (see the constructor).
    [[nodiscard]] TGlobalBundleAdjuster* global_ba() { return global_ba_ ? &*global_ba_ : nullptr; }
    [[nodiscard]] const TGlobalBundleAdjuster* global_ba() const {
        return global_ba_ ? &*global_ba_ : nullptr;
    }

private:
    TTracker tracker_;
    TMapper mapper_;
    TLoopCloser loop_closer_;
    std::optional<TGlobalBundleAdjuster> global_ba_;
    // The Slam's own lifecycle (control plane, not map data).
    std::atomic<events::RunState> state_{events::RunState::Created};
    // Last member: destroyed first, after ~Slam() has stopped every producer.
    std::vector<events::Subscription> subscriptions_;
};

} // namespace voyahslam::slam

#endif // VOYAHSLAM_SLAM_SLAM_H
