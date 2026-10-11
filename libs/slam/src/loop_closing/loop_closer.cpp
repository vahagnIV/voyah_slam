#include <algorithm>
#include <chrono>
#include <optional>
#include <utility>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/loop_closing/loop_closer.h>

#include "loop_closing/common_region_internal.h"

namespace voyahslam::loop_closing {

using detail::FixesScale;
using detail::IsValid;
using detail::PointMatches;
using detail::PointsOf;
using detail::ToSim3;

template <typename TFeatureExtractor>
LoopCloser<TFeatureExtractor>::~LoopCloser() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Stop() {
    const bool was_running = worker_.running();
    worker_.Stop();
    if (was_running && query_stats_.detected > 0) {  // once per run
        log_->info("Loop detection: {} queries, {:.1f} ms mean, {:.1f} ms max; {} queries and {} loops dropped",
                   query_stats_.detected,
                   1e3 * query_stats_.total_seconds / static_cast<double>(query_stats_.detected),
                   1e3 * query_stats_.max_seconds, query_stats_.dropped, num_dropped_loops_);
    }
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ProcessPending() {
    // Every event is applied on receipt and so released at once: however far
    // detection lags, it holds none of the Mapper's pooled transactions. Only
    // the queries (Frame ids) wait.
    bool processed = false;
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;  // acknowledged by the next GetNextEvent()
        if (vocabulary_ && options_.enabled) {
            ApplyEvent(*event);
        }
    }
    if (queries_.size() > options_.max_query_backlog) {
        const size_t dropped = queries_.size() - options_.max_query_backlog;
        if (!dropping_queries_) {  // once until the backlog empties
            log_->warn("Loop detection behind: queries beyond {} dropped, from Frame {} ({} so far)",
                       options_.max_query_backlog, queries_.front().value, query_stats_.dropped + dropped);
        }
        dropping_queries_ = true;
        queries_.erase(queries_.begin(), queries_.begin() + static_cast<std::ptrdiff_t>(dropped));
        query_stats_.dropped += dropped;
    }
    if (queries_.empty()) {
        dropping_queries_ = false;
        return processed;
    }
    // One query per call: the inbox is drained again before the next, and
    // Stop() waits for this one only.
    const map::FrameId query = queries_.front();
    queries_.pop_front();
    DetectQuery(query);
    return true;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    // Only the Mapper's transactions build the Atlas; a TrackingLost changes
    // nothing (the Mapper answers a final one with a transaction).
    if (event.type != events::EventType::MapTransaction) {
        return;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    const std::optional<map::MapId> previous = atlas_.active_map();
    if (!atlas_.Apply(transaction)) {
        return;  // an unknown or removed map's
    }
    const std::optional<map::MapId> active = atlas_.active_map();
    if (transaction.new_map || active != previous) {
        pending_.reset();  // a new active map, or none
        pending_merge_.reset();
        queries_.clear();
    }
    // A pending region's Sim3 is in its match map's coordinates: a correction
    // of that map (a higher epoch) or its removal drops it.
    for (std::optional<PendingRegion>* pending : {&pending_, &pending_merge_}) {
        if (*pending) {
            const tracking::MapReplica<TFeatureExtractor>* match_map = atlas_.map((*pending)->match_map);
            if (match_map == nullptr || match_map->map_epoch() != (*pending)->match_epoch) {
                pending->reset();
            }
        }
    }
    if (transaction.map_id != active) {
        return;  // queries are Frames of the active map only
    }
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        if (replica().keyframe(added.id) != nullptr) {
            queries_.push_back(added.id);
        }
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectQuery(map::FrameId query) {
    // A query culled while it waited is skipped; so is one of a small map.
    if (replica().keyframe(query) == nullptr || replica().num_keyframes() < options_.min_map_frames) {
        return;
    }
    if (query_hook_) {
        query_hook_(query);
    }
    const auto start = std::chrono::steady_clock::now();
    DetectRegions(query);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ++query_stats_.detected;
    query_stats_.total_seconds += seconds;
    query_stats_.max_seconds = std::max(query_stats_.max_seconds, seconds);
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectRegions(map::FrameId query) {
    // ORB-SLAM3 NewDetectCommonRegions: each pending region is first checked
    // against this query; unless one is then confirmed, the database is
    // searched for the regions this query did not confirm.
    const bool loop_in_query = ConfirmPending(pending_, query);
    const bool merge_in_query = ConfirmPending(pending_merge_, query);
    if (!Confirmed(pending_) && !Confirmed(pending_merge_) && !(loop_in_query && merge_in_query)) {
        const RegionCandidates candidates = DetectNBestCandidates(atlas_, query, options_);
        if (!loop_in_query) {
            DetectPending(query, candidates.loop, "loop", pending_, last_stats_);
        }
        if (!merge_in_query) {
            DetectPending(query, candidates.merge, "merge", pending_merge_, last_merge_stats_);
        }
    }
    // A merge first; a loop confirmed at the same query is dropped.
    if (Confirmed(pending_merge_)) {
        PublishLoop(query, *pending_merge_);
        pending_merge_.reset();
        if (Confirmed(pending_)) {
            log_->debug("Loop region of Frame {} dropped for the merge at Frame {}", pending_->match_frame.value,
                        query.value);
            pending_.reset();
        }
    } else if (Confirmed(pending_)) {
        PublishLoop(query, *pending_);
        pending_.reset();
    }
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ConfirmPending(std::optional<PendingRegion>& out_pending, map::FrameId query) {
    if (!out_pending || out_pending->num_coincidences == 0) {
        return false;
    }
    // The last confirming query's Sim3 moved by their relative pose.
    const tracking::MapReplica<TFeatureExtractor>* match_map = atlas_.map(out_pending->match_map);
    const auto* last = replica().keyframe(out_pending->last_query);
    const auto* current = replica().keyframe(query);
    std::optional<CommonRegion> confirmed;
    if (match_map != nullptr && last != nullptr && current != nullptr) {
        const geometry::Sim3 S_query_world =
            geometry::Compose(ToSim3(current->T_cw * last->T_cw.Inverse()), out_pending->S_last_world);
        CommonRegionStats refine_stats;
        confirmed = RefineCommonRegion(replica(), query, *match_map, out_pending->match_frame, S_query_world, options_,
                                       &refine_stats);
        LogNormalRejections(refine_stats);
    }
    if (confirmed) {
        ++out_pending->num_coincidences;
        out_pending->last_query = query;
        out_pending->S_last_world = confirmed->S_query_world;
        out_pending->matches = std::move(confirmed->matches);
        out_pending->num_not_found = 0;
        // A merge's normals are not gated; their share is logged for
        // diagnosis.
        log_->debug("{} region of Frame {} confirmed by Frame {} ({}/{}), {:.2f} of {} normals disagree",
                    out_pending->match_map == replica().map_id() ? "Loop" : "Merge", out_pending->match_frame.value,
                    query.value, out_pending->num_coincidences, options_.required_coincidences,
                    confirmed->normals.disagreement, confirmed->normals.pairs);
        return true;
    }
    if (match_map == nullptr || last == nullptr || ++out_pending->num_not_found >= options_.max_unconfirmed_queries) {
        out_pending.reset();
    }
    return false;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectPending(map::FrameId query, const std::vector<RegionCandidate>& candidates,
                                                  const char* kind, std::optional<PendingRegion>& out_pending,
                                                  CommonRegionStats& out_stats) {
    const CommonRegionResult result = DetectCommonRegion(atlas_, query, candidates, options_);
    out_stats = result.stats;
    LogNormalRejections(result.stats);
    if (!result.region) {
        return;
    }
    const CommonRegion& region = *result.region;
    log_->debug("Frame {} shares a {} region with Frame {} of map {}: {} matches, {} confirmations, {:.2f} of {} "
                "normals disagree",
                query.value, kind, region.match_frame.value, region.match_map.value, region.num_matches,
                region.num_coincidences, region.normals.disagreement, region.normals.pairs);
    out_pending = PendingRegion{region.match_frame,
                                region.match_map,
                                atlas_.map(region.match_map)->map_epoch(),
                                query,
                                region.S_query_world,
                                region.matches,
                                region.num_coincidences,
                                0};
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::LogNormalRejections(const CommonRegionStats& stats) {
    for (const NormalRejection& rejection : stats.normal_rejections) {
        log_->debug("Normal gate: Frame {} against the region of Frame {} rejected ({:.2f} of {} pairs disagree)",
                    rejection.frame.value, rejection.match_frame.value, rejection.agreement.disagreement,
                    rejection.agreement.pairs);
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::PublishLoop(map::FrameId query, const PendingRegion& region) {
    const tracking::MapReplica<TFeatureExtractor>* match_map = atlas_.map(region.match_map);
    const auto* query_keyframe = replica().keyframe(query);
    const auto* match_keyframe = match_map != nullptr ? match_map->keyframe(region.match_frame) : nullptr;
    if (query_keyframe == nullptr || match_keyframe == nullptr) {
        return;
    }
    geometry::Sim3 S_query_match = geometry::Compose(region.S_last_world, ToSim3(match_keyframe->T_cw.Inverse()));
    if (FixesScale(*query_keyframe)) {
        S_query_match.scale = 1.0;
    }
    if (!IsValid(S_query_match)) {
        return;
    }
    auto* event = loop_detected_pool_.TryCreate();
    if (event == nullptr) {
        ++num_dropped_loops_;
        log_->warn("Loop of Frame {} with Frame {} dropped: {} LoopDetected events in flight", query.value,
                   region.match_frame.value, options_.max_events_in_flight);
        return;
    }
    event->map_id = replica().map_id();
    event->query_map_id = replica().map_id();
    event->match_map_id = region.match_map;
    event->query_frame = query;
    event->match_frame = region.match_frame;
    event->S_query_match = S_query_match;
    // Urgent: the Mapper's running local BA stops for the correction. A loop
    // found again before that correction reaches this replica (no new epoch
    // since the last loop published) is a repeat the Mapper drops: it waits
    // for the local BA.
    const std::pair<map::MapId, uint32_t> epoch{replica().map_id(), replica().map_epoch()};
    event->priority = last_loop_epoch_ != epoch;
    last_loop_epoch_ = epoch;
    const PointMatches query_points = PointsOf(replica(), *query_keyframe);
    // A loop's pairs are in one map, where a point matched to itself is no
    // pair; a merge's are in two, whose point ids restart per map, so equal
    // ids are two points.
    const bool is_loop = region.match_map == replica().map_id();
    for (size_t i = 0; i < region.matches.size() && i < query_points.size(); ++i) {
        if (region.matches[i] && query_points[i] && (!is_loop || *region.matches[i] != *query_points[i])) {
            event->matched_points.emplace_back(*query_points[i], *region.matches[i]);
        }
    }
    if (is_loop) {
        log_->info("Loop detected: Frame {} with Frame {} (scale {:.3f}, {} matched points)", query.value,
                   region.match_frame.value, S_query_match.scale, event->matched_points.size());
    } else {
        log_->info("Merge detected: Frame {} of map {} with Frame {} of map {} (scale {:.3f}, {} matched points)",
                   query.value, replica().map_id().value, region.match_frame.value, region.match_map.value,
                   S_query_match.scale, event->matched_points.size());
    }
    observable_.Notify(event);
}

INSTANTIATE_CLASS(LoopCloser)

} // namespace voyahslam::loop_closing
