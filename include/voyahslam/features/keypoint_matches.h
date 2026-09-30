#ifndef VOYAHSLAM_FEATURES_KEYPOINT_MATCHES_H
#define VOYAHSLAM_FEATURES_KEYPOINT_MATCHES_H

#include <Eigen/Dense>

namespace voyahslam::features {

class KeypointMatches {
public:
    KeypointMatches() = default;

    KeypointMatches(
        Eigen::VectorXi&& query_ids,
        Eigen::VectorXi&& train_ids,
        Eigen::VectorXf&& distances)
        : query_ids_(std::move(query_ids))
        , train_ids_(std::move(train_ids))
        , distances_(std::move(distances)) {
        assert(query_ids_.size() == train_ids_.size() &&
               query_ids_.size() == distances_.size());
    }

    KeypointMatches(const KeypointMatches&) = delete;
    KeypointMatches& operator=(const KeypointMatches&) = delete;
    KeypointMatches(KeypointMatches&&) = default;
    KeypointMatches& operator=(KeypointMatches&&) = default;

    const Eigen::VectorXi& query_ids() const { return query_ids_; }
    const Eigen::VectorXi& train_ids() const { return train_ids_; }
    const Eigen::VectorXf& distances() const { return distances_; }

    size_t size() const { return query_ids_.size(); }

    void add_match(int query_id, int train_id, float distance) {
        query_ids_.conservativeResize(query_ids_.size() + 1);
        train_ids_.conservativeResize(train_ids_.size() + 1);
        distances_.conservativeResize(distances_.size() + 1);

        query_ids_.tail(1) << query_id;
        train_ids_.tail(1) << train_id;
        distances_.tail(1) << distance;
    }

private:
    Eigen::VectorXi query_ids_;
    Eigen::VectorXi train_ids_;
    Eigen::VectorXf distances_;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_KEYPOINT_MATCHES_H
