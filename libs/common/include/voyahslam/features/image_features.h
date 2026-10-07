#ifndef VOYAHSLAM_FEATURES_IMAGE_FEATURES_H
#define VOYAHSLAM_FEATURES_IMAGE_FEATURES_H

#include <vector>
#include <voyahslam/defs.h>

namespace voyahslam::features {

// Pyramid level each keypoint was detected at (0 = finest).
typedef Eigen::Matrix<uint8_t, 1, Eigen::Dynamic> TLevels;

template<typename TScalar, unsigned int Length>
class ImageFeatures {
public:
    constexpr static unsigned int kDescSize = Length;
    typedef Eigen::Matrix<TScalar, kDescSize, 1> TDescriptor;
    typedef Eigen::Matrix<TScalar, kDescSize, Eigen::Dynamic> TDescriptors;
    ImageFeatures() = default;

    ImageFeatures(
        geometry::TPoints2DF&& points,
        geometry::TRowVectorF&& strengths,
        geometry::TRowVectorF&& angles,
        TDescriptors&& descriptors,
        TLevels&& levels)
        : points_(std::move(points))
          , strengths_(std::move(strengths))
          , angles_(std::move(angles))
          , descriptors_(std::move(descriptors))
          , levels_(std::move(levels))
    {
    }

    ImageFeatures(const ImageFeatures&) = default;
    ImageFeatures& operator=(const ImageFeatures&) = default;
    ImageFeatures(ImageFeatures&&) = default;
    ImageFeatures& operator=(ImageFeatures&&) = default;

    [[nodiscard]] const geometry::TPoints2DF& points() const { return points_; }
    [[nodiscard]] const geometry::TRowVectorF& strengths() const { return strengths_; }
    [[nodiscard]] const geometry::TRowVectorF& angles() const { return angles_; }
    [[nodiscard]] const TDescriptors& descriptors() const { return descriptors_; }
    [[nodiscard]] const TLevels& levels() const { return levels_; }

protected:
    // Keeps only the features at `indices` (ascending), in that order.
    void KeepFeatures(const std::vector<Eigen::Index>& indices) {
        points_ = points_(Eigen::all, indices).eval();
        strengths_ = strengths_(Eigen::all, indices).eval();
        angles_ = angles_(Eigen::all, indices).eval();
        descriptors_ = descriptors_(Eigen::all, indices).eval();
        levels_ = levels_(Eigen::all, indices).eval();
    }

    geometry::TPoints2DF points_;
    geometry::TRowVectorF strengths_;
    geometry::TRowVectorF angles_;
    TDescriptors descriptors_;
    TLevels levels_;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_IMAGE_FEATURES_H
