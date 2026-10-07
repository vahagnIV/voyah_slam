#include "extract_node.h"

#include <numeric>

namespace voyahslam::features::orb {

ExtractorNode::ExtractorNode(size_t count) : keypoints(count) {
  std::iota(keypoints.begin(), keypoints.end(), size_t(0));
}

void ExtractorNode::Divide(ExtractorNode &c1, ExtractorNode &c2,
                           ExtractorNode &c3, ExtractorNode &c4,
                           const geometry::TPoints2DF &positions) const {
  const int width = br.x() - ul.x();
  const int height = br.y() - ul.y();

  const int half_x = (width + 1) / 2;
  const int half_y = (height + 1) / 2;

  c1.ul = ul;
  c1.br = {ul.x() + half_x, ul.y() + half_y};

  c2.ul = {c1.br.x(), c1.ul.y()};
  c2.br = {br.x(), c1.br.y()};

  c3.ul = {c1.ul.x(), c1.br.y()};
  c3.br = {c1.br.x(), br.y()};

  c4.ul = {c3.br.x(), c3.ul.y()};
  c4.br = br;

  const size_t reserve_guess = keypoints.size() / 4 + 1;
  c1.keypoints.reserve(reserve_guess);
  c2.keypoints.reserve(reserve_guess);
  c3.keypoints.reserve(reserve_guess);
  c4.keypoints.reserve(reserve_guess);

  const int split_x = c1.br.x();
  const int split_y = c1.br.y();

  decltype(c4.keypoints)* targets[] = {&c4.keypoints, &c3.keypoints, &c2.keypoints, &c1.keypoints};

  for (const auto &kp_idx : keypoints) {
    int index = (static_cast<int>(positions(0, kp_idx) < split_x) << 0) |
                (static_cast<int>(positions(1, kp_idx) < split_y) << 1);
    targets[index]->push_back(kp_idx);
  }

  c1.no_more = c1.keypoints.size() <= 1;
  c2.no_more = c2.keypoints.size() <= 1;
  c3.no_more = c3.keypoints.size() <= 1;
  c4.no_more = c4.keypoints.size() <= 1;
}

} // namespace voyahslam::features::orb
