#include "quad_tree_distributor.h"

#include "voyahslam/defs.h"

namespace voyahslam::features::orb {

int QuadTreeDistributor::ComputeInitialNodeCount(int min_x, int max_x, int min_y,
                                                int max_y) {
  const int width = max_x - min_x;
  const int height = max_y - min_y;
  return static_cast<int>(std::round(static_cast<precision_t>(width) / height));
}

void QuadTreeDistributor::CreateInitialNodes(
    NodeList &nodes, std::vector<ExtractorNode *> &initial_nodes, int n_ini,
    int min_x, int max_x, int min_y, int max_y) {
  nodes.clear();
  initial_nodes.resize(n_ini);
  const int height = max_y - min_y;

  const float cell_width = static_cast<float>(max_x - min_x) / n_ini;

  for (int i = 0; i < n_ini; ++i) {
    ExtractorNode node;
    node.ul << static_cast<int>(cell_width * i), 0;
    node.br << static_cast<int>(cell_width * (i + 1)), height;
    node.no_more = false;
    nodes.emplace_back(node);
    initial_nodes[i] = &nodes.back();
  }
}

void QuadTreeDistributor::AssignKeypointsToInitialNodes(
    const geometry::TPoints2DF &input_keypoints,
    std::vector<ExtractorNode *> &initial_nodes, int n_ini, int min_x,
    int max_x) {
  const float cell_width_inv = n_ini / static_cast<float>(max_x - min_x);

  for (int i = 0; i < input_keypoints.cols(); ++i) {
    const int idx =
        static_cast<int>((input_keypoints(0, i) - min_x) * cell_width_inv);
    if (idx >= 0 && idx < n_ini) {
      initial_nodes[idx]->keypoints.push_back(i);
    }
  }
}

void QuadTreeDistributor::PruneInitialNodes(NodeList &nodes) {
  auto it = nodes.begin();
  while (it != nodes.end()) {
    if (it->keypoints.empty()) {
      it = nodes.erase(it);
    } else if (it->keypoints.size() == 1) {
      it->no_more = true;
      ++it;
    } else {
      ++it;
    }
  }
}

void QuadTreeDistributor::AddChildNode(NodeList &nodes, ExtractorNode &child,
                                      NodeSizePtrVec *expandable_nodes,
                                      int *n_to_expand) {
  if (child.keypoints.empty()) {
    return;
  }

  nodes.push_front(child);

  if (child.keypoints.size() > 1 && expandable_nodes != nullptr) {
    expandable_nodes->emplace_back(static_cast<int>(child.keypoints.size()),
                                   &nodes.front());
    nodes.front().self_it = nodes.begin();

    if (n_to_expand) {
      ++(*n_to_expand);
    }
  }
}

void QuadTreeDistributor::FullExpansionPass(
    NodeList &nodes, NodeSizePtrVec &expandable_nodes, int &n_to_expand,
    const geometry::TPoints2DF &input_keypoints) {
  expandable_nodes.clear();
  n_to_expand = 0;

  auto it = nodes.begin();
  while (it != nodes.end()) {
    if (it->no_more) {
      ++it;
      continue;
    }

    ExtractorNode n1, n2, n3, n4;
    it->Divide(n1, n2, n3, n4, input_keypoints);

    AddChildNode(nodes, n1, &expandable_nodes, &n_to_expand);
    AddChildNode(nodes, n2, &expandable_nodes, &n_to_expand);
    AddChildNode(nodes, n3, &expandable_nodes, &n_to_expand);
    AddChildNode(nodes, n4, &expandable_nodes, &n_to_expand);

    it = nodes.erase(it);
  }
}

void QuadTreeDistributor::SelectiveExpansionPhase(
    NodeList &nodes, int n_features, NodeSizePtrVec &expandable_nodes,
    const geometry::TPoints2DF &input_keypoints) {
  int prev_size = 0;

  while (nodes.size() < n_features && nodes.size() != prev_size) {
    prev_size = static_cast<int>(nodes.size());
    NodeSizePtrVec sorted = expandable_nodes;
    expandable_nodes.clear();

    if (sorted.empty()) {
      return;
    }

    std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
      return a.first < b.first;
    });

    for (int j = static_cast<int>(sorted.size()) - 1; j >= 0; --j) {
      ExtractorNode *node_to_split = sorted[j].second;

      ExtractorNode n1, n2, n3, n4;
      node_to_split->Divide(n1, n2, n3, n4, input_keypoints);

      AddChildNode(nodes, n1, &expandable_nodes, nullptr);
      AddChildNode(nodes, n2, &expandable_nodes, nullptr);
      AddChildNode(nodes, n3, &expandable_nodes, nullptr);
      AddChildNode(nodes, n4, &expandable_nodes, nullptr);

      nodes.erase(node_to_split->self_it);

      if (static_cast<int>(nodes.size()) >= n_features) {
        break;
      }
    }
  }
}

void QuadTreeDistributor::BuildOctreeNodes(
    NodeList &nodes, int n_features,
    const geometry::TPoints2DF &input_keypoints) {
  bool finished = false;
  NodeSizePtrVec expandable_nodes;
  expandable_nodes.reserve(nodes.size() * 4);

  while (!finished) {
    const int prev_size = static_cast<int>(nodes.size());

    int n_to_expand = 0;
    FullExpansionPass(nodes, expandable_nodes, n_to_expand, input_keypoints);

    const int current_size = static_cast<int>(nodes.size());

    if (current_size >= n_features || current_size == prev_size) {
      finished = true;
    } else if (current_size + n_to_expand * 3 > n_features) {
      SelectiveExpansionPhase(nodes, n_features, expandable_nodes,
                              input_keypoints);
      finished = true;
    }
  }
}

void QuadTreeDistributor::SelectBestKeypointsFromNodes(
    NodeList &nodes, int n_features, const geometry::TRowVectorF &strengths,
    std::vector<size_t> &out_map_points) {

  out_map_points.clear();
  out_map_points.reserve(n_features);

  for (const auto &node : nodes) {
    const auto &node_keys = node.keypoints;
    if (node_keys.empty()) {
      continue;
    }

    size_t best_kp_id = node_keys[0];
    float best_response = strengths(best_kp_id);

    for (size_t i = 1; i < node_keys.size(); ++i) {
      if (strengths(node_keys[i]) > best_response) {
        best_kp_id = node_keys[i];
        best_response = strengths(node_keys[i]);
      }
    }

    out_map_points.emplace_back(best_kp_id);
  }
}

void QuadTreeDistributor::DistributeOctTree(
    const geometry::TPoints2DF &keypoint_positions,
    const geometry::TRowVectorF &strengths, const int &minX, const int &maxX,
    const int &minY, const int &maxY, const int &n_features,
    std::vector<size_t> &out_map_points) {
  assert(maxX > minX && maxY > minY);
  NodeList nodes;
  std::vector<ExtractorNode *> initial_nodes;

  const int n_ini = ComputeInitialNodeCount(minX, maxX, minY, maxY);
  CreateInitialNodes(nodes, initial_nodes, n_ini, minX, maxX, minY, maxY);
  AssignKeypointsToInitialNodes(keypoint_positions, initial_nodes, n_ini, minX, maxX);
  PruneInitialNodes(nodes);
  BuildOctreeNodes(nodes, n_features, keypoint_positions);
  SelectBestKeypointsFromNodes(nodes, n_features, strengths, out_map_points);
}

} // namespace voyahslam::features::orb
