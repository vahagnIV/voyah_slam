#ifndef VOYAHSLAM_SRC_FEATURES_ORB_QUAD_TREE_DISTRIBUTOR_H
#define VOYAHSLAM_SRC_FEATURES_ORB_QUAD_TREE_DISTRIBUTOR_H

#include "extract_node.h"

namespace voyahslam::features::orb {

class QuadTreeDistributor {
  using NodeList = std::list<ExtractorNode>;

public:
  static void DistributeOctTree(const geometry::TPoints2DF &keypoint_positions,
                         const geometry::TRowVectorF &strengths,
                         const int &minX, const int &maxX, const int &minY,
                         const int &maxY, const int &n_features,
                         std::vector<size_t> &out_map_points);

private:
  using NodeSizePtrVec = std::vector<std::pair<int, ExtractorNode *>>;

  static int ComputeInitialNodeCount(int min_x, int max_x, int min_y, int max_y);

  static void CreateInitialNodes(NodeList &nodes,
                                 std::vector<ExtractorNode *> &initial_nodes,
                                 int n_ini, int min_x, int max_x, int min_y, int max_y);

  static void AssignKeypointsToInitialNodes(
      const geometry::TPoints2DF &input_keypoints,
      std::vector<ExtractorNode *> &initial_nodes, int n_ini, int min_x, int max_x);

  static void PruneInitialNodes(NodeList &nodes);

  static void AddChildNode(NodeList &nodes, ExtractorNode &child,
                           NodeSizePtrVec *expandable_nodes, int *n_to_expand);

  static void FullExpansionPass(NodeList &nodes,
                                NodeSizePtrVec &expandable_nodes,
                                int &n_to_expand,
                                const geometry::TPoints2DF &input_keypoints);

  static void SelectiveExpansionPhase(NodeList &nodes, int n_features,
                          NodeSizePtrVec &expandable_nodes,
                          const geometry::TPoints2DF &input_keypoints);

  static void BuildOctreeNodes(NodeList &nodes, int n_features,
                               const geometry::TPoints2DF &input_keypoints);

  static void SelectBestKeypointsFromNodes(NodeList &nodes, int n_features,
                                    const geometry::TRowVectorF &strengths,
                                    std::vector<size_t> &out_map_points);
};

} // namespace voyahslam::features::orb

#endif // VOYAHSLAM_SRC_FEATURES_ORB_QUAD_TREE_DISTRIBUTOR_H
