#include <gtest/gtest.h>
#include "voyahslam/features/keypoint_matches.h"

namespace voyahslam::features::test {

class KeypointMatchesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create some test data
        query_ids_ = Eigen::VectorXi(3);
        train_ids_ = Eigen::VectorXi(3);
        distances_ = Eigen::VectorXf(3);

        query_ids_ << 1, 2, 3;
        train_ids_ << 4, 5, 6;
        distances_ << 0.1f, 0.2f, 0.3f;
    }

    Eigen::VectorXi query_ids_;
    Eigen::VectorXi train_ids_;
    Eigen::VectorXf distances_;
};

TEST_F(KeypointMatchesTest, DefaultConstructor) {
    KeypointMatches matches;
    EXPECT_EQ(matches.size(), 0);
}

TEST_F(KeypointMatchesTest, ConstructorWithVectors) {
    KeypointMatches matches(
        std::move(query_ids_),
        std::move(train_ids_),
        std::move(distances_));

    EXPECT_EQ(matches.size(), 3);
    EXPECT_EQ(matches.query_ids()(0), 1);
    EXPECT_EQ(matches.query_ids()(1), 2);
    EXPECT_EQ(matches.query_ids()(2), 3);

    EXPECT_EQ(matches.train_ids()(0), 4);
    EXPECT_EQ(matches.train_ids()(1), 5);
    EXPECT_EQ(matches.train_ids()(2), 6);

    EXPECT_FLOAT_EQ(matches.distances()(0), 0.1f);
    EXPECT_FLOAT_EQ(matches.distances()(1), 0.2f);
    EXPECT_FLOAT_EQ(matches.distances()(2), 0.3f);
}

TEST_F(KeypointMatchesTest, AddMatch) {
    KeypointMatches matches;

    matches.add_match(1, 4, 0.1f);
    EXPECT_EQ(matches.size(), 1);
    EXPECT_EQ(matches.query_ids()(0), 1);
    EXPECT_EQ(matches.train_ids()(0), 4);
    EXPECT_FLOAT_EQ(matches.distances()(0), 0.1f);

    matches.add_match(2, 5, 0.2f);
    EXPECT_EQ(matches.size(), 2);
    EXPECT_EQ(matches.query_ids()(1), 2);
    EXPECT_EQ(matches.train_ids()(1), 5);
    EXPECT_FLOAT_EQ(matches.distances()(1), 0.2f);
}

TEST_F(KeypointMatchesTest, MoveSemantics) {
    KeypointMatches matches1(
        std::move(query_ids_),
        std::move(train_ids_),
        std::move(distances_));

    // Test move constructor
    KeypointMatches matches2(std::move(matches1));
    EXPECT_EQ(matches2.size(), 3);
    EXPECT_EQ(matches1.size(), 0);  // Original should be empty

    // Test move assignment
    KeypointMatches matches3;
    matches3 = std::move(matches2);
    EXPECT_EQ(matches3.size(), 3);
    EXPECT_EQ(matches2.size(), 0);  // Original should be empty
}

} // namespace voyahslam::features::test
