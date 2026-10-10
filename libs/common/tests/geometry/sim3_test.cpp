#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "voyahslam/geometry/sim3.h"

namespace voyahslam::geometry {
namespace {

Sim3 RandomSim3(double scale) {
    return Sim3{scale, Eigen::Quaternion<precision_t>::UnitRandom().toRotationMatrix(), TVector3DF::Random()};
}

TEST(Sim3Test, DefaultIsIdentity) {
    const Sim3 S;
    EXPECT_EQ(S.scale, 1.0);
    EXPECT_EQ(S.R, TMatrix33F::Identity());
    EXPECT_EQ(S.t, TVector3DF::Zero());
    const TPoint3DF x(1.0, -2.0, 3.0);
    EXPECT_EQ(Apply(S, x), x);
}

TEST(Sim3Test, ApplyIsScaledRotationPlusTranslation) {
    const Sim3 S = RandomSim3(2.5);
    const TPoint3DF x = TPoint3DF::Random();
    EXPECT_NEAR((Apply(S, x) - (2.5 * S.R * x + S.t)).norm(), 0.0, 1e-12);
}

TEST(Sim3Test, InverseRoundTrip) {
    const Sim3 S = RandomSim3(0.7);
    const TPoint3DF x = TPoint3DF::Random();
    EXPECT_NEAR((Apply(Inverse(S), Apply(S, x)) - x).norm(), 0.0, 1e-12);
    const Sim3 identity = Compose(S, Inverse(S));
    EXPECT_NEAR(identity.scale, 1.0, 1e-12);
    EXPECT_TRUE(identity.R.isApprox(TMatrix33F::Identity(), 1e-12));
    EXPECT_NEAR(identity.t.norm(), 0.0, 1e-12);
}

TEST(Sim3Test, ComposeAppliesRightFirst) {
    const Sim3 S_ab = RandomSim3(1.8);
    const Sim3 S_bc = RandomSim3(0.4);
    const TPoint3DF x = TPoint3DF::Random();
    EXPECT_NEAR((Apply(Compose(S_ab, S_bc), x) - Apply(S_ab, Apply(S_bc, x))).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Compose(S_ab, S_bc).scale, 1.8 * 0.4, 1e-15);
}

}  // namespace
}  // namespace voyahslam::geometry
