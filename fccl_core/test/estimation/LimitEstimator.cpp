#include <gtest/gtest.h>

#include <fccl/estimation/LimitEstimator.h>

using namespace fccl::kdl;
using namespace fccl::estimation;

class LimitEstimatorTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");

      max_vel.init(joint_names);
      max_acc.init(joint_names);
      max_jerk.init(joint_names);

      for(std::size_t i=0; i< joint_names.size(); i++)
      {
        max_vel.numerics()(i) = 1.0;
        max_acc.numerics()(i) = 0.5;
        max_jerk.numerics()(i) = 0.5;
      }
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    JntArray max_vel, max_acc, max_jerk;
};

TEST_F(LimitEstimatorTest, Basics)
{
  LimitEstimator l;
  l.init(max_vel.semantics());
  EXPECT_TRUE(l.maximumVelocity().equals(max_vel));
  EXPECT_TRUE(l.maximumAcceleration().equals(max_acc));
  EXPECT_TRUE(l.maximumJerk().equals(max_jerk));
}
