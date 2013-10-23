#include <gtest/gtest.h>

#include <fccl/estimation/StateEstimator.h>

using namespace fccl::kdl;
using namespace fccl::estimation;

class StateEstimatorTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");

      pos.init(joint_names);
      pos.numerics().data.setConstant(1.0);
      vel.init(joint_names);
      vel.numerics().data.setConstant(1.5);
      acc.init(joint_names);
      acc.numerics().data.setConstant(2.0);
      zero.init(joint_names);
      zero.numerics().data.setZero();
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    JntArray pos, vel, acc, zero;
};

TEST_F(StateEstimatorTest, Basics)
{
  StateEstimator se;
  se.init(zero.semantics());
  EXPECT_TRUE(se.currentPosition().equals(zero));
  EXPECT_TRUE(se.currentVelocity().equals(zero));
  EXPECT_TRUE(se.currentAcceleration().equals(zero));

  se.sensor_update(pos);
  EXPECT_TRUE(se.currentPosition().equals(pos));
  EXPECT_TRUE(se.currentVelocity().equals(zero));
  EXPECT_TRUE(se.currentAcceleration().equals(zero));

  se.control_update(zero, vel, acc);
  EXPECT_TRUE(se.currentPosition().equals(pos));
  EXPECT_TRUE(se.currentVelocity().equals(vel));
  EXPECT_TRUE(se.currentAcceleration().equals(acc));
}
