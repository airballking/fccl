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
      one.init(joint_names);
      one.numerics().data.setConstant(1.0);
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    JntArray pos, vel, acc, zero, one;
};

TEST_F(StateEstimatorTest, Basics)
{
  StateEstimator se;
  se.init(zero.semantics());
  EXPECT_TRUE(se.currentPosition().equals(zero));
  EXPECT_TRUE(se.currentVelocity().equals(zero));
  EXPECT_TRUE(se.currentAcceleration().equals(zero));

  se.start(zero, one, one);
  EXPECT_TRUE(se.currentPosition().equals(zero));
  EXPECT_TRUE(se.currentVelocity().equals(one));
  EXPECT_TRUE(se.currentAcceleration().equals(one));

  se.sensor_update(pos);
  EXPECT_TRUE(se.currentPosition().equals(pos));
  EXPECT_TRUE(se.currentVelocity().equals(one));
  EXPECT_TRUE(se.currentAcceleration().equals(one));

  se.control_update(zero, vel, acc);
  EXPECT_TRUE(se.currentPosition().equals(pos));
  EXPECT_TRUE(se.currentVelocity().equals(vel));
  EXPECT_TRUE(se.currentAcceleration().equals(acc));
}

TEST_F(StateEstimatorTest, Basics2)
{
  StateEstimator se;
  se.init(zero.semantics());
  EXPECT_TRUE(se.currentPosition().equals(zero));
  EXPECT_TRUE(se.currentVelocity().equals(zero));
  EXPECT_TRUE(se.currentAcceleration().equals(zero));

  se.start(zero);
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
