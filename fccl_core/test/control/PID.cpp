#include <gtest/gtest.h>

#include <fccl/control/PID.h>

using namespace fccl::control;
using namespace fccl::kdl;

class PIDTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");

      p.init(joint_names);
      p.numerics().data.setConstant(10.0);
      i.init(joint_names);
      i.numerics()(0) = 10.0;
      i.numerics()(1) = 0.0;
      d.init(joint_names);
      d.numerics().data.setConstant(1.0);
      i_max.init(joint_names);
      i_max.numerics().data.setConstant(15.0);
      i_min.init(joint_names);
      i_min.numerics().data.setConstant(0.0);

      error.init(joint_names);

      cycle_time = 1.0;
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    double cycle_time;
    JntArray p, i, d, i_max, i_min;
    JntArray error;    
};

TEST_F(PIDTest, Basics)
{
  PID pid;
  pid.init(error.semantics());
  pid.setGains(p, i, d, i_max, i_min);

  pid.reset();
  error.numerics().data.setZero();
  JntArray cmd = pid.computeCommand(error, cycle_time);
  EXPECT_TRUE(cmd.semantics().equals(error.semantics()));
  EXPECT_EQ(cmd.numerics()(0), 0.0);
  EXPECT_EQ(cmd.numerics()(1), 0.0);

  error.numerics().data.setConstant(1.0);
  cmd = pid.computeCommand(error, cycle_time);
  EXPECT_TRUE(cmd.semantics().equals(error.semantics()));
  EXPECT_EQ(cmd.numerics()(0), 21.0);
  EXPECT_EQ(cmd.numerics()(1), 11.0);

  cmd = pid.computeCommand(error, cycle_time);
  EXPECT_TRUE(cmd.semantics().equals(error.semantics()));
  EXPECT_EQ(cmd.numerics()(0), 25.0);
  EXPECT_EQ(cmd.numerics()(1), 10.0);
}
