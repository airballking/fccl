#include <gtest/gtest.h>

#include <fccl/base/JointConstraint.h>

using namespace fccl::base;

class JointConstraintTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_name = "right_shoulder_joint";
      joint_semantics.setName(joint_name);
      joint_position = 0.2;
      joint.semantics() = joint_semantics;
      joint.position() = joint_position;

      lower_boundary = 0.5;
      upper_boundary = 1.2;
    }

    virtual void TearDown()
    {

    }

    std::string joint_name;
    fccl::semantics::SemanticsBase joint_semantics;
    double joint_position;
    fccl::kdl::Joint joint;

    double lower_boundary, upper_boundary;
};

TEST_F(JointConstraintTest, Basics)
{
  JointConstraint jc;
  jc.joint() = joint;
  jc.lowerBoundary() = lower_boundary;
  jc.upperBoundary() = upper_boundary;

  ASSERT_TRUE(joint.equals(jc.joint()));
  ASSERT_DOUBLE_EQ(jc.lowerBoundary(), lower_boundary);
  ASSERT_DOUBLE_EQ(jc.upperBoundary(), upper_boundary);

  JointConstraint jc2(jc);
  ASSERT_TRUE(joint.equals(jc2.joint()));
  ASSERT_DOUBLE_EQ(jc2.lowerBoundary(), lower_boundary);
  ASSERT_DOUBLE_EQ(jc2.upperBoundary(), upper_boundary);
  ASSERT_TRUE(jc.equals(jc2));

  JointConstraint jc3;
  jc3 = jc;
  EXPECT_TRUE(jc.equals(jc3));
  
  JointConstraint jc4;
  jc4.joint() = jc.joint();
  jc4.lowerBoundary() = jc.lowerBoundary();
  jc4.upperBoundary() = jc.upperBoundary();
  EXPECT_TRUE(jc.equals(jc4));
  
  JointConstraint jc5;
  EXPECT_FALSE(jc.equals(jc5));
  jc5.joint() = joint;
  EXPECT_FALSE(jc.equals(jc5));
  jc4.lowerBoundary() = jc.lowerBoundary();
  EXPECT_FALSE(jc.equals(jc5));
  jc4.lowerBoundary() = jc.lowerBoundary() + 1.0;
  jc4.upperBoundary() = jc.upperBoundary();
  EXPECT_FALSE(jc.equals(jc5));
  jc4.lowerBoundary() = jc.lowerBoundary();
  jc5.joint().semantics().setID(0);
  EXPECT_FALSE(jc.equals(jc5));
  jc5.joint() = joint;
  EXPECT_TRUE(jc.equals(jc4));
}

TEST_F(JointConstraintTest, calculateDesiredOutput)
{
  JointConstraint jc;
  jc.joint() = joint;
  jc.lowerBoundary() = lower_boundary;
  jc.upperBoundary() = upper_boundary;

  // TODO(Georg): find a way to use private methods margin(), adjusted*Boundary()
  jc.joint().position() = lower_boundary - 1.0;
  EXPECT_DOUBLE_EQ(jc.calculateDesiredOutput(), lower_boundary + 0.05);

  jc.joint().position() = upper_boundary + 1.0;
  EXPECT_DOUBLE_EQ(jc.calculateDesiredOutput(), upper_boundary - 0.05);

  jc.joint().position() = 0.5*lower_boundary + 0.5*upper_boundary;
  EXPECT_DOUBLE_EQ(jc.calculateDesiredOutput(), jc.joint().position());

  jc.joint().position() = lower_boundary;
  EXPECT_DOUBLE_EQ(jc.calculateDesiredOutput(), lower_boundary + 0.05);

  jc.joint().position() = upper_boundary;
  EXPECT_DOUBLE_EQ(jc.calculateDesiredOutput(), upper_boundary - 0.05);
}

TEST_F(JointConstraintTest, calculateWeight)
{
  JointConstraint jc;
  jc.joint() = joint;
  jc.lowerBoundary() = lower_boundary;
  jc.upperBoundary() = upper_boundary;

  // TODO(Georg): find a way to use private methods margin(), adjusted*Boundary()
  jc.joint().position() = lower_boundary - 1.0;
  EXPECT_DOUBLE_EQ(jc.calculateWeight(), 1.0);

  jc.joint().position() = upper_boundary + 1.0;
  EXPECT_DOUBLE_EQ(jc.calculateWeight(), 1.0);

  jc.joint().position() = 0.5*lower_boundary + 0.5*upper_boundary;
  EXPECT_LT(jc.calculateWeight(), 1.0);
  EXPECT_GE(jc.calculateWeight(), 0.0);

  jc.joint().position() = lower_boundary;
  EXPECT_NEAR(jc.calculateWeight(), 1.0, 0.01);

  jc.joint().position() = upper_boundary;
  EXPECT_NEAR(jc.calculateWeight(), 1.0, 0.01);
}
