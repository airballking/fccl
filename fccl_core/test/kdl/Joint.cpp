#include <gtest/gtest.h>

#include <fccl/kdl/Joint.h>

using namespace fccl::kdl;

class JointTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_name = "left_elbow_joint";
      semantics.setName(joint_name);
      joint_position = 1.2;
    }

    virtual void TearDown()
    {

    }

    fccl::semantics::SemanticsBase semantics;
    std::string joint_name;
    double joint_position;
};

TEST_F(JointTest, Basics)
{
  PositionJoint jnt;
  jnt.semantics() = semantics;
  jnt.position() = joint_position;

  ASSERT_STREQ(jnt.semantics().getName().c_str(), joint_name.c_str());
  ASSERT_DOUBLE_EQ(jnt.position(), joint_position);

  PositionJoint jnt2(jnt);
  EXPECT_TRUE(jnt.equals(jnt2));
  EXPECT_STREQ(jnt2.semantics().getName().c_str(), joint_name.c_str());
  EXPECT_DOUBLE_EQ(jnt2.position(), joint_position);

  PositionJoint jnt3;
  jnt3 = jnt;
  EXPECT_TRUE(jnt.equals(jnt3));

  PositionJoint jnt4;
  jnt4.semantics() = jnt.semantics();
  jnt4.position() = jnt.position();
  EXPECT_TRUE(jnt.equals(jnt4));

  PositionJoint jnt5;
  EXPECT_FALSE(jnt.equals(jnt5));
  jnt5.semantics() = semantics;
  EXPECT_FALSE(jnt.equals(jnt5));
  jnt5.position() = joint_position;
  jnt5.semantics().setName("");
  EXPECT_FALSE(jnt.equals(jnt5));
}

TEST_F(JointTest, Addition)
{
  PositionJoint jnt;
  jnt.semantics().setName("neck");
  jnt.position() = 1.13;
  PositionJoint jnt2(jnt);

  PositionJoint jnt3;
  jnt3 = jnt + jnt2;
  EXPECT_TRUE(jnt.semantics().equals(jnt3.semantics()));
  EXPECT_DOUBLE_EQ(jnt3.position(), jnt.position() + jnt2.position());
}

TEST_F(JointTest, Subtraction)
{
  PositionJoint jnt;
  jnt.semantics().setName("neck");
  jnt.position() = 1.13;
  PositionJoint jnt2(jnt);
  jnt2.position() = 1.0;

  PositionJoint jnt3 = jnt -jnt2;
  EXPECT_TRUE(jnt.semantics().equals(jnt3.semantics()));
  EXPECT_DOUBLE_EQ(jnt3.position(), jnt.position() - jnt2.position());
}
// TODO(Georg): test velocity joint
// TODO(Georg): test acceleration joint
