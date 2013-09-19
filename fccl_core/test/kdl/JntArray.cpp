#include <gtest/gtest.h>

#include <fccl/kdl/JntArray.h>
#include <vector>
#include <string>
#include <kdl/jntarray.hpp>

using namespace fccl::kdl;

class JntArrayTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      dof = 3;

      joint_names.resize(dof);
      joint_names[0] = "joint0";
      joint_names[1] = "joint1";
      joint_names[2] = "joint2";

      semantics.resize(dof);
      semantics.setTargetNames(joint_names);

      joint_values.resize(dof);
      for(unsigned int i=0; i<dof; i++)
      {
        joint_values(i) = i;
      }
    }

    virtual void TearDown()
    {

    }

    std::size_t dof;
    std::vector<std::string> joint_names;
    KDL::JntArray joint_values;
    SemanticObjectN semantics;
};

TEST_F(JntArrayTest, Basics)
{
  JntArray q;
  q.resize(dof);
  q.setSemantics(semantics);
  q.setData(joint_values);

  ASSERT_TRUE(q.isValid());

  JntArray q2(semantics, joint_values);

  JntArray q3;
  q3.resize(dof);
  q3 = q;

  JntArray q4(q.getSemantics(), q.getData());

  JntArray q5;
  q5.resize(dof);
  q5.setTargetNames(joint_names);
  q5.setData(joint_values);

  JntArray q6;
  q6.resize(dof);
  for(unsigned int i=0; i<dof; i++)
    q6.setTargetName(i, q.getTargetName(i));
  q6.setData(q.getData());

  JntArray q7;
  q7.init(q.getSemantics());
  q7.setData(q.getData());

  EXPECT_EQ(q, q2);
  EXPECT_EQ(q, q3);
  EXPECT_EQ(q, q4);   
  EXPECT_EQ(q, q5);
  EXPECT_EQ(q, q6);
  EXPECT_EQ(q, q7);
}
