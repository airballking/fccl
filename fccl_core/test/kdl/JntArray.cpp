#include <gtest/gtest.h>

#include <fccl/kdl/JntArray.h>
#include <vector>
#include <string>
#include <kdl/jntarray.hpp>

using namespace fccl::kdl;
using namespace fccl::semantics;

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
      for(std::size_t i=0; i<dof; i++)
        semantics(i).setName(joint_names[i]);

      joint_values.resize(dof);
      for(unsigned int i=0; i<dof; i++)
        joint_values(i) = i;
    }

    virtual void TearDown()
    {

    }

    std::size_t dof;
    std::vector<std::string> joint_names;
    KDL::JntArray joint_values;
    JntArraySemantics semantics;
};

TEST_F(JntArrayTest, Basics)
{
  JntArray q;
  q.resize(dof);
  q.semantics() = semantics;
  q.numerics() = joint_values;

  ASSERT_TRUE(q.isValid());

  JntArray q2(q);

  JntArray q3;
  q3 = q;

  JntArray q4;
  q4.resize(dof);
  for(unsigned int i=0; i<dof; i++)
    q4.numerics()(i) = joint_values(i);
  q4.semantics() = semantics;

  EXPECT_TRUE(q.equals(q2));
  EXPECT_TRUE(q.equals(q3));
  EXPECT_TRUE(q.equals(q4));   
}

TEST_F(JntArrayTest, init)
{
  JntArray q;
  q.init(joint_names);

  ASSERT_TRUE(q.isValid());
  ASSERT_EQ(q.size(), joint_names.size());

  JntArraySemantics s;
  s.init(joint_names);
  EXPECT_TRUE(q.semantics().equals(s));

  JntArray q2;
  q2.init(s);
  
  ASSERT_TRUE(q2.isValid());
  ASSERT_EQ(q2.size(), joint_names.size());
  EXPECT_TRUE(q2.semantics().equals(s));
}
