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
      dof = 4;

      joint_names.resize(dof);
      joint_names[0] = "joint0";
      joint_names[1] = "joint1";
      joint_names[2] = "joint2";
      joint_names[3] = "joint3";

      foo = "foo";
      bar = "bar";

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
    std::string foo, bar;
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

TEST_F(JntArrayTest, partialAssignment)
{
  JntArray q;
  q.init(joint_names);
  q.numerics() = joint_values;

  JntArray q2;
  q2.resize(2);
  q2.semantics()(0).setName(foo);
  q2.semantics()(1).setName(bar);
  q2.numerics()(0) = 42;
  q2.numerics()(1) = 43;

  q.partialAssignment(1, 2, q2);

  ASSERT_TRUE(q.isValid());
  
  EXPECT_STREQ(q.semantics()(0).getName().c_str(), joint_names[0].c_str());
  EXPECT_STREQ(q.semantics()(1).getName().c_str(), foo.c_str());
  EXPECT_STREQ(q.semantics()(2).getName().c_str(), bar.c_str());
  EXPECT_STREQ(q.semantics()(3).getName().c_str(), joint_names[3].c_str());
 
  EXPECT_EQ(q.numerics()(0), joint_values(0));
  EXPECT_EQ(q.numerics()(1), 42); 
  EXPECT_EQ(q.numerics()(2), 43);
  EXPECT_EQ(q.numerics()(3), joint_values(3));
}

TEST_F(JntArrayTest, subtraction)
{
  JntArray q1, q2, res;
  q1.init(joint_names);
  q2.init(joint_names);
  res.init(joint_names);
  
  using Eigen::operator<<;
  q1.numerics().data << 10, 10, 10, 10;
  q2.numerics().data << 6, 6, 6, 6;
  res.numerics().data.setZero();

  substract(q1, q2, res);
  ASSERT_TRUE(q1.semantics().equals(res.semantics()));
  Eigen::Matrix<double, 4, 1> data;
  data << 4, 4, 4, 4;
  EXPECT_TRUE(data.isApprox(res.numerics().data));
}

TEST_F(JntArrayTest, addition)
{
  JntArray q1, q2, res;
  q1.init(joint_names);
  q2.init(joint_names);
  res.init(joint_names);
  
  using Eigen::operator<<;
  q1.numerics().data << 10, 10, 10, 10;
  q2.numerics().data << 6, 6, 6, 6;
  res.numerics().data.setZero();

  add(q1, q2, res);
  ASSERT_TRUE(q1.semantics().equals(res.semantics()));
  Eigen::Matrix<double, 4, 1> data;
  data << 16, 16, 16, 16;
  EXPECT_TRUE(data.isApprox(res.numerics().data));
}
