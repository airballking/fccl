#include <gtest/gtest.h>

#include <fccl/semantics/JointMappingSemantics.h>
#include <vector>
#include <string>

using namespace fccl::semantics;

class JointMappingSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      constraint_names.push_back("height");
      constraint_names.push_back("distance");
      constraint_names.push_back("left off");

      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");
      joint_names.push_back("joint3");
      joint_names.push_back("joint4");
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> constraint_names;
    std::vector<std::string> joint_names;
};

TEST_F(JointMappingSemanticsTest, Basics)
{ 
  JointMappingSemantics A;
  A.row_joints().resize(constraint_names.size());
  A.column_joints().resize(joint_names.size());
  ASSERT_EQ(A.row_joints().size(), constraint_names.size());
  ASSERT_EQ(A.column_joints().size(), joint_names.size());

  for(std::size_t i=0; i<constraint_names.size(); i++)
    A.row_joints()(i).setName(constraint_names[i]);
  for(std::size_t i=0; i<joint_names.size(); i++)
    A.column_joints()(i).setName(joint_names[i]);
  
  for(std::size_t i=0; i<constraint_names.size(); i++)
    EXPECT_STREQ(A.row_joints()(i).getName().c_str(),
        constraint_names[i].c_str());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(A.column_joints()(i).getName().c_str(),
        joint_names[i].c_str());

  JointMappingSemantics A2;
  A2 = A;

  JointMappingSemantics A3(A);

  JointMappingSemantics A4;
  A4.row_joints() = A.row_joints();
  A4.column_joints() = A.column_joints();

  JointMappingSemantics A5;
  A5.row_joints().resize(A.row_joints().size());
  for(std::size_t i=0; i<A5.row_joints().size(); i++)
    A5.row_joints()(i).setName(A.row_joints()(i).getName());
  A5.column_joints().resize(A.column_joints().size());
  for(std::size_t i=0; i<A5.column_joints().size(); i++)
    A5.column_joints()(i) = A.column_joints()(i);

  EXPECT_TRUE(A.equals(A2));
  EXPECT_TRUE(A.equals(A3));
  EXPECT_TRUE(A.equals(A4));
  EXPECT_TRUE(A.equals(A5));

  JointMappingSemantics A6;

  JointMappingSemantics A7;
  A7.row_joints() = A.row_joints();

  JointMappingSemantics A8;
  A8.column_joints() = A.column_joints();

  EXPECT_FALSE(A.equals(A6));
  EXPECT_FALSE(A.equals(A7));
  EXPECT_FALSE(A.equals(A8));
}

TEST_F(JointMappingSemanticsTest, Init)
{ 
  JointMappingSemantics A;
  A.init(constraint_names, joint_names);
  ASSERT_EQ(A.row_joints().size(), constraint_names.size());
  ASSERT_EQ(A.column_joints().size(), joint_names.size());

  for(std::size_t i=0; i<constraint_names.size(); i++)
    EXPECT_STREQ(A.row_joints()(i).getName().c_str(),
        constraint_names[i].c_str());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(A.column_joints()(i).getName().c_str(),
        joint_names[i].c_str());
}
