#include <gtest/gtest.h>

#include <fccl_conversions/JointStateInterpreter.h>
#include <vector>

using namespace fccl::conversions;

class JointStateInterpreterTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("shoulder");
      joint_names.push_back("elbow");
      joint_names.push_back("wrist");

      joint_names_scrambled.push_back("wrist");
      joint_names_scrambled.push_back("elbow");
      joint_names_scrambled.push_back("shoulder");

      joint_values.push_back(1.0);
      joint_values.push_back(2.0);
      joint_values.push_back(3.0);

      joint_values_scrambled.push_back(3.0);
      joint_values_scrambled.push_back(2.0);
      joint_values_scrambled.push_back(1.0);

      semantics.init(joint_names);

      joints.init(semantics);
      for(std::size_t i=0; i<joints.size(); i++)
        joints.numerics()(i) = joint_values[i];

      msg_valid.name = joint_names;
      msg_valid.position = joint_values;

      msg_valid2 = msg_valid;
      msg_valid2.name.push_back("head");
      msg_valid2.position.push_back(0.5);

      msg_valid3.name = joint_names_scrambled;
      msg_valid3.position = joint_values_scrambled;
      
      msg_valid4 = msg_valid3;
      msg_valid4.name.push_back("head");
      msg_valid4.position.push_back(0.5);
 
      msg_invalid = msg_valid;
      msg_invalid.name.pop_back();
      msg_invalid.position.pop_back();
    }

    virtual void TearDown()
    {
    }

  std::vector<std::string> joint_names, joint_names_scrambled;
  std::vector<double> joint_values, joint_values_scrambled;
  fccl::semantics::JntArraySemantics semantics;
  fccl::kdl::JntArray joints;
  sensor_msgs::JointState msg_valid, msg_valid2, msg_valid3, msg_valid4, msg_invalid;
};

TEST_F(JointStateInterpreterTest, Basics)
{
  JointStateInterpreter parser(semantics);

  fccl::kdl::JntArray joints1;
  joints1.init(semantics);
  ASSERT_TRUE(semantics.equals(parser.semantics()));

  ASSERT_TRUE(parser.parseJointState(msg_valid, joints1));  
  EXPECT_TRUE(joints.equals(joints1));

  JointStateInterpreter parser2;
  parser2.init(semantics); 
  ASSERT_TRUE(semantics.equals(parser2.semantics()));

  fccl::kdl::JntArray joints2;
  joints2.init(semantics);
  ASSERT_TRUE(parser2.parseJointState(msg_valid2, joints2));  
  EXPECT_TRUE(joints.equals(joints2));

  fccl::kdl::JntArray joints3;
  joints3.init(semantics);
  ASSERT_TRUE(parser2.parseJointState(msg_valid3, joints3));  
  EXPECT_TRUE(joints.equals(joints3));

  fccl::kdl::JntArray joints4;
  joints4.init(semantics);
  ASSERT_TRUE(parser.parseJointState(msg_valid4, joints4));  
  EXPECT_TRUE(joints.equals(joints4));

  fccl::kdl::JntArray joints5;
  joints5.init(semantics);
  EXPECT_FALSE(parser.parseJointState(msg_invalid, joints5));  
}
