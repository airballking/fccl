#include <gtest/gtest.h>

#include <fccl/kdl/KinematicChain.h>

using namespace fccl::kdl;

class KinematicChainTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      urdf.initFile("/home/georg/ros/groovy/catkin_ws/src/fccl/fccl_core/test_data/pr2.urdf");
      arm_base_name = "torso_lift_link";
      arm_tip_name = "l_gripper_tool_frame";
      semantics = SemanticObject1x1(arm_base_name, arm_tip_name);

      joint_names.push_back("l_shoulder_pan_joint");
      joint_names.push_back("l_shoulder_lift_joint");
      joint_names.push_back("l_upper_arm_roll_joint");
      joint_names.push_back("l_elbow_flex_joint");
      joint_names.push_back("l_forearm_roll_joint");
      joint_names.push_back("l_wrist_flex_joint");
      joint_names.push_back("l_wrist_roll_joint");

      joint_state.resize(7);
      for(std::size_t i=0; i<7; i++)
        joint_state(i) = 0.;
      joint_state.setTargetNames(joint_names);
    }

    virtual void TearDown()
    {

    }

    std::string arm_base_name, arm_tip_name;
    std::vector<std::string> joint_names;
    SemanticObject1x1 semantics;
    JntArray joint_state;
    urdf::Model urdf;
};

TEST_F(KinematicChainTest, Basics)
{
  KinematicChain kinematics1(semantics, urdf);
  
  EXPECT_TRUE(kinematics1.getTransformationSemantics().semanticsEqual(semantics));
  EXPECT_TRUE(kinematics1.getJointSemantics().semanticsEqual(joint_state));

  EXPECT_EQ(kinematics1.getSoftLowerJointLimits().size(), 
      kinematics1.getNumberOfJoints());
  EXPECT_EQ(kinematics1.getSoftUpperJointLimits().size(), 
      kinematics1.getNumberOfJoints());
  EXPECT_EQ(kinematics1.getHardLowerJointLimits().size(), 
      kinematics1.getNumberOfJoints());
  EXPECT_EQ(kinematics1.getHardLowerJointLimits().size(), 
      kinematics1.getNumberOfJoints());
  EXPECT_EQ(kinematics1.getJointNames().size(), 
      kinematics1.getNumberOfJoints());
  EXPECT_EQ(kinematics1.getNumberOfJoints(), 7);

  ASSERT_EQ(joint_names.size(), kinematics1.getJointNames().size());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(joint_names[i].c_str(), kinematics1.getJointNames()[i].c_str());

  Transform ee_pose = kinematics1.calculateForwardKinematics(joint_state);  
  EXPECT_STREQ(ee_pose.getReferenceName().c_str(), arm_base_name.c_str());
  EXPECT_STREQ(ee_pose.getTargetName().c_str(), arm_tip_name.c_str());

  kinematics1.calculateForwardKinematics(joint_state, ee_pose);
  EXPECT_STREQ(ee_pose.getReferenceName().c_str(), arm_base_name.c_str());
  EXPECT_STREQ(ee_pose.getTargetName().c_str(), arm_tip_name.c_str());

  Jacobian jac = kinematics1.calculateJacobian(joint_state);
  EXPECT_STREQ(jac.getReferenceName().c_str(), arm_base_name.c_str());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(joint_names[i].c_str(), jac.getTargetName(i).c_str());

  kinematics1.calculateJacobian(joint_state, jac);
  EXPECT_STREQ(jac.getReferenceName().c_str(), arm_base_name.c_str());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(joint_names[i].c_str(), jac.getTargetName(i).c_str());

  EXPECT_TRUE(kinematics1.getJacobianSemantics().semanticsEqual(SemanticObject1xN(semantics.getReferenceName(), joint_names)));
}
