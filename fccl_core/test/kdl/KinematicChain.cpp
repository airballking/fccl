#include <gtest/gtest.h>

#include <fccl/kdl/KinematicChain.h>

using namespace fccl::kdl;

class KinematicChainTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      urdf.initFile("pr2.urdf");
      arm_base_name = "torso_lift_link";
      arm_tip_name = "l_gripper_tool_frame";
      semantics.init(arm_base_name, arm_tip_name);

      joint_names.push_back("l_shoulder_pan_joint");
      joint_names.push_back("l_shoulder_lift_joint");
      joint_names.push_back("l_upper_arm_roll_joint");
      joint_names.push_back("l_elbow_flex_joint");
      joint_names.push_back("l_forearm_roll_joint");
      joint_names.push_back("l_wrist_flex_joint");
      joint_names.push_back("l_wrist_roll_joint");

      joint_state.resize(7);
      for(std::size_t i=0; i<7; i++)
        joint_state.numerics()(i) = 0.0;
      joint_state.init(joint_names);
    }

    virtual void TearDown()
    {

    }

    std::string arm_base_name, arm_tip_name;
    std::vector<std::string> joint_names;
    fccl::semantics::TransformSemantics semantics;
    JntArray joint_state;
    urdf::Model urdf;
};

TEST_F(KinematicChainTest, Basics)
{
  KinematicChain kinematics1;
  kinematics1.init(semantics, urdf);

  ASSERT_TRUE(kinematics1.isValid());
 
  EXPECT_EQ(kinematics1.softLowerJointLimits().size(), 
      kinematics1.size());
  EXPECT_EQ(kinematics1.softUpperJointLimits().size(), 
      kinematics1.size());
  EXPECT_EQ(kinematics1.hardLowerJointLimits().size(), 
      kinematics1.size());
  EXPECT_EQ(kinematics1.hardLowerJointLimits().size(), 
      kinematics1.size());
  EXPECT_EQ(kinematics1.jointNames().size(), 
      kinematics1.size());
  EXPECT_EQ(kinematics1.size(), 7);

  EXPECT_TRUE(kinematics1.softLowerJointLimits().semantics().equals(
      joint_state.semantics()));
  EXPECT_TRUE(kinematics1.softUpperJointLimits().semantics().equals(
      joint_state.semantics()));
  EXPECT_TRUE(kinematics1.hardLowerJointLimits().semantics().equals(
      joint_state.semantics()));
  EXPECT_TRUE(kinematics1.hardUpperJointLimits().semantics().equals(
      joint_state.semantics()));
  EXPECT_TRUE(kinematics1.semantics().joints().equals(joint_state.semantics()));

  ASSERT_EQ(joint_names.size(), kinematics1.jointNames().size());
  for(std::size_t i=0; i<joint_names.size(); i++)
    EXPECT_STREQ(joint_names[i].c_str(), kinematics1.jointNames()[i].c_str());

  Transform ee_pose = kinematics1.calculateForwardKinematics(joint_state);  
  EXPECT_TRUE(ee_pose.semantics().equals(semantics));
  EXPECT_STREQ(ee_pose.semantics().reference().getName().c_str(), 
      arm_base_name.c_str());
  EXPECT_STREQ(ee_pose.semantics().target().getName().c_str(), 
      arm_tip_name.c_str());
  EXPECT_TRUE(ee_pose.semantics().equals(kinematics1.semantics().transform()));

  kinematics1.calculateForwardKinematics(joint_state, ee_pose);
  EXPECT_TRUE(ee_pose.semantics().equals(semantics));
  EXPECT_STREQ(ee_pose.semantics().reference().getName().c_str(), 
      arm_base_name.c_str());
  EXPECT_STREQ(ee_pose.semantics().target().getName().c_str(), 
      arm_tip_name.c_str());
  EXPECT_TRUE(ee_pose.semantics().equals(kinematics1.semantics().transform()));

  fccl::semantics::JacobianSemantics jac_sem;
  jac_sem.init(joint_names, arm_base_name, arm_tip_name);

  Jacobian jac = kinematics1.calculateJacobian(joint_state);
  EXPECT_TRUE(jac.semantics().equals(jac_sem));

  kinematics1.calculateJacobian(joint_state, jac);
  EXPECT_TRUE(jac.semantics().equals(jac_sem));

  EXPECT_STREQ(jac.semantics().twist().reference().getName().c_str(),
      arm_base_name.c_str());
  EXPECT_STREQ(jac.semantics().twist().target().getName().c_str(),
      arm_tip_name.c_str());
  ASSERT_EQ(jac.size(), joint_names.size());
  for(std::size_t i=0; i<jac.size(); i++)
    EXPECT_STREQ(jac.semantics().joints()(i).getName().c_str(),
        joint_names[i].c_str());
}
