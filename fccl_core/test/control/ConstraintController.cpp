#include <gtest/gtest.h>

#include <fccl/control/ConstraintController.h>

using namespace fccl::control;
using namespace fccl::kdl;
using namespace fccl::utils;
using namespace fccl::base;

using Eigen::operator<<;

class ConstraintControllerTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      // kinematics
      urdf.initFile("pr2.urdf");
      fccl::semantics::TransformSemantics transform_semantics;
      transform_semantics.init("torso_lift_link", "l_gripper_tool_frame");
      kinematics.init(transform_semantics, urdf);

      // joint state vectors
      joint_names.push_back("l_shoulder_pan_joint");
      joint_names.push_back("l_shoulder_lift_joint");
      joint_names.push_back("l_upper_arm_roll_joint");
      joint_names.push_back("l_elbow_flex_joint");
      joint_names.push_back("l_forearm_roll_joint");
      joint_names.push_back("l_wrist_flex_joint");
      joint_names.push_back("l_wrist_roll_joint");
      joint_state.init(joint_names);
      joint_state.numerics().data << 1.0650093105988152,
                                     0.26376743371555295,
                                     1.392565491097796,
                                     -1.629946646305397,
                                     214.04420097719472,
                                     -0.9668414952685922,
                                     -221.66147986216077;

      joint_vel.init(joint_names);

      // timing constants
      cycle_time = 0.01;
      delta_deriv = 0.001;

      // transforms
      tool_calibration.semantics().reference().setName("l_gripper_tool_frame");
      tool_calibration.semantics().target().setName("bottle");
      tool_calibration.numerics() = KDL::Frame(KDL::Rotation::Identity(),
          KDL::Vector(0.0, 0.0, 0.0));

      object_localization.semantics().reference().setName("base_link");
      object_localization.semantics().target().setName("oven");
      object_localization.numerics() = KDL::Frame(KDL::Rotation::Identity(),
          KDL::Vector(0.534, 0.0, 0.746));
      
      torso_transform.semantics().reference().setName("base_link");
      torso_transform.semantics().target().setName("torso_lift_link");
      torso_transform.numerics() = KDL::Frame(KDL::Rotation::Identity(),
          KDL::Vector(-0.05, 0.0, 1.04));

      transform_map.setTransform(multiply(torso_transform, 
          multiply(kinematics.calculateForwardKinematics(joint_state),
              tool_calibration)));
      transform_map.setTransform(
          multiply(kinematics.calculateForwardKinematics(joint_state),
              tool_calibration).inverse());
      transform_map.setTransform(object_localization);

      // features
      bottle_top.position() = KDL::Vector(0.0, 0.0, 0.0825);
      bottle_top.orientation() = KDL::Vector(0.0, 0.0, 0.3);
      bottle_top.semantics().type() = fccl::semantics::PLANE_FEATURE;
      bottle_top.semantics().reference().setName("bottle");
      bottle_top.semantics().name().setName("top plane of bottle cover");

      oven_plane.position() = KDL::Vector(0.0, 0.0, 0.0);
      oven_plane.orientation() = KDL::Vector(0.0, 0.0, 0.1);
      oven_plane.semantics().type() = fccl::semantics::PLANE_FEATURE;
      oven_plane.semantics().reference().setName("oven");
      oven_plane.semantics().name().setName("center of oven plane");

      // constraints
      Constraint above_constraint;
      above_constraint.toolFeature() = bottle_top;
      above_constraint.objectFeature() = oven_plane;
      above_constraint.lowerBoundary() = 0.05;
      above_constraint.upperBoundary() = 0.1;
      above_constraint.semantics().reference().setName("base_link");
      above_constraint.semantics().name().setName("bottle top slightly above oven");
      above_constraint.semantics().type().setName("above"); 

      std::vector<Constraint> cs;
      cs.push_back(above_constraint);
      constraints.init(cs); 

      // gains
      p.init(constraints.names());
      i.init(constraints.names());
      d.init(constraints.names());
      i_min.init(constraints.names());
      i_max.init(constraints.names());

      p.numerics().data.setConstant(100.0);
      i.numerics().data.setConstant(0.0);
      d.numerics().data.setConstant(20.0);
      i_min.numerics().data.setConstant(0.0);
      i_max.numerics().data.setConstant(0.0);
    }

    virtual void TearDown()
    {
    }

    Transform tool_calibration, object_localization, torso_transform;
    std::vector<std::string> joint_names;
    urdf::Model urdf;
    JntArray joint_state, joint_vel;
    TransformMap transform_map;
    KinematicChain kinematics;
    ConstraintArray constraints;    
    Feature bottle_top, oven_plane;
    double cycle_time, delta_deriv;
    JntArray p, i, d, i_min, i_max;
};

TEST_F(ConstraintControllerTest, Basics)
{
  ConstraintController controller;
  controller.init(constraints, kinematics, cycle_time);
  controller.setGains(p, i, d, i_max, i_min);
  controller.start(joint_state, transform_map, delta_deriv, cycle_time);
  ASSERT_FALSE(controller.constraints().areFulfilled());
  
  unsigned int counter = 0;
  while((!controller.constraints().areFulfilled()) && (counter < 100))
  {
    counter++;
    controller.update(joint_state, transform_map, delta_deriv, cycle_time);

    joint_vel = controller.desiredJointVelocities();
    joint_vel.numerics().data = joint_vel.numerics().data * cycle_time;
    substract(joint_state, joint_vel, joint_state);

    transform_map.setTransform(multiply(torso_transform, 
        multiply(kinematics.calculateForwardKinematics(joint_state),
            tool_calibration)));
    transform_map.setTransform(
        multiply(kinematics.calculateForwardKinematics(joint_state),
            tool_calibration).inverse());
  }

  EXPECT_TRUE(controller.constraints().areFulfilled());
}
