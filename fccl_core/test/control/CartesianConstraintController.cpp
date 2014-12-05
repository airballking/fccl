#include <gtest/gtest.h>

#include <fccl/control/CartesianConstraintController.h>

using namespace fccl::control;
using namespace fccl::kdl;
using namespace fccl::utils;
using namespace fccl::base;
using namespace fccl::semantics;

using Eigen::operator<<;

class CartesianConstraintControllerTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      // timing constants
      cycle_time = 0.01;
      delta_deriv = 0.001;

      // transforms
      oven_transform.semantics().reference().setName("world");
      oven_transform.semantics().target().setName("oven");
      oven_transform.numerics() = KDL::Frame(KDL::Rotation::Identity(),
          KDL::Vector(0.534, 0.0, 0.746));
      
      bottle_transform.semantics().reference().setName("world");
      bottle_transform.semantics().target().setName("bottle");
      bottle_transform.numerics() = KDL::Frame(KDL::Rotation::Identity(),
          KDL::Vector(0.504, -0.2, 0.746));

      transform_map.setTransform(oven_transform);
      transform_map.setTransform(bottle_transform);

      // features
      bottle_axis.position() = KDL::Vector(0.0, 0.0, 0.0825);
      bottle_axis.orientation() = KDL::Vector(0.0, 0.0, 0.3);
      bottle_axis.semantics().type() = fccl::semantics::LINE_FEATURE;
      bottle_axis.semantics().reference().setName("bottle");
      bottle_axis.semantics().name().setName("center axis of bottle");

      oven_plane.position() = KDL::Vector(0.0, 0.0, 0.05);
      oven_plane.orientation() = KDL::Vector(0.0, 0.0, 0.1);
      oven_plane.semantics().type() = fccl::semantics::PLANE_FEATURE;
      oven_plane.semantics().reference().setName("oven");
      oven_plane.semantics().name().setName("center plane of upper oven surface");

      // single constraints
      Constraint above_constraint, tilting_constraint;
      above_constraint.toolFeature() = bottle_axis;
      above_constraint.objectFeature() = oven_plane;
      above_constraint.lowerBoundary() = 0.2;
      above_constraint.upperBoundary() = 0.3;
      above_constraint.maxVelocity() = 0.02;
      above_constraint.maxAcceleration() = 0.1;
      above_constraint.maxJerk() = 0.2;
      above_constraint.semantics().reference().setName("world");
      above_constraint.semantics().name().setName("bottle axis above oven");
      above_constraint.semantics().type().setName("above"); 

      tilting_constraint.toolFeature() = bottle_axis;
      tilting_constraint.objectFeature() = oven_plane;
      tilting_constraint.lowerBoundary() = 0.8;
      tilting_constraint.upperBoundary() = 0.95;
      tilting_constraint.maxVelocity() = 0.1;
      tilting_constraint.maxAcceleration() = 0.2;
      tilting_constraint.maxJerk() = 0.4;
      tilting_constraint.semantics().reference().setName("world");
      tilting_constraint.semantics().name().setName("bottle axis tilted w.r.t. oven plane");
      tilting_constraint.semantics().type().setName("perpendicular"); 

      // collecting constraints
      std::vector<Constraint> cs;
      cs.push_back(above_constraint);
      cs.push_back(tilting_constraint);
      constraints.init(cs); 

      // gains
      p.init(constraints.names());
      i.init(constraints.names());
      d.init(constraints.names());
      i_min.init(constraints.names());
      i_max.init(constraints.names());

      p.numerics().data.setConstant(16.0);
      i.numerics().data.setConstant(0.0);
      d.numerics().data.setConstant(8.0);
      i_min.numerics().data.setConstant(0.0);
      i_max.numerics().data.setConstant(0.0);
    }

    virtual void TearDown()
    {
    }

    Transform oven_transform, bottle_transform;
    TransformMap transform_map;
    ConstraintArray constraints;    
    Feature bottle_axis, oven_plane;
    double cycle_time, delta_deriv;
    JntArray p, i, d, i_min, i_max;
};

TEST_F(CartesianConstraintControllerTest, Basics)
{
  CartesianConstraintController controller;
  controller.init(constraints, cycle_time);
  controller.setGains(p, i, d, i_max, i_min);
  controller.start(transform_map, delta_deriv, cycle_time);
  ASSERT_FALSE(controller.constraints().areFulfilled());
  
  Twist twist_des;

  unsigned int counter = 0;
  while((!controller.constraints().areFulfilled()) && (counter < 20))
  {
    counter++;
    controller.update(transform_map, delta_deriv, cycle_time);

    twist_des = controller.desiredTwist();
    ASSERT_STREQ(twist_des.semantics().reference().getName().c_str(), "bottle");
    ASSERT_STREQ(twist_des.semantics().target().getName().c_str(), "bottle");

    twist_des.changeReferenceFrame(bottle_transform);
    bottle_transform.numerics() = KDL::addDelta(bottle_transform.numerics(),
        twist_des.numerics());

    transform_map.setTransform(bottle_transform);
  }

  EXPECT_TRUE(controller.constraints().areFulfilled());
}

TEST_F(CartesianConstraintControllerTest, NecessaryTransforms)
{
  CartesianConstraintController controller;
  controller.init(constraints, cycle_time);
 
  std::set<TransformSemantics> transforms = controller.necessaryTransforms();
  EXPECT_EQ(transforms.size(), 2);

  std::set<TransformSemantics>::iterator it;
  TransformSemantics lookup;

  lookup.reference() = controller.constraints()(0).semantics().reference();
  lookup.target() = controller.constraints()(0).toolFeature().semantics().reference();
  it = transforms.find(lookup);
  EXPECT_NE(it, transforms.end());

  lookup.target() = controller.constraints()(0).objectFeature().semantics().reference();
  it = transforms.find(lookup);
  EXPECT_NE(it, transforms.end());
}
