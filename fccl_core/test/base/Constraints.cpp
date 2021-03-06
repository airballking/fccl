#include <gtest/gtest.h>

#include <fccl/base/Constraints.h>

using namespace fccl::base;
using namespace fccl::kdl;
using namespace fccl::semantics;
using namespace fccl::utils;

class ConstraintsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      // names/identifiers of everything in the world
      view_frame_name = "view";
      tool_frame_name = "spatula blade";
      object_frame_name = "pancake";

      tool_feature_name = "blade front edge";
      object_feature_name ="pancake center plane";

      constraint_name = "height of spatula front edge over pancake center seen from view";

      // numeric data
      tool_feature_pos = KDL::Vector(0,0,0);
      tool_feature_dir = KDL::Vector(1,0,0);
      object_feature_pos = KDL::Vector(0,0,0);
      object_feature_dir = KDL::Vector(0,0,1);
      T_view_tool_data = KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0.4, -0.3, -0.2));
      T_view_object_data = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.5, 0.0, -0.5));
 
      // semanitic transforms
      T_view_tool.semantics().reference().setName(view_frame_name);
      T_view_tool.semantics().target().setName(tool_frame_name);
      T_view_tool.numerics() = T_view_tool_data;

      T_view_object.semantics().reference().setName(view_frame_name);
      T_view_object.semantics().target().setName(object_frame_name);
      T_view_object.numerics() = T_view_object_data;

      // transform map
      transform_map.clear();
      transform_map.setTransform(T_view_tool);
      transform_map.setTransform(T_view_object);

      // features
      object_feature.semantics().reference().setName(object_frame_name);
      object_feature.semantics().name().setName(object_feature_name);
      object_feature.semantics().type() = fccl::semantics::PLANE_FEATURE;
      object_feature.position() = object_feature_pos;
      object_feature.orientation() = object_feature_dir;

      tool_feature.semantics().reference().setName(tool_frame_name);
      tool_feature.semantics().name().setName(tool_feature_name);
      tool_feature.semantics().type() = fccl::semantics::LINE_FEATURE;
      tool_feature.position() = tool_feature_pos;
      tool_feature.orientation() = tool_feature_dir;

      //actual constraint values
      lower_boundary = 0.1; 
      upper_boundary = 0.2;

      //actual limits
      max_vel = 0.5;
      max_acc = 0.6;
      max_jerk = 0.7;
   }

    virtual void TearDown()
    {
    }

    // our tiny little world
    std::string view_frame_name, tool_frame_name, object_frame_name;
    std::string tool_feature_name, object_feature_name;
    std::string constraint_name;
    KDL::Vector object_feature_pos, object_feature_dir;
    KDL::Vector tool_feature_pos, tool_feature_dir;
    KDL::Frame T_view_tool_data, T_view_object_data;
    Transform T_view_tool, T_view_object;
    TransformMap transform_map;
    Feature object_feature, tool_feature;
    double lower_boundary, upper_boundary;
    double max_vel, max_acc, max_jerk;
};

TEST_F(ConstraintsTest, Basics)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
  ac.maxVelocity() = max_vel;
  ac.maxAcceleration() = max_acc;
  ac.maxJerk() = max_jerk;
 
  Constraint ac2(ac);

  Constraint ac3 = ac;

  Constraint ac4;
  ac4 = ac;

  Constraint ac5;
  ac5.semantics() = ac.semantics();
  ac5.toolFeature() = ac.toolFeature();
  ac5.objectFeature() = ac.objectFeature();
  ac5.lowerBoundary() = ac.lowerBoundary();
  ac5.upperBoundary() = ac.upperBoundary();
  ac5.maxVelocity() = ac.maxVelocity();
  ac5.maxAcceleration() = ac.maxAcceleration();
  ac5.maxJerk() = ac.maxJerk();

  EXPECT_TRUE(ac.equals(ac2));
  EXPECT_TRUE(ac.equals(ac3));
  EXPECT_TRUE(ac.equals(ac4));
  EXPECT_TRUE(ac.equals(ac5));

  EXPECT_STREQ(ac.semantics().type().getName().c_str(), "above");
  EXPECT_STREQ(ac2.semantics().type().getName().c_str(), "above");

  EXPECT_STREQ(ac.semantics().reference().getName().c_str(), view_frame_name.c_str());
  EXPECT_STREQ(ac.semantics().name().getName().c_str(), constraint_name.c_str());
  EXPECT_TRUE(ac.toolFeature().equals(tool_feature));
  EXPECT_TRUE(ac.objectFeature().equals(object_feature));
  EXPECT_EQ(ac.lowerBoundary(), lower_boundary);
  EXPECT_EQ(ac.upperBoundary(), upper_boundary);
  EXPECT_EQ(ac.maxVelocity(), max_vel);
  EXPECT_EQ(ac.maxAcceleration(), max_acc);
  EXPECT_EQ(ac.maxJerk(), max_jerk);

  EXPECT_STREQ(ac2.semantics().reference().getName().c_str(), view_frame_name.c_str());
  EXPECT_STREQ(ac2.semantics().name().getName().c_str(), constraint_name.c_str());
  EXPECT_TRUE(ac2.toolFeature().equals(tool_feature));
  EXPECT_TRUE(ac2.objectFeature().equals(object_feature));
  EXPECT_EQ(ac2.lowerBoundary(), lower_boundary);
  EXPECT_EQ(ac2.upperBoundary(), upper_boundary);
  EXPECT_EQ(ac2.maxVelocity(), max_vel);
  EXPECT_EQ(ac2.maxAcceleration(), max_acc);
  EXPECT_EQ(ac2.maxJerk(), max_jerk);
}

TEST_F(ConstraintsTest, Evaluation)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ASSERT_TRUE(ac.functionValid());
  ASSERT_TRUE(ac.isValid());
  ac.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(ac.outputValue(), 0.3);
}

TEST_F(ConstraintsTest, FunctionValid)
{
  Constraint c;
  EXPECT_FALSE(c.functionValid());

  c.semantics().type().setName("above");
  EXPECT_TRUE(c.functionValid());
}

TEST_F(ConstraintsTest, FirstDerivative)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ac.update(T_view_tool, T_view_object);
  InteractionMatrix m = ac.firstDerivative();
  EXPECT_EQ(m.semantics().twist().reference().getID(), tool_feature.semantics().reference().getID());
  ASSERT_EQ(m.size(), 1);
  EXPECT_EQ(m.semantics().joints()(0).getID(), ac.semantics().name().getID());
  ASSERT_EQ(m.numerics().rows(), 1);
  ASSERT_EQ(m.numerics().cols(), 6);
  EXPECT_DOUBLE_EQ(m.numerics()(0,0), 0);
  EXPECT_DOUBLE_EQ(m.numerics()(0,1), 0);
  EXPECT_DOUBLE_EQ(m.numerics()(0,2), 1);
  EXPECT_DOUBLE_EQ(m.numerics()(0,3), 0);
  EXPECT_DOUBLE_EQ(m.numerics()(0,4), 0);
  EXPECT_DOUBLE_EQ(m.numerics()(0,5), 0);
}

TEST_F(ConstraintsTest, NecessaryTransforms)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  std::set<TransformSemantics> transforms = ac.necessaryTransforms();
  EXPECT_EQ(transforms.size(), 2);

  std::set<TransformSemantics>::iterator it;

  TransformSemantics container;
  container.reference() = ac.semantics().reference();
  container.target() = ac.toolFeature().semantics().reference();

  it = transforms.find(container);
  EXPECT_NE(it, transforms.end());

  container.target() = ac.objectFeature().semantics().reference();
  it = transforms.find(container);
  EXPECT_NE(it, transforms.end());
}

TEST_F(ConstraintsTest, isFulfilled)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ac.update(T_view_tool, T_view_object);
  EXPECT_FALSE(ac.isFulfilled());

  ac.upperBoundary() = 0.3;
  ac.update(T_view_tool, T_view_object);
  EXPECT_FALSE(ac.isFulfilled());

  ac.upperBoundary() = 0.31;
  ac.update(T_view_tool, T_view_object);
  EXPECT_TRUE(ac.isFulfilled());

  ac.lowerBoundary() = 0.3 - 0.0001;
  ac.upperBoundary() = 0.3 + 0.0001;
  ac.update(T_view_tool, T_view_object);
  EXPECT_TRUE(ac.isFulfilled());
}

TEST_F(ConstraintsTest, TransformMapUpdate)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ASSERT_TRUE(ac.functionValid());
  ASSERT_TRUE(ac.isValid());
  ac.update(transform_map);
  EXPECT_DOUBLE_EQ(ac.outputValue(), 0.3);
}

TEST_F(ConstraintsTest, Control)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("above");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ASSERT_TRUE(ac.functionValid());
  ASSERT_TRUE(ac.isValid());
  ac.update(transform_map);
 
  EXPECT_DOUBLE_EQ(ac.outputValue(), 0.3);
  EXPECT_DOUBLE_EQ(ac.desiredOutputValue(), 0.15);
  EXPECT_DOUBLE_EQ(ac.taskWeight(), 1.0);
}

// TODO(Georg): move these test into separate TestSuite for ConstraintFunctions
TEST_F(ConstraintsTest, BelowFunction)
{
  Constraint ac;
  ac.semantics().reference().setName(view_frame_name);
  ac.semantics().name().setName(constraint_name);
  ac.semantics().type().setName("below");
  ac.toolFeature() = tool_feature;
  ac.objectFeature() = object_feature;
  ac.lowerBoundary() = lower_boundary;
  ac.upperBoundary() = upper_boundary;
 
  ASSERT_TRUE(ac.functionValid());
  ASSERT_TRUE(ac.isValid());
  ac.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(ac.outputValue(), -0.3);
}

TEST_F(ConstraintsTest, LeftRightFunctions)
{
  Constraint leftc, rightc;
  leftc.semantics().reference().setName(view_frame_name);
  leftc.semantics().name().setName(constraint_name);
  leftc.semantics().type().setName("left");
  leftc.toolFeature() = tool_feature;
  leftc.objectFeature() = object_feature;
  leftc.lowerBoundary() = lower_boundary;
  leftc.upperBoundary() = upper_boundary;

  rightc = leftc;
  rightc.semantics().type().setName("right");
 
  ASSERT_TRUE(leftc.functionValid());
  ASSERT_TRUE(leftc.isValid());
  leftc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(leftc.outputValue(), -0.3);

  ASSERT_TRUE(rightc.functionValid());
  ASSERT_TRUE(rightc.isValid());
  rightc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(leftc.outputValue(), -rightc.outputValue());
}

TEST_F(ConstraintsTest, BehindInfrontFunctions)
{
  Constraint behind, infrontc;
  behind.semantics().reference().setName(view_frame_name);
  behind.semantics().name().setName(constraint_name);
  behind.semantics().type().setName("behind");
  behind.toolFeature() = tool_feature;
  behind.objectFeature() = object_feature;
  behind.lowerBoundary() = lower_boundary;
  behind.upperBoundary() = upper_boundary;

  infrontc = behind;
  infrontc.semantics().type().setName("infront");
 
  ASSERT_TRUE(behind.functionValid());
  ASSERT_TRUE(behind.isValid());
  behind.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(behind.outputValue(), -0.1);

  ASSERT_TRUE(infrontc.functionValid());
  ASSERT_TRUE(infrontc.isValid());
  infrontc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(behind.outputValue(), -infrontc.outputValue());
}

TEST_F(ConstraintsTest, PerpendicularFunction)
{
  Constraint perpc;
  perpc.semantics().reference().setName(view_frame_name);
  perpc.semantics().name().setName(constraint_name);
  perpc.semantics().type().setName("perpendicular");
  perpc.toolFeature() = tool_feature;
  perpc.objectFeature() = object_feature;
  perpc.lowerBoundary() = lower_boundary;
  perpc.upperBoundary() = upper_boundary;

  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.0);

  perpc.toolFeature().orientation() = KDL::Vector(0, -1, 0);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.0);

  perpc.toolFeature().orientation() = KDL::Vector(0, 0, -1);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), -1.0);

  // sanity tests added because Yuen asked for them
  T_view_tool.numerics() = KDL::Frame::Identity();
  T_view_object.numerics() = KDL::Frame::Identity();

  perpc.toolFeature().orientation() = KDL::Vector(0, 0, 1);
  perpc.objectFeature().orientation() = KDL::Vector(-1, 0, 0);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.0);

  perpc.toolFeature().orientation() = KDL::Vector(0, 1, 0);
  perpc.objectFeature().orientation() = KDL::Vector(0, 1, 0);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 1.0);

  perpc.toolFeature().orientation() = KDL::Vector(1, 0, 0);
  perpc.objectFeature().orientation() = KDL::Vector(-1, 0, 0);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), -1.0);

  perpc.toolFeature().orientation() = KDL::Vector(0, 0, 1);
  perpc.objectFeature().orientation() = KDL::Vector(0, 0.5, 0.5);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.7071067811865475);

  perpc.toolFeature().orientation() = KDL::Vector(0, 0, 1);
  perpc.objectFeature().orientation() = KDL::Vector(0.5, 0.5, 0.5);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.5773502691896258);

  perpc.toolFeature().orientation() = KDL::Vector(-0.4620588374492333, 0.8675023433618231, 0.18423168836225917);
  perpc.objectFeature().orientation() = KDL::Vector(-1.4999310963025327E-5, 1.4377279851653142E-7, 0.9999999998875);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.18423874362895884);

  perpc.toolFeature().orientation() = KDL::Vector(-0.787534302288049, 0.6148545577321307, 0.0417563834126201);
  perpc.objectFeature().orientation() = KDL::Vector(-3.777464064713236E-8, -4.999857305595393E-6, 0.9999999999875);
  ASSERT_TRUE(perpc.isValid());
  perpc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(perpc.outputValue(), 0.04175333897587106);


}

TEST_F(ConstraintsTest, PointingFunction)
{
  Constraint pointingc;
  pointingc.semantics().reference().setName(view_frame_name);
  pointingc.semantics().name().setName(constraint_name);
  pointingc.semantics().type().setName("pointing");
  pointingc.toolFeature() = tool_feature;
  pointingc.objectFeature() = object_feature;
  pointingc.lowerBoundary() = lower_boundary;
  pointingc.upperBoundary() = upper_boundary;

  ASSERT_TRUE(pointingc.isValid());
  T_view_tool.numerics() = 
      KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0));
  T_view_object.numerics() = 
      KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(1, 0, 0));
  pointingc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(pointingc.outputValue(), 0.0);

  T_view_object.numerics() = 
     KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(-1, 0, 0));
  pointingc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(pointingc.outputValue(), -2.0);

  T_view_object.numerics() = 
     KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 1, 0));
  pointingc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(pointingc.outputValue(), -1.0);

  T_view_object.numerics() = 
     KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 1));
  pointingc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(pointingc.outputValue(), -1.0);

  T_view_object.numerics() = 
     KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0));
  pointingc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(pointingc.outputValue(), 0.0);
}

TEST_F(ConstraintsTest, DistanceFunction)
{
  Constraint distc;
  distc.semantics().reference().setName(view_frame_name);
  distc.semantics().name().setName(constraint_name);
  distc.semantics().type().setName("distance");
  distc.toolFeature() = tool_feature;
  distc.objectFeature() = object_feature;
  distc.lowerBoundary() = lower_boundary;
  distc.upperBoundary() = upper_boundary;

  ASSERT_TRUE(distc.isValid());
  distc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(distc.outputValue(), std::sqrt(0.19));

  T_view_tool.numerics() = KDL::Frame::Identity();
  T_view_object.numerics() = KDL::Frame::Identity();
  distc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(distc.outputValue(), 0.0);

  T_view_tool.numerics() = 
      KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(3.0, 4.0, 0.0));
  T_view_object.numerics() = KDL::Frame::Identity();
  distc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(distc.outputValue(), 5.0);

  T_view_tool.numerics() = 
      KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(3.0, 4.0, 0.0));
  T_view_object.numerics() = 
      KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0.0, 0.0, 0.0));
  distc.update(T_view_tool, T_view_object);
  EXPECT_DOUBLE_EQ(distc.outputValue(), 5.0);
}
