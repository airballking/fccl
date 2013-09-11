#include <gtest/gtest.h>

#include <fccl/base/Constraints.h>

using namespace fccl::base;
using namespace fccl::kdl;

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
 
      // semanitic KLD ;)
      T_view_tool = Transform(SemanticObject1x1(view_frame_name, tool_frame_name), T_view_tool_data);
      T_view_object = Transform(SemanticObject1x1(view_frame_name, object_frame_name), T_view_object_data);
      
      // features
      object_feature = Feature(SemanticObject1x1(object_frame_name, object_feature_name), object_feature_pos, object_feature_dir, PLANE_FEATURE);
      tool_feature = Feature(SemanticObject1x1(tool_frame_name, tool_feature_name), tool_feature_pos, tool_feature_dir, LINE_FEATURE);

      //actual constraint values
      lower_boundary = 0.1; 
      upper_boundary = 0.2;

      // our sample constraint
      ac.setReferenceName(view_frame_name);
      ac.setTargetName(constraint_name);
      ac.setToolFeature(tool_feature);
      ac.setObjectFeature(object_feature);
      ac.setLowerBoundary(lower_boundary);
      ac.setUpperBoundary(upper_boundary);
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
    Feature object_feature, tool_feature;
    double lower_boundary, upper_boundary;
    AboveConstraint ac;
};

TEST_F(ConstraintsTest, Basics)
{
  AboveConstraint ac2(ac);

  AboveConstraint ac3(SemanticObject1x1(view_frame_name, constraint_name), tool_feature,
      object_feature, lower_boundary, upper_boundary);

  AboveConstraint ac4;
  ac4 = ac;

  AboveConstraint ac5(SemanticObject1x1(ac.getReferenceID(), ac.getTargetID()), ac.getToolFeature(),
    ac.getObjectFeature(), ac.getLowerBoundary(), ac.getUpperBoundary());

  Constraint c(ac);

  EXPECT_EQ(ac, ac2);
  EXPECT_EQ(ac, ac3);
  EXPECT_EQ(ac, ac4);
  EXPECT_EQ(ac, ac5);

  EXPECT_NE(ac, c);

  EXPECT_EQ(ac.getType(), ABOVE_CONSTRAINT);
  EXPECT_EQ(c.getType(), UNKNOWN_CONSTRAINT);
}

TEST_F(ConstraintsTest, Evaluation)
{
  EXPECT_DOUBLE_EQ(ac.calculateValue(T_view_tool, T_view_object), 0.3);
}

TEST_F(ConstraintsTest, TypeConversion)
{
  AboveConstraint* ac_pointer = &ac;
  Constraint* c_pointer = static_cast<Constraint*>(ac_pointer);
  AboveConstraint* ac_pointer2 = static_cast<AboveConstraint*>(c_pointer);

  EXPECT_DOUBLE_EQ(ac_pointer->calculateValue(T_view_tool, T_view_object), 0.3);
  EXPECT_DOUBLE_EQ(ac_pointer2->calculateValue(T_view_tool, T_view_object), 0.3);
  EXPECT_DOUBLE_EQ(c_pointer->calculateValue(T_view_tool, T_view_object), 0.3);
}

TEST_F(ConstraintsTest, FirstDerivative)
{
  InteractionMatrix m = ac.calculateFirstDerivative(T_view_tool, T_view_object);
  EXPECT_EQ(m.getReferenceID(), tool_feature.getReferenceID());
  ASSERT_EQ(m.getTargetIDs().size(), 1);
  EXPECT_EQ(m.getTargetIDs()[0], ac.getTargetID());
  ASSERT_EQ(m.rows(), 1);
  ASSERT_EQ(m.columns(), 6);
  EXPECT_DOUBLE_EQ(m(0,0), 0);
  EXPECT_DOUBLE_EQ(m(0,1), 0);
  EXPECT_DOUBLE_EQ(m(0,2), 1);
  EXPECT_DOUBLE_EQ(m(0,3), 0);
  EXPECT_DOUBLE_EQ(m(0,4), 0);
  EXPECT_DOUBLE_EQ(m(0,5), 0);
}
