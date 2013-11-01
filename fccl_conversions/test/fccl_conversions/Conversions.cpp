#include <gtest/gtest.h>

#include <fccl_conversions/Conversions.h>

using namespace fccl::base;
using namespace fccl::conversions;

class ConversionsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      feature.semantics().reference().setName("world");
      feature.semantics().name().setName("some edge");
      feature.semantics().type() = fccl::semantics::POINT_FEATURE;
      feature.position() = KDL::Vector(1,2,3);
      feature.orientation() = KDL::Vector(-1,-1,-1);

      Feature some_feature;
      some_feature.semantics().reference().setName("reference");
      some_feature.semantics().name().setName("some other edge");
      some_feature.semantics().type() = fccl::semantics::LINE_FEATURE;
      some_feature.position() = KDL::Vector(4,5,6);
      some_feature.orientation() = KDL::Vector(-2,-3,-4);

      Feature some_feature2;
      some_feature2.semantics().reference().setName("reference");
      some_feature2.semantics().name().setName("some other edge2");
      some_feature2.semantics().type() = fccl::semantics::PLANE_FEATURE;
      some_feature2.position() = KDL::Vector(40,50,60);
      some_feature2.orientation() = KDL::Vector(-20,-30,-40);

      constraint.semantics().name().setName("some constraint");
      constraint.semantics().reference().setName("view");
      constraint.semantics().type().setName("above");
      constraint.toolFeature() = some_feature;
      constraint.objectFeature() = feature;
      constraint.lowerBoundary() = -0.1;
      constraint.upperBoundary() = 0.2;

      Constraint some_constraint;
      some_constraint.semantics().name().setName("some other constraint");
      some_constraint.semantics().reference().setName("view");
      some_constraint.semantics().type().setName("above");
      some_constraint.toolFeature() = some_feature;
      some_constraint.objectFeature() = some_feature2;
      some_constraint.lowerBoundary() = -0.1234;
      some_constraint.upperBoundary() = 0.2345;

      constraints.resize(2);
      constraints(0) = constraint;
      constraints(1) = some_constraint;
      constraints.prepare();
    }

    virtual void TearDown()
    {
    }

    Feature feature;
    Constraint constraint;
    ConstraintArray constraints;
};

TEST_F(ConversionsTest, Feature)
{
  fccl_msgs::Feature msg;
  toMsg(feature, msg);
  Feature feature2;
  fromMsg(msg, feature2);
  EXPECT_TRUE(feature.equals(feature2));

  Feature feature3 = fromMsg(toMsg(feature));
  EXPECT_TRUE(feature.equals(feature3));
}

TEST_F(ConversionsTest, Constraint)
{
  fccl_msgs::Constraint msg;
  toMsg(constraint, msg);
  Constraint constraint2;
  fromMsg(msg, constraint2);
  EXPECT_TRUE(constraint.equals(constraint2)); 

  Constraint constraint3 = fromMsg(toMsg(constraint));
  EXPECT_TRUE(constraint.equals(constraint3));
}

TEST_F(ConversionsTest, Constraints)
{
  ASSERT_TRUE(constraints.isValid());
  std::vector<fccl_msgs::Constraint> msg;
  msg.resize(constraints.size());
  toMsg(constraints, msg); 
  ConstraintArray constraints2;
  constraints2.resize(constraints.size());
  fromMsg(msg, constraints2);
  EXPECT_TRUE(constraints.equals(constraints2));

  ConstraintArray constraints3 = fromMsg(toMsg(constraints));
  EXPECT_TRUE(constraints.equals(constraints3));
}
