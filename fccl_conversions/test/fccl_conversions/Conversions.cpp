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


      constraint.semantics().name().setName("some constraint");
      constraint.semantics().reference().setName("view");
      constraint.semantics().type().setName("above");
      constraint.toolFeature() = feature;
      constraint.objectFeature() = some_feature;
      constraint.lowerBoundary() = -0.1;
      constraint.upperBoundary() = 0.2;
    }

    virtual void TearDown()
    {
    }

    Feature feature;
    Constraint constraint;
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
