#include <gtest/gtest.h>

#include <fccl/base/ConstraintArray.h>
#include <fccl/utils/TransformMap.h>
#include <vector>

using namespace fccl::base;
using namespace fccl::semantics;

class ConstraintArrayTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      // frame names
      tool = "tool";
      object = "object";
      view = "view";

      // features
      tool_feature.semantics().reference().setName(tool);
      tool_feature.semantics().name().setName("tool plane");
      tool_feature.semantics().type() = fccl::semantics::PLANE_FEATURE;
      tool_feature.position() = KDL::Vector(0,0,0);
      tool_feature.orientation() = KDL::Vector(0,0,1);

      object_feature.semantics().reference().setName(object);
      object_feature.semantics().name().setName("object corner");
      object_feature.semantics().type() = fccl::semantics::POINT_FEATURE;
      object_feature.position() = KDL::Vector(0,0,0);

      object_feature2.semantics().reference().setName(object);
      object_feature2.semantics().name().setName("object edge");
      object_feature2.semantics().type() = fccl::semantics::LINE_FEATURE;
      object_feature2.position() = KDL::Vector(0,0,0.02);
      object_feature2.orientation() = KDL::Vector(0,0,1);

      // constraints
      ac1.semantics().reference().setName(view);
      ac1.semantics().name().setName("height of tool over object corner");
      ac1.semantics().type().setName("above");
      ac1.toolFeature() = tool_feature;
      ac1.objectFeature() = object_feature;
      ac1.lowerBoundary() = 0.1;
      ac1.upperBoundary() = 0.2;
      ac1.maxVelocity() = 0.3;
      ac1.maxAcceleration() = 0.4;
      ac1.maxJerk() = 0.5;

      ac2.semantics().reference().setName(view);
      ac2.semantics().name().setName("height of tool over object edge");
      ac2.semantics().type().setName("above");
      ac2.toolFeature() = tool_feature;
      ac2.objectFeature() = object_feature2;
      ac2.lowerBoundary() = 0.1;
      ac2.upperBoundary() = 0.2;
      ac2.maxVelocity() = 1.3;
      ac2.maxAcceleration() = 1.4;
      ac2.maxJerk() = 1.5;

      constraints.push_back(ac1);
      constraints.push_back(ac2);

      // transforms
      tool_transform.semantics().reference().setName(view);
      tool_transform.semantics().target().setName(tool);
      tool_transform.numerics() = 
          KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0.4, -0.3, -0.2));

      object_transform.semantics().reference().setName(view);
      object_transform.semantics().target().setName(object);
      object_transform.numerics() = 
          KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.5, 0.0, -0.5));

      transform_map.setTransform(tool_transform);
      transform_map.setTransform(object_transform);
    }

    virtual void TearDown()
    {
    }

    std::string tool, object, view;
    std::vector<Constraint> constraints;
    Feature tool_feature, object_feature, object_feature2;
    Constraint ac1, ac2;
    fccl::kdl::Transform tool_transform, object_transform;
    fccl::utils::TransformMap transform_map;
};

TEST_F(ConstraintArrayTest, Basics)
{
  ConstraintArray cs;
  ASSERT_TRUE(cs.isValid());
  ASSERT_EQ(cs.size(), 0);  

  cs.resize(2);
  ASSERT_FALSE(cs.isValid());
  cs.prepare();
  ASSERT_FALSE(cs.isValid());
  ASSERT_EQ(cs.size(), 2);
  
  cs(0) = ac1;
  cs.constraints()[1] = ac2;
  cs.prepare();
  ASSERT_TRUE(cs.isValid());
  ASSERT_EQ(cs.size(), 2);
  EXPECT_TRUE(cs(0).equals(ac1));
  EXPECT_TRUE(cs(1).equals(ac2));

  ConstraintArray cs2;
  EXPECT_FALSE(cs.equals(cs2));
  cs2.init(constraints);
  EXPECT_TRUE(cs2.isValid());
  EXPECT_TRUE(cs.equals(cs2));

  ConstraintArray cs3(cs);
  EXPECT_TRUE(cs.equals(cs3));

  ConstraintArray cs4;
  cs4 = cs;
  EXPECT_TRUE(cs.equals(cs4));

  ConstraintArray cs5 = cs;
  EXPECT_TRUE(cs.equals(cs5));
}

TEST_F(ConstraintArrayTest, OutputValues)
{
  ConstraintArray cs;
  cs.init(constraints);

  cs.update(transform_map, 0.001); 
  ASSERT_TRUE(cs.isValid());
  ASSERT_TRUE(cs.outputValues().isValid());
  ASSERT_EQ(cs.size(), cs.outputValues().size());
  ASSERT_EQ(cs.size(), 2);

  EXPECT_TRUE(cs.outputValues().semantics()(0).equals(cs(0).semantics().name()));
  EXPECT_TRUE(cs.outputValues().semantics()(1).equals(cs(1).semantics().name()));

  EXPECT_DOUBLE_EQ(cs.outputValues().numerics()(0), 0.3);
  EXPECT_DOUBLE_EQ(cs.outputValues().numerics()(1), 0.28);
}

TEST_F(ConstraintArrayTest, DesiredOutputValues)
{
  ConstraintArray cs;
  cs.init(constraints);

  cs.update(transform_map, 0.001); 
  ASSERT_TRUE(cs.isValid());
  ASSERT_TRUE(cs.desiredOutputValues().isValid());
  ASSERT_EQ(cs.size(), cs.desiredOutputValues().size());
  ASSERT_EQ(cs.size(), 2);

  EXPECT_TRUE(cs.desiredOutputValues().semantics()(0).equals(cs(0).semantics().name()));
  EXPECT_TRUE(cs.desiredOutputValues().semantics()(1).equals(cs(1).semantics().name()));

  EXPECT_DOUBLE_EQ(cs.desiredOutputValues().numerics()(0), 0.15);
  EXPECT_DOUBLE_EQ(cs.desiredOutputValues().numerics()(1), 0.15);
}

TEST_F(ConstraintArrayTest, TaskWeights)
{
  ConstraintArray cs;
  cs.init(constraints);

  cs.update(transform_map, 0.001); 
  ASSERT_TRUE(cs.isValid());
  ASSERT_TRUE(cs.taskWeights().isValid());
  ASSERT_EQ(cs.size(), cs.taskWeights().size());
  ASSERT_EQ(cs.size(), 2);

  EXPECT_TRUE(cs.taskWeights().semantics()(0).equals(cs(0).semantics().name()));
  EXPECT_TRUE(cs.taskWeights().semantics()(1).equals(cs(1).semantics().name()));

  EXPECT_DOUBLE_EQ(cs.taskWeights().numerics()(0), 1.0);
  EXPECT_DOUBLE_EQ(cs.taskWeights().numerics()(1), 1.0);
}

TEST_F(ConstraintArrayTest, FirstDerivative)
{
  ConstraintArray cs;
  cs.init(constraints);

  cs.update(transform_map, 0.001); 
  ASSERT_TRUE(cs.isValid());
  ASSERT_TRUE(cs.firstDerivative().isValid());
  ASSERT_EQ(cs.size(), cs.firstDerivative().size());
  ASSERT_EQ(cs.size(), 2);

  EXPECT_TRUE(cs.firstDerivative().semantics().joints()(0).equals(cs(0).semantics().name()));
  EXPECT_TRUE(cs.firstDerivative().semantics().joints()(1).equals(cs(1).semantics().name()));
  EXPECT_TRUE(cs.firstDerivative().semantics().twist().reference().equals(cs(0).toolFeature().semantics().reference()));
  EXPECT_TRUE(cs.firstDerivative().semantics().twist().target().equals(cs(0).toolFeature().semantics().reference()));

  Eigen::Matrix< double, 2, 6 > data;
  data.setZero();
  data(0, 2) = 1.0;
  data(1, 2) = 1.0;
  EXPECT_TRUE(data.isApprox(cs.firstDerivative().numerics()));
}

TEST_F(ConstraintArrayTest, NecessaryTransforms)
{
  ConstraintArray cs;
  cs.init(constraints);

  std::set<TransformSemantics> transforms = cs.necessaryTransforms();
  EXPECT_EQ(transforms.size(), 2);

  std::set<TransformSemantics>::iterator it;

  TransformSemantics container;
  container.reference().setName(view);
  container.target().setName(tool);

  it = transforms.find(container);
  EXPECT_NE(it, transforms.end());

  container.target().setName(object);
  it = transforms.find(container);
  EXPECT_NE(it, transforms.end());
}

TEST_F(ConstraintArrayTest, Limits)
{
  ConstraintArray cs;
  cs.init(constraints);

  ASSERT_EQ(cs.maxVelocities().size(), constraints.size());
  ASSERT_EQ(cs.maxAccelerations().size(), constraints.size());
  ASSERT_EQ(cs.maxJerks().size(), constraints.size());

  EXPECT_TRUE(cs.maxVelocities().semantics().equals(cs.maxAccelerations().semantics()));
  EXPECT_TRUE(cs.maxVelocities().semantics().equals(cs.maxJerks().semantics()));

  for(std::size_t i=0; i<constraints.size(); i++)
  {
    EXPECT_TRUE(cs.maxVelocities().semantics()(i).equals(cs(i).semantics().name()));
    EXPECT_TRUE(cs.maxAccelerations().semantics()(i).equals(cs(i).semantics().name()));
    EXPECT_TRUE(cs.maxJerks().semantics()(i).equals(cs(i).semantics().name()));
  }

  EXPECT_DOUBLE_EQ(cs.maxVelocities().numerics()(0), constraints[0].maxVelocity());
  EXPECT_DOUBLE_EQ(cs.maxAccelerations().numerics()(0), constraints[0].maxAcceleration());
  EXPECT_DOUBLE_EQ(cs.maxJerks().numerics()(0), constraints[0].maxJerk());

  EXPECT_DOUBLE_EQ(cs.maxVelocities().numerics()(1), constraints[1].maxVelocity());
  EXPECT_DOUBLE_EQ(cs.maxAccelerations().numerics()(1), constraints[1].maxAcceleration());
  EXPECT_DOUBLE_EQ(cs.maxJerks().numerics()(1), constraints[1].maxJerk());


  EXPECT_DOUBLE_EQ(cs.maxVelocities().numerics()(0), 0.3);
  EXPECT_DOUBLE_EQ(cs.maxAccelerations().numerics()(0), 0.4);
  EXPECT_DOUBLE_EQ(cs.maxJerks().numerics()(0), 0.5);

  EXPECT_DOUBLE_EQ(cs.maxVelocities().numerics()(1), 1.3);
  EXPECT_DOUBLE_EQ(cs.maxAccelerations().numerics()(1), 1.4);
  EXPECT_DOUBLE_EQ(cs.maxJerks().numerics()(1), 1.5);
}
