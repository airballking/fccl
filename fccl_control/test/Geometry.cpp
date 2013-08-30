#include <gtest/gtest.h>

#include <fccl_control/Geometry.h>

using namespace fccl;

class GeometryTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      parent_frame = "parent";
      child_frame = "child";
      transform = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
    }

    virtual void TearDown()
    {

    }

    std::string parent_frame, child_frame;
    KDL::Frame transform;
    KDL::Vector vector;
};

TEST_F(GeometryTest, TransformConstructor)
{
  Transform t(parent_frame, child_frame, transform);
  EXPECT_STREQ(parent_frame.c_str(), t.getParentFrame().c_str()); 
  EXPECT_STREQ(child_frame.c_str(), t.getChildFrame().c_str());
  EXPECT_TRUE(KDL::Equal(transform, t.getTransform()));
}

TEST_F(GeometryTest, TransformGettersAndSetters)
{
  Transform t;
  t.setParentFrame(parent_frame);
  t.setChildFrame(child_frame);
  t.setTransform(transform);
  EXPECT_STREQ(parent_frame.c_str(), t.getParentFrame().c_str()); 
  EXPECT_STREQ(child_frame.c_str(), t.getChildFrame().c_str());
  EXPECT_TRUE(KDL::Equal(transform, t.getTransform()));
}

TEST_F(GeometryTest, VectorConstructor)
{
  Vector v(vector, parent_frame);
  EXPECT_STREQ(parent_frame.c_str(), v.getFrameName().c_str());
  EXPECT_TRUE(KDL::Equal(vector, v.getVector()));
}

TEST_F(GeometryTest, VectorGettersAndSetters)
{
  Vector v;
  v.setFrameName(parent_frame);
  v.setVector(vector);
  EXPECT_STREQ(parent_frame.c_str(), v.getFrameName().c_str());
  EXPECT_TRUE(KDL::Equal(vector, v.getVector()));
}

TEST_F(GeometryTest, VectorTransformation)
{
  Transform t(parent_frame, child_frame, transform);
  Vector v(vector, child_frame);
  EXPECT_FALSE(v.isTransformApplicable(t));
  v.setFrameName(parent_frame);
  ASSERT_TRUE(v.isTransformApplicable(t));
  v.changeReferenceFrame(t);
  EXPECT_STREQ(v.getFrameName().c_str(), child_frame.c_str());
  EXPECT_TRUE(KDL::Equal(v.getVector(), transform.Inverse(vector)));
  EXPECT_TRUE(KDL::Equal(v.getVector(), KDL::Vector(0.0, -3.0, 0.0)));
}
