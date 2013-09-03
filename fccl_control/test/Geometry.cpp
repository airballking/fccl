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

  Transform t2(t);
  EXPECT_STREQ(t.getParentFrame().c_str(), t2.getParentFrame().c_str());
  EXPECT_STREQ(t.getChildFrame().c_str(), t2.getChildFrame().c_str());
  EXPECT_TRUE(KDL::Equal(t.getTransform(), t2.getTransform()));
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

TEST_F(GeometryTest, TransformInversion)
{
  Transform t(parent_frame, child_frame, transform);
  Transform t2(t);
  t2.invert();
  EXPECT_STREQ(t2.getParentFrame().c_str(), child_frame.c_str());
  EXPECT_STREQ(t2.getChildFrame().c_str(), parent_frame.c_str());
  t*=t2;
  EXPECT_STREQ(parent_frame.c_str(), t.getParentFrame().c_str());
  EXPECT_STREQ(parent_frame.c_str(), t.getChildFrame().c_str());
  EXPECT_TRUE(KDL::Equal(t.getTransform(), KDL::Frame::Identity()));
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

TEST_F(GeometryTest, VectorComparison)
{
  Vector v(vector, parent_frame);
  Vector v2(vector, parent_frame);
  Vector v3(vector+KDL::Vector(1,1,1), parent_frame);
  Vector v4(vector, child_frame);

  EXPECT_TRUE(v == v2);
  EXPECT_FALSE(v != v2);

  EXPECT_FALSE(v == v3);
  EXPECT_TRUE(v != v3);

  EXPECT_FALSE(v == v4);
  EXPECT_TRUE(v != v4);

  EXPECT_EQ(v, v2);
  EXPECT_NE(v, v3);
  EXPECT_NE(v, v4);
}

TEST_F(GeometryTest, TwistDerivative)
{
  TwistDerivative td("child", "bla");
  for(unsigned int i=0; i<6; i++)
    td(i) = i;
  
  Transform tf(parent_frame, child_frame, transform);

  td.changeReferenceFrame(tf);

  EXPECT_DOUBLE_EQ(td(0), 0);
  EXPECT_DOUBLE_EQ(td(1), -2);
  EXPECT_DOUBLE_EQ(td(2), 1);
  EXPECT_DOUBLE_EQ(td(3), 9);
  EXPECT_DOUBLE_EQ(td(4), -5);
  EXPECT_DOUBLE_EQ(td(5), 4);
}
