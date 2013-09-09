#include <gtest/gtest.h>

#include <fccl/kdl/Vector.h>

using namespace fccl::kdl;

class VectorTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "parent";
      target = "child";
      transform = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
      vector == KDL::Vector(1.0, 2.0, 3.0);
    }

    virtual void TearDown()
    {

    }

    std::string reference, target, world;
    KDL::Frame transform;
    KDL::Vector vector;
};

TEST_F(VectorTest, Basics)
{
  Vector v(reference, target, vector);
  Vector v2(v);
  Vector v3(v.getReferenceID(), v.getTargetID(), v.getVector());

  EXPECT_EQ(v, v2);
  EXPECT_EQ(v, v3);
}

TEST_F(VectorTest, Basics2)
{
  Vector v(reference, target, vector);
  Vector v2, v3, v4;

  v2.setReferenceID(v.getReferenceID());
  v2.setTargetID(v.getTargetID());
  v2.setVector(v.getVector());

  v3.setReferenceName(reference);
  v3.setTargetName(target);
  v3.setVector(vector);

  v4 = v;

  EXPECT_EQ(v, v2);
  EXPECT_EQ(v, v3);
  EXPECT_EQ(v, v4);
}

TEST_F(VectorTest, TransformMultiplication)
{
  KDL::Frame T = transform;
  Transform trans(world, reference, T);

  Vector v(reference, target, vector);

  Vector v2 = trans*v;
  Vector v3(world, target, T*vector);
  Vector v4(v);
  v4.changeReferenceFrame(trans);

  EXPECT_EQ(v2, v3);
  EXPECT_EQ(v2, v4);
  EXPECT_NE(v, v2);
}
