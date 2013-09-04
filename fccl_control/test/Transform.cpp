#include <gtest/gtest.h>

#include <fccl_control/Transform.h>
#include <fccl_control/Hashing.h>

using namespace fccl;

class TransformTest : public ::testing::Test
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

TEST_F(TransformTest, Basics)
{
  Transform t(parent_frame, child_frame, transform);
  Transform t2(t);
  Transform t3(t.getReferenceID(), t.getTargetID(), t.getTransform());

  EXPECT_EQ(t, t2);
  EXPECT_EQ(t, t3);
}

TEST_F(TransformTest, Basics2)
{
  Transform t(parent_frame, child_frame, transform);
  Transform t2, t3;

  t2.setReferenceID(t.getReferenceID());
  t2.setTargetID(t.getTargetID());
  t2.setTransform(t.getTransform());

  t3.setReferenceFrame(parent_frame);
  t3.setTargetFrame(child_frame);
  t3.setTransform(transform);

  EXPECT_EQ(t, t2);
  EXPECT_EQ(t, t3);
}

TEST_F(TransformTest, TransformInversion)
{
  Transform t(parent_frame, child_frame, transform);
  Transform t_inv = t.inverse();
  Transform t_inv2(child_frame, parent_frame, transform.Inverse());

  EXPECT_EQ(t_inv, t_inv2);
  EXPECT_NE(t, t_inv);
}

TEST_F(TransformTest, TransformMultiplication)
{
  KDL::Frame T_f_m1, T_m1_m2;
  T_f_m1 = KDL::Frame(KDL::Rotation::EulerZYX(M_PI/2.0, M_PI/3.0, M_PI/4.0), KDL::Vector(0.1, 0.2, 0.3));
  T_m1_m2 = transform;

  Transform t("f", "m1", T_f_m1);
  Transform t2("m1", "m2", T_m1_m2);
  Transform t3 = t*t2;
  Transform t4("f", "m2", T_f_m1*T_m1_m2);

  EXPECT_EQ(t3, t4);
}
