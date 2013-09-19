#include <gtest/gtest.h>

#include <fccl/kdl/Transform.h>

using namespace fccl::kdl;

class TransformTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      parent_frame = "parent";
      child_frame = "child";

      semantics.reference().setName(parent_frame);
      semantics.target().setName(child_frame);

      transform = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
    }

    virtual void TearDown()
    {

    }

    std::string parent_frame, child_frame;
    fccl::semantics::TransformSemantics semantics;
    KDL::Frame transform;
};

TEST_F(TransformTest, Basics)
{
  Transform t;
  t.numerics() = transform;
  t.semantics() = semantics;

  KDL::Equal(transform, t.numerics());
  EXPECT_TRUE(semantics.equals(t.semantics()));

  Transform t2(t);
  EXPECT_TRUE(t.equals(t2));

  Transform t3;
  t3.numerics() = transform;
  
  Transform t4;
  t4.semantics() = semantics;

  EXPECT_FALSE(t.equals(t3));
  EXPECT_FALSE(t.equals(t4));
}

TEST_F(TransformTest, TransformInversion)
{
  Transform t;
  t.numerics() = transform;
  t.semantics() = semantics;

  Transform t_inv = t.inverse();
  
  Transform t_inv2(t);
  t_inv2.invert();

  Transform t_inv3;
  t_inv3.numerics() = transform.Inverse();
  t_inv3.semantics().reference().setName(child_frame);
  t_inv3.semantics().target().setName(parent_frame);

  EXPECT_TRUE(t_inv.equals(t_inv2));
  EXPECT_TRUE(t_inv.equals(t_inv3));

  EXPECT_FALSE(t.equals(t_inv));
}

TEST_F(TransformTest, TransformMultiplication)
{
  KDL::Frame T_f_m1, T_m1_m2;
  T_f_m1 = KDL::Frame(KDL::Rotation::EulerZYX(M_PI/2.0, M_PI/3.0, M_PI/4.0), KDL::Vector(0.1, 0.2, 0.3));
  T_m1_m2 = transform;

  Transform t;
  t.numerics() = T_f_m1;
  t.semantics().reference().setName("f");
  t.semantics().target().setName("m1");

  Transform t2;
  t2.numerics() = T_m1_m2;
  t2.semantics().reference().setName("m1");
  t2.semantics().target().setName("m2");


  Transform t3 = multiply(t, t2);

  Transform t4;
  t4.numerics() = T_f_m1*T_m1_m2;
  t4.semantics().reference().setName("f");
  t4.semantics().target().setName("m2");

  EXPECT_TRUE(t3.equals(t4));
}
