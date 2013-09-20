#include <gtest/gtest.h>

#include <fccl/kdl/Twist.h>

using namespace fccl::kdl;

class TwistTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "parent";
      target = "child";
      transform = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
      twist = KDL::Twist(KDL::Vector(3, 4, 5), KDL::Vector(0,1,2));
    }

    virtual void TearDown()
    {

    }

    std::string reference, target, world;
    KDL::Frame transform;
    KDL::Twist twist;
};

TEST_F(TwistTest, Basics)
{
  Twist t;
  t.semantics().reference().setName(reference);
  t.semantics().target().setName(target);
  t.numerics() = twist;

  Twist t2(t);

  Twist t3 = t;

  Twist t4;
  t4 = t;

  EXPECT_TRUE(t.equals(t2));
  EXPECT_TRUE(t.equals(t3));
  EXPECT_TRUE(t.equals(t4));
}

TEST_F(TwistTest, ChangeReferenceFrame)
{
  Transform trans;
  trans.semantics().reference().setName(world);
  trans.semantics().target().setName(reference);
  trans.numerics() = transform;

  Twist t;
  t.semantics().reference().setName(reference);
  t.semantics().target().setName(target);
  t.numerics() = twist;

  Twist t2(t);
  t2.changeReferenceFrame(trans);

  Twist t3;
  t3.semantics().reference().setName(world);
  t3.semantics().target().setName(target);
  t3.numerics() = KDL::Twist(KDL::Vector(9, -5, 4), KDL::Vector(0, -2, 1));

  Twist t4;
  t4.semantics() = t3.semantics();
  t4.numerics() = transform*twist;

  EXPECT_TRUE(t2.equals(t3));
  EXPECT_FALSE(t.equals(t2));
  EXPECT_TRUE(t3.equals(t4));
}
