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
  Twist t(reference, target, twist);
  Twist t2(t);
  Twist t3(t.getReferenceID(), t.getTargetID(), t.getTwist());

  EXPECT_EQ(t, t2);
  EXPECT_EQ(t, t3);
}

TEST_F(TwistTest, Basics2)
{
  Twist t(reference, target, twist);
  Twist t2, t3;

  t2.setReferenceID(t.getReferenceID());
  t2.setTargetID(t.getTargetID());
  t2.setTwist(t.getTwist());

  t3.setReferenceName(reference);
  t3.setTargetName(target);
  t3.setTwist(twist);

  EXPECT_EQ(t, t2);
  EXPECT_EQ(t, t3);
}

TEST_F(TwistTest, ChangeReferenceFrame)
{
  Transform trans(world, reference, transform);

  Twist t(reference, target, twist);

  Twist t2(t);
  t2.changeReferenceFrame(trans);
  Twist t3(world, target, KDL::Twist(KDL::Vector(9, -5, 4), KDL::Vector(0, -2, 1)));

  Twist t4(world, target, transform*twist);

  EXPECT_EQ(t2, t3);
  EXPECT_NE(t, t2);
  EXPECT_EQ(t3, t4);
}
