#include <gtest/gtest.h>

#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>

using namespace fccl::kdl;

class InteractionMatrixTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "parent";
      target = "child";
      transform_data = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
      twist_data = KDL::Twist(KDL::Vector(3, 4, 5), KDL::Vector(0,1,2));
      transform = Transform(world, reference, transform_data);
      twist = Twist(reference, target, twist_data);

    }

    virtual void TearDown()
    {

    }

    std::string reference, target, world;
    KDL::Frame transform_data;
    KDL::Twist twist_data;
    Transform transform;
    Twist twist;
};

TEST_F(InteractionMatrixTest, Basics)
{
  InteractionMatrix m;
  m.setReferenceName(reference);
  m.resize(1);
  m.setRow(0, twist);
  InteractionMatrix m2(m);

  std::vector<std::string> targets;
  targets.push_back(target);
  InteractionMatrix m3(reference, targets, m.getData());

  InteractionMatrix m4(m.getReferenceID(), m.getTargetIDs(), m.getData());

  InteractionMatrix m5(reference, m.rows());
  m5.setTargetIDs(m.getTargetIDs());
  m5.setData(m.getData());

  InteractionMatrix m6(m.getReferenceID(), m.rows());
  m6.setTargetIDs(m.getTargetIDs());
  m6.setData(m.getData());

  EXPECT_EQ(m, m2);
  EXPECT_EQ(m, m3);
  EXPECT_EQ(m, m4);
  EXPECT_EQ(m, m5);
  EXPECT_EQ(m, m6);
  EXPECT_EQ(m.getRow(0), twist);
}

TEST_F(InteractionMatrixTest, ChangeReferenceFrame)
{
  InteractionMatrix m(reference, 1);
  m.setRow(0, twist);

  // compare with KDL's method for change of reference frames of twists
  twist.changeReferenceFrame(transform);
  InteractionMatrix m2(world, 1);
  m2.setRow(0, twist);

  // our own method to change reference frames
  InteractionMatrix m3 = m;
  m3.changeReferenceFrame(transform);

  EXPECT_EQ(m2, m3);
  EXPECT_NE(m, m2);
}
