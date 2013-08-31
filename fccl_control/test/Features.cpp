#include <gtest/gtest.h>

#include <fccl_control/Features.h>

using namespace fccl;

class FeaturesTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      name = "corner";
      pos.setFrameName("parent");
      dir.setFrameName("parent");
      pos.setVector(KDL::Vector(0, 0, 0));
      dir.setVector(KDL::Vector(0, 0, 1));
      transform = Transform("parent", "child", KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0, 0, 1)));
     
    }

    virtual void TearDown()
    {
    }

    std::string name;
    Vector pos, dir;
    Transform transform;
};

TEST_F(FeaturesTest, Point)
{
  // basic constructor, getter, and setter tests
  Point point(name, pos);
  Point point2;
  point2.setName(name);
  point2.setPosition(pos);
  EXPECT_EQ(point.getPosition(), point2.getPosition());
  EXPECT_STREQ(point.getName().c_str(), point2.getName().c_str());
  EXPECT_STRNE(point.getName().c_str(), "corner2");
  EXPECT_STRNE(point.getName().c_str(), "");

  // changing reference frame
  point.changeReferenceFrame(transform);
  EXPECT_NE(point.getPosition(), point2.getPosition());
  EXPECT_EQ(point.getPosition(), Vector(KDL::Vector(0,0,-1), "child"));
}
