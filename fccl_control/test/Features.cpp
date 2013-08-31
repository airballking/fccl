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
  point2.setName("corner");
  point2.setPosition(pos);
  EXPECT_TRUE(KDL::Equal(point.getPosition().getVector(), point2.getPosition().getVector()));
  EXPECT_STREQ(point.getPosition().getFrameName().c_str(), point2.getPosition().getFrameName().c_str());
  EXPECT_STREQ(point.getName().c_str(), point2.getName().c_str());
  EXPECT_STRNE(point.getName().c_str(), "corner2");
  EXPECT_STRNE(point.getName().c_str(), "");

  // changing reference frame
  point.changeReferenceFrame(transform);
  EXPECT_FALSE(KDL::Equal(point.getPosition().getVector(), point2.getPosition().getVector()));
  EXPECT_TRUE(KDL::Equal(point.getPosition().getVector(), KDL::Vector(0,0,-1)));
  EXPECT_STREQ(point.getName().c_str(), point2.getName().c_str());
  EXPECT_STRNE(point.getPosition().getFrameName().c_str(), point2.getPosition().getFrameName().c_str());
}
