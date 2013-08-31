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

  // basic equality
  EXPECT_NE(point, point2);
  point.setPosition(pos);
  EXPECT_EQ(point.getPosition(), point2.getPosition());
  EXPECT_STREQ(point.getName().c_str(), point2.getName().c_str());
  EXPECT_EQ(point, point2);
  point.setName("corner2");
  EXPECT_NE(point, point2);
}

TEST_F(FeaturesTest, Plane)
{
  //basic constructor, getter, and setter tests
  Plane plane(name, pos, dir);
  Plane plane2;
  plane2.setName(name);
  plane2.setPosition(pos);
  plane2.setNormal(dir);
  EXPECT_EQ(plane.getPosition(), plane2.getPosition());
  EXPECT_EQ(plane.getNormal(), plane2.getNormal());
  EXPECT_EQ(plane.getOrientation(), plane2.getNormal());
  EXPECT_STREQ(plane.getName().c_str(), plane2.getName().c_str());
  EXPECT_EQ(plane, plane2);

  // changing reference frame
  plane.changeReferenceFrame(transform);
  EXPECT_NE(plane.getPosition(), plane2.getPosition());
  EXPECT_NE(plane.getNormal(), plane2.getNormal());
  EXPECT_EQ(plane.getPosition(), Vector(KDL::Vector(0, 0, -1), "child"));
  EXPECT_EQ(plane.getNormal(), Vector(KDL::Vector(0,0,0), "child"));

  // basic equality
  EXPECT_NE(plane, plane2);
  plane.setPosition(pos);
  plane.setOrientation(dir);
  EXPECT_EQ(plane, plane2);

  // some casting trick ;)
  Feature* base_pointer2 = dynamic_cast<Feature*>(&plane2);
  EXPECT_EQ(plane, *base_pointer2);
}

TEST_F(FeaturesTest, Line)
{
  //basic constructor, getter, and setter tests
  Line line(name, pos, dir);
  Line line2;
  line2.setName(name);
  line2.setPosition(pos);
  line2.setDirection(dir);
  EXPECT_EQ(line.getPosition(), line2.getPosition());
  EXPECT_EQ(line.getDirection(), line2.getDirection());
  EXPECT_EQ(line.getOrientation(), line2.getDirection());
  EXPECT_STREQ(line.getName().c_str(), line2.getName().c_str());
  EXPECT_EQ(line, line2);

  // changing reference frame
  line.changeReferenceFrame(transform);
  EXPECT_NE(line.getPosition(), line2.getPosition());
  EXPECT_NE(line.getDirection(), line2.getDirection());
  EXPECT_EQ(line.getPosition(), Vector(KDL::Vector(0, 0, -1), "child"));
  EXPECT_EQ(line.getDirection(), Vector(KDL::Vector(0,0,0), "child"));

  // basic equality
  EXPECT_NE(line, line2);
  line.setPosition(pos);
  line.setOrientation(dir);
  EXPECT_EQ(line, line2);
}

TEST_F(FeaturesTest, MultipleEquality)
{
  Plane plane;
  Line line;
  Point point;

  EXPECT_NE(plane, line);
  EXPECT_NE(plane, point);
  EXPECT_NE(line, point);
}
