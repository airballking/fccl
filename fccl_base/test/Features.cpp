#include <gtest/gtest.h>

#include <fccl_base/Features.h>
#include <fccl_kdl/Vector.h>
#include <fccl_kdl/Transform.h>
#include <kdl/frames.hpp>

using namespace fccl;

class FeaturesTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      // names/identifers of stuff
      name = "corner";
      parent = "parent";
      child = "child";
      world = "world"; 

      // kdl data
      pos_data = KDL::Vector(0, 0, 0);
      dir_data = KDL::Vector(0, 0, 1);
      transform_data = KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0, 0, 1));
      // semantic kdl ;)
      pos = Vector(parent, name, pos_data);
      dir = Vector(parent, name, dir_data);
      transform = Transform(world, parent, transform_data);
    }

    virtual void TearDown()
    {
    }

    std::string name, parent, child, world;
    KDL::Vector pos_data, dir_data;
    Vector pos, dir;
    KDL::Frame transform_data;
    Transform transform;
};

TEST_F(FeaturesTest, Point)
{
  Point p;
  p.setName(name);
  p.setPosition(pos); 

  Point p2(name, pos);

  Point p3(p);

  Point p4(p.getID(), p.getPosition());

  Point p5;
  p5.setID(p.getID());
  p5.setPosition(p.getPosition());

  Point p6 = p;

  EXPECT_EQ(p, p2);
  EXPECT_EQ(p, p3);
  EXPECT_EQ(p, p4);
  EXPECT_EQ(p, p5);
  EXPECT_EQ(p, p6);

  Point p7(p);
  p7.changeReference(transform);
  
  EXPECT_NE(p, p7);
  EXPECT_EQ(p7, Point(name, transform * pos));
}

TEST_F(FeaturesTest, Line)
{
  Line p;
  p.setName(name);
  p.setPosition(pos); 
  p.setOrientation(dir);

  Line p2(name, pos, dir);

  Line p3(p);

  Line p4(p.getID(), p.getPosition(), p.getOrientation());

  Line p5;
  p5.setID(p.getID());
  p5.setPosition(p.getPosition());
  p5.setOrientation(p.getOrientation());

  Line p6 = p;

  EXPECT_EQ(p, p2);
  EXPECT_EQ(p, p3);
  EXPECT_EQ(p, p4);
  EXPECT_EQ(p, p5);
  EXPECT_EQ(p, p6);

  Line p7(p);
  p7.changeReference(transform);
  
  EXPECT_NE(p, p7);
  EXPECT_EQ(p7, Line(name, transform * pos, transform * dir));
}

TEST_F(FeaturesTest, Plane)
{
  Plane p;
  p.setName(name);
  p.setPosition(pos); 
  p.setOrientation(dir);

  Plane p2(name, pos, dir);

  Plane p3(p);

  Plane p4(p.getID(), p.getPosition(), p.getOrientation());

  Plane p5;
  p5.setID(p.getID());
  p5.setPosition(p.getPosition());
  p5.setOrientation(p.getOrientation());

  Plane p6 = p;

  EXPECT_EQ(p, p2);
  EXPECT_EQ(p, p3);
  EXPECT_EQ(p, p4);
  EXPECT_EQ(p, p5);
  EXPECT_EQ(p, p6);

  Plane p7(p);
  p7.changeReference(transform);
  
  EXPECT_NE(p, p7);
  EXPECT_EQ(p7, Plane(name, transform * pos, transform * dir));
}
