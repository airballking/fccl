#include <gtest/gtest.h>

#include <fccl/base/Features.h>
#include <fccl/kdl/Transform.h>
#include <kdl/frames.hpp>

using namespace fccl::kdl;
using namespace fccl::base;

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
      pos = KDL::Vector(0, 0, 0);
      dir = KDL::Vector(0, 0, 1);
      transform_data = KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0, 0, 1));
      // semantic kdl ;)
      transform = Transform(world, parent, transform_data);
    }

    virtual void TearDown()
    {
    }

    std::string name, parent, child, world;
    KDL::Vector pos, dir;
    KDL::Frame transform_data;
    Transform transform;
};

TEST_F(FeaturesTest, Basics)
{
  Feature f;
  f.setTargetName(name);
  f.setReferenceName(parent);
  f.setPosition(pos); 
  f.setOrientation(dir);
  f.setType(LINE_FEATURE);

  Feature f2(parent, name, pos, dir, LINE_FEATURE);

  Feature f3(f);

  Feature f4(f.getReferenceID(), f.getTargetID(), f.getPosition(), f.getOrientation(), f.getType());

  Feature f5;
  f5.setTargetID(f.getTargetID());
  f5.setReferenceID(f.getReferenceID());
  f5.setPosition(f.getPosition());
  f5.setOrientation(f.getOrientation());
  f5.setType(f.getType());

  Feature f6 = f;

  EXPECT_EQ(f, f2);
  EXPECT_EQ(f, f3);
  EXPECT_EQ(f, f4);
  EXPECT_EQ(f, f5);
  EXPECT_EQ(f, f6);

  Feature f7(f);
  f7.changeReference(transform);
  
  EXPECT_NE(f, f7);
  EXPECT_EQ(f7, Feature(world, name, transform.getTransform() * pos, transform.getTransform() * dir, LINE_FEATURE));
}

TEST_F(FeaturesTest, Equalities)
{
  Feature f(parent, name, pos, dir, POINT_FEATURE);
  Feature f2(f);
  f2.setType(PLANE_FEATURE);
  Feature f3(f);
  f3.setType(LINE_FEATURE);

  EXPECT_NE(f, f2);
  EXPECT_NE(f, f3);
  EXPECT_NE(f2, f3);
}
