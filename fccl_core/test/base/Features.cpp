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
      world = "world"; 

      // kdl data
      pos = KDL::Vector(0, 0, 0);
      dir = KDL::Vector(0, 0, 0);
      transform_data = KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0, 0, 1));
      // semantic kdl
      transform.numerics() = transform_data;
      transform.semantics().reference().setName(world);
      transform.semantics().target().setName(parent);

      // semantics for our feature
      feature_semantics.name().setName(name);
      feature_semantics.reference().setName(parent);
      feature_semantics.type() = fccl::semantics::POINT_FEATURE;
    }

    virtual void TearDown()
    {
    }

    std::string name, parent, world;
    KDL::Vector pos, dir;
    KDL::Frame transform_data;
    Transform transform;
    fccl::semantics::FeatureSemantics feature_semantics;
};

TEST_F(FeaturesTest, Basics)
{
  Feature f;
  f.semantics() = feature_semantics;
  f.position() = pos; 
  f.orientation() = dir;

  Feature f2;
  f2 = f;

  Feature f3(f);

  Feature f4 = f;

  Feature f5;
  f5.semantics() = f.semantics();
  f5.position() = f.position();
  f5.orientation() = f.orientation();

  Feature f6;
  f6.semantics().reference() = f.semantics().reference();
  f6.semantics().name() = f.semantics().name();
  f6.semantics().type() = f.semantics().type();
  f6.position() = f.position();
  f6.orientation() = f.orientation();

  EXPECT_TRUE(f.equals(f2));
  EXPECT_TRUE(f.equals(f3));
  EXPECT_TRUE(f.equals(f4));
  EXPECT_TRUE(f.equals(f5));
  EXPECT_TRUE(f.equals(f6));

  EXPECT_STREQ(f.semantics().reference().getName().c_str(), parent.c_str());
  EXPECT_STREQ(f.semantics().name().getName().c_str(), name.c_str());
  EXPECT_EQ(f.semantics().type(), fccl::semantics::POINT_FEATURE);
  EXPECT_TRUE(KDL::Equal(f.position(), pos));
  EXPECT_TRUE(KDL::Equal(f.orientation(), dir));

  EXPECT_STREQ(f2.semantics().reference().getName().c_str(), parent.c_str());
  EXPECT_STREQ(f2.semantics().name().getName().c_str(), name.c_str());
  EXPECT_EQ(f2.semantics().type(), fccl::semantics::POINT_FEATURE);
  EXPECT_TRUE(KDL::Equal(f2.position(), pos));
  EXPECT_TRUE(KDL::Equal(f2.orientation(), dir));

  Feature f7(f);
  f7.position().x(f7.position().x() + 1);
  EXPECT_FALSE(f.equals(f7));

  Feature f8(f);
  f8.orientation().x(f8.orientation().x() +1);
  EXPECT_FALSE(f.equals(f8));

  Feature f9(f);
  f9.semantics().name().setName("gibberish");
  EXPECT_FALSE(f.equals(f9));
}

TEST_F(FeaturesTest, ValidityCheck)
{
  Feature f;
  f.semantics() = feature_semantics;
  f.position() = pos; 
  f.orientation() = dir;

  EXPECT_TRUE(f.isOrientationValid());
  EXPECT_TRUE(f.isValid());

  f.semantics().type() = fccl::semantics::UNKNOWN_FEATURE;
  EXPECT_TRUE(f.isOrientationValid());
  EXPECT_FALSE(f.isValid());

  f.semantics().type() = fccl::semantics::FEATURE_COUNT;
  EXPECT_TRUE(f.isOrientationValid());
  EXPECT_FALSE(f.isValid());

  f.semantics().type() = fccl::semantics::LINE_FEATURE;
  EXPECT_FALSE(f.isOrientationValid());
  EXPECT_FALSE(f.isValid());

  f.semantics().type() = fccl::semantics::LINE_FEATURE;
  EXPECT_FALSE(f.isOrientationValid());
  EXPECT_FALSE(f.isValid());

  f.orientation() = KDL::Vector(1, 0, 0);
  EXPECT_TRUE(f.isOrientationValid());
  EXPECT_TRUE(f.isValid());

  f.semantics().type() = fccl::semantics::PLANE_FEATURE;
  EXPECT_TRUE(f.isOrientationValid());
  EXPECT_TRUE(f.isValid());
}

TEST_F(FeaturesTest, ChangeReferenceFrame)
{
  Feature f;
  f.semantics() = feature_semantics;
  f.position() = pos; 
  f.orientation() = dir;

  Feature f2(f);
  ASSERT_TRUE(f2.changeReferencePossible(transform));
  f2.changeReferenceFrame(transform);
  EXPECT_STREQ(f2.semantics().reference().getName().c_str(), world.c_str());
  EXPECT_STREQ(f2.semantics().name().getName().c_str(), name.c_str());
  EXPECT_EQ(f2.semantics().type(), fccl::semantics::POINT_FEATURE);
  EXPECT_TRUE(KDL::Equal(f2.position(), transform.numerics() * pos));
  EXPECT_TRUE(KDL::Equal(f2.orientation(), transform.numerics() * dir));
  EXPECT_TRUE(KDL::Equal(f2.position(), KDL::Vector(0, 0, 1)));  
  EXPECT_TRUE(KDL::Equal(f2.orientation(), KDL::Vector(0, 0, 1)));

  EXPECT_FALSE(f.equals(f2));
  Feature f3;
  f3.semantics().reference() = transform.semantics().reference();
  f3.semantics().name() = f.semantics().name();
  f3.semantics().type() = f.semantics().type();
  f3.position() =  transform.numerics() * pos;
  f3.orientation() = transform.numerics() * dir;
  EXPECT_TRUE(f2.equals(f3));
}
