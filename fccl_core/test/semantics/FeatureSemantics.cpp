#include <gtest/gtest.h>

#include <fccl/semantics/FeatureSemantics.h>
#include <fccl/semantics/TransformSemantics.h>

using namespace fccl::semantics;

class FeatureSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      name = "corner";
      parent_frame = "parent";
      world_frame = "world";

      transform.reference().setName(world_frame);
      transform.target().setName(parent_frame);
    }

    virtual void TearDown()
    {

    }

    std::string name, world_frame, parent_frame;
    fccl::semantics::TransformSemantics transform;
};

TEST_F(FeatureSemanticsTest, Basics)
{
  FeatureSemantics fs;
  fs.reference().setName(parent_frame);
  fs.name().setName(name);
  fs.type() = POINT_FEATURE; 

  FeatureSemantics fs2(fs);

  FeatureSemantics fs3 = fs;

  FeatureSemantics fs4;
  fs4 = fs;

  FeatureSemantics fs5;
  fs5.reference() = fs.reference();
  fs5.name() = fs.name();
  fs5.type() = fs.type();

  EXPECT_TRUE(fs.equals(fs2));
  EXPECT_TRUE(fs.equals(fs3));
  EXPECT_TRUE(fs.equals(fs4));
  EXPECT_TRUE(fs.equals(fs5));

  EXPECT_STREQ(fs.reference().getName().c_str(), parent_frame.c_str());
  EXPECT_STREQ(fs.name().getName().c_str(), name.c_str());
  EXPECT_EQ(fs.type(), POINT_FEATURE);

  EXPECT_STREQ(fs2.reference().getName().c_str(), parent_frame.c_str());
  EXPECT_STREQ(fs2.name().getName().c_str(), name.c_str());
  EXPECT_EQ(fs2.type(), POINT_FEATURE);

  FeatureSemantics fs7;
  EXPECT_FALSE(fs.equals(fs7));
  fs7.reference().setName(parent_frame);
  EXPECT_FALSE(fs.equals(fs7));
  fs7.name().setName(name);
  EXPECT_FALSE(fs.equals(fs7));
  fs7.type() = POINT_FEATURE;
  EXPECT_TRUE(fs.equals(fs7));

  FeatureSemantics fs8(fs);
  fs8.reference().setName(world_frame);
  EXPECT_FALSE(fs.equals(fs8));

  FeatureSemantics fs9(fs);
  fs9.name().setName("pizza");
  EXPECT_FALSE(fs.equals(fs9));

  FeatureSemantics fs10(fs);
  fs10.type() = LINE_FEATURE;
  EXPECT_FALSE(fs.equals(fs10));
}

TEST_F(FeatureSemanticsTest, ValidityChecking)
{
  FeatureSemantics fs;
  fs.reference().setName(parent_frame);
  fs.name().setName(name);
  fs.type() = POINT_FEATURE;

  EXPECT_TRUE(fs.isValid());

  fs.type() = LINE_FEATURE;
  EXPECT_TRUE(fs.isValid());

  fs.type() = PLANE_FEATURE;
  EXPECT_TRUE(fs.isValid());

  fs.type() = UNKNOWN_FEATURE;
  EXPECT_FALSE(fs.isValid());

  fs.type() = FEATURE_COUNT;
  EXPECT_FALSE(fs.isValid());
}

TEST_F(FeatureSemanticsTest, ChangeReferenceFrame)
{
  FeatureSemantics fs;
  fs.reference().setName(parent_frame);
  fs.name().setName(name);
  fs.type() = POINT_FEATURE;

  ASSERT_TRUE(fs.changeReferencePossible(transform));
  EXPECT_FALSE(fs.changeReferencePossible(transform.inverse()));

  fs.changeReferenceFrame(transform);
  EXPECT_STREQ(fs.reference().getName().c_str(), world_frame.c_str());
  EXPECT_STREQ(fs.name().getName().c_str(), name.c_str());
  EXPECT_EQ(fs.type(), POINT_FEATURE);
} 
