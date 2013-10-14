#include <gtest/gtest.h>

#include <fccl/semantics/TwistSemantics.h>

using namespace fccl::semantics;

class TwistSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "parent";
      target = "child";

      transform.init(world, reference); 
    }

    virtual void TearDown()
    {

    }

    std::string reference, target, world;
    fccl::semantics::TransformSemantics transform;
};

TEST_F(TwistSemanticsTest, Basics)
{
  TwistSemantics t;
  t.reference().setName(reference);
  t.target().setName(target);

  TwistSemantics t2(t);

  TwistSemantics t3 = t;

  TwistSemantics t4;
  t4 = t;

  TwistSemantics t5;
  t5.init(reference, target);

  EXPECT_TRUE(t.equals(t2));
  EXPECT_TRUE(t.equals(t3));
  EXPECT_TRUE(t.equals(t4));
  EXPECT_TRUE(t.equals(t5));

  EXPECT_STREQ(t.reference().getName().c_str(), reference.c_str());
  EXPECT_STREQ(t.target().getName().c_str(), target.c_str());

  EXPECT_STREQ(t2.reference().getName().c_str(), reference.c_str());
  EXPECT_STREQ(t2.target().getName().c_str(), target.c_str());
}

TEST_F(TwistSemanticsTest, ChangeReferenceFrame)
{
  TwistSemantics t;
  t.init(reference, target);

  TwistSemantics t2(t);
  t2.changeReferenceFrame(transform);

  TwistSemantics t3;
  t3.init(world, target);

  EXPECT_TRUE(t2.equals(t3));
  EXPECT_FALSE(t.equals(t2));
}
