#include <gtest/gtest.h>

#include <fccl/semantics/TransformSemantics.h>

using namespace fccl::semantics;

class TransformSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      parent_frame = "parent";
      child_frame = "child";
      world_frame = "world";
    }

    virtual void TearDown()
    {

    }

    std::string world_frame, parent_frame, child_frame;
};

TEST_F(TransformSemanticsTest, Basics)
{
  TransformSemantics ts;

  ts.reference().setName(parent_frame);
  ts.target().setName(child_frame);

  TransformSemantics ts2(ts);

  TransformSemantics ts3;
  ts3 = ts;  

  TransformSemantics ts4;
  ts4.target() = ts.target();
  ts4.reference() = ts.reference();

  EXPECT_TRUE(ts.equals(ts2));
  EXPECT_TRUE(ts.equals(ts3));
  EXPECT_TRUE(ts.equals(ts4));
  
}

TEST_F(TransformSemanticsTest, Inversion)
{
  TransformSemantics ts;

  ts.reference().setName(parent_frame);
  ts.target().setName(child_frame);

  TransformSemantics ts2(ts);
  ts2.invert();

  TransformSemantics ts3(ts2);
  ts3.invert();

  TransformSemantics ts4;
  ts4.reference().setName(child_frame);
  ts4.target().setName(parent_frame);

  EXPECT_FALSE(ts.equals(ts2));
  EXPECT_TRUE(ts.equals(ts3));
  EXPECT_TRUE(ts2.equals(ts4));

  TransformSemantics ts5 = ts.inverse();
  EXPECT_TRUE(ts2.equals(ts5));
} 

TEST_F(TransformSemanticsTest, Multiplication)
{
  TransformSemantics ts;

  ts.reference().setName(parent_frame);
  ts.target().setName(child_frame);

  TransformSemantics ts2;
  ts2.reference().setName(world_frame);
  ts2.target().setName(parent_frame);

  ASSERT_TRUE(areMultipliable(ts2, ts));
  ASSERT_FALSE(areMultipliable(ts, ts2));
  TransformSemantics ts3 = multiply(ts2, ts);
  
  TransformSemantics ts4;
  ts4.reference().setName(world_frame);
  ts4.target().setName(child_frame);

  EXPECT_TRUE(ts3.equals(ts4));
}
