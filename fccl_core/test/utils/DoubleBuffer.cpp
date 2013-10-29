#include <gtest/gtest.h>

#include <fccl/utils/DoubleBuffer.h>
#include <fccl/kdl/Transform.h>

using namespace fccl::utils;
using namespace fccl::kdl;

class DoubleBufferTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      view = "view";
      object = "object";
      tool = "tool";

      tf1.semantics().reference().setName(world);
      tf1.semantics().target().setName(view);
      tf1.numerics() = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(1,0,0));

      tf2.semantics().reference().setName(world);
      tf2.semantics().target().setName(object);
      tf2.numerics() = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0,1,0));

      tf3.semantics().reference().setName(world);
      tf3.semantics().target().setName(tool);
      tf3.numerics() = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0,0,1));
    }

    virtual void TearDown()
    {

    }

    std::string view, world, object, tool;
    Transform tf1, tf2, tf3;
};

TEST_F(DoubleBufferTest, Basics)
{
  TransformDoubleBuffer double_buffer;
  ASSERT_EQ(double_buffer.inBuffer().size(), 0);
  ASSERT_EQ(double_buffer.outBuffer().size(), 0);

  double_buffer.inBuffer().setTransform(tf1);
  double_buffer.inBuffer().setTransform(tf2);
  EXPECT_EQ(double_buffer.inBuffer().size(), 2);
  EXPECT_EQ(double_buffer.outBuffer().size(), 0);
  ASSERT_TRUE(double_buffer.inBuffer().hasTransform(tf1.semantics()));
  ASSERT_TRUE(double_buffer.inBuffer().hasTransform(tf2.semantics()));
  EXPECT_FALSE(double_buffer.inBuffer().hasTransform(tf3.semantics()));

  EXPECT_TRUE(double_buffer.inBuffer().getTransform(tf1.semantics()).equals(tf1));
  EXPECT_TRUE(double_buffer.inBuffer().getTransform(tf2.semantics()).equals(tf2));

  double_buffer.outBuffer().setTransform(tf3);
  EXPECT_EQ(double_buffer.inBuffer().size(), 2);
  EXPECT_EQ(double_buffer.outBuffer().size(), 1);
  ASSERT_TRUE(double_buffer.inBuffer().hasTransform(tf1.semantics()));
  ASSERT_TRUE(double_buffer.inBuffer().hasTransform(tf2.semantics()));
  EXPECT_FALSE(double_buffer.inBuffer().hasTransform(tf3.semantics()));
  ASSERT_TRUE(double_buffer.outBuffer().hasTransform(tf3.semantics()));
  EXPECT_FALSE(double_buffer.outBuffer().hasTransform(tf1.semantics()));
  EXPECT_FALSE(double_buffer.outBuffer().hasTransform(tf2.semantics()));

  EXPECT_TRUE(double_buffer.inBuffer().getTransform(tf1.semantics()).equals(tf1));
  EXPECT_TRUE(double_buffer.inBuffer().getTransform(tf2.semantics()).equals(tf2));
  EXPECT_TRUE(double_buffer.outBuffer().getTransform(tf3.semantics()).equals(tf3));

  double_buffer.swap();
  EXPECT_EQ(double_buffer.outBuffer().size(), 2);
  EXPECT_EQ(double_buffer.inBuffer().size(), 1);
  ASSERT_TRUE(double_buffer.outBuffer().hasTransform(tf1.semantics()));
  ASSERT_TRUE(double_buffer.outBuffer().hasTransform(tf2.semantics()));
  EXPECT_FALSE(double_buffer.outBuffer().hasTransform(tf3.semantics()));
  ASSERT_TRUE(double_buffer.inBuffer().hasTransform(tf3.semantics()));
  EXPECT_FALSE(double_buffer.inBuffer().hasTransform(tf1.semantics()));
  EXPECT_FALSE(double_buffer.inBuffer().hasTransform(tf2.semantics()));

  EXPECT_TRUE(double_buffer.outBuffer().getTransform(tf1.semantics()).equals(tf1));
  EXPECT_TRUE(double_buffer.outBuffer().getTransform(tf2.semantics()).equals(tf2));
  EXPECT_TRUE(double_buffer.inBuffer().getTransform(tf3.semantics()).equals(tf3));
}
