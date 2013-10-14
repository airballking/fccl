#include <gtest/gtest.h>

#include <fccl/semantics/ConstraintSemantics.h>
#include <fccl/semantics/TransformSemantics.h>

using namespace fccl::semantics;

class ConstraintSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      name = "above constraint";
      view = "kinect frame";
      tool = "tool frame";

      transform.reference().setName(view);
      transform.target().setName(tool);
    }

    virtual void TearDown()
    {

    }

    std::string name, view, tool;
    fccl::semantics::TransformSemantics transform;
};

TEST_F(ConstraintSemanticsTest, Basics)
{
  ConstraintSemantics cs;
  EXPECT_NE(cs.type(), ABOVE_CONSTRAINT);
  EXPECT_EQ(cs.reference().getID(), 0);
  EXPECT_EQ(cs.name().getID(), 0);
  cs.reference().setName(view);
  cs.name().setName(name);
  cs.type() = ABOVE_CONSTRAINT;

  ConstraintSemantics cs2(cs);

  ConstraintSemantics cs3 = cs;

  ConstraintSemantics cs4;
  cs4 = cs;

  ConstraintSemantics cs5;
  cs5.reference() = cs.reference();
  cs5.name() = cs.name();
  cs5.type() = cs.type();

  EXPECT_TRUE(cs.equals(cs2));
  EXPECT_TRUE(cs.equals(cs3));
  EXPECT_TRUE(cs.equals(cs4));
  EXPECT_TRUE(cs.equals(cs5));

  EXPECT_STREQ(cs.reference().getName().c_str(), view.c_str());
  EXPECT_STREQ(cs.name().getName().c_str(), name.c_str());
  EXPECT_EQ(cs.type(), ABOVE_CONSTRAINT);

  EXPECT_STREQ(cs2.reference().getName().c_str(), view.c_str());
  EXPECT_STREQ(cs2.name().getName().c_str(), name.c_str());
  EXPECT_EQ(cs2.type(), ABOVE_CONSTRAINT);

  ConstraintSemantics cs7;
  EXPECT_FALSE(cs.equals(cs7));
  cs7.reference().setName(view);
  EXPECT_FALSE(cs.equals(cs7));
  cs7.name().setName(name);
  EXPECT_FALSE(cs.equals(cs7));
  cs7.type() = ABOVE_CONSTRAINT;
  EXPECT_TRUE(cs.equals(cs7));

  ConstraintSemantics cs8(cs);
  cs8.reference().setName(tool);
  EXPECT_FALSE(cs.equals(cs8));

  ConstraintSemantics cs9(cs);
  cs9.name().setName("below constraint");
  EXPECT_FALSE(cs.equals(cs9));

  ConstraintSemantics cs10(cs);
  cs10.type() = UNKNOWN_CONSTRAINT;
  EXPECT_FALSE(cs.equals(cs10));
}

TEST_F(ConstraintSemanticsTest, ValidityCheck)
{
  ConstraintSemantics cs;
  cs.reference().setName(view);
  cs.name().setName(name);
  cs.type() = ABOVE_CONSTRAINT;

  EXPECT_TRUE(cs.isValid());
  cs.type() = UNKNOWN_CONSTRAINT;
  EXPECT_FALSE(cs.isValid());
  cs.type() = CONSTRAINT_COUNT;
  EXPECT_FALSE(cs.isValid());
}

TEST_F(ConstraintSemanticsTest, InteractionMatrix)
{
  ConstraintSemantics cs;
  cs.reference().setName(view);
  cs.name().setName(name);
  cs.type() = ABOVE_CONSTRAINT;

  InteractionMatrixSemantics ims = cs.calculateFirstDerivative(transform);
  ASSERT_EQ(ims.size(), 1);
  EXPECT_STREQ(ims.twist().reference().getName().c_str(), tool.c_str());
  EXPECT_STREQ(ims.twist().target().getName().c_str(), tool.c_str());
  EXPECT_STREQ(ims.joints()(0).getName().c_str(), name.c_str());
}
