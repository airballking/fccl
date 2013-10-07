#include <gtest/gtest.h>

#include <fccl/semantics/InteractionMatrixSemantics.h>
#include <fccl/semantics/TwistSemantics.h>
#include <fccl/semantics/JacobianSemantics.h>
#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/JointMappingSemantics.h>
#include <fccl/semantics/TransformSemantics.h>

using namespace fccl::semantics;

class InteractionMatrixSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "reference";
      target = "target";

      twist.reference().setName(reference);
      twist.target().setName(target);

      constraints.resize(3);
      constraints(0).setName("height");
      constraints(1).setName("left of");
      constraints(2).setName("behind of");

      joints.resize(4);
      joints(0).setName("joint0");
      joints(1).setName("joint0");
      joints(2).setName("joint0");
      joints(3).setName("joint0");

      jacobian.twist() = twist;
      jacobian.joints() = joints;

      A.row_joints() = constraints;
      A.column_joints() = joints;

      transform.reference().setName(world);
      transform.target().setName(reference);
    }

    virtual void TearDown()
    {
    }

    std::string world, reference, target;
    TwistSemantics twist;
    JntArraySemantics constraints;
    JntArraySemantics joints;
    JacobianSemantics jacobian;
    JointMappingSemantics A;
    TransformSemantics transform;
};

TEST_F(InteractionMatrixSemanticsTest, Basics)
{ 
  InteractionMatrixSemantics H;
  H.joints() = constraints;
  H.twist() = twist;
  
  InteractionMatrixSemantics H2(H);
  
  InteractionMatrixSemantics H3;
  H3.joints().resize(constraints.size());
  H3.joints() = constraints;
  H3.twist() = twist;

  EXPECT_TRUE(H.equals(H2));
  EXPECT_TRUE(H.equals(H3));

  InteractionMatrixSemantics H4;
  H4.joints() = constraints;

  InteractionMatrixSemantics H5;
  H5.twist() = twist; 

  EXPECT_FALSE(H.equals(H4));
  EXPECT_FALSE(H.equals(H5));

  InteractionMatrixSemantics H6;
  EXPECT_EQ(H6.joints().size(), 0);
  H6.resize(2);
  EXPECT_EQ(H6.joints().size(), 2);
  EXPECT_EQ(H6.size(), 2);
}

TEST_F(InteractionMatrixSemanticsTest, JacobianMultiplication)
{ 
  InteractionMatrixSemantics H;
  H.joints() = constraints;
  H.twist() = twist;

  ASSERT_TRUE(areMultipliable(H, jacobian));
  JointMappingSemantics A2;
  multiply(H, jacobian, A2);
  
  EXPECT_TRUE(A.equals(A2));
}

TEST_F(InteractionMatrixSemanticsTest, ChangeReferenceFrame)
{ 
  InteractionMatrixSemantics H;
  H.joints() = constraints;
  H.twist() = twist;

  ASSERT_TRUE(H.changeReferencePossible(transform));
  
  H.changeReferenceFrame(transform);

  EXPECT_STREQ(H.twist().reference().getName().c_str(), world.c_str());
  EXPECT_STREQ(H.twist().target().getName().c_str(), target.c_str());
  EXPECT_TRUE(H.joints().equals(constraints));
}
