#include <gtest/gtest.h>

#include <fccl/semantics/InteractionMatrixSemantics.h>
#include <fccl/semantics/TwistSemantics.h>
#include <fccl/semantics/JacobianSemantics.h>
#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/JointMappingSemantics.h>

using namespace fccl::semantics;

class InteractionMatrixSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      twist.reference().setName("world");
      twist.target().setName("tool");

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
    }

    virtual void TearDown()
    {
    }

    TwistSemantics twist;
    JntArraySemantics constraints;
    JntArraySemantics joints;
    JacobianSemantics jacobian;
    JointMappingSemantics A;
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
