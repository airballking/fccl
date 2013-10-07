#include <gtest/gtest.h>

#include <fccl/semantics/JacobianSemantics.h>
#include <vector>

using namespace fccl::semantics;

class JacobianSemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");
      joint_names.push_back("joint3");

      parent = "parent";
      child = "child";
      world = "world";

      joints.resize(joint_names.size());
      for(std::size_t i=0; i<joints.size(); i++)
        joints(i).setName(joint_names[i]);

      twist.reference().setName(parent);
      twist.target().setName(child);

      transform.reference().setName(world);
      transform.target().setName(parent); 
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    std::string parent, child, world;

    JntArraySemantics joints;
    TwistSemantics twist;
    TransformSemantics transform;
};

TEST_F(JacobianSemanticsTest, Basics)
{ 
  JacobianSemantics jac;
  jac.twist() = twist;
  jac.joints() = joints;

  JacobianSemantics jac2(jac);

  JacobianSemantics jac3;
  jac3 = jac;

  EXPECT_TRUE(jac.equals(jac2)); 
  EXPECT_TRUE(jac.equals(jac3)); 
}

TEST_F(JacobianSemanticsTest, JntArrayMultiplication)
{ 
  JacobianSemantics jac;
  jac.twist() = twist;
  jac.joints() = joints;

  TwistSemantics result = multiply(jac, joints);
  EXPECT_TRUE(result.equals(twist));

  EXPECT_STREQ(result.reference().getName().c_str(), parent.c_str());
  EXPECT_STREQ(result.target().getName().c_str(), child.c_str());

  joints(1).setName("blub");
  EXPECT_FALSE(areMultipliable(jac, joints));

  joints(1).setName("joint1");
  jac.twist().reference().setName("huhu");
  EXPECT_TRUE(areMultipliable(jac, joints));

  jac.twist().target().setName("haha");
  EXPECT_TRUE(areMultipliable(jac, joints));
}

TEST_F(JacobianSemanticsTest, ChangeReference)
{ 
  JacobianSemantics jac;
  jac.twist() = twist;
  jac.joints() = joints;

  ASSERT_TRUE(jac.changeReferencePossible(transform));
  ASSERT_FALSE(jac.changeReferencePossible(transform.inverse()));

  jac.changeReferenceFrame(transform);

  EXPECT_STREQ(jac.twist().reference().getName().c_str(), world.c_str());
  EXPECT_STREQ(jac.twist().target().getName().c_str(), child.c_str());
}
