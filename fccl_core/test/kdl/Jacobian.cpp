#include <gtest/gtest.h>

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>

using namespace fccl::kdl;

class JacobianTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world_frame = "world";
      reference_frame = "reference";
      joint_name = "joint";
      joint_names.push_back(joint_name);

      twist_data = KDL::Twist(KDL::Vector(3,4,5), KDL::Vector(0,1,2));
      jacobian_data = KDL::Jacobian(1);
      jacobian_data.setColumn(0, twist_data);
      transform_data = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));

      twist.semantics().reference().setName(reference_frame);
      twist.semantics().target().setName(joint_name);
      twist.numerics() = twist_data;

      transform.semantics().reference().setName(world_frame);
      transform.semantics().target().setName(reference_frame);
      transform.numerics() = transform_data;

      jacobian_semantics.joints().resize(joint_names.size());
      for(unsigned int i=0; i<joint_names.size(); i++)
        jacobian_semantics.joints()(i).setName(joint_names[i]);
      jacobian_semantics.twist() = twist.semantics();
    }

    virtual void TearDown()
    {

    }

    std::string world_frame, reference_frame, joint_name;
    std::vector<std::string> joint_names;
    KDL::Twist twist_data;
    KDL::Jacobian jacobian_data;
    KDL::Frame transform_data;
    Twist twist;
    Transform transform;
    fccl::semantics::JacobianSemantics jacobian_semantics;    
};

TEST_F(JacobianTest, Basics)
{
  Jacobian j;
  j.resize(1);
  ASSERT_EQ(j.semantics().joints().size(), j.numerics().columns());
  ASSERT_EQ(j.numerics().columns(), 1);
  j.semantics() = jacobian_semantics;
  j.numerics() = jacobian_data;

  Jacobian j2(j);

  Jacobian j3;
  j3.semantics() = j.semantics();
  j3.numerics() = j.numerics();

  Jacobian j4;
  j4.resize(1);
  j4.semantics() = j.semantics();
  j4.numerics() = j.numerics();

  EXPECT_TRUE(j.equals(j2));
  EXPECT_TRUE(j.equals(j3));
  EXPECT_TRUE(j.equals(j4));

  EXPECT_TRUE(j.isValid());
  EXPECT_TRUE(j2.isValid());
  EXPECT_TRUE(j3.isValid());
  EXPECT_TRUE(j4.isValid());

  EXPECT_DOUBLE_EQ(j.numerics()(0,0), 3);
  EXPECT_DOUBLE_EQ(j.numerics()(1,0), 4);
  EXPECT_DOUBLE_EQ(j.numerics()(2,0), 5);
  EXPECT_DOUBLE_EQ(j.numerics()(3,0), 0);
  EXPECT_DOUBLE_EQ(j.numerics()(4,0), 1);
  EXPECT_DOUBLE_EQ(j.numerics()(5,0), 2);

  EXPECT_STREQ(j.semantics().twist().reference().getName().c_str(), reference_frame.c_str());
  EXPECT_STREQ(j.semantics().twist().target().getName().c_str(), joint_name.c_str());
  for(std::size_t i=0; i<j.semantics().joints().size(); i++)
    EXPECT_STREQ(j.semantics().joints()(i).getName().c_str(), joint_names[i].c_str());
  EXPECT_EQ(j.size(), joint_names.size());
}

TEST_F(JacobianTest, ChangeReferenceFrame)
{
  Jacobian j;
  j.resize(1);
  j.semantics() = jacobian_semantics;
  j.numerics() = jacobian_data;

  j.changeReferenceFrame(transform);

  EXPECT_STREQ(j.semantics().twist().reference().getName().c_str(), world_frame.c_str());
  EXPECT_STREQ(j.semantics().twist().target().getName().c_str(), joint_name.c_str());
  EXPECT_DOUBLE_EQ(j.numerics()(0,0), 9);
  EXPECT_DOUBLE_EQ(j.numerics()(1,0), -5);
  EXPECT_DOUBLE_EQ(j.numerics()(2,0), 4);
  EXPECT_DOUBLE_EQ(j.numerics()(3,0), 0);
  EXPECT_DOUBLE_EQ(j.numerics()(4,0), -2);
  EXPECT_DOUBLE_EQ(j.numerics()(5,0), 1);
}
