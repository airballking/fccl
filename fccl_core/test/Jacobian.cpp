#include <gtest/gtest.h>

#include <fccl/kdl/Jacobian.h>

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

      twist = Twist(SemanticObject1x1(reference_frame, joint_name), twist_data);
      transform = Transform(SemanticObject1x1(world_frame, reference_frame), transform_data);
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
};

TEST_F(JacobianTest, Basics)
{
  Jacobian j;
  j.resize(1);
  j.setSemantics(SemanticObject1xN(reference_frame, joint_names));
  j.setData(jacobian_data);

  Jacobian j2(j);

  Jacobian j3(j.getSemantics(), j.getData());

  Jacobian j4;
  j4.resize(1);
  j4.setSemantics(j.getSemantics());
  j4.setData(j.getData());

  Jacobian j5(1);
  j5.setTargetNames(j.getTargetNames());
  j5.setReferenceName(j.getReferenceName());
  j5.setData(j.getData());

  Jacobian j6;
  j6.init(j.getSemantics());
  j6.setData(j.getData());

  EXPECT_EQ(j, j2);
  EXPECT_EQ(j, j3);
  EXPECT_EQ(j, j4);
  EXPECT_EQ(j, j5);
  EXPECT_EQ(j, j6);

  EXPECT_TRUE(j.columnIndexValid(0));
  EXPECT_TRUE(j.rowIndexValid(5));
  EXPECT_FALSE(j.columnIndexValid(1));
  EXPECT_FALSE(j.rowIndexValid(6));

  EXPECT_TRUE(j.isValid());

  EXPECT_EQ(j.rows(), 6);
  EXPECT_EQ(j.columns(), 1);

  EXPECT_EQ(twist, j.getColumn(0));

  EXPECT_DOUBLE_EQ(j(0,0), 3);
  EXPECT_DOUBLE_EQ(j(1,0), 4);
  EXPECT_DOUBLE_EQ(j(2,0), 5);
  EXPECT_DOUBLE_EQ(j(3,0), 0);
  EXPECT_DOUBLE_EQ(j(4,0), 1);
  EXPECT_DOUBLE_EQ(j(5,0), 2);

  Jacobian j7(2);
  Jacobian j8(1);
  Jacobian j9(1);
  j.setReferenceName(reference_frame);
  j.setData(jacobian_data);
  Jacobian j10(1);
  j.setTargetNames(joint_names);
  j.setData(jacobian_data);
  Jacobian j11(1);
  j.setReferenceName(reference_frame);
  j.setTargetNames(joint_names);

  EXPECT_NE(j, j7);
  EXPECT_NE(j, j8);
  EXPECT_NE(j, j9);
  EXPECT_NE(j, j10);
  EXPECT_NE(j, j11);
}

TEST_F(JacobianTest, ChangeReferenceFrame)
{
  Jacobian j(SemanticObject1xN(reference_frame, joint_names), jacobian_data);

  j.changeReferenceFrame(transform);
  twist.changeReferenceFrame(transform);

  EXPECT_EQ(j.getColumn(0), twist);

  EXPECT_STREQ(j.getReferenceName().c_str(), world_frame.c_str());
  EXPECT_STREQ(j.getTargetName(0).c_str(), joint_name.c_str());
  EXPECT_DOUBLE_EQ(j(0,0), 9);
  EXPECT_DOUBLE_EQ(j(1,0), -5);
  EXPECT_DOUBLE_EQ(j(2,0), 4);
  EXPECT_DOUBLE_EQ(j(3,0), 0);
  EXPECT_DOUBLE_EQ(j(4,0), -2);
  EXPECT_DOUBLE_EQ(j(5,0), 1);

}
