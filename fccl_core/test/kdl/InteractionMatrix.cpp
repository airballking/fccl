#include <gtest/gtest.h>

#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>

using namespace fccl::kdl;

class InteractionMatrixTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      world = "world";
      reference = "parent";
      target = "child";
      joint = "joint";

      joint_names.push_back(joint);

      transform_data = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.0, 3.0));
      twist_data = KDL::Twist(KDL::Vector(3, 4, 5), KDL::Vector(0,1,2));

      transform.semantics().reference().setName(world);
      transform.semantics().target().setName(reference);
      transform.numerics() = transform_data;

      twist.semantics().reference().setName(reference);
      twist.semantics().target().setName(target);
      twist.numerics() = twist_data;

      semantics.twist() = twist.semantics();
      semantics.joints().resize(1);
      semantics.joints()(0).setName(joint);
      for(std::size_t i=0; i<6; i++)
        data(0,i) = twist_data(i);
    }

    virtual void TearDown()
    {

    }

    std::string reference, target, world, joint;
    std::vector<std::string> joint_names;
    KDL::Frame transform_data;
    KDL::Twist twist_data;
    Transform transform;
    Twist twist;
    fccl::semantics::InteractionMatrixSemantics semantics;
    Eigen::Matrix< double, 1, 6 > data;
};

TEST_F(InteractionMatrixTest, Basics)
{
  InteractionMatrix m;
  ASSERT_EQ(m.semantics().joints().size(), 0);
  m.resize(1);
  ASSERT_EQ(m.semantics().joints().size(), 1);
  m.semantics() = semantics;
  m.numerics() = data;

  InteractionMatrix m2(m);

  InteractionMatrix m3;
  m3 = m;

  EXPECT_TRUE(m.equals(m2));
  EXPECT_TRUE(m.equals(m3));
 
  EXPECT_TRUE(m.numerics().isApprox(data));
  EXPECT_STREQ(m.semantics().twist().reference().getName().c_str(),
      reference.c_str());
  EXPECT_STREQ(m.semantics().twist().target().getName().c_str(),
      target.c_str());
  ASSERT_EQ(m.semantics().joints().size(), 1);
  EXPECT_STREQ(m.semantics().joints()(0).getName().c_str(), joint.c_str());
}

TEST_F(InteractionMatrixTest, ChangeReferenceFrame)
{
  InteractionMatrix m;
  m.semantics() = semantics;
  m.numerics() = data;

  // compare with KDL's method for change of reference frames of twists
  twist.changeReferenceFrame(transform);
  InteractionMatrix m2;
  m2.resize(1);
  m2.semantics().twist() = twist.semantics();
  m2.semantics().joints()(0).setName(joint);
  ASSERT_EQ(m2.size(), 1);
  for(std::size_t i=0; i<6; i++)
    m2.numerics()(0,i) = twist.numerics()(i);
  
  // our own method to change reference frames
  InteractionMatrix m3(m);
  m3.changeReferenceFrame(transform);

  EXPECT_TRUE(m2.equals(m3));
  EXPECT_FALSE(m.equals(m2));
}

TEST_F(InteractionMatrixTest, Init)
{
  InteractionMatrix m;
  m.init(joint_names, reference, target);

  ASSERT_TRUE(m.isValid());
  ASSERT_EQ(m.size(), joint_names.size());
  ASSERT_EQ(m.numerics().rows(), joint_names.size());
  ASSERT_EQ(m.numerics().cols(), 6);
  ASSERT_EQ(joint_names.size(), 1);

  EXPECT_STREQ(m.semantics().twist().reference().getName().c_str(),
      reference.c_str());
  EXPECT_STREQ(m.semantics().twist().target().getName().c_str(),
      target.c_str());
  for(std::size_t i=0; i<m.size(); i++)
    EXPECT_STREQ(m.semantics().joints()(i).getName().c_str(), 
        joint_names[i].c_str());
}
