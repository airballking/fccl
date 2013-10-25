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

      joint_names2.push_back("joint0");
      joint_names2.push_back("joint1");
      joint_names2.push_back("joint2");
      joint_names2.push_back("joint3");

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
    std::vector<std::string> joint_names, joint_names2;
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

  InteractionMatrix m2;
  m2.init(m.semantics());
  EXPECT_TRUE(m.equals(m2));
}

TEST_F(InteractionMatrixTest, PartialAssignment)
{
  InteractionMatrix m;
  m.resize(4);

  m.semantics().twist().reference().setName(reference);
  m.semantics().twist().target().setName(target); 

  ASSERT_EQ(m.size(), joint_names2.size());
  for(std::size_t i=0; i<m.size(); i++)
    m.semantics().joints()(i).setName(joint_names2[i]);

  for(std::size_t row=0; row<m.numerics().rows(); row++)
    for(std::size_t col=0; col<m.numerics().cols(); col++)
      m.numerics()(row,col) = col + 10*row;

  InteractionMatrix m2;
  m2.resize(2);

  m2.semantics().twist().reference().setName(reference);
  m2.semantics().twist().target().setName(target); 

  std::string foo = "foo";
  std::string bar = "bar"; 

  m2.semantics().joints()(0).setName(foo);
  m2.semantics().joints()(1).setName(bar);
  m2.numerics().setZero();

  m.partialAssignment(1, 2, m2);

  ASSERT_TRUE(m.isValid());
  
  EXPECT_STREQ(m.semantics().twist().reference().getName().c_str(), 
      reference.c_str());
  EXPECT_STREQ(m.semantics().twist().target().getName().c_str(), target.c_str());

  EXPECT_STREQ(m.semantics().joints()(0).getName().c_str(), 
      joint_names2[0].c_str());
  EXPECT_STREQ(m.semantics().joints()(1).getName().c_str(), foo.c_str());
  EXPECT_STREQ(m.semantics().joints()(2).getName().c_str(), bar.c_str());
  EXPECT_STREQ(m.semantics().joints()(3).getName().c_str(), 
      joint_names2[3].c_str());
  
  for(std::size_t col=0; col<m.numerics().cols(); col++)
    EXPECT_EQ(m.numerics()(0, col), col);

  for(std::size_t col=0; col<m.numerics().cols(); col++)
    EXPECT_EQ(m.numerics()(1, col), 0);

  for(std::size_t col=0; col<m.numerics().cols(); col++)
    EXPECT_EQ(m.numerics()(2, col), 0);

  for(std::size_t col=0; col<m.numerics().cols(); col++)
    EXPECT_EQ(m.numerics()(3, col), col + 30);
}

TEST_F(InteractionMatrixTest, Multiplication)
{
  std::vector<std::string> robot_joints;
  robot_joints.push_back("elbow");
  robot_joints.push_back("shoulder");
  robot_joints.push_back("hip");

  std::vector<std::string> constraints;
  constraints.push_back("above");
  constraints.push_back("behind");
  constraints.push_back("left");
  constraints.push_back("facing");

  InteractionMatrix H;
  H.init(constraints, reference, target);

  Jacobian J;
  J.init(robot_joints, reference, target);
  
  using Eigen::operator<<;
  H.numerics() << 1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0;

  J.numerics().data << 0, 1, 2,
                       3, 4, 5,
                       10, 11, 12,
                       13, 14, 15,
                       20, 21, 22,
                       23, 24, 25;

  JointMappingMatrix A;
  A.init(constraints, robot_joints);
  ASSERT_TRUE(areMultipliable(H, J));
  multiply(H, J, A);
  EXPECT_TRUE(A.semantics().row_joints().equals(H.semantics().joints()));
  EXPECT_TRUE(A.semantics().column_joints().equals(J.semantics().joints()));

  Eigen::Matrix<double, 4, 3> result_data;
  result_data <<  0, 1, 2,
                  3, 4, 5,
                  0, 0, 0,
                  0, 0, 0;
  EXPECT_TRUE(result_data.isApprox(A.numerics()));
}
