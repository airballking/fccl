#include <gtest/gtest.h>

#include <fccl/kdl/JointMappingMatrix.h>
#include <fccl/utils/Equalities.h>

using namespace fccl::kdl;

class JointMappingMatrixTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");

      constraint_names.push_back("height");
      constraint_names.push_back("left");
      constraint_names.push_back("behind");

      data.resize(constraint_names.size(), joint_names.size());
      for(unsigned int i=0; i<data.rows(); i++)
        for(unsigned int j=0; j<data.cols(); j++)
          data(i, j) = i*j;

      semantics.row_joints().resize(constraint_names.size());
      for(std::size_t i=0; i< constraint_names.size(); i++)
        semantics.row_joints()(i).setName(constraint_names[i]);
      semantics.column_joints().resize(joint_names.size());
      for(std::size_t i=0; i< joint_names.size(); i++)
        semantics.column_joints()(i).setName(joint_names[i]);
    }

    virtual void TearDown()
    {

    }

    std::vector<std::string> joint_names, constraint_names;
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > data;
    fccl::semantics::JointMappingSemantics semantics;
};

TEST_F(JointMappingMatrixTest, Basics)
{
  JointMappingMatrix A;
  ASSERT_TRUE(A.isValid());

  EXPECT_EQ(A.numerics().rows(), 0);
  EXPECT_EQ(A.numerics().cols(), 0);
  EXPECT_EQ(A.semantics().row_joints().size(), 0);
  EXPECT_EQ(A.semantics().column_joints().size(), 0);

  A.resize(data.rows(), data.cols());
  EXPECT_EQ(A.numerics().rows(), data.rows());
  EXPECT_EQ(A.numerics().cols(), data.cols());
  EXPECT_EQ(A.semantics().row_joints().size(), data.rows());
  EXPECT_EQ(A.semantics().column_joints().size(), data.cols());

  EXPECT_FALSE(data.isApprox(A.numerics()));
  A.numerics() = data;
  EXPECT_TRUE(data.isApprox(A.numerics()));

  A.semantics() = semantics;
  for(std::size_t i=0; i< constraint_names.size(); i++)
    EXPECT_STREQ(semantics.row_joints()(i).getName().c_str(),
        constraint_names[i].c_str());
  for(std::size_t i=0; i< joint_names.size(); i++)
    EXPECT_STREQ(semantics.column_joints()(i).getName().c_str(),
        joint_names[i].c_str());

  JointMappingMatrix A2(A);
  EXPECT_TRUE(A.equals(A2));

  JointMappingMatrix A3;
  EXPECT_FALSE(A.equals(A3));
  A3.resize(A.semantics().row_joints().size(), A.semantics().column_joints().size());
  EXPECT_FALSE(A.equals(A3));
  A3.numerics() = data;
  EXPECT_FALSE(A.equals(A3));
  for(std::size_t i=0; i< constraint_names.size(); i++)
    semantics.row_joints()(i).setName(constraint_names[i]);
  EXPECT_FALSE(A.equals(A3));
  for(std::size_t i=0; i< joint_names.size(); i++)
    semantics.column_joints()(i).setName(joint_names[i]);
  EXPECT_TRUE(A.equals(A2));
}

TEST_F(JointMappingMatrixTest, Init)
{
  JointMappingMatrix A;
  A.init(constraint_names, joint_names);

  ASSERT_EQ(A.numerics().rows(), constraint_names.size());
  ASSERT_EQ(A.numerics().cols(), joint_names.size());

  ASSERT_EQ(A.semantics().row_joints().size(), constraint_names.size());
  ASSERT_EQ(A.semantics().column_joints().size(), joint_names.size());

  ASSERT_EQ(constraint_names.size(), 3);
  ASSERT_EQ(joint_names.size(), 2);
 
  for(std::size_t i=0; i< constraint_names.size(); i++)
    EXPECT_STREQ(semantics.row_joints()(i).getName().c_str(),
        constraint_names[i].c_str());
  for(std::size_t i=0; i< joint_names.size(); i++)
    EXPECT_STREQ(semantics.column_joints()(i).getName().c_str(),
        joint_names[i].c_str());
}
