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
    }

    virtual void TearDown()
    {

    }

    std::vector<std::string> joint_names, constraint_names;
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > data;
};

TEST_F(JointMappingMatrixTest, Basics)
{
  JointMappingMatrix A;
  ASSERT_TRUE(A.isValid());

  EXPECT_EQ(A.rows(), 0);
  EXPECT_EQ(A.columns(), 0);
  EXPECT_EQ(A.size().first, 0);
  EXPECT_EQ(A.size().second, 0);

  A.resize(std::pair<std::size_t, std::size_t>(data.rows(), data.cols()));
  EXPECT_EQ(A.rows(), data.rows());
  EXPECT_EQ(A.columns(), data.cols());
  EXPECT_EQ(A.size().first, data.rows());
  EXPECT_EQ(A.size().second, data.cols());  

  EXPECT_TRUE(A.rowIndexValid(0));
  EXPECT_TRUE(A.rowIndexValid(data.rows() - 1));
  EXPECT_FALSE(A.rowIndexValid(data.rows()));

  EXPECT_TRUE(A.columnIndexValid(0));
  EXPECT_TRUE(A.columnIndexValid(data.cols() - 1));
  EXPECT_FALSE(A.columnIndexValid(data.cols()));

  EXPECT_FALSE(data.isApprox(A.getData()));
  A.setData(data);
  EXPECT_TRUE(data.isApprox(A.getData()));

  // TODO(Georg): get this in, once hashing throws exceptions
//  EXPECT_FALSE(fccl::utils::Equal(A.getReferenceNames(), joint_names));
  A.setReferenceNames(joint_names);
  EXPECT_TRUE(fccl::utils::Equal(A.getReferenceNames(), joint_names));

  A.setTargetNames(constraint_names);
  EXPECT_TRUE(fccl::utils::Equal(A.getTargetNames(), constraint_names));

  JointMappingMatrix A2(A);
  EXPECT_EQ(A, A2);

  JointMappingMatrix A3;
  EXPECT_NE(A, A3);
  A3.resize(A.size());
  EXPECT_NE(A, A3);
  A3.setData(data);
  EXPECT_NE(A, A3);
  A3.setReferenceNames(joint_names);
  EXPECT_NE(A, A3);
  A3.setTargetNames(constraint_names);
  EXPECT_EQ(A, A3);

  JointMappingMatrix A4;
  A4.init(A.getSemantics());
  EXPECT_NE(A, A4);
  A4.setData(A.getData());
  EXPECT_EQ(A, A4);
}
