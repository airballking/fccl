#include <gtest/gtest.h>

#include <fccl/kdl/JointArray.h>

using namespace fccl::kdl;

class JointArrayTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joints.resize(3);
      joints2.resize(3);

      joints(0, 0).setName("shoulder");
      joints(1, 0).setName("elbow");
      joints(2, 0).setName("neck");

      joints2 = joints;

      for(std::size_t i=0; i<joints.rows(); i++)
      {
        joints(i, 0).position() = 1.0 + 0.1*i;
        joints(i, 0).velocity() = 2.0 + 0.1*i;
        joints(i, 0).acceleration() = 3.0 + 0.1*i;

        joints2(i, 0).position() = -1.0 - 0.1*i;
        joints2(i, 0).velocity() = -2.0 - 0.1*i;
        joints2(i, 0).acceleration() = -3.0 - 0.1*i;
      }
    }

    virtual void TearDown()
    {
    }

    Eigen::Matrix< AccelerationJoint, Eigen::Dynamic, 1> joints, joints2;
};

TEST_F(JointArrayTest, Basics)
{
  AccelerationJointArray jointArray;
  EXPECT_EQ(jointArray.size(), 0);
  jointArray.resize(joints.rows());
  ASSERT_EQ(jointArray.size(), joints.rows());
  
  for(std::size_t i=0; i<joints.rows(); i++)
  {
    jointArray(i) = joints(i, 0);
    EXPECT_TRUE(jointArray(i).equals(joints(i, 0)));
  }

  AccelerationJointArray jointArray2(jointArray);
  EXPECT_TRUE(jointArray.equals(jointArray2));
  ASSERT_EQ(jointArray.size(), jointArray2.size());
  for(std::size_t i=0; i<jointArray.size(); i++)
    EXPECT_TRUE(jointArray(i).equals(jointArray2(i)));
 
  AccelerationJointArray jointArray3;
  jointArray3 = jointArray;
  EXPECT_TRUE(jointArray.equals(jointArray3));
}

TEST_F(JointArrayTest, AdditionSubtraction)
{
  AccelerationJointArray jointArray, jointArray2;
  jointArray.resize(joints.rows());
  jointArray2.resize(joints2.rows());

  ASSERT_EQ(joints.size(), jointArray.size());
  ASSERT_EQ(joints2.size(), jointArray2.size());
  ASSERT_EQ(jointArray.size(), jointArray2.size());
  for(std::size_t i=0; i<joints.size(); i++)
  {
    jointArray(i) = joints(i, 0);
    jointArray2(i) = joints2(i, 0);
  }

  AccelerationJointArray jointArray3;
  jointArray3 = jointArray + jointArray2;
  ASSERT_EQ(jointArray3.size(), jointArray.size());
  for(std::size_t i=0; i<jointArray.size(); i++)
    EXPECT_TRUE(jointArray3(i).equals(jointArray(i) + jointArray2(i)));

  AccelerationJointArray jointArray4;
  jointArray4 = jointArray - jointArray2;
  ASSERT_EQ(jointArray4.size(), jointArray.size());
  for(std::size_t i=0; i<jointArray.size(); i++)
    EXPECT_TRUE(jointArray4(i).equals(jointArray(i) - jointArray2(i)));

}

TEST_F(JointArrayTest, Init)
{
  using namespace fccl::base;
  typedef Array<SemanticsBase, SemanticsBase> SemanticsArray;
  SemanticsArray array1;
  array1.resize(3);
  array1(0).setName("shoulder");
  array1(1).setName("elbow");
  array1(2).setName("neck");
  PositionJointArray array2;
  array2.init(array1);

  ASSERT_EQ(array2.size(), array1.size());
  for(std::size_t i=0; i<array2.size(); i++)
    EXPECT_TRUE(array1(i).semantics().equals(array2(i).semantics()));

  PositionJointArray array3;
  array3.resize(2);
  array3(0).setName("joint0");
  array3(1).setName("joint1");
  AccelerationJointArray array4;
  array4.init(array3);
  
  ASSERT_EQ(array4.size(), array3.size());
  for(std::size_t i=0; i<array4.size(); i++)
    EXPECT_TRUE(array4(i).semantics().equals(array3(i).semantics()));
}
