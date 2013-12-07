#include <gtest/gtest.h>

#include <fccl/kdl/JointArray.h>

using namespace fccl::kdl;

class JointArrayTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {

    }

};

TEST_F(JointArrayTest, Basics)
{
  AccelerationJointArray joints;
  joints.resize(2);
  
  joints(0).setName("shoulder");
  joints(1).setName("elbow");
  joints(0).position() = 1.0;
  joints(1).position() = -1.0;
  joints(0).velocity() = 2.0;
  joints(1).velocity() = -2.0;
  joints(0).acceleration() = 2.0;
  joints(1).acceleration() = -2.0;

  std::cout << joints;
}
