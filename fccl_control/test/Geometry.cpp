#include <gtest/gtest.h>

#include <fccl_control/Geometry.h>

using namespace fccl;

class GeometryTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      parent_frame = "parent";
      child_frame = "child";
      transform = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(1.0, 2.0, 3.0));
    }

    virtual void TearDown()
    {

    }

    std::string parent_frame, child_frame;
    KDL::Frame transform;
};

TEST_F(GeometryTest, TESTNAME)
{
  Transform t(parent_frame, child_frame, transform);
  EXPECT_STREQ(parent_frame.c_str(), t.getParentFrame().c_str()); 
  EXPECT_STREQ(child_frame.c_str(), t.getChildFrame().c_str());
  EXPECT_TRUE(KDL::Equal(transform, t.getTransform()));
}
