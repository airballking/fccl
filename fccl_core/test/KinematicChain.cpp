#include <gtest/gtest.h>

#include <urdf/model.h>

class KinematicChainTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      urdf.initFile("pr2.urdf");
    }

    virtual void TearDown()
    {

    }

    urdf::Model urdf;
};

TEST_F(KinematicChainTest, Basics)
{
}
