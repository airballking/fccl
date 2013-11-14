#include <gtest/gtest.h>

#include <fccl/control/Gains.h>

using namespace fccl::control;

class GainsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");

      semantics.init(joint_names);

      p.init(semantics);
      i.init(semantics);
      d.init(semantics);
      i_max.init(semantics);
      i_min.init(semantics);

      p.numerics().data.setRandom(p.size());
      i.numerics().data.setRandom(i.size());
      d.numerics().data.setRandom(d.size());
      i_max.numerics().data.setRandom(i_max.size());
      i_min.numerics().data.setRandom(i_min.size());
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    fccl::semantics::JntArraySemantics semantics;
    fccl::kdl::JntArray p, i, d, i_max, i_min;
};

TEST_F(GainsTest, Basics)
{
  PIDGains gains1;
  gains1.init(semantics);
  ASSERT_TRUE(gains1.areValid());
  ASSERT_TRUE(gains1.p().semantics().equals(semantics));
  EXPECT_EQ(gains1.size(), semantics.size());

  PIDGains gains2;
  gains2.init(semantics);
  ASSERT_TRUE(gains2.areValid());
  ASSERT_TRUE(gains2.p().semantics().equals(semantics));

  gains1.p() = p; 
  gains1.i() = i; 
  gains1.d() = d; 
  gains1.i_max() = i_max;
  gains1.i_min() = i_min;

  ASSERT_TRUE(gains1.areValid());
  ASSERT_TRUE(gains1.p().equals(p));
  ASSERT_TRUE(gains1.i().equals(i));
  ASSERT_TRUE(gains1.d().equals(d));
  ASSERT_TRUE(gains1.i_max().equals(i_max));
  ASSERT_TRUE(gains1.i_min().equals(i_min));

  gains2 = gains1;
  ASSERT_TRUE(gains2.areValid());
  EXPECT_TRUE(gains1.equals(gains2));

  PIDGains gains3;
  gains3.resize(3);
  ASSERT_EQ(gains3.size(), 3);
  ASSERT_FALSE(gains3.areValid());
  gains3.setSemantics(semantics);
  ASSERT_TRUE(gains3.areValid());
  EXPECT_TRUE(gains3.p().semantics().equals(semantics));
  EXPECT_FALSE(gains3.equals(gains1));
  gains3.p() = p; 
  EXPECT_FALSE(gains3.equals(gains1));
  gains3.i() = i; 
  EXPECT_FALSE(gains3.equals(gains1));
  gains3.d() = d; 
  EXPECT_FALSE(gains3.equals(gains1));
  gains3.i_max() = i_max;
  EXPECT_FALSE(gains3.equals(gains1));
  gains3.i_min() = i_min;
  EXPECT_TRUE(gains3.equals(gains1));
}
