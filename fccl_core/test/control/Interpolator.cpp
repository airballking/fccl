#include <gtest/gtest.h>

#include <fccl/control/Interpolator.h>

using namespace fccl::kdl;
using namespace fccl::control;

class InterpolatorTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      cycle_time = 0.001;

      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");
      
      current_pos.init(joint_names); 
      current_vel.init(joint_names);
      current_acc.init(joint_names);
      target_pos.init(joint_names); 
      target_vel.init(joint_names);
      target_acc.init(joint_names);
      max_vel.init(joint_names);
      max_acc.init(joint_names); 
      max_jerk.init(joint_names);

      current_pos.numerics()(0) = 100.0;
      current_pos.numerics()(1) = 0.0;
      current_pos.numerics()(2) = 50.0;

      current_vel.numerics()(0) = 100.0;
      current_vel.numerics()(1) = -220.0;
      current_vel.numerics()(2) = -50.0;

      current_acc.numerics()(0) = -150.0;
      current_acc.numerics()(1) = 250.0;
      current_acc.numerics()(2) = -50.0;

      max_vel.numerics()(0) = 300.0;
      max_vel.numerics()(1) = 100.0;
      max_vel.numerics()(2) = 300.0;

      max_acc.numerics()(0) = 300.0;
      max_acc.numerics()(1) = 200.0;
      max_acc.numerics()(2) = 100.0;

      max_jerk.numerics()(0) = 400.0;
      max_jerk.numerics()(1) = 300.0;
      max_jerk.numerics()(2) = 200.0;

      target_pos.numerics()(0) = -600.0;
      target_pos.numerics()(1) = -200.0;
      target_pos.numerics()(2) = -350.0;

      target_vel.numerics()(0) = 50.0;
      target_vel.numerics()(1) = -50.0;
      target_vel.numerics()(2) = -200.0;

      target_acc.numerics()(0) = 0.0; 
      target_acc.numerics()(1) = 0.0; 
      target_acc.numerics()(2) = 0.0; 
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    JntArray current_pos, current_vel, current_acc;
    JntArray target_pos, target_vel, target_acc;
    JntArray max_vel, max_acc, max_jerk;
    double cycle_time;
};

TEST_F(InterpolatorTest, Basics)
{
  Interpolator i;
  i.init(current_pos.semantics(), cycle_time);  
  EXPECT_TRUE(i.inputValid());
  EXPECT_TRUE(i.semantics().equals(current_pos.semantics()));
  EXPECT_TRUE(i.nextPosition().semantics().equals(current_pos.semantics()));
  EXPECT_TRUE(i.nextVelocity().semantics().equals(current_pos.semantics()));
  EXPECT_FALSE(i.interpolationFinished());
}

TEST_F(InterpolatorTest, InterpolationSimple)
{
  Interpolator i;
  i.init(current_pos.semantics(), cycle_time);  
  ASSERT_TRUE(i.semantics().equals(current_pos.semantics()));

  i.setCurrentPosition(current_pos);
  i.setCurrentVelocity(current_vel);
  i.setCurrentAcceleration(current_acc);
 
  i.setTargetPosition(target_pos);
  i.setTargetVelocity(target_vel);

  i.setMaxVelocity(max_vel);
  i.setMaxAcceleration(max_acc);
  i.setMaxJerk(max_jerk);

  ASSERT_TRUE(i.inputValid());
  ASSERT_FALSE(i.interpolationFinished());

  for(std::size_t j=0; j<current_pos.semantics().size(); j++)
    i.setDimensionActivity(j, true);

  while(!i.interpolationFinished())
  {
    i.interpolate();

    i.setCurrentPosition(i.nextPosition());
    i.setCurrentVelocity(i.nextVelocity());
    i.setCurrentAcceleration(i.nextAcceleration());
  }
  
  EXPECT_TRUE(i.nextPosition().semantics().equals(target_pos.semantics()));
  EXPECT_TRUE(i.nextVelocity().semantics().equals(target_vel.semantics()));
  EXPECT_TRUE(i.nextAcceleration().semantics().equals(target_acc.semantics()));

  for(std::size_t j=0; j<current_pos.semantics().size(); j++)
  {
    EXPECT_NEAR(i.nextPosition().numerics()(j), target_pos.numerics()(j), 0.1);
    EXPECT_NEAR(i.nextVelocity().numerics()(j), target_vel.numerics()(j), 0.1);
    EXPECT_NEAR(i.nextAcceleration().numerics()(j), target_acc.numerics()(j), 0.1);
  }
}

TEST_F(InterpolatorTest, InterpolationParams)
{
  Interpolator i;
  i.init(current_pos.semantics(), cycle_time);  
  ASSERT_TRUE(i.semantics().equals(current_pos.semantics()));

  ASSERT_TRUE(i.inputValid());
  ASSERT_FALSE(i.interpolationFinished());

  for(std::size_t j=0; j<current_pos.semantics().size(); j++)
    i.setDimensionActivity(j, true);

  i.interpolate(target_pos, target_vel, current_pos, current_vel,
      current_acc, max_vel, max_acc, max_jerk);

  while(!i.interpolationFinished())
  {
    i.interpolate(target_pos, target_vel, i.nextPosition(), i.nextVelocity(),
        i.nextAcceleration(), max_vel, max_acc, max_jerk);
  }
  
  EXPECT_TRUE(i.nextPosition().semantics().equals(target_pos.semantics()));
  EXPECT_TRUE(i.nextVelocity().semantics().equals(target_vel.semantics()));
  EXPECT_TRUE(i.nextAcceleration().semantics().equals(target_acc.semantics()));

  for(std::size_t j=0; j<current_pos.semantics().size(); j++)
  {
    EXPECT_NEAR(i.nextPosition().numerics()(j), target_pos.numerics()(j), 0.1);
    EXPECT_NEAR(i.nextVelocity().numerics()(j), target_vel.numerics()(j), 0.1);
    EXPECT_NEAR(i.nextAcceleration().numerics()(j), target_acc.numerics()(j), 0.1);
  }
}
