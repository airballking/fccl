#include <gtest/gtest.h>

#include <fccl/utils/TransformMap.h>
#include <fccl/kdl/Transform.h>
#include <fccl/semantics/TransformSemantics.h>
#include <kdl/frames.hpp>

using namespace fccl::kdl;
using namespace fccl::semantics;
using namespace fccl::utils;

class TransformMapTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      base = "base";
      torso = "torso";
      left = "left";

      T_base_torso_data = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(-0.1, 0.0, 0.55));
      T_torso_left_data = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.4, 0.3));

      T_base_torso_semantics.reference().setName(base);
      T_base_torso_semantics.target().setName(torso);

      T_torso_left_semantics.reference().setName(torso);
      T_torso_left_semantics.target().setName(left);

      T_base_torso.numerics() = T_base_torso_data;
      T_base_torso.semantics() = T_base_torso_semantics;
   
      T_torso_left.numerics() = T_torso_left_data;
      T_torso_left.semantics() = T_torso_left_semantics;
    }

    virtual void TearDown()
    {

    }

    std::string base, torso, left;
    KDL::Frame T_base_torso_data, T_torso_left_data;
    TransformSemantics T_base_torso_semantics, T_torso_left_semantics;
    Transform T_base_torso, T_torso_left;
};

TEST_F(TransformMapTest, Basics)
{
  TransformMap tm;
  EXPECT_FALSE(tm.hasTransform(T_base_torso_semantics));
  EXPECT_FALSE(tm.hasTransform(T_torso_left_semantics));
  tm.setTransform(T_base_torso);
  tm.setTransform(T_torso_left);
  EXPECT_TRUE(tm.getTransform(T_base_torso_semantics).equals(T_base_torso));
  EXPECT_TRUE(tm.getTransform(T_torso_left_semantics).equals(T_torso_left));
  EXPECT_TRUE(tm.hasTransform(T_base_torso_semantics));
  EXPECT_TRUE(tm.hasTransform(T_torso_left_semantics));
  EXPECT_TRUE(tm.getTransform(T_base_torso_semantics.reference(), 
      T_base_torso_semantics.target()).equals(T_base_torso));
  EXPECT_TRUE(tm.getTransform(T_torso_left_semantics.reference(), 
      T_torso_left_semantics.target()).equals(T_torso_left));
  EXPECT_TRUE(tm.hasTransform(T_base_torso_semantics.reference(), 
      T_base_torso_semantics.target()));
  EXPECT_TRUE(tm.hasTransform(T_torso_left_semantics.reference(), 
      T_torso_left_semantics.target()));
  
  EXPECT_NO_THROW(tm.removeTransform(T_base_torso_semantics));
  EXPECT_ANY_THROW(tm.getTransform(T_base_torso_semantics));
  EXPECT_FALSE(tm.hasTransform(T_base_torso_semantics));
  EXPECT_TRUE(tm.hasTransform(T_torso_left_semantics));

  EXPECT_NO_THROW(tm.clear());
  EXPECT_ANY_THROW(tm.getTransform(T_torso_left_semantics));  

  EXPECT_NO_THROW(tm.setTransform(T_base_torso));
  EXPECT_TRUE(tm.getTransform(T_base_torso_semantics).equals(T_base_torso));

  Transform fake_transform;
  fake_transform.numerics() = KDL::Frame::Identity();
  fake_transform.semantics() = T_base_torso_semantics;
  EXPECT_NO_THROW(tm.setTransform(fake_transform));
  EXPECT_FALSE(tm.getTransform(fake_transform.semantics()).equals(T_base_torso));
  EXPECT_TRUE(tm.getTransform(T_base_torso_semantics).equals(fake_transform));
  EXPECT_TRUE(tm.hasTransform(T_base_torso_semantics));

  EXPECT_ANY_THROW(tm.getTransform(T_torso_left_semantics));
}
