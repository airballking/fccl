#include <gtest/gtest.h>

#include <fccl/kdl/Transform.h>
#include <fccl/utils/TransformMap.h>
#include <kdl/frames.hpp>

using namespace fccl::kdl;
using namespace fccl::utils;

class TransformMapTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      base = "base";
      torso = "torso";
      left = "left";

      t_base_torso_data = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(-0.1, 0.0, 0.55));
      t_torso_left_data = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0), KDL::Vector(0.0, 0.4, 0.3));

      t_base_torso_semantics = SemanticObject1x1(base, torso);
      t_torso_left_semantics = SemanticObject1x1(torso, left);
      T_base_torso = Transform(t_base_torso_semantics, t_base_torso_data);
      T_torso_left = Transform(t_torso_left_semantics, t_torso_left_data);
    }

    virtual void TearDown()
    {

    }

    std::string base, torso, left;
    KDL::Frame t_base_torso_data, t_torso_left_data;
    SemanticObject1x1 t_base_torso_semantics, t_torso_left_semantics;
    Transform T_base_torso, T_torso_left;
};

TEST_F(TransformMapTest, Basics)
{
  TransformMap tm;
  tm.setTransform(T_base_torso);
  tm.setTransform(T_torso_left);
  EXPECT_EQ(tm.getTransform(t_base_torso_semantics), T_base_torso);
  EXPECT_EQ(tm.getTransform(t_torso_left_semantics), T_torso_left);
  EXPECT_EQ(tm.getTransform(T_base_torso), T_base_torso);
  EXPECT_EQ(tm.getTransform(T_torso_left), T_torso_left);
  
  EXPECT_NO_THROW(tm.removeTransform(t_base_torso_semantics));
  EXPECT_NO_THROW(tm.removeTransform(T_base_torso));
  EXPECT_ANY_THROW(tm.getTransform(T_base_torso));

  EXPECT_NO_THROW(tm.clear());
  EXPECT_ANY_THROW(tm.getTransform(t_torso_left_semantics));  

  EXPECT_NO_THROW(tm.setTransform(T_base_torso));
  EXPECT_EQ(tm.getTransform(t_base_torso_semantics), T_base_torso);
  EXPECT_NO_THROW(tm.setTransform(Transform(t_base_torso_semantics, KDL::Frame::Identity())));
  EXPECT_NE(tm.getTransform(T_base_torso), T_base_torso); 
  EXPECT_EQ(tm.getTransform(T_base_torso), Transform(t_base_torso_semantics, KDL::Frame::Identity()));

  EXPECT_ANY_THROW(tm.getTransform(T_torso_left));
}
