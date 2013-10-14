#include <gtest/gtest.h>

#include <fccl/semantics/JntArraySemantics.h>
#include <vector>

using namespace fccl::semantics;

class JntArraySemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");
      joint_names.push_back("joint3");

      joint_names2.push_back("shoulder");
      joint_names2.push_back("elbow");

      joint_names3.push_back("joint0");
      joint_names3.push_back("shoulder");
      joint_names3.push_back("elbow");
      joint_names3.push_back("joint3");
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> joint_names;
    std::vector<std::string> joint_names2, joint_names3;
};

TEST_F(JntArraySemanticsTest, Basics)
{ 
  JntArraySemantics js;
  js.resize(joint_names.size());
  ASSERT_EQ(joint_names.size(), js.size());

  for(std::size_t i=0; i < js.size(); i++)
    js(i).setName(joint_names[i]);

  JntArraySemantics js2(js);

  JntArraySemantics js3;
  js3.resize(js.size());
  for(std::size_t i=0; i<js.size(); i++)
    js3(i).setName(js(i).getName());

  JntArraySemantics js4;
  js4 = js;

  JntArraySemantics js5;
  js5.joints() = js.joints();

  EXPECT_TRUE(js.equals(js2));
  EXPECT_TRUE(js.equals(js3));
  EXPECT_TRUE(js.equals(js4));
  EXPECT_TRUE(js.equals(js5));
}

TEST_F(JntArraySemanticsTest, partialAssignement)
{
  JntArraySemantics js, js2, js3;

  js.resize(joint_names.size());
  for(std::size_t i=0; i < js.size(); i++)
    js(i).setName(joint_names[i]);

  js2.resize(joint_names2.size());
  for(std::size_t i=0; i < js2.size(); i++)
    js2(i).setName(joint_names2[i]);


  js3.resize(joint_names3.size());
  for(std::size_t i=0; i < js3.size(); i++)
    js3(i).setName(joint_names3[i]);

  js.partialAssignment(1, 2, js2);
  
  EXPECT_TRUE(js.equals(js3));
}

TEST_F(JntArraySemanticsTest, init)
{
  JntArraySemantics js;
  js.init(joint_names);

  ASSERT_EQ(js.size(), joint_names.size());
  for(std::size_t i=0; i<js.size(); i++)
    EXPECT_STREQ(js(i).getName().c_str(), joint_names[i].c_str());
}
