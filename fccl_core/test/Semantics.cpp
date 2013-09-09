#include <gtest/gtest.h>

#include <fccl/kdl/Semantics.h>

using namespace fccl::kdl;

class SemanticsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      reference = "reference";
      reference2 = "reference2";
      target = "target";
      target2 = "target2";
    }

    virtual void TearDown()
    {
    }

    std::string reference, reference2;
    std::string target, target2;
};

TEST_F(SemanticsTest, TwofoldSemanticObject)
{
  TwofoldSemanticObject s;
  s.setReferenceName(reference);
  s.setTargetName(target);

  EXPECT_STREQ(s.getReferenceName().c_str(), reference.c_str());
  EXPECT_STREQ(s.getTargetName().c_str(), target.c_str());

  TwofoldSemanticObject s2(s);

  TwofoldSemanticObject s3(s);
 
  TwofoldSemanticObject s4;
  s4.setReferenceID(s.getReferenceID());
  s4.setTargetID(s.getTargetID());

  TwofoldSemanticObject s5(reference, target);

  TwofoldSemanticObject s6(s.getReferenceID(), s.getTargetID());

  TwofoldSemanticObject s9;
  s9.setReferenceName(s.getReferenceName());
  s9.setTargetName(s.getTargetName());


  EXPECT_TRUE(s.semanticsEqual(s2));
  EXPECT_TRUE(s.semanticsEqual(s3));
  EXPECT_TRUE(s.semanticsEqual(s4));
  EXPECT_TRUE(s.semanticsEqual(s5));
  EXPECT_TRUE(s.semanticsEqual(s6));
  EXPECT_TRUE(s.semanticsEqual(s9));

  TwofoldSemanticObject s7(reference, target2);
  TwofoldSemanticObject s8(reference2, target);

  EXPECT_FALSE(s.semanticsEqual(s7));
  EXPECT_FALSE(s.semanticsEqual(s8));
}
