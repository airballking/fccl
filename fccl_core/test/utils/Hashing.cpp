#include <gtest/gtest.h>

#include <fccl/utils/Hashing.h>

using namespace fccl::utils;

class HashingTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      string1 = "spatula_frame";
      string2 = "pancake_frame";
    }

    virtual void TearDown()
    {
    }
    
    std::string string1, string2;
};

TEST_F(HashingTest, Basics)
{
  std::size_t hash1 = Hasher::hash(string1);
  std::size_t hash2 = Hasher::hash(string2);

  EXPECT_NE(hash1, hash2);

  EXPECT_EQ(hash1, Hasher::hash(string1));
  EXPECT_EQ(hash2, Hasher::hash(string2));

  EXPECT_STREQ(string1.c_str(), Hasher::retrieveValue(hash1).c_str());
  EXPECT_STREQ(string2.c_str(), Hasher::retrieveValue(hash2).c_str());
  EXPECT_STRNE(string2.c_str(), Hasher::retrieveValue(hash1).c_str());
}
