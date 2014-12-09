#include <gtest/gtest.h>

#include <fccl_conversions/YamlParser.h>
#include <fstream>

using namespace fccl::conversions;

class YamlParserTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

};

TEST_F(YamlParserTest, Features)
{
  std::ifstream file_in("features.yaml");
  ASSERT_TRUE(file_in.good());
  YAML::Parser parser(file_in);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  fccl::base::Feature feature;
  for(unsigned int i=0; i<doc.size(); ++i)
  {
    doc[i] >> feature;
    std::cout << feature << "\n";
  }
}
