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

TEST_F(YamlParserTest, FeaturesParsingBasics)
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
    EXPECT_TRUE(feature.isValid());
  }
}

TEST_F(YamlParserTest, ConstraintsParsingBasics)
{
  std::ifstream file_in("constraints.yaml");
  ASSERT_TRUE(file_in.good());
  YAML::Parser parser(file_in);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  fccl::base::Constraint constraint;
  for(unsigned int i=0; i<doc.size(); ++i)
  {
    doc[i] >> constraint;
    EXPECT_TRUE(constraint.isValid());
  }
}

TEST_F(YamlParserTest, ConstraintArrayParsingBasics)
{
  std::ifstream file_in("constraints.yaml");
  ASSERT_TRUE(file_in.good());
  YAML::Parser parser(file_in);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  fccl::base::ConstraintArray constraints;
  doc >> constraints;
  EXPECT_TRUE(constraints.isValid());
}
