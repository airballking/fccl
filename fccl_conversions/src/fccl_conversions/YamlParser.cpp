#include <fccl_conversions/YamlParser.h>

namespace fccl
{
  namespace conversions
  {
    void operator>> (const YAML::Node& node, KDL::Vector& v)
    {
      for(unsigned int i=0; i<3; ++i)
        node[i] >> v.data[i];
    }

    void operator>> (const YAML::Node& node, fccl::base::Feature& f)
    {
      std::string tmp_string;
      node["name"] >> tmp_string;
      f.semantics().name().setName(tmp_string);
      node["reference"] >> tmp_string;
      f.semantics().reference().setName(tmp_string);
      node["type"] >> tmp_string;
      f.semantics().type() = fccl::semantics::featureTypeFromString(tmp_string);
      node["position"] >> f.position();
      node["orientation"] >> f.orientation();
    }
  } // namespace conversions
} // namespace fccl
