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

    void operator>> (const YAML::Node& node, fccl::base::Constraint& c)
    {
      std::string tmp_string;
      node["name"] >> tmp_string;
      c.semantics().name().setName(tmp_string);
      node["reference"] >> tmp_string;
      c.semantics().reference().setName(tmp_string);
      node["type"] >> tmp_string;
      std::transform(tmp_string.begin(), tmp_string.end(), 
          tmp_string.begin(), ::tolower);
      c.semantics().type().setName(tmp_string);
      node["tool-feature"] >> c.toolFeature();
      node["object-feature"] >> c.objectFeature();
      node["lower-boundary"] >> c.lowerBoundary();
      node["upper-boundary"] >> c.upperBoundary();
      node["max-velocity"] >> c.maxVelocity();
      node["max-acceleration"] >> c.maxAcceleration();
      node["max-jerk"] >> c.maxJerk();
    }
  } // namespace conversions
} // namespace fccl
