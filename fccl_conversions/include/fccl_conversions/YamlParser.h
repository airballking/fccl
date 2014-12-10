#ifndef FCCL_CONVERSIONS_YAML_PARSER_H
#define FCCL_CONVERSIONS_YAML_PARSER_H

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fccl/base/Features.h>
#include <fccl/base/Constraints.h>

namespace fccl
{
  namespace conversions
  {
    void operator>> (const YAML::Node& node, KDL::Vector& v);
 
    void operator>> (const YAML::Node& node, fccl::base::Feature& f);

    void operator>> (const YAML::Node& node, fccl::base::Constraint& c);

  } // namespace conversions
} // namespace fccl
#endif // FCCL_CONVERSIONS_YAML_PARSER_H
