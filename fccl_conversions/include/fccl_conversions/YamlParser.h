#ifndef FCCL_CONVERSIONS_YAML_PARSER_H
#define FCCL_CONVERSIONS_YAML_PARSER_H

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fccl/base/Features.h>

namespace fccl
{
  namespace conversions
  {
    void operator>> (const YAML::Node& node, fccl::base::Feature& f);
  } // namespace conversions
} // namespace fccl
#endif // FCCL_CONVERSIONS_YAML_PARSER_H
