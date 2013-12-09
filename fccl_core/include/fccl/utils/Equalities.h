#ifndef FCCL_UTILS_EQUALITIES_H
#define FCCL_UTILS_EQUALITIES_H

#include <string>
#include <vector>
#include <kdl/segment.hpp>

namespace fccl
{
  namespace utils
  {
    bool Equal(const std::vector<std::size_t>& v1, const std::vector<std::size_t>& v2);

    bool Equal(const std::vector<std::string>& v1, const std::vector<std::string>& v2);

    bool areEqual(double a, double b);

    bool isJoint(const KDL::Segment& segment);
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_EQUALITIES_H
