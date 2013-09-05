#ifndef FCCL_UTILS_PRINTING_H
#define FCCL_UTILS_PRINTING_H

#include <iostream>
#include <kdl/frames.hpp>

namespace fccl
{

  std::ostream& operator<<(std::ostream& out, const KDL::Vector& v);
  std::ostream& operator<<(std::ostream& out, const KDL::Rotation& v);
  std::ostream& operator<<(std::ostream& out, const KDL::Frame& f);

} // namespace fccl
#endif // FCCL_UTILS_PRINTING_H
