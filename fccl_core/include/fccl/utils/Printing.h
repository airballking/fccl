#ifndef FCCL_UTILS_PRINTING_H
#define FCCL_UTILS_PRINTING_H

#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Core>

namespace fccl
{
  namespace utils
  {
    std::ostream& operator<<(std::ostream& out, const KDL::Vector& v);
    std::ostream& operator<<(std::ostream& out, const KDL::Rotation& v);
    std::ostream& operator<<(std::ostream& out, const KDL::Frame& f);
    std::ostream& operator<<(std::ostream& out, const KDL::Twist& t);
    std::ostream& operator<<(std::ostream& out, const KDL::JntArray& j);
    std::ostream& operator<<(std::ostream& out, const KDL::Jacobian& j);
    std::ostream& operator<<(std::ostream& out, const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& m);
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_PRINTING_H
