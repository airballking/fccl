#ifndef FCCL_KDL_JOINT_ARRAY_H
#define FCCL_KDL_JOINT_ARRAY_H

#include <fccl/base/Array.h>
#include <fccl/kdl/Joint.h>

namespace fccl
{
  namespace kdl
  {
    typedef fccl::base::Array<PositionJoint> PositionJointArray;
    typedef fccl::base::Array<VelocityJoint> VelocityJointArray;
    typedef fccl::base::Array<AccelerationJoint> AccelerationJointArray;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_ARRAY_H
