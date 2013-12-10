#ifndef FCCL_KDL_JOINT_ARRAY_H
#define FCCL_KDL_JOINT_ARRAY_H

#include <fccl/base/Array.h>
#include <fccl/kdl/Joint.h>

namespace fccl
{
  namespace kdl
  {
    typedef fccl::base::Array<PositionJoint, SemanticsBase> PositionJointArray;
    typedef fccl::base::Array<VelocityJoint, SemanticsBase> VelocityJointArray;
    typedef fccl::base::Array<AccelerationJoint, SemanticsBase> AccelerationJointArray;
    typedef fccl::base::Array<JerkJoint, SemanticsBase> JerkJointArray;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_ARRAY_H
