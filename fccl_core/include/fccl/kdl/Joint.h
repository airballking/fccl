#ifndef FCCL_KDL_JOINT_H
#define FCCL_KDL_JOINT_H

#include <fccl/utils/StatePolicies.h>
#include <fccl/semantics/SemanticsBase.h>

using namespace fccl::utils;
using namespace fccl::semantics;

namespace fccl
{
  namespace kdl
  {
    template <class T, template <class> class StatePolicy, class SemanticsPolicy>
    class Joint : public StatePolicy<T>, public SemanticsPolicy
    {
      public:
        virtual bool equals(const Joint& other) const
        {
          return this->semantics().equals(other.semantics()) &&
              this->state().equals(other.state());
        }
    };

    typedef Joint<double, PositionState, SemanticsBase> PositionJoint;
    typedef Joint<double, VelocityState, SemanticsBase> VelocityJoint;
    typedef Joint<double, AccelerationState, SemanticsBase> AccelerationJoint;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
