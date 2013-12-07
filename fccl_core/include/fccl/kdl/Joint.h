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
    template <class StatePolicy, class SemanticsPolicy>
    class Joint : public StatePolicy, public SemanticsPolicy
    {
      public:
        virtual bool equals(const Joint& other) const
        {
          return this->semantics().equals(other.semantics()) &&
              this->state().equals(other.state());
        }
    };

    typedef Joint<DoublePositionState, SemanticsBase> PositionJoint;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
