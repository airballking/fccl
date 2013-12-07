#ifndef FCCL_KDL_JOINT_H
#define FCCL_KDL_JOINT_H

#include <fccl/utils/StatePolicies.h>
#include <fccl/semantics/SemanticsBase.h>

using namespace fccl::utils;
using namespace fccl::semantics;
using namespace std;

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

    template <class T, template <class> class StatePolicy, class SemanticsPolicy>
    inline ostream& operator<<(ostream& os, 
        const Joint<T, StatePolicy, SemanticsPolicy>& joint)
    {
      os << static_cast< const StatePolicy<T>& >(joint) << "\n";
      os << static_cast< const SemanticsPolicy& >(joint);
      return os;
    }

    typedef Joint<double, PositionState, SemanticsBase> PositionJoint;
    typedef Joint<double, VelocityState, SemanticsBase> VelocityJoint;
    typedef Joint<double, AccelerationState, SemanticsBase> AccelerationJoint;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
