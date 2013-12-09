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
          return this->as<const SemanticsPolicy>().equals(other.as<const SemanticsPolicy>()) &&
              this->as< const StatePolicy<T> >().equals(other.as< const StatePolicy<T> >());
        }

        Joint& operator+=(const Joint& rhs)
        {
          this->as<SemanticsPolicy>() += rhs.as<const SemanticsPolicy>();
          this->as< StatePolicy<T> >() += rhs.as< const StatePolicy<T> >();

          return *this;
        }

        template<class U>
        U& as()
        {
          return static_cast< U& >(*this);
        }

        template<class U>
        const U& as() const
        {
          return static_cast< const U& >(*this);
        }
    };

    template <class T, template <class> class StatePolicy, class SemanticsPolicy>
    inline ostream& operator<<(ostream& os, 
        const Joint<T, StatePolicy, SemanticsPolicy>& joint)
    {
      os << joint.template as< const StatePolicy<T> >() << "\n";
      os << joint.template as< const SemanticsPolicy >();
      return os;
    }

    template <class T, template <class> class StatePolicy, class SemanticsPolicy>
    inline Joint<T, StatePolicy, SemanticsPolicy> operator+(
        Joint<T, StatePolicy, SemanticsPolicy> lhs,
        const Joint<T, StatePolicy, SemanticsPolicy>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    typedef Joint<double, PositionState, SemanticsBase> PositionJoint;
    typedef Joint<double, VelocityState, SemanticsBase> VelocityJoint;
    typedef Joint<double, AccelerationState, SemanticsBase> AccelerationJoint;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
