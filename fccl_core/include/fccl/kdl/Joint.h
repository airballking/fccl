#ifndef FCCL_KDL_JOINT_H
#define FCCL_KDL_JOINT_H

#include <fccl/base/Policies.h>
#include <fccl/semantics/SemanticsBase.h>

using namespace fccl::utils;

namespace fccl
{
  namespace kdl
  {
    template <class StatePolicy>
    class Joint : public StatePolicy
    {
      public:
        const fccl::semantics::SemanticsBase& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::SemanticsBase& semantics()
        {
          return semantics_;
        }

        virtual bool equals(const Joint& other) const
        {
          return this->semantics().equals(other.semantics()) &&
              this->state().equals(other.state());
        }
      private:
        fccl::semantics::SemanticsBase semantics_;
    };

    typedef Joint<DoublePositionState> PositionJoint;
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
