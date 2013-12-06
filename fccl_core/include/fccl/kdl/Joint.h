#ifndef FCCL_KDL_JOINT_H
#define FCCL_KDL_JOINT_H

#include <fccl/semantics/SemanticsBase.h>
#include <limits>

namespace fccl
{
  namespace kdl
  {
    // TODO(Georg): move this to utils
    inline bool areEqual(double a, double b)
    {
      return fabs(a-b) < std::numeric_limits<double>::epsilon();
    }

    class Joint
    {
      public:
        double position() const
        {
          return position_;
        }

        double& position()
        {
          return position_;
        }

        const fccl::semantics::SemanticsBase& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::SemanticsBase& semantics()
        {
          return semantics_;
        }

        bool equals(const Joint& other) const
        {
          return semantics().equals(other.semantics()) &&
              areEqual(position(), other.position());
        }
      private:
        double position_;
        fccl::semantics::SemanticsBase semantics_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JOINT_H
