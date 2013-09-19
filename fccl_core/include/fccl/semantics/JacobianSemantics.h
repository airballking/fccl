#ifndef FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H
#define FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H

#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/TwistSemantics.h>

namespace fccl
{
  namespace semantics
  {
    class JacobianSemantics
    {
      public:
        const JntArraySemantics& joints() const
        {
          return joints_;
        }

        JntArraySemantics& joints()
        {
          return joints_;
        }

        const TwistSemantics& twist() const
        {
          return twist_;
        }

        TwistSemantics& twist()
        {
          return twist_;
        }

        bool equals(const JacobianSemantics& other) const
        {
          return twist().equals(other.twist()) &&
              joints().equals(other.joints());
        }

      private:
        TwistSemantics twist_;
        JntArraySemantics joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const JacobianSemantics& obj)
    {
      os << "twist:\n " << obj.twist() << "\n";
      os << "joints:\n " << obj.joints();
      return os;
    }

    inline bool areMultipliable(const JacobianSemantics& lhs, const JntArraySemantics& rhs)
    {
      return lhs.joints().equals(rhs);
    }

    inline TwistSemantics multiply(const JacobianSemantics& lhs, const JntArraySemantics& rhs)
    {
      assert(areMultipliable(lhs, rhs));

      return lhs.twist();
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H
