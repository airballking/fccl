#ifndef FCCL_SEMANTICS_TWIST_SEMANTICS_H
#define FCCL_SEMANTICS_TWIST_SEMANTICS_H

#include <fccl/semantics/SemanticsBase.h>

namespace fccl
{
  namespace semantics
  {
    class TwistSemantics
    {
      public:
        const SemanticsBase& reference() const
        {
          return reference_;
        }

        SemanticsBase& reference()
        {
          return reference_;
        }

        const SemanticsBase& target() const
        {
          return target_;
        }

        SemanticsBase& target()
        {
          return target_;
        }

        bool equals(const TwistSemantics& other) const
        {
          return reference().equals(other.reference()) &&
              target().equals(other.target());
        }

      private:
        SemanticsBase reference_;
        SemanticsBase target_;
    };

    inline std::ostream& operator<<(std::ostream& os, const TwistSemantics& obj)
    {
      os << "reference: " << obj.reference() << "\n";
      os << "target: " << obj.target();
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_TWIST_SEMANTICS_H
