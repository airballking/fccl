#ifndef FCCL_SEMANTICS_TRANSFORM_SEMANTICS_H
#define FCCL_SEMANTICS_TRANSFORM_SEMANTICS_H

#include <fccl/semantics/SemanticsBase.h>

namespace fccl
{
  namespace semantics
  {
    class TransformSemantics
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

        bool equals(const TransformSemantics& other) const
        {
          return reference().equals(other.reference()) &&
              target().equals(other.target());
        }

        TransformSemantics inverse() const
        {
          TransformSemantics result(*this);

          result.invert();

          return result;
        }

        void invert()
        {
          SemanticsBase tmp(reference());
          reference() = target();
          target() = tmp;
        }

      private:
        SemanticsBase reference_;
        SemanticsBase target_;
    };

    inline std::ostream& operator<<(std::ostream& os, const TransformSemantics& obj)
    {
      os << "reference: " << obj.reference() << "\n";
      os << "target: " << obj.target();
      return os;
    }

    inline bool areMultipliable(const TransformSemantics& lhs, const TransformSemantics& rhs)
    {
      return lhs.target().equals(rhs.reference());
    }

    inline TransformSemantics multiply(const TransformSemantics& lhs, const TransformSemantics& rhs)
    {
      assert(areMultipliable(lhs, rhs));

      TransformSemantics result(lhs);
      result.target() = rhs.target();

      return result;
    }

    inline bool operator<(const TransformSemantics& lhs, const TransformSemantics& rhs)
    {
      return (lhs.reference().getID() < rhs.reference().getID()) ||
          ((lhs.reference().getID() == rhs.reference().getID()) &&
           (lhs.target().getID() < rhs.target().getID()));
    }

  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_TRANSFORM_SEMANTICS_H
