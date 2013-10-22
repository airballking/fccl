#ifndef FCCL_SEMANTICS_CONSTRAINT_SEMANTICS_H
#define FCCL_SEMANTICS_CONSTRAINT_SEMANTICS_H

#include <fccl/semantics/SemanticsBase.h>
#include <fccl/semantics/TransformSemantics.h>
#include <fccl/semantics/InteractionMatrixSemantics.h>
#include <iostream>

namespace fccl
{
  namespace semantics
  {
    class ConstraintSemantics
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

        const SemanticsBase& name() const
        {
          return name_;
        }

        SemanticsBase& name()
        {
          return name_;
        }

        const SemanticsBase& type() const
        {
          return type_;
        }

        SemanticsBase& type() 
        {
          return type_;
        }

        bool equals(const ConstraintSemantics& other) const
        {
          return type().equals(other.type()) && name().equals(other.name()) &&
              reference().equals(other.reference());
        }

      private:
        SemanticsBase reference_;
        SemanticsBase name_;
        SemanticsBase type_; 
    };

    inline std::ostream& operator<<(std::ostream& os, const ConstraintSemantics& obj)
    {
      os << "reference: " << obj.reference() << "\n";
      os << "name: " << obj.name() << "\n";
      os << "type: " << obj.type();
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_CONSTRAINT_SEMANTICS_H
