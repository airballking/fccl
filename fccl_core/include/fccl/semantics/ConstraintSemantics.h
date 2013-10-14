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
    enum ConstraintTypes
    {
      UNKNOWN_CONSTRAINT = 0,
      
      ABOVE_CONSTRAINT = 1,

      CONSTRAINT_COUNT
    };

    inline bool constraintTypeValid(int constraint_type)
    {
      return (UNKNOWN_CONSTRAINT < constraint_type) && (constraint_type < CONSTRAINT_COUNT);
    }
 
    class ConstraintSemantics
    {
      public:
        ConstraintSemantics() : 
            reference_( fccl::semantics::SemanticsBase() ),
            name_( fccl::semantics::SemanticsBase() ),
            type_( UNKNOWN_CONSTRAINT )
        {
          first_derivative_.resize(1);
        }

        ConstraintSemantics(const ConstraintSemantics& other) :
            reference_( other.reference() ), name_( other.name() ), 
            type_( other.type() )
        {
          first_derivative_.resize(1);
        }

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

        const ConstraintTypes& type() const
        {
          return type_;
        }

        ConstraintTypes& type() 
        {
          return type_;
        }

        bool equals(const ConstraintSemantics& other) const
        {
          return (type() == other.type()) &&
              reference().equals(other.reference()) &&
              name().equals(other.name());
        }

        bool isValid() const
        {
          return constraintTypeValid(type());
        }

        const fccl::semantics::InteractionMatrixSemantics& calculateFirstDerivative(
            const fccl::semantics::TransformSemantics& tool_transform)
        {
          assert(first_derivative_.size() == 1);

          first_derivative_.twist().reference() = tool_transform.target();
          first_derivative_.twist().target() = tool_transform.target();
          first_derivative_.joints()(0) = name(); 
          
          return first_derivative_;
        }
            
      private:
        SemanticsBase reference_;
        SemanticsBase name_;
        ConstraintTypes type_; 
        // pre-allocated memory to return first derivative
        InteractionMatrixSemantics first_derivative_;
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
