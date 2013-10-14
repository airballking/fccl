#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl/base/Features.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/semantics/ConstraintSemantics.h>
#include <string>
#include <iostream>

namespace fccl
{
  namespace base
  {
    class Constraint
    {
      public:
        Constraint() :
            semantics_( fccl::semantics::ConstraintSemantics() ),
            tool_feature_( Feature() ), object_feature_( Feature() ),
            lower_boundary_( 0.0 ), upper_boundary_( 0.0 )
        {
          semantics().type() = fccl::semantics::UNKNOWN_CONSTRAINT;
          first_derivative_.resize(1);
        }

        Constraint(const Constraint& other) :
            semantics_( other.semantics() ),
            tool_feature_( other.toolFeature() ), 
            object_feature_( other.objectFeature() ),
            lower_boundary_( other.lowerBoundary() ),
            upper_boundary_( other.upperBoundary() )
        {
          semantics().type() = fccl::semantics::UNKNOWN_CONSTRAINT;
          first_derivative_.resize(1);
        }

        Constraint& operator=(const Constraint& rhs)
        {
          // protect against self-assignment
          if(this != &rhs)
          {
            assert(semantics().type() == rhs.semantics().type());

            semantics() = rhs.semantics();
            toolFeature() = rhs.toolFeature();
            objectFeature() = rhs.objectFeature();
            lowerBoundary() = rhs.lowerBoundary();
            upperBoundary() = rhs.upperBoundary();
          }
      
          return *this; 
        }
 
        const Feature& toolFeature() const
        {
          return tool_feature_;
        }

        Feature& toolFeature() 
        {
          return tool_feature_;
        }

        const Feature& objectFeature() const
        {
          return object_feature_;
        }

        Feature& objectFeature() 
        {
          return object_feature_;
        }

        double lowerBoundary() const
        {
          return lower_boundary_;
        }

        double& lowerBoundary()
        {
          return lower_boundary_;
        }

        double upperBoundary() const
        {
          return upper_boundary_;
        }

        double& upperBoundary()
        {
          return upper_boundary_;
        }

        const fccl::semantics::ConstraintSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::ConstraintSemantics& semantics()
        {
          return semantics_;
        }

        bool equals(const Constraint& other) const
        {
          return semantics().equals(other.semantics()) &&
            (lowerBoundary() == other.lowerBoundary()) &&
            (upperBoundary() == other.upperBoundary()) &&
            toolFeature().equals(other.toolFeature()) &&
            objectFeature().equals(other.objectFeature());
        }

        bool isValid() const
        {
          return semantics().isValid() &&
            toolFeature().isValid() && objectFeature().isValid();
        }
 
        virtual double calculateValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) const;
  
        const fccl::kdl::InteractionMatrix& calculateFirstDerivative(
            const fccl::kdl::Transform& tool_transform, 
            const fccl::kdl::Transform& object_transform, double delta=0.001); 
  
      protected:
        // semantics
        fccl::semantics::ConstraintSemantics semantics_;

        // feature on the controllable tool
        Feature tool_feature_;
     
        // feature on the non-controllable object
        Feature object_feature_;
  
        // lower boundary for output of feature function
        double lower_boundary_;
  
        // upper boundary for output of feature function
        double upper_boundary_;
  
        // memory to store first derivative of constraint
        fccl::kdl::InteractionMatrix first_derivative_;
    };

    inline std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
    {
      os << "semantics: " << constraint.semantics() << "\n";
      os << "tool feature: " << constraint.toolFeature() << "\n";
      os << "object feature: " << constraint.objectFeature() << "\n";
      os << "lower boundary: " << constraint.lowerBoundary() << "\n";
      os << "upper boundary: " << constraint.upperBoundary();

      return os;
    }
 
    class AboveConstraint : public Constraint
    {
      public:
        AboveConstraint() : Constraint()
        {
          semantics().type() = fccl::semantics::ABOVE_CONSTRAINT;
        }

        AboveConstraint(const AboveConstraint& other) : Constraint(other)
        {
          semantics().type() = fccl::semantics::ABOVE_CONSTRAINT;
        }

        AboveConstraint& operator=(const AboveConstraint& rhs)
        {
          Constraint* lhs_p = dynamic_cast<Constraint*>(this);
          const Constraint* rhs_p = dynamic_cast<const Constraint*>(&rhs);

          *lhs_p = *rhs_p;

          return *this;
        }
 
        virtual double calculateValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) const;
    };
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
