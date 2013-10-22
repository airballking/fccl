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
    typedef double (*ConstraintFunction) 
        (const fccl::semantics::SemanticsBase& view_frame,
         const Feature& tool_feature, const Feature& object_feature,
         const fccl::kdl::Transform& tool_transform,
         const fccl::kdl::Transform& object_transform);

    class Constraint
    {
      public:
        Constraint() :
            semantics_( fccl::semantics::ConstraintSemantics() ),
            tool_feature_( Feature() ), object_feature_( Feature() ),
            lower_boundary_( 0.0 ), upper_boundary_( 0.0 )
        {
          first_derivative_.resize(1);
        }

        Constraint(const Constraint& other) :
            semantics_( other.semantics() ),
            tool_feature_( other.toolFeature() ),
            object_feature_( other.objectFeature() ),
            lower_boundary_( other.lowerBoundary() ),
            upper_boundary_( other.upperBoundary() )
        {
          first_derivative_.resize(1);
        }

        Constraint& operator=(const Constraint& rhs)
        {
          // protect against self-assignment
          if(this != &rhs)
          {
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

        bool functionValid() const
        {
          using namespace fccl::semantics;

          std::map<SemanticsBase, ConstraintFunction>::const_iterator it =
              function_map_.find(semantics().type());

          return it != function_map_.end();
        }

        bool isValid() const
        {
          return functionValid() && toolFeature().isValid() && 
              objectFeature().isValid();
        }
 
        double calculateValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) const
        {
          assert(functionValid());

          using namespace fccl::semantics;

          std::map<SemanticsBase, ConstraintFunction>::const_iterator it =
              function_map_.find(semantics().type());

          return it->second(semantics().reference(), toolFeature(), objectFeature(),
              tool_transform, object_transform);
        }
  
        const fccl::kdl::InteractionMatrix& calculateFirstDerivative(
            const fccl::kdl::Transform& tool_transform, 
            const fccl::kdl::Transform& object_transform, double delta=0.001); 
  
      private:
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

        // map used to lookup constraint functions dynamically
        static const std::map<fccl::semantics::SemanticsBase, ConstraintFunction>
            function_map_;

        // auxiliary function to fill function map with correct correspondences
        static std::map<fccl::semantics::SemanticsBase, ConstraintFunction> 
            createFunctionMap();

        // auxiliary function for numeric derivation
        void calculateInteractionSemantics(
            const fccl::semantics::TransformSemantics& tool_transform);
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

    double above(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
