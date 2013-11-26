#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl/base/Features.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/semantics/ConstraintSemantics.h>
#include <fccl/utils/TransformMap.h>
#include <string>
#include <iostream>
#include <vector>
#include <set>

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
            lower_boundary_( 0.0 ), upper_boundary_( 0.0 ),
            output_value_( 0.0 ), desired_output_value_( 0.0 ),
            desired_output_velocity_( 0.0 ), task_weight_( 0.0 )
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

        double outputValue() const
        {
          return output_value_;
        }

        double desiredOutputValue() const
        {
          return desired_output_value_;
        }

        double desiredOutputVelocity() const
        {
          return desired_output_velocity_;
        }

        double taskWeight() const
        {
          return task_weight_;
        }

        const fccl::kdl::InteractionMatrix& firstDerivative() const
        {
          return first_derivative_;
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

        bool isFulfilled() const
        {
          return taskWeight() < 1.0;
        }

        bool isValid() const
        {
          return functionValid() && toolFeature().isValid() && 
              objectFeature().isValid();
        }
 
        void update(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform, double delta=0.001)
        {
          updateOutputValue(tool_transform, object_transform);
          updateFirstDerivative(tool_transform, object_transform, delta);
          updateWeightAndDesiredOutput();
        }
 
        void update(const fccl::utils::TransformMap& transform_map, double delta=0.001) 
        {
          fccl::kdl::Transform tool_transform = transform_map.getTransform(
              semantics().reference(), toolFeature().semantics().reference());

          fccl::kdl::Transform object_transform = transform_map.getTransform(
              semantics().reference(), objectFeature().semantics().reference());
 
          update(tool_transform, object_transform, delta);
        }
  
        // NOT REAL-TIME-SAFE
        std::set<fccl::semantics::TransformSemantics> necessaryTransforms() const;
  
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

        // member variables to cache return values
        double output_value_;
        double desired_output_value_;
        double desired_output_velocity_;
        double task_weight_;

        // auxiliary function to fill function map with correct correspondences
        static std::map<fccl::semantics::SemanticsBase, ConstraintFunction> 
            createFunctionMap();

        // auxiliary function for numeric derivation
        void calculateInteractionSemantics(
            const fccl::semantics::TransformSemantics& tool_transform);

        double calculateOutputValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) 
        {
          assert(functionValid());

          using namespace fccl::semantics;

          std::map<SemanticsBase, ConstraintFunction>::const_iterator it =
              function_map_.find(semantics().type());

          return it->second(semantics().reference(), toolFeature(),
              objectFeature(), tool_transform, object_transform);
        }
 
        void updateOutputValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform)
        {
          output_value_ = calculateOutputValue(tool_transform, object_transform);
        }

        const fccl::kdl::InteractionMatrix& calculateFirstDerivative(
            const fccl::kdl::Transform& tool_transform, 
            const fccl::kdl::Transform& object_transform, double delta=0.001); 

        void updateFirstDerivative(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform, double delta)
        {
          first_derivative_ = calculateFirstDerivative(tool_transform,
              object_transform, delta);
        }

        void updateWeightAndDesiredOutput()
        {
          // TODO(Georg): refactor this param into every constraint
          double s = 0.05;

          double lo = lowerBoundary();
          double hi = upperBoundary();
        
          // adjust margin if range is too small
          double ss = (hi - lo < 2*s) ? (hi - lo) / 2 : s;
        
          // desired output values..
          if(outputValue() > hi - ss)
          {
            desired_output_value_ = hi - ss;
          }
          else if(outputValue() < lo + ss)
          {
            desired_output_value_ = lo + ss;
          }
          else
          {
            desired_output_value_ = outputValue();
          }
        
          // weights..
          if(outputValue() > hi || outputValue() < lo)
          {
            task_weight_ = 1.0;
          }
          else
          {
            double w_lo = (1/s)*(-hi + outputValue())+1;
            double w_hi = (1/s)*( lo - outputValue())+1;
        
            w_lo = (w_lo > 0.0) ? w_lo : 0.0;
            w_hi = (w_hi > 0.0) ? w_hi : 0.0;
        
            task_weight_ = (w_lo > w_hi) ? w_lo : w_hi;
          }
        }
    };

    inline std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
    {
      os << "semantics: " << constraint.semantics() << "\n";
      os << "tool feature: " << constraint.toolFeature() << "\n";
      os << "object feature: " << constraint.objectFeature() << "\n";
      os << "lower boundary: " << constraint.lowerBoundary() << "\n";
      os << "upper boundary: " << constraint.upperBoundary() << "\n";

      os << "output value: " << constraint.outputValue() << "\n";
      os << "desired output value: " << constraint.desiredOutputValue() << "\n",
      os << "desired output velocity: " << constraint.desiredOutputVelocity() << "\n";
      os << "task weight: " << constraint.taskWeight() << "\n";
      os << "first derivative:\n" << constraint.firstDerivative();

      return os;
    }

    double above(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
