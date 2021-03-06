#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl/base/ConstraintFunctions.h>
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
    class Constraint
    {
      public:
        Constraint() :
            semantics_( fccl::semantics::ConstraintSemantics() ),
            tool_feature_( Feature() ), object_feature_( Feature() ),
            lower_boundary_( 0.0 ), upper_boundary_( 0.0 ),
            max_velocity_( 0.0 ), max_acceleration_( 0.0 ), max_jerk_( 0.0 ),
            output_value_( 0.0 ), desired_output_value_( 0.0 ), task_weight_( 0.0 )
        {
          first_derivative_.resize(1);
        }

        Constraint(const Constraint& other) :
            semantics_( other.semantics() ),
            tool_feature_( other.toolFeature() ),
            object_feature_( other.objectFeature() ),
            lower_boundary_( other.lowerBoundary() ),
            upper_boundary_( other.upperBoundary() ),
            max_velocity_( other.maxVelocity() ),
            max_acceleration_( other.maxAcceleration() ),
            max_jerk_( other.maxJerk() )
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
            maxVelocity() = rhs.maxVelocity();
            maxAcceleration() = rhs.maxAcceleration();
            maxJerk() = rhs.maxJerk();
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

        double maxVelocity() const
        {
          return max_velocity_;
        }

        double& maxVelocity()
        {
          return max_velocity_;
        }

        double maxAcceleration() const
        {
          return max_acceleration_;
        }

        double& maxAcceleration()
        {
          return max_acceleration_;
        }

        double maxJerk() const
        {
          return max_jerk_;
        }

        double& maxJerk()
        {
          return max_jerk_;
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
            (maxVelocity() == other.maxVelocity()) &&
            (maxAcceleration() == other.maxAcceleration()) &&
            (maxJerk() == other.maxJerk()) &&
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
  
        // velocity, acceleration, and jerk limits
        double max_velocity_, max_acceleration_, max_jerk_;

        // memory to store first derivative of constraint
        fccl::kdl::InteractionMatrix first_derivative_;

        // map used to lookup constraint functions dynamically
        // TODO(Georg): make a typedef for the map
        static const std::map<fccl::semantics::SemanticsBase, ConstraintFunction>
            function_map_;

        // member variables to cache return values
        double output_value_;
        double desired_output_value_;
        double task_weight_;

        // auxiliary function to fill function map with correct correspondences
        static std::map<fccl::semantics::SemanticsBase, ConstraintFunction> 
            createFunctionMap();


        // auxiliary function to put a new function into the function-map
        static void registerConstraintFunction(const std::string& function_name,
            const ConstraintFunction& function,
            std::map<fccl::semantics::SemanticsBase, ConstraintFunction>& map);

        // auxiliary function for numeric derivation
        void calculateInteractionSemantics(
            const fccl::semantics::TransformSemantics& tool_transform);

        // TODO(Georg): move this into a cpp-file
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

        // TODO(Georg): move this into a cpp-file
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

    // TODO(Georg): move this into a cpp-file
    inline std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
    {
      os << "semantics: " << constraint.semantics() << "\n";
      os << "tool feature: " << constraint.toolFeature() << "\n";
      os << "object feature: " << constraint.objectFeature() << "\n";
      os << "lower boundary: " << constraint.lowerBoundary() << "\n";
      os << "upper boundary: " << constraint.upperBoundary() << "\n";

      os << "max velocity: " << constraint.maxVelocity() << "\n";
      os << "max acceleration: " << constraint.maxAcceleration() << "\n";
      os << "max jerk: " << constraint.maxJerk() << "\n";

      os << "output value: " << constraint.outputValue() << "\n";
      os << "desired output value: " << constraint.desiredOutputValue() << "\n",
      os << "task weight: " << constraint.taskWeight() << "\n";
      os << "first derivative:\n" << constraint.firstDerivative();

      return os;
    }
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
