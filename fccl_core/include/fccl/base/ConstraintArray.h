#ifndef FCCL_BASE_CONSTRAINT_ARRAY_H
#define FCCL_BASE_CONSTRAINT_ARRAY_H

#include <fccl/base/Constraints.h>
#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/utils/TransformMap.h>
#include <iostream>
#include <vector>

namespace fccl
{
  namespace base
  {
    class ConstraintArray
    {
      public:
        const std::vector<fccl::base::Constraint>& constraints() const
        {
          return data_;
        }

        std::vector<fccl::base::Constraint>& constraints()
        {
          return data_;
        }

        const fccl::base::Constraint& operator()(std::size_t index) const
        {
          return constraints()[index];
        }

        fccl::base::Constraint& operator()(std::size_t index) 
        {
          return constraints()[index];
        }

        const fccl::kdl::InteractionMatrix& firstDerivative() const
        {
          return first_derivative_;
        }

        const fccl::kdl::JntArray& outputValues() const
        {
          return output_values_;
        }

        const fccl::kdl::JntArray& desiredOutputValues() const
        {
          return desired_output_values_;
        }

        const fccl::kdl::JntArray& taskWeights() const
        {
          return task_weights_;
        }

        bool equals(const ConstraintArray& other) const
        {
          assert(isValid());
          assert(other.isValid());

          if(size() != other.size())
            return false;

          for(std::size_t i=0; i<size(); i++)
            if(!this->operator()(i).equals(other(i)))
              return false;

          return true;
        }

        bool isValid() const
        {
          for(std::size_t i=0; i<size(); i++)
            if(!this->operator()(i).isValid())
              return false;

          return hasCommonReference() && hasCommonToolReference();
        }

        void init(const std::vector<fccl::base::Constraint>& constraints)
        {
          resize(constraints.size());

          copyConstraints(constraints);

          prepare();
        }

        bool areFulfilled() const
        {
          // TODO(Georg): refactor this into Constraint
          assert(taskWeights().size() == size());

          for(std::size_t i=0; i<size(); i++)
            if(!(taskWeights().numerics()(i) < 1.0))
              return false;

          return true;
        }

        // NOT REAL-TIME-SAFE
        std::vector<std::string> names() const
        {
          std::vector<std::string> result;
          for(std::size_t i=0; i<size(); i++)
            result.push_back(operator()(i).semantics().name().getName());

          return result;
        }

        // NOT REAL-TIME-SAFE
        std::set<fccl::semantics::TransformSemantics> necessaryTransforms() const
        {
          assert(isValid());

          std::set<fccl::semantics::TransformSemantics> result;
          for(std::size_t i=0; i<size(); i++)
          {
            std::set<fccl::semantics::TransformSemantics> subset = 
                operator()(i).necessaryTransforms();
            result.insert(subset.begin(), subset.end());
          }

          return result;
        }

        void prepare()
        {
          initFirstDerivative();

          initOutputValuesAndWeights();
        }

        std::size_t size() const
        {
          return constraints().size();
        }

        void resize(std::size_t new_size)
        {
          constraints().resize(new_size);
          first_derivative_.resize(new_size);
          output_values_.resize(new_size);
          desired_output_values_.resize(new_size);
          task_weights_.resize(new_size);
        }

        void update(fccl::utils::TransformMap& transform_map, double delta)
        {
          // TODO(Georg): refactor this into Constraint
          calculateOutputValues(transform_map, delta);
          calculateDerivative(transform_map, delta);
          calculateWeightsAndDesiredOutputs();
        }

      private:
        // vector of constraints
        std::vector<fccl::base::Constraint> data_;

        // pre-allocated memory for return variables
        fccl::kdl::InteractionMatrix first_derivative_;
        fccl::kdl::JntArray output_values_;
        fccl::kdl::JntArray desired_output_values_;
        fccl::kdl::JntArray task_weights_;

        void calculateOutputValues(fccl::utils::TransformMap& transform_map,
            double delta)
        {
          assert(size() == output_values_.size());

          for(std::size_t i=0; i<size(); i++)
          {
            assert(output_values_.semantics()(i).equals(operator()(i).semantics().name()));

            fccl::kdl::Transform tool_transform = transform_map.getTransform(
                operator()(i).semantics().reference(),
                operator()(i).toolFeature().semantics().reference());

            fccl::kdl::Transform object_transform = transform_map.getTransform(
                operator()(i).semantics().reference(),
                operator()(i).objectFeature().semantics().reference());

            output_values_.numerics()(i) = 
                operator()(i).calculateValue(tool_transform, object_transform);
          }
        }

        void calculateDerivative(fccl::utils::TransformMap& transform_map, 
            double delta)
        {
          assert(size() == firstDerivative().size());

          for(std::size_t i=0; i<size(); i++)
          {
            fccl::kdl::Transform tool_transform = transform_map.getTransform(
                operator()(i).semantics().reference(),
                operator()(i).toolFeature().semantics().reference());

            fccl::kdl::Transform object_transform = transform_map.getTransform(
                operator()(i).semantics().reference(),
                operator()(i).objectFeature().semantics().reference());

            first_derivative_.partialAssignment(i, 1,
                operator()(i).calculateFirstDerivative(
                    tool_transform, object_transform, delta));
          }
        }

        void calculateWeightsAndDesiredOutputs()
        {
          assert(size() == desiredOutputValues().size());
          assert(size() == taskWeights().size());

          // TODO(Georg): refactor this param into every constraint
          double s = 0.05;

          // TODO(Georg): refactor this method into Constraint 
          for(std::size_t i=0; i<size(); i++)
          {
            double value = output_values_.numerics()(i);
        
            double lo = operator()(i).lowerBoundary();
            double hi = operator()(i).upperBoundary();
        
            // adjust margin if range is too small
            double ss = (hi - lo < 2*s) ? (hi - lo) / 2 : s;
        
            // desired output values..
            if(value > hi - ss)
            {
              desired_output_values_.numerics()(i) = hi - ss;
            }
            else if(value < lo + ss)
            {
              desired_output_values_.numerics()(i) = lo + ss;
            }
            else
            {
              desired_output_values_.numerics()(i) = value;
            }
        
            // weights..
            if(value > hi || value < lo)
            {
              task_weights_.numerics()(i) = 1.0;
            }
            else
            {
              double w_lo = (1/s)*(-hi + value)+1;
              double w_hi = (1/s)*( lo - value)+1;
        
              w_lo = (w_lo > 0.0) ? w_lo : 0.0;
              w_hi = (w_hi > 0.0) ? w_hi : 0.0;
        
              task_weights_.numerics()(i) = (w_lo > w_hi) ? w_lo : w_hi;
            }
          }
        }
 
        void copyConstraints(const std::vector<fccl::base::Constraint>& constraints)
        {
          assert(size() == constraints.size());

          for(std::size_t i=0; i<size(); i++)
            operator()(i) = constraints[i];
        }

        bool hasCommonReference() const
        {
          if(size() > 0)
          {
            fccl::semantics::SemanticsBase common_reference = 
                this->operator()(0).semantics().reference();
            for(std::size_t i=0; i<size(); i++)
              if(!this->operator()(i).semantics().reference().equals(common_reference))
                return false;
          }

          return true; 
        }

        bool hasCommonToolReference() const
        {
          if(size() > 0)
          {
            fccl::semantics::SemanticsBase common_reference = 
                this->operator()(0).toolFeature().semantics().reference();
            for(std::size_t i=0; i<size(); i++)
              if(!this->operator()(i).toolFeature().semantics().reference().equals(
                    common_reference))
                return false;
          }

          return true; 
        }

        void initFirstDerivative()
        {
          assert(size() == firstDerivative().size());
          assert(hasCommonToolReference());

          for(std::size_t i=0; i<size(); i++)
            first_derivative_.semantics().joints()(i) = 
                operator()(i).semantics().name();

          first_derivative_.semantics().twist().reference() =
              operator()(0).toolFeature().semantics().reference();

          first_derivative_.semantics().twist().target() =
              operator()(0).toolFeature().semantics().reference();
        }

        void initOutputValuesAndWeights()
        {
          assert(size() == output_values_.size());
          assert(size() == desiredOutputValues().size());
          assert(size() == taskWeights().size());

          for(std::size_t i=0; i<size(); i++)
          {
            output_values_.semantics()(i) = operator()(i).semantics().name();
            desired_output_values_.semantics()(i) = operator()(i).semantics().name();
            task_weights_.semantics()(i) = operator()(i).semantics().name();
          }
        }
    }; 

    inline std::ostream& operator<<(std::ostream& os, const ConstraintArray& constraints)
    {
      os << "constraints:\n";

      for(std::size_t i=0; i<constraints.size(); i++)
      {
        os << constraints(i);

        if(i<(constraints.size()-1))
          os << "\n";
      }

      return os;
    }
  } // namespace kdl
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINT_ARRAY_H
