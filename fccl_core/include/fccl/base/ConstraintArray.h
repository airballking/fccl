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

        const fccl::kdl::JntArray& maxVelocities() const
        {
          return max_velocities_;
        }

        const fccl::kdl::JntArray maxVelocities()
        {
          return max_velocities_;
        }

        const fccl::kdl::JntArray& maxAccelerations() const
        {
          return max_accelerations_;
        }

        const fccl::kdl::JntArray maxAccelerations()
        {
          return max_accelerations_;
        }

        const fccl::kdl::JntArray& maxJerks() const
        {
          return max_jerks_;
        }

        const fccl::kdl::JntArray maxJerks()
        {
          return max_jerks_;
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
          assert(taskWeights().size() == size());

          for(std::size_t i=0; i<size(); i++)
            if(!operator()(i).isFulfilled())
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

          copyLimits();
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
          max_velocities_.resize(new_size);
          max_accelerations_.resize(new_size);
          max_jerks_.resize(new_size);
        }

        void update(const fccl::utils::TransformMap& transform_map, double delta=0.001)
        {
          assert(size() == output_values_.size());
          assert(size() == desired_output_values_.size());
          assert(size() == task_weights_.size());
          assert(size() == first_derivative_.size());

          for(std::size_t i=0; i<size(); i++)
          {
            assert(output_values_.semantics()(i).equals(
                operator()(i).semantics().name()));
            assert(desired_output_values_.semantics()(i).equals(
                operator()(i).semantics().name()));
            assert(task_weights_.semantics()(i).equals(
                operator()(i).semantics().name()));
            assert(first_derivative_.semantics().joints()(i).equals(
                operator()(i).semantics().name()));
           
            // TODO(Georg): relax these two assumptions!
            assert(first_derivative_.semantics().twist().reference().equals(
                 operator()(i).toolFeature().semantics().reference()));
            assert(first_derivative_.semantics().twist().target().equals(
                 operator()(i).toolFeature().semantics().reference()));
 
            operator()(i).update(transform_map, delta);

            // copy all the results from this particular constraint
            output_values_.numerics()(i) = operator()(i).outputValue();
            desired_output_values_.numerics()(i) =
                operator()(i).desiredOutputValue();
            task_weights_.numerics()(i) = operator()(i).taskWeight();

            first_derivative_.partialAssignment(i, 1,
                operator()(i).firstDerivative());
          }
        }

      private:
        // vector of constraints
        std::vector<fccl::base::Constraint> data_;

        // pre-allocated memory for return variables
        fccl::kdl::InteractionMatrix first_derivative_;
        fccl::kdl::JntArray output_values_;
        fccl::kdl::JntArray desired_output_values_;
        fccl::kdl::JntArray task_weights_;
        fccl::kdl::JntArray max_velocities_;
        fccl::kdl::JntArray max_accelerations_;
        fccl::kdl::JntArray max_jerks_;

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

        void copyLimits()
        {
          assert(size() == max_velocities_.size());
          assert(size() == max_accelerations_.size());
          assert(size() == max_jerks_.size());

          for(std::size_t i=0; i<size(); i++)
          {
            max_velocities_.semantics()(i) = operator()(i).semantics().name();
            max_accelerations_.semantics()(i) = operator()(i).semantics().name();
            max_jerks_.semantics()(i) = operator()(i).semantics().name();

            max_velocities_.numerics()(i) = operator()(i).maxVelocity();
            max_accelerations_.numerics()(i) = operator()(i).maxAcceleration();
            max_jerks_.numerics()(i) = operator()(i).maxJerk();
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
