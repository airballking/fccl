#ifndef FCCL_CONTROL_INTERPOLATOR_H
#define FCCL_CONTROL_INTERPOLATOR_H

#include <ReflexxesAPI.h>
#include <fccl/base/Array.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/kdl/JointArray.h>
#include <fccl/semantics/SemanticsBase.h>

using namespace fccl::base;
using namespace fccl::semantics;
using namespace fccl::kdl;

namespace fccl
{
  namespace control
  {
    class Interpolator
    {
      public:
        Interpolator() :
            generator_(NULL), input_(NULL), output_(NULL), 
            flags_(RMLPositionFlags()), result_value_(0),
            next_position_(JntArray()), next_velocity_(JntArray()),
            next_acceleration_(JntArray())
        {
        }

        Interpolator(const Interpolator& other) :
            generator_(other.generator_), input_(other.input_), 
            output_(other.output_), flags_(other.flags_), 
            result_value_(other.result_value_),
            next_position_(other.next_position_),
            next_velocity_(other.next_velocity_),
            next_acceleration_(other.next_acceleration_)
        {
        }

        ~Interpolator()
        {
          deletePointers();
        }

        // TODO(Georg): deprecate this
        void init(JntArraySemantics semantics, double delta_t)
        {
          deletePointers();

          generator_ = new ReflexxesAPI(semantics.size(), delta_t);
          input_ = new RMLPositionInputParameters(semantics.size());
          output_ = new RMLPositionOutputParameters(semantics.size());
          result_value_ = 0;

          next_position_.init(semantics);
          next_velocity_.init(semantics);
          next_acceleration_.init(semantics);

          next_state_.resize(semantics.size());
          for(std::size_t i=0; i<next_state_.size(); i++)
            next_state_(i).semantics() = semantics(i);
        }

        // TODO(Georg): make this a template of the class
        void init(const Array<SemanticsBase, SemanticsBase>& state, double delta_t)
        {
          deletePointers();

          generator_ = new ReflexxesAPI(state.size(), delta_t);
          input_ = new RMLPositionInputParameters(state.size());
          output_ = new RMLPositionOutputParameters(state.size());
          result_value_ = 0;

          next_state_.init(state);
        }

        bool inputValid() const
        {
          assert(input_);

          return input_->CheckForValidity();
        }

        // TODO(Georg): renamed once other interpolate-methods are gone
        const AccelerationJointArray& performInterpolation()
        {
          assert(input_);
          assert(inputValid());
          assert(output_);
          assert(generator_);
          assert(output_->NumberOfDOFs == next_state_.size());

          result_value_ = generator_->RMLPosition(*input_, output_, flags_);

          if(result_value_ < 0)
          {
            std::cout << "Reflexxes interpolator returned with error: " << 
                result_value_ << "\n";
            // TODO(Georg): set result to Zero
            return next_state_;
          }

          for(std::size_t i=0; i<next_state_.size(); i++)
          {
            output_->GetNewPositionVectorElement(&(next_state_(i).position()), i);
            output_->GetNewVelocityVectorElement(&(next_state_(i).velocity()), i);
            output_->GetNewAccelerationVectorElement(
                &(next_state_(i).acceleration()), i);
          }

          return next_state_;
        }

        // TODO(Georg): deprecate this
        void interpolate()
        {
          assert(input_);
          assert(inputValid());
          assert(output_);
          assert(generator_);
          assert(output_->NumberOfDOFs == nextPosition().size());
          assert(output_->NumberOfDOFs == nextVelocity().size());
          assert(output_->NumberOfDOFs == nextAcceleration().size());

          result_value_ = generator_->RMLPosition(*input_, output_, flags_);

          if(result_value_ < 0)
          {
            std::cout << "Reflexxes interpolator returned with error: " << 
                result_value_ << "\n";
            return;
          }

          output_->GetNewPositionVector(next_position_.numerics().data.data(), 
              nextPosition().size()*sizeof(double));
          output_->GetNewVelocityVector(next_velocity_.numerics().data.data(), 
              nextVelocity().size()*sizeof(double));
          output_->GetNewAccelerationVector(
              next_acceleration_.numerics().data.data(), 
              nextAcceleration().size()*sizeof(double));
        }

        // TODO(Georg): deprecate this
        void interpolate(const JntArray& target_pos, const JntArray& target_vel,
            const JntArray& current_pos, const JntArray& current_vel,
            const JntArray& current_acc, const JntArray& max_vel, 
            const JntArray& max_acc, const JntArray max_jerk)
        {
          setTargetPosition(target_pos);
          setTargetVelocity(target_vel);
          setCurrentPosition(current_pos);
          setCurrentVelocity(current_vel);
          setCurrentAcceleration(current_acc);
          setMaxVelocity(max_vel);
          setMaxAcceleration(max_acc);
          setMaxJerk(max_jerk);
          
          interpolate();
        }

        bool interpolationFinished() const
        {
          return result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED;
        }

        // TODO(Georg): deprecate this
        const JntArraySemantics& semantics() const
        {
          assert(nextPosition().semantics().equals(nextVelocity().semantics()));
          assert(nextPosition().semantics().equals(nextAcceleration().semantics()));

          return nextPosition().semantics();
        }

        // TODO(Georg): deprecate this
        const JntArray& nextPosition() const
        {
          return next_position_;
        }

	// TODO(Georg): deprecate this
        const JntArray& nextVelocity() const
        {
          return next_velocity_;
        }
	// TODO(Georg): deprecate this
        const JntArray& nextAcceleration() const
        {
          return next_acceleration_;
        }

        void setCurrentState(const AccelerationJointArray& current)
        {
          assert(current.size() == semantics().size());
          for(std::size_t i=0; i<current.size(); i++)
            current(i).semantics().equals(semantics()(i));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          for(std::size_t i=0; i<current.size(); i++)
          {
            input_->SetCurrentPositionVectorElement(current(i).position(), i);
            input_->SetCurrentVelocityVectorElement(current(i).velocity(), i);
            input_->SetCurrentAccelerationVectorElement(current(i).acceleration(), i);
          }
        }
	// TODO(Georg): deprecate this
        void setCurrentPosition(const JntArray& current_position)
        {
          assert(semantics().equals(current_position.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetCurrentPositionVector(current_position.numerics().data.data());
        }
	// TODO(Georg): deprecate this
        void setCurrentVelocity(const JntArray& current_velocity)
        {
          assert(semantics().equals(current_velocity.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetCurrentVelocityVector(current_velocity.numerics().data.data());
        }
	// TODO(Georg): deprecate this
        void setCurrentAcceleration(const JntArray& current_acceleration)
        {
          assert(semantics().equals(current_acceleration.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetCurrentAccelerationVector(current_acceleration.numerics().data.data());
        }

        void setDimensionActivity(std::size_t dim, bool active)
        {
          assert(dim < semantics().size());
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetSelectionVectorElement(active, dim);
        }

        void setMaxMotionState(const JerkJointArray& max_specs)
        {
          assert(max_specs.size() == semantics().size());
          for(std::size_t i=0; i<max_specs.size(); i++)
            max_specs(i).semantics().equals(semantics()(i));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          for(std::size_t i=0; i<max_specs.size(); i++)
          {
            input_->SetMaxVelocityVectorElement(max_specs(i).velocity(), i);
            input_->SetMaxAccelerationVectorElement(max_specs(i).acceleration(), i);
            input_->SetMaxJerkVectorElement(max_specs(i).jerk(), i);
          }
        }
	// TODO(Georg): deprecate this
        void setMaxVelocity(const JntArray& max_velocity)
        {
          assert(semantics().equals(max_velocity.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxVelocityVector(max_velocity.numerics().data.data());
        }
	// TODO(Georg): deprecate this
        void setMaxAcceleration(const JntArray& max_acceleration)
        {
          assert(semantics().equals(max_acceleration.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxAccelerationVector(max_acceleration.numerics().data.data());
        }
	// TODO(Georg): deprecate this
        void setMaxJerk(const JntArray& max_jerk)
        {
          assert(semantics().equals(max_jerk.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxJerkVector(max_jerk.numerics().data.data());
        }

        void setTargetState(const VelocityJointArray& target)
        {
          assert(target.size() == semantics().size());
          for(std::size_t i=0; i<target.size(); i++)
            target(i).semantics().equals(semantics()(i));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          for(std::size_t i=0; i<target.size(); i++)
          {
            input_->SetTargetPositionVectorElement(target(i).position(), i);
            input_->SetTargetVelocityVectorElement(target(i).velocity(), i);
          }
        }
	// TODO(Georg): deprecate this
        void setTargetPosition(const JntArray& target_position)
        {
          assert(semantics().equals(target_position.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetTargetPositionVector(target_position.numerics().data.data());
        }
	// TODO(Georg): deprecate this
        void setTargetVelocity(const JntArray& target_velocity)
        {
          assert(semantics().equals(target_velocity.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetTargetVelocityVector(target_velocity.numerics().data.data());
        }
  
      private:
        // reflexxes data structures needed for interpolation
        ReflexxesAPI* generator_;
        RMLPositionInputParameters* input_;
        RMLPositionOutputParameters* output_;
        RMLPositionFlags flags_;
        int result_value_;
        // pre-allocated memory for output
        JntArray next_position_;
        JntArray next_velocity_;
        JntArray next_acceleration_;
        AccelerationJointArray next_state_;

        void deletePointers()
        {
          if(generator_)
            delete generator_;

          if(input_)
            delete input_;

          if(output_)
            delete output_;
        }
    };
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_INTERPOLATOR_H
