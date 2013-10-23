#ifndef FCCL_CONTROL_INTERPOLATOR_H
#define FCCL_CONTROL_INTERPOLATOR_H

#include <ReflexxesAPI.h>
#include <fccl/kdl/JntArray.h>

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
        }

        bool inputValid() const
        {
          assert(input_);

          return input_->CheckForValidity();
        }

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

        const JntArraySemantics& semantics() const
        {
          assert(nextPosition().semantics().equals(nextVelocity().semantics()));
          assert(nextPosition().semantics().equals(nextAcceleration().semantics()));

          return nextPosition().semantics();
        }

        const JntArray& nextPosition() const
        {
          return next_position_;
        }

        const JntArray& nextVelocity() const
        {
          return next_velocity_;
        }

        const JntArray& nextAcceleration() const
        {
          return next_acceleration_;
        }

        void setCurrentPosition(const JntArray& current_position)
        {
          assert(semantics().equals(current_position.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetCurrentPositionVector(current_position.numerics().data.data());
        }

        void setCurrentVelocity(const JntArray& current_velocity)
        {
          assert(semantics().equals(current_velocity.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetCurrentVelocityVector(current_velocity.numerics().data.data());
        }

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

        void setMaxVelocity(const JntArray& max_velocity)
        {
          assert(semantics().equals(max_velocity.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxVelocityVector(max_velocity.numerics().data.data());
        }

        void setMaxAcceleration(const JntArray& max_acceleration)
        {
          assert(semantics().equals(max_acceleration.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxAccelerationVector(max_acceleration.numerics().data.data());
        }

        void setMaxJerk(const JntArray& max_jerk)
        {
          assert(semantics().equals(max_jerk.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetMaxJerkVector(max_jerk.numerics().data.data());
        }

        void setTargetPosition(const JntArray& target_position)
        {
          assert(semantics().equals(target_position.semantics()));
          assert(input_);
          assert(input_->NumberOfDOFs == semantics().size());

          input_->SetTargetPositionVector(target_position.numerics().data.data());
        }

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
