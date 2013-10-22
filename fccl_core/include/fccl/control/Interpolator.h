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

          next_position_.init(semantics);
          next_velocity_.init(semantics);
        }

        void interpolate()
        {
          assert(input_);
          assert(output_);
          assert(generator_);
          assert(output_->NumberOfDOFs == nextPosition().size());
          assert(output_->NumberOfDOFs == nextVelocity().size());

          int return_value = generator_->RMLPosition(*input_, output_, flags_);

          if(return_value < 0)
          {
            std::cout << "Reflexxes interpolator returned with error: " << 
                return_value << "\n";
            return;
          }

          output_->GetNewPositionVector(next_position_.numerics().data.data(), 
              nextPosition().size()*sizeof(double));
          output_->GetNewVelocityVector(next_velocity_.numerics().data.data(), 
              nextVelocity().size()*sizeof(double));
        }

        void interpolate(const JntArray& target_pos, const JntArray& target_vel,
            const JntArray& current_pos, const JntArray& current_vel,
            const JntArray& max_vel, const JntArray& max_acc, 
            const JntArray max_jerk)
        {
          setTargetPosition(target_pos);
          setTargetVelocity(target_vel);
          setCurrentPosition(current_pos);
          setCurrentVelocity(current_vel);
          setMaxVelocity(max_vel);
          setMaxAcceleration(max_acc);
          setMaxJerk(max_jerk);
          
          interpolate();
        }

        const JntArraySemantics& semantics() const
        {
          assert(nextPosition().semantics().equals(nextVelocity().semantics()));

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
        // pre-allocated memory for output
        JntArray next_position_;
        JntArray next_velocity_;

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
