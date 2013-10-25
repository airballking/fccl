#ifndef FCCL_ESTIMATION_STATE_ESTIMATOR_H
#define FCCL_ESTIMATION_STATE_ESTIMATOR_H

#include <fccl/kdl/JntArray.h>

namespace fccl
{
  namespace estimation
  {
    class StateEstimator
    {
      public:
        void init(const fccl::semantics::JntArraySemantics& semantics)
        {
          current_position_.init(semantics);
          current_velocity_.init(semantics);
          current_acceleration_.init(semantics);
          tmp_.init(semantics);
        }

        const fccl::kdl::JntArray& currentPosition() const
        {
          return current_position_;
        }

        const fccl::kdl::JntArray& currentVelocity() const
        {
          return current_velocity_;
        }

        const fccl::kdl::JntArray& currentAcceleration() const
        {
          return current_acceleration_;
        }

        void start(const fccl::kdl::JntArray& pos_init,
            const fccl::kdl::JntArray& vel_init, 
            const fccl::kdl::JntArray& acc_init)
        {
          current_position_ = pos_init;
          current_velocity_ = vel_init;
          current_acceleration_ = acc_init;
        }

        void start(const fccl::kdl::JntArray& pos_init)
        {
          tmp_.numerics().data.setZero();
          start(pos_init, tmp_, tmp_);
        }
 
        void sensor_update(const fccl::kdl::JntArray& pos_estimate)
        {
          // TODO(Georg): implement sth more useful in the future
          current_position_ = pos_estimate;
        }

        void control_update(const fccl::kdl::JntArray& pos_cmd,
            const fccl::kdl::JntArray& vel_cmd, const fccl::kdl::JntArray& acc_cmd)
        {
          // TODO(Georg): implement sth more useful in the future
          current_velocity_ = vel_cmd;
          current_acceleration_ = acc_cmd;
        }

      private:
        fccl::kdl::JntArray current_position_;
        fccl::kdl::JntArray current_velocity_;
        fccl::kdl::JntArray current_acceleration_;
        fccl::kdl::JntArray tmp_;
    };
  } // namespace estimation
} // namespace fccl
#endif // FCCL_ESTIMATION_STATE_ESTIMATOR_H
