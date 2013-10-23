#ifndef FCCL_ESTIMATION_LIMIT_ESTIMATOR_H
#define FCCL_ESTIMATION_LIMIT_ESTIMATOR_H

#include <fccl/kdl/JntArray.h>

namespace fccl
{
  namespace estimation
  {
    class LimitEstimator
    {
      public:
        void init(const fccl::semantics::JntArraySemantics& semantics)
        {
          max_velocity_.init(semantics);
          max_acceleration_.init(semantics);
          max_jerk_.init(semantics);

          // TODO(Georg): implement sth more useful in the future
          max_velocity_.numerics().data.setConstant(2.0);
          max_acceleration_.numerics().data.setConstant(1.0); 
          max_jerk_.numerics().data.setConstant(1.0);
        }

        const fccl::kdl::JntArray& maximumVelocity() const
        {
          return max_velocity_;
        }

        const fccl::kdl::JntArray& maximumAcceleration() const
        {
          return max_acceleration_;
        }

        const fccl::kdl::JntArray& maximumJerk() const
        {
          return max_jerk_;
        }

        void update()
        {
          // TODO(Georg): implement sth more useful in the future
        }

      private:
        fccl::kdl::JntArray max_velocity_;
        fccl::kdl::JntArray max_acceleration_;
        fccl::kdl::JntArray max_jerk_;
    };
  } // namespace estimation
} // namespace fccl
#endif // FCCL_ESTIMATION_LIMIT_ESTIMATOR_H
