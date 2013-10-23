#ifndef FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
#define FCCL_CONTROL_CONSTRAINT_CONTROLLER_H

#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/KinematicChain.h>
#include <fccl/base/Constraints.h>
#include <fccl/utils/TransformMap.h>
#include <fccl/kdl/JointMappingMatrix.h>
#include <vector>

namespace fccl
{
  namespace control
  {
    class ConstraintController
    {
      public:
        void init(const fccl::base::ConstraintArray& constraints, 
            const fccl::kdl::KinematicChain& kinematics, double cycle_time);

        void start(const fccl::kdl::JntArray& joint_state,
            const fccl::utils::TransformMap& transform_map, double delta=0.001);

        void update(const fccl::kdl::JntArray& joint_state,
            const fccl::utils::TransformMap& transform_map, double delta=0.001);
 
        void stop();

        const fccl::kdl::JntArray& desiredJointVelocities() const;
        const fccl::kdl::JointMappingMatrix& nullSpaceProjector() const;

        const fccl::base::ConstraintArray& constraints() const;
        const fccl::kdl::KinematicChain& kinematics() const;
 
      private:
        // actual state variables of the controller
        std::vector<fccl::base::Constraint> constraints_;
        fccl::kdl::KinematicChain& kinematics_;
        fccl::control::Interpolator interpolator_;
        fccl::estimation::LimitEstimator limit_estimator_;
        fccl::estimation::StateEstimator constraint_estimator_;
        fccl::control::PID pid_;

        // memory for output values
        fccl::kdl::JntArray desired_joint_velocities_;
        fccl::kdl::JointMappingMatrix null_space_projector_;

        // intermediates results...
        fccl::kdl::JointMappingMatrix A_;
        fccl::kdl::InteractionMatrix H_;
        fccl::kdl::Jacobian JR_;
        fccl::kdl::JntArray desired_output_velocities_;
        fccl::kdl::JntArray desired_target_velocities_;
        fccl::kdl::JntArray tmp_constraint_space_;
        fccl::kdl::JntArray joint_weights_;

        void changeInterpolatorActivity(const fccl::kdl::JntArray& task_weights);
        void interpolate();
        void assembleEquation();
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
