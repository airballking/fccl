#ifndef FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
#define FCCL_CONTROL_CONSTRAINT_CONTROLLER_H

#include <fccl/kdl/JointMappingMatrix.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/kdl/KinematicChain.h>
#include <fccl/base/ConstraintArray.h>
#include <fccl/control/Interpolator.h>
#include <fccl/control/PID.h>
#include <fccl/estimation/LimitEstimator.h>
#include <fccl/estimation/StateEstimator.h>
#include <fccl/solvers/WeightedSolver.h>
#include <fccl/utils/TransformMap.h>
#include <vector>
#include <set>

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
            const fccl::utils::TransformMap& transform_map, double delta=0.001,
            double cycle_time=0.01);

        void update(const fccl::kdl::JntArray& joint_state,
            const fccl::utils::TransformMap& transform_map, double delta=0.001,
            double cycle_time=0.01);
 
        void stop();

        void setGains(const fccl::kdl::JntArray& p, const fccl::kdl::JntArray& i,
            const fccl::kdl::JntArray& d, const fccl::kdl::JntArray& i_max,
            const fccl::kdl::JntArray& i_min);

        void setGains(const fccl::control::PIDGains& gains); 
 
        const fccl::kdl::JntArray& desiredJointVelocities() const;
        const fccl::kdl::JointMappingMatrix& nullSpaceProjector() const;

        const fccl::base::ConstraintArray& constraints() const;
        const fccl::kdl::KinematicChain& kinematics() const;

        const std::set<fccl::semantics::TransformSemantics>& necessaryTransforms() const
        {
          return necessary_transforms_;
        }
 
      private:
        // actual state variables of the controller
        fccl::base::ConstraintArray constraints_;
        fccl::kdl::KinematicChain kinematics_;
        fccl::control::Interpolator interpolator_;
        fccl::estimation::LimitEstimator limit_estimator_;
        fccl::estimation::StateEstimator state_estimator_;
        fccl::control::PID pid_;
        fccl::solvers::WeightedSolver solver_;

        // memory for output values
        fccl::kdl::JntArray desired_joint_velocities_;
        fccl::kdl::JointMappingMatrix null_space_projector_;

        // intermediates results...
        fccl::kdl::JointMappingMatrix A_;
        fccl::kdl::InteractionMatrix H_;
        fccl::kdl::Jacobian JR_;
        fccl::kdl::JntArray output_error_;
        fccl::kdl::JntArray desired_output_velocities_;
        fccl::kdl::JntArray desired_target_velocities_;
        fccl::kdl::JntArray tmp_constraint_space_;
        fccl::kdl::JointMappingMatrix joint_weights_;
        fccl::kdl::JointMappingMatrix task_weights_;
        std::set<fccl::semantics::TransformSemantics> necessary_transforms_;

        void changeInterpolatorActivity(const fccl::kdl::JntArray& task_weights);
        void interpolate();
        void assembleEquation(const fccl::kdl::JntArray& joint_state,
            const fccl::utils::TransformMap& transform_map);
        void assembleNecessaryTransforms();
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
