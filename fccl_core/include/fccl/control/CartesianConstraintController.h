#ifndef FCCL_CONTROL_CARTESIAN_CONSTRAINT_CONTROLLER_H
#define FCCL_CONTROL_CARTESIAN_CONSTRAINT_CONTROLLER_H

#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/kdl/JointMappingMatrix.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/kdl/Twist.h>
#include <fccl/base/ConstraintArray.h>
#include <fccl/control/Interpolator.h>
#include <fccl/control/PID.h>
#include <fccl/solvers/WeightedSolver.h>
#include <fccl/utils/TransformMap.h>
#include <vector>
#include <set>

namespace fccl
{
  namespace control
  {
    class CartesianConstraintController
    {
      public:
        void init(const fccl::base::ConstraintArray& constraints, double cycle_time=0.01);

        void start(const fccl::utils::TransformMap& transform_map, double delta=0.001,
            double cycle_time=0.01);

        void update(const fccl::utils::TransformMap& transform_map, double delta=0.001,
            double cycle_time=0.01);
 
        void stop();

        void setGains(const fccl::kdl::JntArray& p, const fccl::kdl::JntArray& i,
            const fccl::kdl::JntArray& d, const fccl::kdl::JntArray& i_max,
            const fccl::kdl::JntArray& i_min);

        void setGains(const fccl::control::PIDGains& gains); 
 
        const fccl::kdl::Twist& desiredTwist() const;

        const fccl::base::ConstraintArray& constraints() const;

        const std::set<fccl::semantics::TransformSemantics>& necessaryTransforms() const
        {
          return necessary_transforms_;
        }
 
      private:
        // actual state variables of the controller
        fccl::base::ConstraintArray constraints_;
        fccl::control::Interpolator interpolator_;
        fccl::control::PID pid_;
        fccl::solvers::WeightedSolver solver_;

        // memory for output values
        fccl::kdl::Twist desired_twist_;

        // intermediates results...
        fccl::kdl::InteractionMatrix H_;
        fccl::kdl::JntArray output_error_;
        fccl::kdl::JntArray desired_output_velocities_;
        fccl::kdl::JntArray desired_target_velocities_;
        fccl::kdl::Twist desired_twist;
        fccl::kdl::JointMappingMatrix twist_weights_;
        fccl::kdl::JointMappingMatrix task_weights_;
        std::set<fccl::semantics::TransformSemantics> necessary_transforms_;

        void changeInterpolatorActivity(const fccl::kdl::JntArray& task_weights);
        void interpolate();
        void assembleNecessaryTransforms();
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_CARTESIAN_CONSTRAINT_CONTROLLER_H
