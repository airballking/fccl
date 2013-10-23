#include <fccl/control/ConstraintController.h>

namespace fccl
{
  namespace control
  {
    void ConstraintController::init(const fccl::base::ConstraintArray& constraints,
        const fccl::kdl::KinematicChain& kinematics, double cycle_time)
    {
      // copy input
      assert(kinematics.isValid());
      assert(constraints.isValid());

      constraints_ = constraints;
      kinematics_ = kinematics;

      // init sub-modules
      interpolator_.init(constraints_.outputValues().semantics(), cycle_time);
      limit_estimator_.init(constraints_.semantics().joints());
      state_estimator_.init(constraints_.semantics().joints());
      pids_.init(constraints_.semantics().joints());

      // init return values or intermediate results
      desired_joint_velocities_.init(kinematics_.semantics().joints());
      desired_target_velocities_.init(kinematics_.semantic().joints());
      null_space_projector_.init(kinematics_.semantics().joints(), 
          kinematics_.semantics().joints());
      A_.init(constraints_.semantics().joints(), kinematics_.semantics().joints());
      H_.init(constraints_.firstDerivative().semantics());
      JR_.init(kinematics_.semantics().joints(), 
          constraints_.firstDerivative().semantics().twist()); 
      joint_weights_.init(kinematics_.semantics().joints());
      joint_weights_.numerics().data.setConstant(1.0);
    }

    void ConstraintController::setGains(const fccl::kdl::JntArray& p, 
        const fccl::kdl::JntArray& i, const fccl::kdl::JntArray& d, 
        const fccl::kdl::JntArray& i_max, const fccl::kdl::JntArray& i_min)
    {
      pids_.setGains(p, i, d, i_max, i_min);
    }
 
    void ConstraintController::start(const JntArray& joint_state,
        const fccl::utils::TransformMap& transform_map, double derivative_delta,
        double cycle_time)
    {
       constraints_.update(transfrom_map, delta);

       state_estimator_.start(constraints_.outputValues(), Zero, Zero);

       pids_.reset();
    }

    void ConstraintController::update(const JntArray& joint_state,
        const fccl::utils::TransformMap& transform_map, double derivative_delta,
        double cycle_time)
    {
      constraints_.update(transfrom_map, delta);

      assembleEquation();

      state_estimator_.sensor_update(constraints_.outputValues());

      // TODO(Georg): do sth real here
      limit_estimator_.update();

      interpolate();

      state_estimator_.control_update(interpolator_.nextPosition(), 
          interpolator_.nextVelocity(), interpolator_.nextAcceleration());

      desired_output_velocities_ = pids_.computeCommand(
          constraints_.outputValues() - interpolator_.nextPosition(), cycle_time);
 
      // TODO(Georg):
      //  - wrap solver with semantics
      solver_.solve(A_, desired_output_velocities_, constraints_.taskWeights(), joint_weights_, desired_joint_velocites_, null_space_projector_);
    }

    void ConstraintController::stop()
    {
    }

    const fccl::kdl::JntArray& ConstraintController::desiredJointVelocities() const
    {
      return desired_joint_velocities_;
    }

    const fccl::kdl::MatrixNxN& ConstraintController::nullSpaceProjector() const
    {
      return nullSpaceProjector_;
    }

    const std::vector<fccl::base::Constraint>& ConstraintController::constraints() const
    {
      return constraints_;
    }

    const fccl::kdl::KinematicChain& ConstraintController::kinematics() const
    {
      return kinematics_;
    }

    void ConstraintController::assembleEquation()
    {
      H_ = constraints_.firstDerivative();
      // get robot jacobian in reference frame w.r.t. which constraints are defined
      JR_ = kinematics_.calculateJacobian(joint_state);
      JR_.changeReferenceFrame(transform_map.getTransform(
          H_.semantics().twist().reference(), JR_.semantics().twist().reference()));
      A_ = H_ * JR_;
    }

    void ConstraintController::interpolate()
    {
      changeInterpolatorActivity(constraints_.taskWeights());

      interpolator_.setMaximumVelocity(limit_estimator_.maximumVelocity());
      interpolator_.setMaximumAcceleration(limit_estimator_.maximumAcceleration());
      interpolator_.setMaximumJerk(limit_estimator_.maximumJerk());

      interpolator_.setCurrentPosition(constraint_estimator_.currentPosition());
      interpolator_.setCurrentVelocity(constraint_estimator_.currentVelocity());
      interpolator_.setCurrentAcceleration(constraint_estimator_.currentAcceleration());

      interpolator_.setTargetPosition(constraints_.desiredOutputValues());
      interpolator_.setTargetVelocity(desired_target_velocities_);

      interpolator_.interpolate();
    }

    void ConstraintController::changeInterpolatorActivity(const 
        fccl::kdl::JntArray& task_weights)
    {
      assert(task_weights.semantics().equals(interpolator_.semantics()));

      for(std::size_t i=0; i<task_weights.size(); i++)
      {
        assert(task_weights.numerics()(i) >= 0.0);
        assert(task_weights.numerics()(i) <= 1.0);

        interpolator.setDimensionActivity(i, task_weights.numerics()(i) == 1.0);
      }
    }
  } // namespace control
} // namespace fccl
