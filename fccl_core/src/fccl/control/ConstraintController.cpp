#include <fccl/control/ConstraintController.h>

using namespace fccl::semantics;

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
      limit_estimator_.init(constraints_.outputValues().semantics());
      state_estimator_.init(constraints_.outputValues().semantics());
      pid_.init(constraints_.outputValues().semantics());
      solver_.init(constraints_.outputValues().semantics(), 
          kinematics_.semantics().joints());

      // init return values or intermediate results
      desired_joint_velocities_.init(kinematics_.semantics().joints());
      desired_target_velocities_.init(constraints_.outputValues().semantics());
      output_error_.init(constraints_.outputValues().semantics());
      null_space_projector_.init(kinematics_.semantics().joints(), 
          constraints_.outputValues().semantics());
      A_.init(constraints_.outputValues().semantics(), kinematics_.semantics().joints());
      H_.init(constraints_.firstDerivative().semantics());
      // TODO(Georg): extend KinematicChain to get JacobianSemantics
      JR_.init(kinematics_.semantics().joints().jointNames(), 
          kinematics_.semantics().transform().reference().getName(),
          kinematics_.semantics().transform().target().getName());
      joint_weights_.init(kinematics_.semantics().joints(), 
          kinematics_.semantics().joints());
      joint_weights_.numerics().setIdentity();
      task_weights_.init(constraints_.outputValues().semantics(), 
          constraints_.outputValues().semantics());
      task_weights_.numerics().setIdentity();
      tmp_constraint_space_.init(constraints_.outputValues().semantics());

      assembleNecessaryTransforms();
    }

    void ConstraintController::setGains(const fccl::kdl::JntArray& p, 
        const fccl::kdl::JntArray& i, const fccl::kdl::JntArray& d, 
        const fccl::kdl::JntArray& i_max, const fccl::kdl::JntArray& i_min)
    {
      pid_.setGains(p, i, d, i_max, i_min);
    }
 
    void ConstraintController::start(const JntArray& joint_state,
        fccl::utils::TransformMap& transform_map, double derivative_delta,
        double cycle_time)
    {
       constraints_.update(transform_map, derivative_delta);

       state_estimator_.start(constraints_.outputValues());

       pid_.reset();
    }

    void ConstraintController::update(const JntArray& joint_state,
        fccl::utils::TransformMap& transform_map, double derivative_delta,
        double cycle_time)
    {
      constraints_.update(transform_map, derivative_delta);

      assembleEquation(joint_state, transform_map);

      state_estimator_.sensor_update(constraints_.outputValues());

      // TODO(Georg): do sth real here
      limit_estimator_.update();

      interpolate();

      state_estimator_.control_update(interpolator_.nextPosition(), 
          interpolator_.nextVelocity(), interpolator_.nextAcceleration());


      fccl::kdl::substract(constraints_.outputValues(),
          interpolator_.nextPosition(), output_error_);
      desired_output_velocities_ = pid_.computeCommand(output_error_, cycle_time);

      solver_.solve(A_, desired_output_velocities_, joint_weights_, task_weights_,
          desired_joint_velocities_, null_space_projector_);

    }

    void ConstraintController::stop()
    {
    }

    const fccl::kdl::JntArray& ConstraintController::desiredJointVelocities() const
    {
      return desired_joint_velocities_;
    }

    const fccl::kdl::JointMappingMatrix& ConstraintController::nullSpaceProjector() const
    {
      return null_space_projector_;
    }

    const fccl::base::ConstraintArray& ConstraintController::constraints() const
    {
      return constraints_;
    }

    const fccl::kdl::KinematicChain& ConstraintController::kinematics() const
    {
      return kinematics_;
    }

    void ConstraintController::assembleEquation(const fccl::kdl::JntArray& 
        joint_state, fccl::utils::TransformMap& transform_map)
    {
      H_ = constraints_.firstDerivative();

      // get robot jacobian in reference frame w.r.t. which constraints are defined
      JR_ = kinematics_.calculateJacobian(joint_state);

      Transform T = transform_map.getTransform(H_.semantics().twist().reference(),
          JR_.semantics().twist().reference());

      JR_.changeReferenceFrame(T);

      multiply(H_, JR_, A_);
    }

    void ConstraintController::interpolate()
    {
      changeInterpolatorActivity(constraints_.taskWeights());

      interpolator_.setMaxVelocity(limit_estimator_.maximumVelocity());
      interpolator_.setMaxAcceleration(limit_estimator_.maximumAcceleration());
      interpolator_.setMaxJerk(limit_estimator_.maximumJerk());

      interpolator_.setCurrentPosition(state_estimator_.currentPosition());
      interpolator_.setCurrentVelocity(state_estimator_.currentVelocity());
      interpolator_.setCurrentAcceleration(state_estimator_.currentAcceleration());

      interpolator_.setTargetPosition(constraints_.desiredOutputValues());
      interpolator_.setTargetVelocity(desired_target_velocities_);

      interpolator_.interpolate();
    }

    void ConstraintController::changeInterpolatorActivity(const 
        fccl::kdl::JntArray& task_weights)
    {
      assert(task_weights.semantics().equals(interpolator_.semantics()));
      assert(task_weights_.semantics().row_joints().equals(
          task_weights.semantics()));
      assert(task_weights_.semantics().column_joints().equals(
          task_weights.semantics()));

      task_weights_.numerics().diagonal() = task_weights.numerics().data;

      for(std::size_t i=0; i<task_weights.size(); i++)
      {
        assert(task_weights.numerics()(i) >= 0.0);
        assert(task_weights.numerics()(i) <= 1.0);

        interpolator_.setDimensionActivity(i, task_weights.numerics()(i) == 1.0);
      }
    }

    // NOT REAL-TIME-SAFE
    void ConstraintController::assembleNecessaryTransforms()
    {
      assert(constraints_.isValid());
      assert(kinematics_.isValid());

      necessary_transforms_.clear();

      // TODO(Georg): refactor this into a private method which we can also use
      //              in assembleEquation()
      TransformSemantics transform;
      transform.reference() = 
          constraints_.firstDerivative().semantics().twist().reference();
      // TODO(Georg): extend KinematicChain to get JacobianSemantics
      transform.target() = kinematics_.semantics().transform().reference();
      necessary_transforms_.insert(transform);

      std::set<TransformSemantics> constraint_transforms = constraints().necessaryTransforms();
      necessary_transforms_.insert(constraint_transforms.begin(), constraint_transforms.end()); 
    }
  } // namespace control
} // namespace fccl
