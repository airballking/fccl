#include <fccl/control/CartesianConstraintController.h>

using namespace fccl::semantics;
using namespace fccl::utils;

namespace fccl
{
  namespace control
  {
    void CartesianConstraintController::init(
        const fccl::base::ConstraintArray& constraints, double cycle_time)
    {
      // copy input
      assert(constraints.isValid());

      constraints_ = constraints;

      // init sub-modules
      interpolator_.init(constraints_.outputValues().semantics(), cycle_time);
      pid_.init(constraints_.outputValues().semantics());
      solver_.init(constraints_.outputValues().semantics(), 
          constraints_.firstDerivative().semantics().twist());

      // init return values or intermediate results
      H_.init(constraints_.firstDerivative().semantics());

      output_error_.init(constraints_.outputValues().semantics());

      desired_target_velocities_.init(constraints_.outputValues().semantics());

      desired_output_velocities_.init(constraints_.outputValues().semantics());

      desired_twist.semantics() = constraints_.firstDerivative().semantics().twist();

      twist_weights_.resize(6, 6);
      twist_weights_.numerics().setIdentity();

      task_weights_.init(constraints_.outputValues().semantics(), 
          constraints_.outputValues().semantics());
      task_weights_.numerics().setIdentity();

      assembleNecessaryTransforms();
    }

    void CartesianConstraintController::setGains(const fccl::kdl::JntArray& p, 
        const fccl::kdl::JntArray& i, const fccl::kdl::JntArray& d, 
        const fccl::kdl::JntArray& i_max, const fccl::kdl::JntArray& i_min)
    {
      pid_.setGains(p, i, d, i_max, i_min);
    }

    void CartesianConstraintController::setGains(const fccl::control::PIDGains& gains)
    {
      pid_.setGains(gains);
    }

    void CartesianConstraintController::start(const fccl::utils::TransformMap&
        transform_map, double derivative_delta, double cycle_time)
    {
       constraints_.update(transform_map, derivative_delta);

       pid_.reset();
    }

    void CartesianConstraintController::update(const TransformMap& transform_map,
        double derivative_delta, double cycle_time)
    {
      constraints_.update(transform_map, derivative_delta);

      H_ = constraints_.firstDerivative();

      interpolate();

      fccl::kdl::substract(interpolator_.nextPosition(), 
          constraints_.outputValues(), output_error_);

      desired_output_velocities_ = pid_.computeCommand(output_error_, cycle_time);

      solver_.solve(H_, desired_output_velocities_, twist_weights_, task_weights_,
          desired_twist_);
    }

    void CartesianConstraintController::stop()
    {
      KDL::SetToZero(desired_twist_.numerics());
    }

    const fccl::kdl::Twist& CartesianConstraintController::desiredTwist() const
    {
      return desired_twist_;
    }

    const fccl::base::ConstraintArray& CartesianConstraintController::constraints() const
    {
      return constraints_;
    }

    void CartesianConstraintController::interpolate()
    {

      interpolator_.setMaxVelocity(constraints_.maxVelocities());
      interpolator_.setMaxAcceleration(constraints_.maxAccelerations());
      interpolator_.setMaxJerk(constraints_.maxJerks());

      changeInterpolatorActivity(constraints_.taskWeights());

      interpolator_.setCurrentPosition(constraints_.outputValues());
      // Hack: this is assuming the last commanded values got really applied
      interpolator_.setCurrentVelocity(interpolator_.nextVelocity());
      interpolator_.setCurrentAcceleration(interpolator_.nextAcceleration());
 
      interpolator_.setTargetPosition(constraints_.desiredOutputValues());
      interpolator_.setTargetVelocity(desired_output_velocities_);

      interpolator_.interpolate();
    }

    void CartesianConstraintController::changeInterpolatorActivity(const 
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
    void CartesianConstraintController::assembleNecessaryTransforms()
    {
      assert(constraints_.isValid());

      necessary_transforms_.clear();

      std::set<TransformSemantics> constraint_transforms = constraints().necessaryTransforms();
      necessary_transforms_.insert(constraint_transforms.begin(), constraint_transforms.end()); 
    }
  } // namespace control
} // namespace fccl
