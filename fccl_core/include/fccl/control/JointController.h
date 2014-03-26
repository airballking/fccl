#ifndef FCCL_CONTROL_JOINT_CONTROLLER_H
#define FCCL_CONTROL_JOINT_CONTROLLER_H

#include <fccl/kdl/JointArray.h>
#include <fccl/base/JointConstraintArray.h>

using namespace fccl::base;
using namespace fccl::kdl;

namespace fccl
{
  namespace control
  {
    class JointControllerInitException : public std::runtime_error
    {
      public:
        JointControllerInitException(const std::string& message) : 
            std::runtime_error(message) {}
    };
 
    template <class T>
    class DummyLimitEstimator
    {
      public
        void init(const Array<T>& array)
        {
          limits_ = array;
        }

        const Array<T>& getLimits() const
        {
          return limits_;
        }

      private:
        Array<T> limits_;
    };

    template <class T>
    class DummyStateEstimator
    {
      public:
        void init(const Array<T>& initial_estimate)
        {
          estimate_ = initial_estimate;
        }

        const Array<T>& currentEstimate() const
        {
          return estimate_;
        }

        void control_update(const Array<T>& command)
        {
          estimate_ = command;
        }

        void sensor_update(const Array<T>& percept)
        {
          estimate_ = percept;
        }
 
      private:
        Array<T> estimate_;
    };
    // TODO(Georg): Do we dare making a templated controller?
    // TODO(Georg): add limitEstimationPolicy
    // TODO(Georg): add stateEstimationPolicy
    template <
      class T, 
      class LimitEstimationPolicy<T>, 
      class StateEstimationPolicy<T> 
    >
    class JointController
    {
      public:
        void init(const JointConstraintArray& constraints, double cycle_time)
            throw (JointControllerInitException)
        {
          if(!constraints.isValid())
            throw JointControllerInitException("Init of joint controller failed because of invalid constraint command.");
          constraints_ = constraints;

          control_output_.init(constraints);

          interpolator_.init(constraints, cycle_time);

          // TODO(Georg): init from constraints
          pids_.init(control_output_);
          pids_.reset();

          // TODO(Georg): move this away from here
          limits.init(constraints);
          limits.position().setValue(0.0);
          limits.velocity().setValue(2.0);
          limits.acceleration().setValue(1.0);
          limits.jerk().setValue(1.0);
          limit_estimator_.init
          limit_estimator_.init(limits);

          state_estimator_.init(constraints);
        }
 
        void start(const AccelerationJointStateArray& joint_state)
        {
          pids_.reset();
          control_output_.setZero();
        }
 
        void update(const AccelerationJointStateArray& joint_state, 
            const JerkJointStateArray& max_specs, double cycle_time)
        {
          assert(pids_.size() == constraints.size());
          assert(desired_joint_velocities_.size() == joint_state.size());

          for(std::size_t i=0; i<constraints.size(); i++)
          {
            // TODO(Georg): add some joint filter/map
            assert(constraints(i).semantics().equals(joint_state()(i).semantics()));
            constraints(i).output() = joint_state()(i)
            // TODO(Georg): add some joint filter/map
            weights(i) = constraints(i).calculateWeight();
            des_joint_pos(i) = constraints(i).calculateDesiredOutput();
            des_joint_vel(i) = 0.0;
          }

          // TODO(Georg): add some joint filter/map
          interpolator_.setCurrentState(joint_state);

          interpolator_.setDesiredState(des_state);

          interpolator_.setMaxState(max_specs);

          desired_constraint_state = interpolator_.performInterpolation();

          for(std::size_t i=0; i<constraints.size(); i++)
            assert(desired_constraint_state(i).semantics().equals(
                joint_state(what).semantics()));
            assert(desired_constraint_state(i).semantics().equals(
                joint_state(what).semantics()));
 
            pids_(i).computeCommand(desired_constraint_state(i).position() - 
                joint_state(what).position());
 
        }
 
        void stop()
        {
          pids_.reset();
          control_output_.setZero();
        }

        const JointConstraintArray& constraints() const
        {
          return constraints_;
        }

        const JointArray& desiredJointVelocities() const
        {
          return control_output_;
        }

        // TODO(Georg): move this to a cpp-file
        // NOT REALTIME-SAFE
        StandardPIDGainArray gains() const
        {
          StandardPIDGainArray result;
          result.size(pids_.size());
          for(std::size_t i=0; i<pids_.size(); i++)
            result(i) = pids_(i).getGains();
          return result;
        }

        // TODO(Georg): move this to a cpp-file
        void setGains(const StandardPIDGainArray& gains); 
        {
          for(std::size_t i=0; i<gains.size(); i++)
          {
            assert(kinematics().semantics().joints()(i).equals(
                gains(i).semantics()));

            this->gains_(i) = gains(i);
          }
        }
 
      private:
        // state of the controller
        Interpolator interpolator_;
        StandardPIDArray pids_;
        JointConstraintArray constraints_;
 
        // memory for output values
        VelocityJointArray control_output_;

        // some estimation
        LimitEstimationPolicy limit_estimator_;
        StateEstimationPolicy state_estimator_;

        void buildMapping(joints, constraints)
        {
          // TODO(Georg): implement me
        }

        size_t lookUp(constraint)
        {
          // TODO(Georg): implement me
        }
    };
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_JOINT_CONTROLLER_H
