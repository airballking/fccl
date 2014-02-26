#ifndef FCCL_CONTROL_PID_H
#define FCCL_CONTROL_PID_H

#include <fccl/control/Gains.h>
#include <fccl/kdl/JntArray.h>
// TODO(Georg): consider replacing control_toolbox/pid with sth ros-agnostic
#include <control_toolbox/pid.h>
#include <ros/duration.h>
#include <vector>

namespace fccl
{
  namespace control
  {
    template <class T, class SemanticsPolicy>
    class PIDController : public SemanticsPolicy
    {
      public:
        const PIDGains2<T, SemanticsPolicy>& getGains() const
        {
          return this->gains_;
        }

        void setGains(const PIDGains2<T, SemanticsPolicy>& gains)
        {
          assert(gains.semantics().equals(this->semantics()));

          this->gains_ = gains;
        }

        void reset()
        {
          p_last_error_ = 0.0;
          i_error_term_ = 0.0;
          cmd_ = 0.0;
        }

        // error = target - state
        const T& computeCommand(const T& error, const T& dt)
        {
          if (!inputValid(error, dt))
            return 0.0;

          cmd_ = - calcProportionalErrorTerm(error)
                 - calcIntegralErrorTerm(error, dt)
                 - calcDerivativeErrorTerm(error, dt);

          return cmd_;
        }
      
      private:
        PIDGains2<T, SemanticsPolicy> gains_;

        // Save last position error to calculate derivated error
        T p_last_error_;
        // Integral error term
        T i_error_term_;

        // Memory for command output
        T cmd_;

        bool inputValid(const T& error, const T& dt) const
        {
          return dt <= 0.0 || std::isnan(error) || std::isinf(error);
        }

        T calcProportionalErrorTerm(const T& error) const
        {
          return getGains().p() * error;
        }

        T calcIntegralErrorTerm(const T& error, const T& dt)
        {
          i_error_term_ += getGains().i() * dt * error;

          // truncate integral error term
          i_error_term_ =
              std::max( getGains().i_min(),
                        std::min( i_error_term_, getGains().i_max()) );

          return i_error_term_;
        }

        T calcDerivativeErrorTerm(const T& error, const T& dt)
        {
          // calculate derivative error numerically
          T d_error_term = (error - p_last_error_) / dt;
          // remember current error for next iteration
          p_last_error_ = error;

          return getGains().d() * d_error_term;
        }
    };

    typedef PIDController<double, fccl::semantics::SemanticsBase> StandardPID;
    typedef Array<StandardPID, fccl::semantics::SemanticsBase> StandardPIDArray;

    // TODO(Georg): refactor this into Array<template> + single-PID
    class PID
    {
      public:
        void init(const fccl::semantics::JntArraySemantics& semantics)
        {
          effort_out_.init(semantics);
          pids_.resize(semantics.size());
        }

        void reset()
        {
          for(std::size_t i=0; i<pids_.size(); i++)
            pids_[i].reset();
        }

        void setGains(const fccl::kdl::JntArray& p, const fccl::kdl::JntArray& i,
            const fccl::kdl::JntArray& d, const fccl::kdl::JntArray& i_max,
            const fccl::kdl::JntArray& i_min)
        {
          assert(p.semantics().equals(effort_out_.semantics()));
          assert(i.semantics().equals(effort_out_.semantics()));
          assert(d.semantics().equals(effort_out_.semantics()));
          assert(i_max.semantics().equals(effort_out_.semantics()));
          assert(i_min.semantics().equals(effort_out_.semantics()));
          assert(pids_.size() == effort_out_.size());

          for(std::size_t j=0; j<p.size(); j++)
            pids_[j].setGains(p.numerics()(j), i.numerics()(j), d.numerics()(j),
                i_max.numerics()(j), i_min.numerics()(j));
        }

        void setGains(const fccl::control::PIDGains& gains)
        {
          setGains(gains.p(), gains.i(), gains.d(), gains.i_max(), gains.i_min());
        }

        const fccl::kdl::JntArray& computeCommand(const fccl::kdl::JntArray& error,
            double cycle_time)
        {
          assert(error.semantics().equals(effort_out_.semantics()));
          assert(cycle_time > 0.0);
          assert(error.size() == pids_.size());

          for(std::size_t i=0; i<pids_.size(); i++)
             effort_out_.numerics()(i) = pids_[i].computeCommand(
                 error.numerics()(i), ros::Duration(cycle_time));

          return effort_out_;
        }
 
      private:
        fccl::kdl::JntArray effort_out_;
        std::vector<control_toolbox::Pid> pids_;
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_PID_H
