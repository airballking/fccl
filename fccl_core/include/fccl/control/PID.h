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
