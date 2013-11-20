#ifndef FCCL_NODES_COMMAND_PUBLISHER_H
#define FCCL_NODES_COMMAND_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <fccl/kdl/JntArray.h>

using namespace std_msgs;

namespace fccl
{
  namespace nodes
  {
    class CommandPublisher
    {
      public:
        CommandPublisher(const ros::NodeHandle& nh, const std::string& topic) : 
          nh_(nh), msg_(), pub_(nh_.advertise<Float64MultiArray>(topic, 1)),
          semantics_()
        {
        }

        ~CommandPublisher()
        {
        }

        void resize(const fccl::semantics::JntArraySemantics& semantics)
        {
          semantics_=semantics;
          msg_.data.resize(semantics.size());
        }

        void publish(const fccl::kdl::JntArray& command)
        {
          assert(command.semantics().equals(semantics_));
          assert(msg_.data.size() == command.size());

          for(std::size_t i=0; i<command.size(); i++)
          {
            msg_.data[i] = command.numerics()(i);
          }

          pub_.publish(msg_);
        }

      private:
        ros::NodeHandle nh_;
        Float64MultiArray msg_;
        ros::Publisher pub_;
        fccl::semantics::JntArraySemantics semantics_;
    }; // class CommandPublisher
  } // namespace nodes
} // namespace fccl
#endif // FCCL_NODES_COMMAND_PUBLISHER_H
