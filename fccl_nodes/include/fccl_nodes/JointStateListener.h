#ifndef FCCL_NODES_JOINT_STATE_LISTENER_H
#define FCCL_NODES_JOINT_STATE_LISTENER_H

#include <fccl_conversions/JointStateInterpreter.h>
#include <sensor_msgs/JointState.h>
#include <fccl/kdl/JntArray.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

using namespace fccl::semantics;
using namespace fccl::kdl;
using namespace fccl::conversions;

// TODO(Georg): put into namespace fccl::nodes
// TODO(Georg): split this into h and cpp file
class JointStateListener
{
  public:
    JointStateListener() :
        mutex_(), thread_(NULL), node_handle_(), subscriber_(), callback_queue_(),
        keep_listening_(true), current_(), parser_()
    {
      // fancy subscription to run its callback in separate thread
      //TODO(Georg): move this topic somewhere
      ros::SubscribeOptions options = 
          ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_state", 1,
              boost::bind(&JointStateListener::callback, this, _1), 
              ros::VoidPtr(), &callback_queue_);
      subscriber_ = node_handle_.subscribe(options);

      // assign thread to work on callback queue
      thread_ = new boost::thread(boost::bind(&JointStateListener::spin, this));
    }

    ~JointStateListener()
    {
      // signal thread to stop
      keep_listening_ = false;

      // kill thread
      if(thread_)
      {
        thread_->join();
        delete thread_;
      }
    }

    void setDesiredSemantics(const JntArraySemantics& semantics) 
    {
       boost::mutex::scoped_lock scoped_lock(mutex_);
       current_.init(semantics);
       parser_.init(semantics); 
    }

    const JntArray& currentJointState() const 
    { 
      boost::mutex::scoped_lock scoped_lock(mutex_);
      return current_; 
    }

    bool currentJointStateValid() const
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
      return parser_.semantics().equals(current_.semantics());
    }

  private:
    // communication infrastructure
    mutable boost::mutex mutex_;
    boost::thread* thread_;
    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::CallbackQueue callback_queue_;
    bool keep_listening_;

    // information processing infrastructure
    JntArray current_;
    JointStateInterpreter parser_;

    void spin()
    {
      while (keep_listening_ && ros::ok())
      {
        // TODO(Georg): move this number somewhere
        callback_queue_.callAvailable(ros::WallDuration(0.01));
      }
    }

    void callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
      if(!parser_.parseJointState(*msg, current_))
        ROS_ERROR("Could not parse joint state.");
    }
};
#endif // FCCL_NODES_JOINT_STATE_LISTENER_H
