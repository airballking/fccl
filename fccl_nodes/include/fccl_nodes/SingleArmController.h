#ifndef FCCL_NODES_SINGLE_ARM_CONTROLLER_H
#define FCCL_NODES_SINGLE_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fccl_msgs/SingleArmMotionAction.h>
#include <urdf/model.h>

#include <fccl_nodes/TFWorker.h>
#include <fccl_nodes/JointStateListener.h>
#include <fccl_conversions/Conversions.h>
#include <fccl/control/ConstraintController.h>

#include <boost/thread.hpp>
#include <iostream>
#include <stdexcept>

using namespace boost;
using namespace fccl::control;
using namespace fccl::conversions;
using namespace fccl::kdl;
using namespace fccl::base;
using namespace fccl::semantics;
using namespace ros;

namespace fccl
{
  namespace nodes
  {
    class SingleArmInitException : public std::runtime_error
    {
      public:
        SingleArmInitException(const std::string& message) : 
            std::runtime_error(message) {}
    };
    
    class SingleArmController
    { 
      public:
        SingleArmController(const NodeHandle& node_handle);
        ~SingleArmController();
    
        void init(const fccl_msgs::SingleArmMotionGoalConstPtr& goal) 
            throw (SingleArmInitException, ConversionException);
        void start(const JntArray& joint_state, const TransformMap& transform_map);
        void stop();
        void update(const JntArray& joint_state, const TransformMap& transform_map);
    
      private:
        // ROS communication infrastructure
        NodeHandle node_handle_;
        actionlib::SimpleActionServer<fccl_msgs::SingleArmMotionAction> action_server_;
    
        // TF infrastructure
        thread* tf_thread_;
        TFWorker tf_worker_;
    
        // joint state infrastructure
        JointStateListener js_listener_;
    
        // control infrastructure
        ConstraintController controller_;
        const double cycle_time, delta_deriv;
    
        void commandGoalCallback();
        void commandPreemptCallback();
    
        void initTFRequests(const std::set<TransformSemantics> requests)
            throw (SingleArmInitException);
        void loopTF();
    
        void initJointState(const JntArraySemantics& joints)
            throw (SingleArmInitException);
        void initControllerGains(const ConstraintArray& constraints)
            throw (SingleArmInitException);
    };
  } // namespace nodes
} // namespace fccl
#endif // FCCL_NODES_SINGLE_ARM_CONTROLLER_H
