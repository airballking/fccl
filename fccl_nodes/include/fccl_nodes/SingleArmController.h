#ifndef FCCL_NODES_SINGLE_ARM_CONTROLLER_H
#define FCCL_NODES_SINGLE_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fccl_msgs/SingleArmMotionAction.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include <fccl_nodes/TFWorker.h>
#include <fccl_nodes/JointStateListener.h>
#include <fccl_nodes/CommandPublisher.h>
#include <fccl_conversions/Conversions.h>
#include <fccl/control/ConstraintController.h>
#include <fccl/control/FSM.h>

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

        void run();
        void shutdown();
   
      private:
        // ROS communication infrastructure
        NodeHandle node_handle_;
        actionlib::SimpleActionServer<fccl_msgs::SingleArmMotionAction> action_server_;
    
        // TF infrastructure
        TFWorker tf_worker_;
    
        // joint state infrastructure
        // member to obtain joint state with correct semantics
        JointStateListener js_listener_;

        // joint command publishing infrastructure
        CommandPublisher qdot_publisher_;

        // finite state machine
        fccl::control::ControllerFSM<fccl_msgs::SingleArmMotionGoalConstPtr> fsm_;
   
        // control infrastructure
        ConstraintController controller_;
        const double cycle_time_, delta_deriv_;

        // pacing infrastructure
        // ... listener pacing controller by calling update on every new joint-state
        Subscriber js_subscriber_;
        // ... callback to trigger the FSM with every new joint-state
        void js_callback(const sensor_msgs::JointState::ConstPtr& msg);
   
        // action interface
        void commandGoalCallback() throw ();
        void commandPreemptCallback() throw ();
    
        // init helpers 
        void initTFRequests(const std::set<TransformSemantics> requests)
            throw (TFWorkerException);
        void initJointState(const JntArraySemantics& joints)
            throw (SingleArmInitException);
        void initControllerGains(const ConstraintArray& constraints)
            throw (SingleArmInitException);

        // hooks provided to ControllerFSM
        bool init(const fccl_msgs::SingleArmMotionGoalConstPtr& goal) 
            throw ();
        void start() throw ();
        void stop() throw ();
        void update() throw ();
    };
  } // namespace nodes
} // namespace fccl
#endif // FCCL_NODES_SINGLE_ARM_CONTROLLER_H
