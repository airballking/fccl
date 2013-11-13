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

// TODO(Georg): split this into h and cpp file
namespace fccl
{
  namespace nodes
  {
    class SingleArmInitException : public std::runtime_error
    {
      public:
        SingleArmInitException(const std::string& message) : 
            std::runtime_error(message) { }
    };
    
    class SingleArmController
    { 
      public:
        SingleArmController(const NodeHandle& node_handle) :
            node_handle_(node_handle), action_server_(node_handle, "command", false),
            tf_thread_(NULL), js_listener_(), 
            cycle_time(0.01), delta_deriv(0.001), p_(JntArray()),
            i_(JntArray()), d_(JntArray()), i_max_(JntArray()), i_min_(JntArray())
        {
          tf_thread_ = new thread( bind( &SingleArmController::loopTF, this ) );
    
          action_server_.registerGoalCallback( bind( 
              &SingleArmController::commandGoalCallback, this ) );
          action_server_.registerPreemptCallback( bind( 
              &SingleArmController::commandPreemptCallback, this ) );
          action_server_.start();
        }
    
        ~SingleArmController()
        {
          if(tf_thread_)
          {
            tf_thread_->interrupt();
            tf_thread_->join();
            delete tf_thread_;
            tf_thread_ = NULL;
          }
        }
    
        void init(const fccl_msgs::SingleArmMotionGoalConstPtr& goal) 
            throw (SingleArmInitException, ConversionException)
        {
          ConstraintArray constraints = fromMsg(goal->constraints);
    
          if(!constraints.isValid())
            throw SingleArmInitException("Given constraints not valid. Aborting.");
    
          if(!urdf_.initParam("robot_description"))
            throw SingleArmInitException("No urdf 'robot_description' on param-server.");
    
          KinematicChain kinematics = fromMsg(goal->kinematics, urdf_);
    
          if(!kinematics.isValid())
            throw SingleArmInitException("Given kinematics not valid. Aborting.");
    
          initTFRequests(constraints.necessaryTransforms());
    
          initJointState(kinematics.semantics().joints());
    
          controller_.init(constraints, kinematics, cycle_time);
    
          // TODO(Georg): get gains
          controller_.setGains(p_, i_, d_, i_max_, i_min_);
        }
    
        void start(const JntArray& joint_state, const TransformMap& transform_map)
        {
          controller_.start(joint_state, transform_map, delta_deriv, cycle_time); 
        }
    
        void stop()
        {
          controller_.stop();
          tf_worker_.clear();
        }
    
        void update(const JntArray& joint_state, const TransformMap& transform_map)
        {
        }
    
      private:
        // ROS communication infrastructure
        NodeHandle node_handle_;
        actionlib::SimpleActionServer<fccl_msgs::SingleArmMotionAction> action_server_;
        Subscriber js_subscriber_;
    
        // TF infrastructure
        thread* tf_thread_;
        TFWorker tf_worker_;
    
        // joint state infrastructure
        JointStateListener js_listener_;
    
        // urdf-stuff
        urdf::Model urdf_;
    
        // control infrastructure
        ConstraintController controller_;
        const double cycle_time, delta_deriv;
        JntArray p_, i_, d_, i_max_, i_min_;
    
        void commandGoalCallback()
        {
          fccl_msgs::SingleArmMotionGoalConstPtr goal = action_server_.acceptNewGoal();
    
          if(action_server_.isPreemptRequested())
          {
            commandPreemptCallback();
            return;
          }
    
          try
          {
            init(goal);
          }
          catch (std::exception& e)
          {
            ROS_INFO("Error during init of SingleArmController: '%s'", e.what());
            commandPreemptCallback();
            return;
          }
    
          start(js_listener_.currentJointState(), tf_worker_.currentTransforms());
        }
    
        void commandPreemptCallback()
        {
          stop();
          action_server_.setPreempted();
        }
    
        void initTFRequests(const std::set<TransformSemantics> requests)
            throw (SingleArmInitException)
        {
          tf_worker_.clear();
     
          tf_worker_.addRequests(requests);
     
          Time timeout = Time::now() + Duration(0.1);
          Duration short_time(0.01);
     
          do
          {
            short_time.sleep();
          } 
          while((Time::now() < timeout) && !tf_worker_.allRequestsFound());
    
          if(!tf_worker_.allRequestsFound())
            throw SingleArmInitException("TF was not aware of all necessary transforms. Aborting.");
        }
    
        void loopTF()
        {
          // TODO(Georg): consider moving this into TFWorker which then spawns its own thread
          Rate sleep_rate(20.0);
    
          while(ros::ok())
          {
            sleep_rate.sleep();
            tf_worker_.lookupTransforms();
          }
        }
    
        void initJointState(const JntArraySemantics& joints)
            throw (SingleArmInitException)
        {
          js_listener_.setDesiredSemantics(joints);
    
          Time timeout = Time::now() + Duration(0.1);
          Duration short_time(0.01);
     
          do
          {
            short_time.sleep();
          } 
          while((Time::now() < timeout) && !js_listener_.currentJointStateValid());
    
          if(!js_listener_.currentJointStateValid())
            throw SingleArmInitException("Joint State could not be parsed for too long. Aborting.");
        }
    };
  } // namespace nodes
} // namespace fccl
#endif // FCCL_NODES_SINGLE_ARM_CONTROLLER_H
