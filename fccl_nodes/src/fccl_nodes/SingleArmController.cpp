#include <fccl_nodes/SingleArmController.h>

namespace fccl
{
  namespace nodes
  {
    SingleArmController::SingleArmController(const NodeHandle& node_handle) :
        node_handle_(node_handle), action_server_(node_handle, "command", false),
        tf_thread_(NULL), js_listener_(), 
        cycle_time(0.01), delta_deriv(0.001)
    {
      tf_thread_ = new thread( bind( &SingleArmController::loopTF, this ) );

      action_server_.registerGoalCallback( bind( 
          &SingleArmController::commandGoalCallback, this ) );
      action_server_.registerPreemptCallback( bind( 
          &SingleArmController::commandPreemptCallback, this ) );
      action_server_.start();
    }

    SingleArmController::~SingleArmController()
    {
      if(tf_thread_)
      {
        tf_thread_->interrupt();
        tf_thread_->join();
        delete tf_thread_;
        tf_thread_ = NULL;
      }
    }

    void SingleArmController::init(const fccl_msgs::SingleArmMotionGoalConstPtr& goal) 
        throw (SingleArmInitException, ConversionException)
    {
      // TODO(Georg): add some FSM
      ConstraintArray constraints = fromMsg(goal->constraints);

      if(!constraints.isValid())
        throw SingleArmInitException("Given constraints not valid. Aborting.");

      urdf::Model urdf;
      if(!urdf.initParam("robot_description"))
        throw SingleArmInitException("No urdf 'robot_description' on param-server.");

      KinematicChain kinematics = fromMsg(goal->kinematics, urdf);

      if(!kinematics.isValid())
        throw SingleArmInitException("Given kinematics not valid. Aborting.");

      initTFRequests(constraints.necessaryTransforms());

      initJointState(kinematics.semantics().joints());

      controller_.init(constraints, kinematics, cycle_time);

      initControllerGains(constraints);
    }

    void SingleArmController::start()
    {
      // TODO(Georg): add some FSM
      controller_.start(js_listener_.currentJointState(), 
          tf_worker_.currentTransforms(), delta_deriv, cycle_time); 
    }

    void SingleArmController::stop()
    {
      // TODO(Georg): add some FSM
      controller_.stop();
    }

    void SingleArmController::update()
    {
      // TODO(Georg): implement me
      // TODO(Georg): have me periodically called
      // TODO(Georg): add some FSM
    }

    void SingleArmController::commandGoalCallback()
    {
      // TODO(Georg): add some FSM
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

      start();
    }

    void SingleArmController::commandPreemptCallback()
    {
      // TODO(Georg): add some FSM
      stop();
      action_server_.setPreempted();
    }

    void SingleArmController::initTFRequests(const std::set<TransformSemantics> requests)
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

    void SingleArmController::loopTF()
    {
      // TODO(Georg): consider moving this into TFWorker which then spawns its own thread
      Rate sleep_rate(20.0);

      while(ros::ok())
      {
        sleep_rate.sleep();
        tf_worker_.lookupTransforms();
      }
    }

    void SingleArmController::initJointState(const JntArraySemantics& joints)
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

    void SingleArmController::initControllerGains(const ConstraintArray& constraints)
        throw (SingleArmInitException)
    {
     if(!constraints.isValid())
       throw SingleArmInitException("Given constraints not valid. Aborting.");
      
      PIDGains gains;
      gains.init(constraints.outputValues().semantics());
      // TODO(Georg): move these magic numbers somewhere clever
      gains.p().numerics().data.setConstant(100.0);
      gains.i().numerics().data.setConstant(20.0);
      gains.d().numerics().data.setConstant(0.0);
      gains.i_max().numerics().data.setConstant(0.0);
      gains.i_min().numerics().data.setConstant(0.0);

      controller_.setGains(gains);
    }
  } // namespace nodes
} // namespace fccl
