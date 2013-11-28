#include <fccl_nodes/SingleArmController.h>

namespace fccl
{
  namespace nodes
  {
    SingleArmController::SingleArmController(const NodeHandle& node_handle) :
        node_handle_(node_handle), action_server_(node_handle, "command", false),
        tf_worker_(), js_listener_(), qdot_publisher_(node_handle, "qdot"),
        js_subscriber_(), cycle_time_(0.01), delta_deriv_(0.001)
    {
      fsm_.start();

      action_server_.registerGoalCallback( bind( 
          &SingleArmController::commandGoalCallback, this ) );
      action_server_.registerPreemptCallback( bind( 
          &SingleArmController::commandPreemptCallback, this ) );
      action_server_.start();
      // TODO(Georg): publish feedback for visualization periodically
    }

    SingleArmController::~SingleArmController()
    {
    }

    void SingleArmController::run()
    {
      js_subscriber_ = node_handle_.subscribe<sensor_msgs::JointState>("joint_state",
          1, &SingleArmController::js_callback, this);
    }

    void SingleArmController::shutdown()
    {
      js_subscriber_.shutdown();
    }

    bool SingleArmController::init(const fccl_msgs::SingleArmMotionGoalConstPtr& goal) 
        throw ()
    {
      try
      {
        ConstraintArray constraints = fromMsg(goal->constraints);

        if(!constraints.isValid())
          throw SingleArmInitException("Given constraints not valid. Aborting.");

        urdf::Model urdf;
        if(!urdf.initParam("robot_description"))
          throw SingleArmInitException("No urdf 'robot_description' on param-server.");

        KinematicChain kinematics = fromMsg(goal->kinematics, urdf);

        if(!kinematics.isValid())
          throw SingleArmInitException("Given kinematics not valid. Aborting.");

        controller_.init(constraints, kinematics, cycle_time_);

        initTFRequests(controller_.necessaryTransforms());

        initJointState(kinematics.semantics().joints());

        initControllerGains(constraints);

        qdot_publisher_.resize(kinematics.semantics().joints());

        feedback_msg_.constraints.resize(constraints.size());
      }
      catch (std::exception& e)
      {
        ROS_INFO("Error during init of SingleArmController: '%s'", e.what());
        commandPreemptCallback();
        return false;
      }

      return true;
    }

    void SingleArmController::start() throw ()
    {
      controller_.start(js_listener_.currentJointState(), 
          tf_worker_.currentTransforms(), delta_deriv_, cycle_time_); 
    }

    void SingleArmController::stop() throw ()
    {
      controller_.stop();
      qdot_publisher_.publish(controller_.desiredJointVelocities());
    }

    void SingleArmController::update() throw()
    {
      controller_.update(js_listener_.currentJointState(), 
          tf_worker_.currentTransforms(), delta_deriv_, cycle_time_); 

      qdot_publisher_.publish(controller_.desiredJointVelocities());

      publishFeedback();
    }

    void SingleArmController::js_callback(const
       sensor_msgs::JointState::ConstPtr& msg)
    {
      // ensure FSM is running before...
      if(fsm_.isRunning())
        // ... sending the signal to update using the function-hook
        fsm_.process_event(UpdateEvent(
            boost::bind(&SingleArmController::update, this)));
    }
 
    void SingleArmController::commandGoalCallback() throw ()
    {
      fccl_msgs::SingleArmMotionGoalConstPtr goal = action_server_.acceptNewGoal();

      if(action_server_.isPreemptRequested())
      {
        commandPreemptCallback();
        return;
      }

      // send event 'Init' to FSM, and use our local init-function together with
      // ActionGoal as hook
      fsm_.process_event(InitEvent<fccl_msgs::SingleArmMotionGoalConstPtr>(
          boost::bind(&SingleArmController::init, this, _1), goal));

      // verifying that init worked properly and we're now in state 'Stopped'
      if(!fsm_.isStopped())
        return;

      // send event 'Start' to FSM and use our local start-function as hook
      fsm_.process_event(StartEvent(boost::bind(&SingleArmController::start, this)));
    }

    void SingleArmController::commandPreemptCallback() throw ()
    {
      // verify that we are running before signalling FSM to stop
      if(fsm_.isRunning())
        // send event 'Stop' to FSM and use our local stop-function as hook
        fsm_.process_event(StopEvent(boost::bind(&SingleArmController::stop, this)));

      action_server_.setPreempted();
    }

    void SingleArmController::publishFeedback() throw ()
    {
      toMsg(controller_.constraints(), feedback_msg_.constraints);
      action_server_.publishFeedback(feedback_msg_);
    }
 
    void SingleArmController::initTFRequests(const std::set<TransformSemantics> requests)
        throw (TFWorkerException)
    {
      // TODO(Georg): magic number
      // TODO(Georg): no longer needs its own method
      tf_worker_.init(requests, 0.1);
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
