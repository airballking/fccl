#ifndef FCCL_NODES_SINGLE_ARM_CONTROLLER_H
#define FCCL_NODES_SINGLE_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <iostream>

using namespace boost;

class SingleArmController
{
  public:
    SingleArmController() : tf_thread_(NULL)
    {
      tf_thread_ = new thread( bind( &SingleArmController::loopTF, this) );
    }

    ~SingleArmController()
    {
      if(tf_thread_)
      {
        tf_thread_->join();
        delete tf_thread_;
        tf_thread_ = NULL;
      }
    }

    bool init()
    {
      return false;
    }

    void start()
    {
    }

    void stop()
    {
    }

    void update()
    {
    }

  private:
    thread* tf_thread_;    

    bool lookupTF()
    {

    }

    void loopTF()
    {
      while(ros::ok())
        lookupTF();
    }
};
#endif // FCCL_NODES_SINGLE_ARM_CONTROLLER_H
