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
    }

    ~SingleArmController()
    {
      destroyTFThread();
    }

    bool init()
    {
      createTFThread();
    }

    void start() {}

    void stop() {}

    void update() {}

  private:
    thread* tf_thread_;    

    void createTFThread()
    {
      assert(!tf_thread_);

      tf_thread_ = new thread( bind( &SingleArmController::workTF, this) );
    }    

    void destroyTFThread()
    {
      if(tf_thread_)
      {
        tf_thread_->join();
        delete tf_thread_;
      }
    } 

    void workTF()
    {
      int a;
      while(ros::ok())
        a = 1;
    }
};
#endif // FCCL_NODES_SINGLE_ARM_CONTROLLER_H
