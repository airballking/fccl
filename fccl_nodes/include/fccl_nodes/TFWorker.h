#ifndef FCCL_NODES_TF_WORKER_H
#define FCCL_NODES_TF_WORKER_H

// TODO(Georg): Move TransformDoubleBuffer into its own h-file for clarity
#include <fccl/utils/DoubleBuffer.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <tf_conversions/tf_kdl.h>
// TODO(Georg): move to TF2 and use BufferClient
#include <tf/transform_listener.h>
#include <ros/ros.h>

using namespace fccl::semantics;
using namespace fccl::utils;

typedef std::set<TransformSemantics>::iterator SetIterator;
typedef std::set<TransformSemantics>::const_iterator ConstSetIterator;

class TFWorker
{
  public:
    TFWorker() :
        mutex_(), requests_(), buffer_(), tf_listener_(), tf_transform_(),
        thread_(NULL)
    {
      thread_ = new boost::thread( boost::bind( &TFWorker::loopTF, this ) );
    }

    ~TFWorker()
    {
      if(thread_)
      {
        thread_->interrupt();
        thread_->join();
        delete thread_;
        thread_ = NULL;
      }
    }

    void addRequests(const std::set<TransformSemantics>& transforms)
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      requests_.insert(transforms.begin(), transforms.end());
    }

    const std::set<TransformSemantics>& currentRequests() const
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      return requests_;
    }

    void removeRequests(const std::set<TransformSemantics>& transforms)
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      requests_.erase(transforms.begin(), transforms.end());
    }

    void lookupTransforms()
    {
     boost::mutex::scoped_lock scoped_lock(mutex_);
 
     for(ConstSetIterator it=requests_.begin();
          it!=requests_.end(); ++it)
        lookupTransform(*it);
    }

    bool allRequestsFound() const
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      for(ConstSetIterator it=requests_.begin();
          it!=requests_.end(); ++it)
        if(!buffer_.inBuffer().hasTransform(*it))
          return false;

      return true;
    }

    const TransformMap& currentTransforms()
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      buffer_.swap();
      return buffer_.outBuffer(); 
    }

    void clear()
    {
      boost::mutex::scoped_lock scoped_lock(mutex_);
 
      requests_.clear();
 
      buffer_.clear();
    }

  private:
    mutable boost::mutex mutex_;
    boost::thread* thread_;
    std::set<TransformSemantics> requests_;
    TransformDoubleBuffer buffer_;
    tf::TransformListener tf_listener_;
    // pre-allocated memory for tf-lookup
    tf::StampedTransform tf_transform_;

    void lookupTransform(const TransformSemantics& request)
    {
      fccl::kdl::Transform transform;
      try
      {
        tf_listener_.lookupTransform(request.reference().getName(),
            request.target().getName(), ros::Time(0), tf_transform_);
      }
      catch (tf::TransformException ex)
      {
        return;
      }

      tf::transformTFToKDL(tf_transform_, transform.numerics());
      transform.semantics() = request;
 
      buffer_.inBuffer().setTransform(transform);
    }

    void loopTF()
    {
      ros::Rate sleep_rate(20.0);

      while(ros::ok())
      {
        sleep_rate.sleep();
        lookupTransforms();
      }
    }
};
#endif // FCCL_NODES_TF_WORKER_H
