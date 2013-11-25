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

// TODO(Georg): into namespace fccl::nodes

class TFWorkerException : public std::runtime_error
{
  public:
    TFWorkerException(const std::string& message) : 
        std::runtime_error(message) {}
};
 
class TFWorker
{
  public:
    TFWorker() :
        internal_mutex_(), external_mutex_(), requests_(), buffer_(),
        tf_listener_(), tf_transform_(), thread_(NULL)
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

    void init(const std::set<TransformSemantics>& requests, double timeout)
        throw (TFWorkerException)
    {
      boost::mutex::scoped_lock scoped_lock(external_mutex_);

      initRequests(requests);
 
      waitForBuffersToFill(timeout);
    }

    const std::set<TransformSemantics>& currentRequests() const
    {
      boost::mutex::scoped_lock scoped_lock(external_mutex_);
 
      return requests_;
    }

    bool allRequestsFound() const
    {
      boost::mutex::scoped_lock scoped_lock(external_mutex_);
 
      return inBufferHasAllRequests() && outBufferHasAllRequests();
    }

    const TransformMap& currentTransforms()
    {
      boost::mutex::scoped_lock scoped_lock(external_mutex_);
 
      // TODO(Georg): move 'swap' somewhere else ??
      buffer_.swap();
      return buffer_.outBuffer(); 
    }

  private:
    mutable boost::mutex internal_mutex_, external_mutex_;
    boost::thread* thread_;
    std::set<TransformSemantics> requests_;
    TransformDoubleBuffer buffer_;
    tf::TransformListener tf_listener_;
    // pre-allocated memory for tf-lookup
    tf::StampedTransform tf_transform_;

    void waitForBuffersToFill(double timeout)
        throw (TFWorkerException)
    {
      // fill buffer #1
      waitForInputBufferToFill(timeout/2.0);

      // fill buffer #2
      swapBuffers();
      waitForInputBufferToFill(timeout/2.0);
    }

    void waitForInputBufferToFill(double timeout)
        throw (TFWorkerException)
    {
      ros::Time timeout_time = ros::Time::now() + ros::Duration(timeout);
      ros::Duration short_time(0.01);
 
      do
      {
        short_time.sleep();
      } 
      while((ros::Time::now() < timeout_time) && !inBufferHasAllRequests());

      if(!inBufferHasAllRequests())
        throw TFWorkerException("TF timedout while initializing. Aborting.");
    }
 
    void initRequests(const std::set<TransformSemantics>& requests)
    {
      boost::mutex::scoped_lock scoped_lock(internal_mutex_);

      requests_.clear();
      requests_.insert(requests.begin(), requests.end());

      buffer_.clear();
    }
 
    bool inBufferHasAllRequests() const
    {
      boost::mutex::scoped_lock scoped_lock(internal_mutex_);
      
      for(ConstSetIterator it=requests_.begin();
          it!=requests_.end(); ++it)
        if(!buffer_.inBuffer().hasTransform(*it))
          return false;

      return true;
    }

    bool outBufferHasAllRequests() const
    {
      boost::mutex::scoped_lock scoped_lock(internal_mutex_);
 
      for(ConstSetIterator it=requests_.begin();
          it!=requests_.end(); ++it)
        if(!buffer_.outBuffer().hasTransform(*it))
          return false;

      return true;
    }

    void swapBuffers()
    {
      boost::mutex::scoped_lock scoped_lock(internal_mutex_);
      
      buffer_.swap();
    }

    void lookupTransforms()
    {
     boost::mutex::scoped_lock scoped_lock(internal_mutex_);
 
     for(ConstSetIterator it=requests_.begin();
          it!=requests_.end(); ++it)
        lookupTransform(*it);
    }

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
