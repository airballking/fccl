#ifndef FCCL_NODES_TF_WORKER_H
#define FCCL_NODES_TF_WORKER_H

// TODO(Georg): Move TransformDoubleBuffer into its own h-file for clarity
#include <fccl/utils/DoubleBuffer.h>
#include <boost/thread/mutex.hpp>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

using namespace fccl::semantics;
using namespace fccl::utils;

typedef std::set<TransformSemantics>::iterator SetIterator;
typedef std::set<TransformSemantics>::const_iterator ConstSetIterator;

class TFWorker
{
  public:
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
};
#endif // FCCL_NODES_TF_WORKER_H
