#ifndef FCCL_UTILS_TRANSFORM_MAP_H
#define FCCL_UTILS_TRANSFORM_MAP_H

#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/Transform.h>
#include <boost/thread/mutex.hpp>
#include <map>

namespace fccl
{
  namespace utils
  {
    class TransformMap 
    {
      public:
        void setTransform(const fccl::kdl::Transform& transform);
        const fccl::kdl::Transform& getTransform(
            const fccl::kdl::SemanticObject1x1& semantics);
        void removeTransform(const fccl::kdl::SemanticObject1x1& semantics);
        void clear();
        bool hasTransform(const fccl::kdl::SemanticObject1x1& semantics);

        boost::mutex& getMutex(); 

      private:
        boost::mutex mutex_;
        std::map<fccl::kdl::SemanticObject1x1, fccl::kdl::Transform> map_;
    };
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_TRANSFORM_MAP_H
