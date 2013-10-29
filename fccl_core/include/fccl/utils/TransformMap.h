#ifndef FCCL_UTILS_TRANSFORM_MAP_H
#define FCCL_UTILS_TRANSFORM_MAP_H

#include <fccl/kdl/Transform.h>
#include <fccl/semantics/TransformSemantics.h>
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
            const fccl::semantics::TransformSemantics& semantics);
        const fccl::kdl::Transform& getTransform(
            const fccl::semantics::SemanticsBase& reference,
            const fccl::semantics::SemanticsBase& target);
        void removeTransform(const fccl::semantics::TransformSemantics& semantics);

        void clear();
        bool hasTransform(const fccl::semantics::TransformSemantics& semantics);
        bool hasTransform(const fccl::semantics::SemanticsBase& reference,
            const fccl::semantics::SemanticsBase& target);
 
        std::size_t size() const
        {
          return map_.size();
        }

        boost::mutex& getMutex(); 

      private:
        boost::mutex mutex_;
        std::map<fccl::semantics::TransformSemantics, fccl::kdl::Transform> map_;
    };
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_TRANSFORM_MAP_H
