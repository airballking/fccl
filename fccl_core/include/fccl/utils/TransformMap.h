#ifndef FCCL_UTILS_TRANSFORM_MAP_H
#define FCCL_UTILS_TRANSFORM_MAP_H

#include <fccl/kdl/Transform.h>
#include <fccl/semantics/TransformSemantics.h>
#include <boost/thread/mutex.hpp>
#include <map>
#include <vector>

namespace fccl
{
  namespace utils
  {
    class TransformMap 
    {
      public:
        void setTransform(const fccl::kdl::Transform& transform);
        const fccl::kdl::Transform& getTransform(
            const fccl::semantics::TransformSemantics& semantics) const;
        const fccl::kdl::Transform& getTransform(
            const fccl::semantics::SemanticsBase& reference,
            const fccl::semantics::SemanticsBase& target) const;
        void removeTransform(const fccl::semantics::TransformSemantics& semantics);

        void clear();
        bool hasTransform(const fccl::semantics::TransformSemantics& semantics) const;
        bool hasTransform(const fccl::semantics::SemanticsBase& reference,
            const fccl::semantics::SemanticsBase& target) const;
 
        std::size_t size() const
        {
          return map_.size();
        }

        // NOT REAL-TIME-SAFE
        std::vector<fccl::kdl::Transform> allTransforms() const;
        // TODO(Georg): Is this still necessary?
        boost::mutex& getMutex() const; 

      private:
        mutable boost::mutex mutex_;
        std::map<fccl::semantics::TransformSemantics, fccl::kdl::Transform> map_;
    };
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_TRANSFORM_MAP_H
