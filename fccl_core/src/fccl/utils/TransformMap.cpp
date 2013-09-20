#include <fccl/utils/TransformMap.h>

namespace fccl
{
  namespace utils
  {

    typedef std::pair<fccl::semantics::TransformSemantics, fccl::kdl::Transform> EntryType;
    typedef std::map<fccl::semantics::TransformSemantics, fccl::kdl::Transform> MapType;

    void TransformMap::setTransform(const fccl::kdl::Transform& transform)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      std::pair<MapType::iterator, bool> insert_result =
        map_.insert(EntryType(transform.semantics(), transform));

      if(!insert_result.second)
        // key was already present, we want to update
        insert_result.first->second = transform;
    }

    const fccl::kdl::Transform& TransformMap::getTransform(
        const fccl::semantics::TransformSemantics& semantics)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      return map_.at(semantics);
    }

    void TransformMap::removeTransform(const fccl::semantics::TransformSemantics& semantics)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());
     
      map_.erase(semantics); 
    }

    void TransformMap::clear()
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      map_.clear();
    }

    bool TransformMap::hasTransform(const fccl::semantics::TransformSemantics& semantics)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());
     
      return map_.find(semantics) != map_.end();
    }

    INLINE 
    boost::mutex& TransformMap::getMutex()
    {
      return mutex_;
    }
  } // namespace utils
} // namespace fccl
