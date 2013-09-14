#include <fccl/utils/TransformMap.h>

namespace fccl
{
  namespace utils
  {

    typedef std::pair<fccl::kdl::SemanticObject1x1, fccl::kdl::Transform> EntryType;
    typedef std::map<fccl::kdl::SemanticObject1x1, fccl::kdl::Transform> MapType;

    void TransformMap::setTransform(const fccl::kdl::Transform& transform)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      std::pair<MapType::iterator, bool> insert_result =
        map_.insert(EntryType(transform.getSemantics(), transform));

      if(!insert_result.second)
        // key was already present, we want to update
        insert_result.first->second = transform;
    }

    const fccl::kdl::Transform& TransformMap::getTransform(
        const fccl::kdl::SemanticObject1x1& semantics)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      return map_.at(semantics);
    }

    void TransformMap::removeTransform(const fccl::kdl::SemanticObject1x1& semantics)
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());
     
      map_.erase(semantics); 
    }

    void TransformMap::clear()
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      map_.clear();
    }

    INLINE 
    boost::mutex& TransformMap::getMutex()
    {
      return mutex_;
    }
  } // namespace utils
} // namespace fccl
