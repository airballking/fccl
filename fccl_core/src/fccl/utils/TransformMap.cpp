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
        const fccl::semantics::TransformSemantics& semantics) const
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());

      return map_.at(semantics);
    }

    const fccl::kdl::Transform& TransformMap::getTransform(
        const fccl::semantics::SemanticsBase& reference,
        const fccl::semantics::SemanticsBase& target) const
    {
      fccl::semantics::TransformSemantics semantics;
      semantics.reference() = reference;
      semantics.target() = target;

      return getTransform(semantics);
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

    bool TransformMap::hasTransform(const fccl::semantics::TransformSemantics& semantics) const
    {
      boost::mutex::scoped_lock scoped_lock(getMutex());
     
      return map_.find(semantics) != map_.end();
    }

    bool TransformMap::hasTransform(const fccl::semantics::SemanticsBase& reference,
        const fccl::semantics::SemanticsBase& target) const
    {
      fccl::semantics::TransformSemantics semantics;
      semantics.reference() = reference;
      semantics.target() = target;

      return hasTransform(semantics);
    }

    // NOT REAL-TIME-SAFE
    std::vector<fccl::kdl::Transform> TransformMap::allTransforms() const
    {
      std::vector<fccl::kdl::Transform> result;
// TODO(Georg): typedefs
      for (std::map<fccl::semantics::TransformSemantics, fccl::kdl::Transform>::const_iterator it=map_.begin(); it!=map_.end(); ++it)
        result.push_back(it->second);
      return result;
    }
 
    INLINE 
    boost::mutex& TransformMap::getMutex() const
    {
      return mutex_;
    }
  } // namespace utils
} // namespace fccl
