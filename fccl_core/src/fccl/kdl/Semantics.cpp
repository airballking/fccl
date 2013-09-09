#include <fccl/kdl/Semantics.h>
#include <fccl/utils/Hashing.h>

namespace fu = fccl::utils;

namespace fccl
{
  namespace kdl
  {
    TwofoldSemanticObject::TwofoldSemanticObject()
    {
    }
  
    TwofoldSemanticObject::TwofoldSemanticObject(const TwofoldSemanticObject& other) :
        reference_id_(other.getReferenceID()), target_id_(other.getTargetID())
    {
    }

    TwofoldSemanticObject::TwofoldSemanticObject(const std::string& reference_name, const std::string& target_name) :
        reference_id_(fu::hash(reference_name)), target_id_(fu::hash(target_name))
    {
    }

    TwofoldSemanticObject::TwofoldSemanticObject(std::size_t reference_id, std::size_t target_id) :
        reference_id_(reference_id), target_id_(target_id)
    {
    }
  
    TwofoldSemanticObject::~TwofoldSemanticObject()
    {
    }
  
    std::size_t TwofoldSemanticObject::getReferenceID() const
    {
      return reference_id_;
    }

    void TwofoldSemanticObject::setReferenceID(std::size_t reference_id)
    {
      reference_id_ = reference_id;
    }
  
    const std::string& TwofoldSemanticObject::getReferenceName() const
    {
      return fu::retrieveValue(getReferenceID());
    }

    void TwofoldSemanticObject::setReferenceName(const std::string& reference_name)
    {
      reference_id_ = fu::hash(reference_name);
    }
  
    std::size_t TwofoldSemanticObject::getTargetID() const
    {
      return target_id_;
    }

    void TwofoldSemanticObject::setTargetID(std::size_t target_id)
    {
      target_id_ = target_id;
    }
  
    const std::string& TwofoldSemanticObject::getTargetName() const
    {
      return fu::retrieveValue(getTargetID());
    }

    void TwofoldSemanticObject::setTargetName(const std::string& target_name)
    {
      target_id_ = fu::hash(target_name);
    }
  
    bool TwofoldSemanticObject::semanticsEqual(const TwofoldSemanticObject& other) const
    {
      return (target_id_ == other.getTargetID()) && (reference_id_ == other.getReferenceID());
    }
  } // namespace kdl
} // namespace fccl
