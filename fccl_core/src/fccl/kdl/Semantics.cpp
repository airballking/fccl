#include <fccl/kdl/Semantics.h>
#include <fccl/utils/Hashing.h>
#include <assert.h>

namespace fu = fccl::utils;

namespace fccl
{
  namespace kdl
  {
    int SemanticObject::getSemanticType() const
    {
      return semantic_type_;
    }
 
    bool SemanticObject::semanticTypesEqual(const SemanticObject& other) const
    {
      return (getSemanticType() == other.getSemanticType());
    }

    SemanticObject1x1::SemanticObject1x1()
    {
      semantic_type_ = SEMANTICS_1x1;
    }
  
    SemanticObject1x1::SemanticObject1x1(const SemanticObject1x1& other) :
        reference_id_(other.getReferenceID()), target_id_(other.getTargetID())
    {
      semantic_type_ = SEMANTICS_1x1;
    }

    SemanticObject1x1::SemanticObject1x1(const std::string& reference_name, const std::string& target_name) :
        reference_id_(fu::hash(reference_name)), target_id_(fu::hash(target_name))
    {
      semantic_type_ = SEMANTICS_1x1;
    }

    SemanticObject1x1::SemanticObject1x1(std::size_t reference_id, std::size_t target_id) :
        reference_id_(reference_id), target_id_(target_id)
    {
      semantic_type_ = SEMANTICS_1x1;
    }
  
    SemanticObject1x1::~SemanticObject1x1()
    {
    }
  
    std::size_t SemanticObject1x1::getReferenceID() const
    {
      return reference_id_;
    }

    void SemanticObject1x1::setReferenceID(std::size_t reference_id)
    {
      reference_id_ = reference_id;
    }
  
    const std::string& SemanticObject1x1::getReferenceName() const
    {
      return fu::retrieveValue(getReferenceID());
    }

    void SemanticObject1x1::setReferenceName(const std::string& reference_name)
    {
      reference_id_ = fu::hash(reference_name);
    }
  
    std::size_t SemanticObject1x1::getTargetID() const
    {
      return target_id_;
    }

    void SemanticObject1x1::setTargetID(std::size_t target_id)
    {
      target_id_ = target_id;
    }
  
    const std::string& SemanticObject1x1::getTargetName() const
    {
      return fu::retrieveValue(getTargetID());
    }

    void SemanticObject1x1::setTargetName(const std::string& target_name)
    {
      target_id_ = fu::hash(target_name);
    }
  
    bool SemanticObject1x1::semanticsEqual(const SemanticObject& other) const
    {
      if(!semanticTypesEqual(other))
        return false;

      const SemanticObject1x1* other_p = static_cast<const SemanticObject1x1*>(&other);
      assert(other_p);     

      return (getTargetID() == other_p->getTargetID()) && (getReferenceID() == other_p->getReferenceID());
    }
  } // namespace kdl
} // namespace fccl
