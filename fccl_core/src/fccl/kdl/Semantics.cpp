#include <fccl/kdl/Semantics.h>
#include <fccl/utils/Hashing.h>
#include <fccl/utils/Equalities.h>
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

    SemanticObject1xN::SemanticObject1xN()
    {
      semantic_type_ = SEMANTICS_1xN;
    }

    SemanticObject1xN::SemanticObject1xN(const SemanticObject1xN& other) :
        reference_id_(other.getReferenceID()), target_ids_(other.getTargetIDs())
    {
      semantic_type_ = SEMANTICS_1xN;
    }

    SemanticObject1xN::SemanticObject1xN(const std::string& reference_name, const
            std::vector<std::string>& target_names) :
        reference_id_(fu::hash(reference_name))
    {
      semantic_type_ = SEMANTICS_1xN;

      target_ids_.resize(target_names.size());
      for(unsigned int i=0; i<target_names.size(); i++)
        target_ids_[i] = fu::hash(target_names[i]);
    }


    SemanticObject1xN::SemanticObject1xN(std::size_t reference_id, const 
            std::vector<std::size_t>& target_ids) :
        reference_id_(reference_id), target_ids_(target_ids)
    {
      semantic_type_ = SEMANTICS_1xN;
    }

    SemanticObject1xN::~SemanticObject1xN()
    {
    }

    std::size_t SemanticObject1xN::getReferenceID() const
    {
      return reference_id_;
    }

    void SemanticObject1xN::setReferenceID(std::size_t reference_id)
    {
      reference_id_ = reference_id;
    }

    const std::string& SemanticObject1xN::getReferenceName() const
    {
      return fu::retrieveValue(reference_id_);
    }

    void SemanticObject1xN::setReferenceName(const std::string& reference_name)
    {
      reference_id_ = fu::hash(reference_name);
    }

    std::size_t SemanticObject1xN::getTargetID(std::size_t index) const
    {
      assert(targetIndexValid(index));

      return target_ids_[index];
    }

    void SemanticObject1xN::setTargetID(std::size_t index, std::size_t target_id)
    {
      assert(targetIndexValid(index));

      target_ids_[index] = target_id;
    }

    const std::string& SemanticObject1xN::getTargetName(std::size_t index) const
    {
      assert(targetIndexValid(index));

      return fu::retrieveValue(target_ids_[index]);
    }

    void SemanticObject1xN::setTargetName(std::size_t index, const std::string& target_name)
    {
      assert(targetIndexValid(index));
 
      target_ids_[index] = fu::hash(target_name);
    }

    const std::vector<std::size_t>& SemanticObject1xN::getTargetIDs() const
    {
      return target_ids_;
    }

    void SemanticObject1xN::setTargetIDs(const std::vector<std::size_t>& target_ids)
    {
      assert(targets() == target_ids.size());

      target_ids_ = target_ids;
    }

    std::vector<std::string> SemanticObject1xN::getTargetNames() const
    {
      std::vector<std::string> result;

      for(unsigned int i=0; i<result.size(); i++)
        result.push_back(getTargetName(i));

      return result;
    }

    void SemanticObject1xN::setTargetNames(const std::vector<std::string>& target_names)
    {
      assert(targets() == target_names.size());

      for(unsigned int i=0; i<targets(); i++)
        setTargetName(i, target_names[i]);
    }

    bool SemanticObject1xN::semanticsEqual(const SemanticObject& other) const
    {
      if(!semanticTypesEqual(other))
        return false;

      const SemanticObject1xN* other_p = static_cast<const SemanticObject1xN*>(&other);
      assert(other_p);     

      return (fu::Equal(getTargetIDs(), other_p->getTargetIDs()) 
          && (getReferenceID() == other_p->getReferenceID()));
    }

    std::size_t SemanticObject1xN::targets() const
    {
      return target_ids_.size();
    }

    void SemanticObject1xN::resizeTargets(std::size_t new_size)
    {
      target_ids_.resize(new_size);
    }
 
    bool SemanticObject1xN::targetIndexValid(std::size_t index) const
    {
      return (index < target_ids_.size());
    }

    std::size_t SemanticObject1xN::getTargetIndex(size_t target_id) const
    {
      for(std::size_t i=0; i<targets(); i++)
        if(target_ids_[i] == target_id)
          return i;

      return targets();
    }

    std::size_t SemanticObject1xN::getTargetIndex(const std::string& target_name) const
    {
      return getTargetIndex(fu::hash(target_name));
    }
  } // namespace kdl
} // namespace fccl
