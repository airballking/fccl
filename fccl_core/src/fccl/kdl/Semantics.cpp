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
        semantics_(other.getReferenceID(), other.getTargetID())
    {
      semantic_type_ = SEMANTICS_1x1;
    }

    SemanticObject1x1::SemanticObject1x1(const std::string& reference_name, 
        const std::string& target_name)
    {
      semantic_type_ = SEMANTICS_1x1;
      setReferenceName(reference_name);
      setTargetName(target_name);
    }


    SemanticObject1x1::SemanticObject1x1(std::size_t reference_id, std::size_t target_id) :
        semantics_(reference_id, target_id)
    {
      semantic_type_ = SEMANTICS_1x1;
    }


    SemanticObject1x1::~SemanticObject1x1()
    {
    }
  
    SemanticObject1x1& SemanticObject1x1::operator=(const SemanticObject1x1& other)
    {
      if(this != &other)
      {
        setReferenceID(other.getReferenceID());
        setTargetID(other.getTargetID());
      }

      return *this;
    }

    const SemanticObject1x1& SemanticObject1x1::getSemantics() const
    {
      return *this;
    }

    void SemanticObject1x1::setSemantics(const SemanticObject1x1& semantics)
    {
      *this = semantics;
    }
 
    std::size_t SemanticObject1x1::getReferenceID() const
    {
      return semantics_.first;
    }

    void SemanticObject1x1::setReferenceID(std::size_t reference_id)
    {
      semantics_.first = reference_id;
    }
  
    const std::string& SemanticObject1x1::getReferenceName() const
    {
      return fu::retrieveValue(getReferenceID());
    }

    void SemanticObject1x1::setReferenceName(const std::string& reference_name)
    {
      setReferenceID(fu::hash(reference_name));
    }
  
    std::size_t SemanticObject1x1::getTargetID() const
    {
      return semantics_.second;
    }

    void SemanticObject1x1::setTargetID(std::size_t target_id)
    {
      semantics_.second = target_id;
    }
  
    const std::string& SemanticObject1x1::getTargetName() const
    {
      return fu::retrieveValue(getTargetID());
    }

    void SemanticObject1x1::setTargetName(const std::string& target_name)
    {
      setTargetID(fu::hash(target_name));
    }
  
    bool SemanticObject1x1::semanticsEqual(const SemanticObject& other) const
    {
      if(!semanticTypesEqual(other))
        return false;

      const SemanticObject1x1* other_p = static_cast<const SemanticObject1x1*>(&other);
      assert(other_p);     

      return (getTargetID() == other_p->getTargetID()) && (getReferenceID() == other_p->getReferenceID());
    }

    bool SemanticObject1x1::operator<(const SemanticObject1x1& rhs) const
    {
      // THIS SCREAMS FOR REFACTORING:
      // (1) have semanticInterface, where Semantic1x1 inherits from this
      // (2) have semanticObjectInterface, which holds a SemanticInterface
      return semantics_ < rhs.semantics_; 
    }

    SemanticObject1x1 SemanticObject1x1::inverse() const
    {
      SemanticObject1x1 result;
      result.setReferenceID(getTargetID());
      result.setTargetID(getReferenceID());
      return result;
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

    SemanticObject1xN& SemanticObject1xN::operator=(const SemanticObject1xN& other)
    {
      if(this != &other)
      {
        assert(size() == other.size());
        
        setReferenceID(other.getReferenceID());
        setTargetIDs(other.getTargetIDs());
      }

      return *this;
    }

    const SemanticObject1xN& SemanticObject1xN::getSemantics() const
    {
      return *this;
    }

    void SemanticObject1xN::setSemantics(const SemanticObject1xN& semantics)
    {
      *this = semantics;
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
      assert(size() == target_ids.size());

      target_ids_ = target_ids;
    }

    std::vector<std::string> SemanticObject1xN::getTargetNames() const
    {
      std::vector<std::string> result;

      for(unsigned int i=0; i<size(); i++)
        result.push_back(getTargetName(i));

      return result;
    }

    void SemanticObject1xN::setTargetNames(const std::vector<std::string>& target_names)
    {
      assert(size() == target_names.size());

      for(unsigned int i=0; i<size(); i++)
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

    std::size_t SemanticObject1xN::size() const
    {
      return target_ids_.size();
    }

    void SemanticObject1xN::resize(std::size_t new_size)
    {
      target_ids_.resize(new_size);
    }
 
    bool SemanticObject1xN::targetIndexValid(std::size_t index) const
    {
      return (index < target_ids_.size());
    }

    std::size_t SemanticObject1xN::getTargetIndex(size_t target_id) const
    {
      for(std::size_t i=0; i<size(); i++)
        if(target_ids_[i] == target_id)
          return i;

      return size();
    }

    std::size_t SemanticObject1xN::getTargetIndex(const std::string& target_name) const
    {
      return getTargetIndex(fu::hash(target_name));
    }

    std::ostream& operator<<(std::ostream& os, const SemanticObject1xN& semantics)
    {
      os << "reference: " << semantics.getReferenceName() << " (";
      os << semantics.getReferenceID() << ")\n";
      os << "targets: \n";
      for(std::size_t i=0; i<semantics.size(); i++)
      {
        os << semantics.getTargetName(i) << " (";
        os << semantics.getTargetID(i) << ")";
        if(i < (semantics.size() - 1))
          os << "\n";
      }

      return os;
    }

    SemanticObjectNxM::SemanticObjectNxM()
    {
      semantic_type_ = SEMANTICS_NxM;
    }

    SemanticObjectNxM::SemanticObjectNxM(const SemanticObjectNxM& other) :
        reference_ids_(other.getReferenceIDs()), target_ids_(other.getTargetIDs())
    {
      semantic_type_ = SEMANTICS_NxM;
    }

   
    SemanticObjectNxM::SemanticObjectNxM(const std::vector<std::string>& reference_names, const std::vector<std::string>& target_names)
    {
      semantic_type_ = SEMANTICS_NxM;
      setReferenceNames(reference_names);
      setTargetNames(target_names);
    }


    SemanticObjectNxM::SemanticObjectNxM(const std::vector<std::size_t>& reference_ids, const std::vector<std::size_t>& target_ids) :
        reference_ids_(reference_ids), target_ids_(target_ids)
    {
      semantic_type_ = SEMANTICS_NxM;
    }

    SemanticObjectNxM::~SemanticObjectNxM()
    {
    }
 
    SemanticObjectNxM& SemanticObjectNxM::operator=(const SemanticObjectNxM& other)
    {
      // protect against self-assignment
      if(this != &other)
      {
        setReferenceIDs(other.getReferenceIDs());
        setTargetIDs(other.getTargetIDs());
      }

      return *this;
    }
 
    const SemanticObjectNxM& SemanticObjectNxM::getSemantics() const
    {
      return *this;
    }
 
    void SemanticObjectNxM::setSemantics(const SemanticObjectNxM& semantics)
    {
      *this = semantics;
    }
 
    std::size_t SemanticObjectNxM::getReferenceID(std::size_t index) const
    {
      assert(referenceIndexValid(index));

      return reference_ids_[index];
    }

    void SemanticObjectNxM::setReferenceID(std::size_t index, std::size_t reference_id)
    {
      assert(referenceIndexValid(index));

      reference_ids_[index] = reference_id;
    }
 
    const std::string& SemanticObjectNxM::getReferenceName(std::size_t index) const
    {
      return fu::retrieveValue(getReferenceID(index));
    }

    void SemanticObjectNxM::setReferenceName(std::size_t index, const std::string& reference_name)
    {
      setReferenceID(index, fu::hash(reference_name));
    }
 
    const std::vector<std::size_t>& SemanticObjectNxM::getReferenceIDs() const
    {
      return reference_ids_;
    }

    void SemanticObjectNxM::setReferenceIDs(const std::vector<std::size_t>& reference_ids)
    {
      assert(sizeReferences() == reference_ids.size());

      reference_ids_ = reference_ids;
    }

    std::vector<std::string> SemanticObjectNxM::getReferenceNames() const
    {
      std::vector<std::string> result;

      for(std::size_t i=0; i<sizeReferences(); i++)
        result.push_back(getReferenceName(i));

      return result;
    }

    void SemanticObjectNxM::setReferenceNames(const std::vector<std::string>& reference_names)
    {
      assert(sizeReferences() == reference_names.size());

      for(std::size_t i=0; i<sizeReferences(); i++)
        setReferenceName(i, reference_names[i]);
    }

    std::size_t SemanticObjectNxM::getTargetID(std::size_t index) const
    {
      assert(targetIndexValid(index));
      
      return target_ids_[index];
    }

    void SemanticObjectNxM::setTargetID(std::size_t index, std::size_t target_id)
    {
      assert(targetIndexValid(index));

      target_ids_[index] = target_id;
    }
 
    const std::string& SemanticObjectNxM::getTargetName(std::size_t index) const
    {
      return fu::retrieveValue(getTargetID(index));
    }

    void SemanticObjectNxM::setTargetName(std::size_t index, const std::string& target_name)
    {
      setTargetID(index, fu::hash(target_name));
    }
 
    const std::vector<std::size_t>& SemanticObjectNxM::getTargetIDs() const
    {
      return target_ids_;
    }

    void SemanticObjectNxM::setTargetIDs(const std::vector<std::size_t>& target_ids)
    {
      assert(sizeTargets() == target_ids.size());

      target_ids_ = target_ids;
    }
 
    std::vector<std::string> SemanticObjectNxM::getTargetNames() const
    {
      std::vector<std::string> result;

      for(std::size_t i=0; i<sizeTargets(); i++)
        result.push_back(getTargetName(i));

      return result;
    }

    void SemanticObjectNxM::setTargetNames(const std::vector<std::string>& target_names)
    {
      assert(sizeTargets() == target_names.size());

      for(std::size_t i=0; i<sizeTargets(); i++)
        setTargetName(i, target_names[i]);
    }
 
    bool SemanticObjectNxM::semanticsEqual(const SemanticObject& other) const
    {
      if(!semanticTypesEqual(other))
        return false;

      const SemanticObjectNxM* other_p = static_cast<const SemanticObjectNxM*>(&other);
      assert(other_p);     

      return (fu::Equal(getTargetIDs(), other_p->getTargetIDs()) 
          && (fu::Equal(getReferenceIDs(), other_p->getReferenceIDs())));
    }
 
    std::pair<std::size_t, std::size_t> SemanticObjectNxM::size() const
    {
      return std::pair<std::size_t, std::size_t>(sizeTargets(), sizeReferences());
    }

    void SemanticObjectNxM::resize(const std::pair<std::size_t, std::size_t>& new_size)
    {
      resizeTargets(new_size.first);
      resizeReferences(new_size.second);
    }
 
    std::size_t SemanticObjectNxM::sizeReferences() const
    {
      return reference_ids_.size();
    }

    void SemanticObjectNxM::resizeReferences(std::size_t new_size)
    {
      reference_ids_.resize(new_size);
    }
 
    std::size_t SemanticObjectNxM::sizeTargets() const
    {
      return target_ids_.size();
    }

    void SemanticObjectNxM::resizeTargets(std::size_t new_size)
    {
      target_ids_.resize(new_size);
    }
 
    bool SemanticObjectNxM::referenceIndexValid(std::size_t index) const
    {
      return index < sizeReferences();
    }

    std::size_t SemanticObjectNxM::getReferenceIndex(size_t reference_id) const
    {
      for(std::size_t i=0; i<sizeReferences(); i++)
        if(getReferenceID(i) == reference_id)
          return i;

      return sizeReferences();
    }

    std::size_t SemanticObjectNxM::getReferenceIndex(const std::string& reference_name) const
    {
      return getReferenceIndex(fu::hash(reference_name));
    } 
 
    bool SemanticObjectNxM::targetIndexValid(std::size_t index) const
    {
      return index < sizeTargets();
    }

    std::size_t SemanticObjectNxM::getTargetIndex(size_t target_id) const
    {
      for(std::size_t i=0; i<sizeTargets(); i++)
        if(getTargetID(i) == target_id)
          return i;

      return sizeTargets();
    }

    std::size_t SemanticObjectNxM::getTargetIndex(const std::string& target_name) const
    {
      return getTargetIndex(fu::hash(target_name));
    }
 
    std::ostream& operator<<(std::ostream& os, const SemanticObjectNxM& semantics)
    {
      os << "references: \n";
      for(std::size_t i=0; i<semantics.sizeReferences(); i++)
      {
        os << semantics.getReferenceName(i) << " (";
        os << semantics.getReferenceID(i) << ")";
      }
      os << "targets: \n";
      for(std::size_t i=0; i<semantics.sizeTargets(); i++)
      {
        os << semantics.getTargetName(i) << " (";
        os << semantics.getTargetID(i) << ")";
        if(i < (semantics.sizeTargets() - 1))
          os << "\n";
      }

      return os;
    }
 
    SemanticObjectN::SemanticObjectN()
    {
      semantic_type_ = SEMANTICS_N;
    }

    SemanticObjectN::SemanticObjectN(const SemanticObjectN& other) :
        target_ids_(other.getTargetIDs())
    {
      semantic_type_ = SEMANTICS_N;
    }

    SemanticObjectN::SemanticObjectN(const std::vector<std::size_t>& target_ids) :
        target_ids_(target_ids)
    {
      semantic_type_ = SEMANTICS_N;
    }

    SemanticObjectN::SemanticObjectN(const std::vector<std::string>& target_names)
    {
      semantic_type_ = SEMANTICS_N;

      resize(target_names.size());
      setTargetNames(target_names);
    }

    SemanticObjectN::~SemanticObjectN()
    {
    }

    SemanticObjectN& SemanticObjectN::operator=(const SemanticObjectN& other)
    {
      if(this != &other)
      {
        assert(size() == other.size());

        setTargetIDs(other.getTargetIDs());
      }

      return *this;
    }

    const SemanticObjectN& SemanticObjectN::getSemantics() const
    {
      return *this;
    }

    void SemanticObjectN::setSemantics(const SemanticObjectN& semantics)
    {
      *this = semantics;
    }
 
    std::size_t SemanticObjectN::getTargetID(std::size_t index) const
    {
      assert(indexValid(index));

      return target_ids_[index];
    }

    void SemanticObjectN::setTargetID(std::size_t index, std::size_t target_id)
    {
      assert(indexValid(index));

      target_ids_[index] = target_id;
    }

    const std::string& SemanticObjectN::getTargetName(std::size_t index) const
    {
      assert(indexValid(index));

      return fu::retrieveValue(target_ids_[index]);
    }

    void SemanticObjectN::setTargetName(std::size_t index, const std::string& target_name)
    {
      assert(indexValid(index));
 
      target_ids_[index] = fu::hash(target_name);
    }

    const std::vector<std::size_t>& SemanticObjectN::getTargetIDs() const
    {
      return target_ids_;
    }

    void SemanticObjectN::setTargetIDs(const std::vector<std::size_t>& target_ids)
    {
      assert(size() == target_ids.size());

      target_ids_ = target_ids;
    }

    std::vector<std::string> SemanticObjectN::getTargetNames() const
    {
      std::vector<std::string> result;

      for(unsigned int i=0; i<size(); i++)
        result.push_back(getTargetName(i));

      return result;
    }

    void SemanticObjectN::setTargetNames(const std::vector<std::string>& target_names)
    {
      assert(size() == target_names.size());

      for(unsigned int i=0; i<size(); i++)
        setTargetName(i, target_names[i]);
    }

    bool SemanticObjectN::semanticsEqual(const SemanticObject& other) const
    {
      if(!semanticTypesEqual(other))
        return false;

      const SemanticObjectN* other_p = static_cast<const SemanticObjectN*>(&other);
      assert(other_p);     

      return fu::Equal(getTargetIDs(), other_p->getTargetIDs());
    }

    std::size_t SemanticObjectN::size() const
    {
      return target_ids_.size();
    }

    void SemanticObjectN::resize(std::size_t new_size)
    {
      target_ids_.resize(new_size);
    }
 
    bool SemanticObjectN::indexValid(std::size_t index) const
    {
      return (index < target_ids_.size());
    }

    std::size_t SemanticObjectN::getIndex(size_t target_id) const
    {
      for(std::size_t i=0; i<size(); i++)
        if(target_ids_[i] == target_id)
          return i;

      return size();
    }

    std::size_t SemanticObjectN::getIndex(const std::string& target_name) const
    {
      return getIndex(fu::hash(target_name));
    }
 
  } // namespace kdl
} // namespace fccl
