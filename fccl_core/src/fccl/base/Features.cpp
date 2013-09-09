#include <fccl/base/Features.h>
#include <fccl/utils/Hashing.h>

namespace fu = fccl::utils;

namespace fccl
{
  namespace base
  {
    Feature::Feature()
    {
      type_ = UNKNOWN_FEATURE;
    }
  
    Feature::Feature(const Feature& other) :
        id_(other.getID()), position_(other.getPosition()), 
        orientation_(other.getOrientation()), type_(other.getType())
    {
      assert(featureTypeValid(other.getType()));
    }
  
    Feature::Feature(std::size_t id, const fccl::kdl::Vector& position,
        const fccl::kdl::Vector& orientation, FeatureTypes type) :
        id_(id), position_(position), orientation_(orientation), type_(type)
    {
      assert(featureTypeValid(type));
    }
   
    Feature::Feature(const std::string& name, const fccl::kdl::Vector& position,
        const fccl::kdl::Vector& orientation, FeatureTypes type) :
        id_(fu::hash(name)), position_(position), orientation_(orientation), type_(type)
    {
      assert(featureTypeValid(type));
    }
  
    Feature::~Feature() {}
  
    Feature& Feature::operator=(const Feature& rhs)
    {
      // protect against self-assignement
      if(this != &rhs)
      {
        setID(rhs.getID());
        setPosition(rhs.getPosition());
        setOrientation(rhs.getOrientation());
        setType(rhs.getType());
      }
  
      return *this;
    }
  
    std::size_t Feature::getID() const
    {
      return id_;
    }
  
    void Feature::setName(const std::string& name)
    {
      id_ = fu::hash(name);
    }
  
    void Feature::setID(std::size_t id)
    {
      id_ = id;
    }
  
    const fccl::kdl::Vector& Feature::getPosition() const
    {
      return position_;
    }
  
    void Feature::setPosition(const fccl::kdl::Vector& position)
    {
      position_ = position;
    }
  
    const fccl::kdl::Vector& Feature::getOrientation() const
    {
      return orientation_;
    }
  
    void Feature::setOrientation(const fccl::kdl::Vector& orientation)
    {
      orientation_ = orientation;
    }
  
    FeatureTypes Feature::getType() const
    {
      return (FeatureTypes) type_;
    }
  
    void Feature::setType(FeatureTypes type)
    {
      assert(featureTypeValid(type));
  
      type_ = type;
    }
  
    void Feature::changeReference(const fccl::kdl::Transform& transform)
    {
      assert(getReferenceID() == transform.getTargetID());
  
      setPosition(transform * getPosition());
      setOrientation(transform * getOrientation());
    }
  
    std::size_t Feature::getReferenceID() const
    {
      assert(getPosition().getReferenceID() == getOrientation().getReferenceID());
  
      return getPosition().getReferenceID();
    }
   
    bool Feature::operator==(const Feature& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }
  
    bool Feature::operator!=(const Feature& other) const
    {
      return !(*this == other);
    }
  
    bool Feature::semanticsEqual(const Feature& other) const
    {
      return (getType() == other.getType()) && (getID() == other.getID());
    }
  
    bool Feature::numericsEqual(const Feature& other) const
    {
      return (getPosition() == other.getPosition())
          && (getOrientation() == other.getOrientation());
    }
  
    std::ostream& operator<<(std::ostream& os, const Feature& feature)
    {
      os << "position:\n" << feature.getPosition() << "\n";
      os << "direction:\n" << feature.getOrientation() << "\n";
      os << "id: " << feature.getID() << "\n";
      os << "type: " << feature.getType();
  
      return os;
    }
  } // namespace base
} // namespace fccl
