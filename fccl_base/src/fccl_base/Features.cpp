#include <fccl_base/Features.h>
#include <fccl_base/FeatureTypes.h>
#include <fccl_utils/Hashing.h>
#include <exception>

namespace fccl
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

  Feature::Feature(std::size_t id, const fccl::Vector& position,
      const fccl::Vector& orientation, int type) :
      id_(id), position_(position), orientation_(orientation), type_(type)
  {
    assert(featureTypeValid(type));
  }
 
  Feature::Feature(const std::string& name, const fccl::Vector& position,
      const fccl::Vector& orientation, int type) :
      id_(hash(name)), position_(position), orientation_(orientation), type_(type)
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
    id_ = hash(name);
  }

  void Feature::setID(std::size_t id)
  {
    id_ = id;
  }

  const fccl::Vector& Feature::getPosition() const
  {
    return position_;
  }

  void Feature::setPosition(const fccl::Vector& position)
  {
    position_ = position;
  }

  const fccl::Vector& Feature::getOrientation() const
  {
    return orientation_;
  }

  void Feature::setOrientation(const fccl::Vector& orientation)
  {
    orientation_ = orientation;
  }

  int Feature::getType() const
  {
    return type_;
  }

  void Feature::setType(int type)
  {
    assert(featureTypeValid(type));

    type_ = type;
  }

  void Feature::changeReference(const fccl::Transform& transform)
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

} // namespace fccl
