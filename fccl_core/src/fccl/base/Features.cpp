#include <fccl/base/Features.h>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace base
  {
    Feature::Feature() : SemanticObject1x1()
    {
      type_ = UNKNOWN_FEATURE;
    }
  
    Feature::Feature(const Feature& other) :
        SemanticObject1x1(other), position_(other.getPosition()), 
        orientation_(other.getOrientation()), type_(other.getType())
    {
      assert(featureTypeValid(other.getType()));
    }

    Feature::Feature(const std::string& reference_name, const std::string&
            target_name, const KDL::Vector& position, const KDL::Vector&
            orientation, FeatureTypes type) :
        SemanticObject1x1(reference_name, target_name), position_(position),
        orientation_(orientation), type_(type)
    {
      assert(featureTypeValid(type));
    }
  
    Feature::Feature(std::size_t reference_id, std::size_t target_id,
            const KDL::Vector& position, const KDL::Vector& orientation,
            FeatureTypes type) :
        SemanticObject1x1(reference_id, target_id), position_(position), 
        orientation_(orientation), type_(type)
    {
      assert(featureTypeValid(type));
    }
 
    Feature::~Feature() {}
  
    Feature& Feature::operator=(const Feature& rhs)
    {
      // protect against self-assignement
      if(this != &rhs)
      {
        setTargetID(rhs.getTargetID());
        setReferenceID(rhs.getReferenceID());
        setPosition(rhs.getPosition());
        setOrientation(rhs.getOrientation());
        setType(rhs.getType());
      }
  
      return *this;
    }
  
    const KDL::Vector& Feature::getPosition() const
    {
      return position_;
    }
  
    void Feature::setPosition(const KDL::Vector& position)
    {
      position_ = position;
    }
  
    const KDL::Vector& Feature::getOrientation() const
    {
      return orientation_;
    }
  
    void Feature::setOrientation(const KDL::Vector& orientation)
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
  
      setReferenceID(transform.getReferenceID());

      setPosition(transform.getTransform() * getPosition());
      setOrientation(transform.getTransform() * getOrientation());
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
      return (getType() == other.getType()) && 
          SemanticObject1x1::semanticsEqual(other);
    }
  
    bool Feature::numericsEqual(const Feature& other) const
    {
      return (KDL::Equal(getPosition(), other.getPosition()))
          && (KDL::Equal(getOrientation(), other.getOrientation()));
    }
  
    std::ostream& operator<<(std::ostream& os, const Feature& feature)
    {
      using fccl::utils::operator<<;

      os << "position:\n" << feature.getPosition() << "\n";
      os << "direction:\n" << feature.getOrientation() << "\n";
      os << "reference: " << feature.getReferenceName() << " (";
      os << feature.getReferenceID() << ")\n";
      os << "target: " << feature.getTargetName() << " (";
      os << feature.getTargetID() << ")\n";
      os << "type: " << feature.getType();
  
      return os;
    }
  } // namespace base
} // namespace fccl
