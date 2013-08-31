#include <fccl_control/Features.h>
#include <exception>
#include <iostream>

namespace fccl
{
  Feature::Feature()
  {
    name_.reserve(STRING_SIZE);
    type_ = -1;
  }

  Feature::Feature(const std::string& name, const fccl::Vector& position) :
      name_(name), position_(position)
  {
    name_.reserve(STRING_SIZE);
    type_ = -1;
  }

  Feature::~Feature() {}

  const std::string& Feature::getName() const
  {
    return name_;
  }

  void Feature::setName(const std::string& name)
  {
    name_ = name;
  }

  const fccl::Vector& Feature::getPosition() const
  {
    return position_;
  }

  void Feature::setPosition(const fccl::Vector& position)
  {
    position_ = position;
  }

  int Feature::getType() const
  {
    return type_;
  }

  bool Feature::operator==(const Feature& other) const
  {
    return (type_ == other.getType())
        && (name_.compare(other.getName()) == 0)
        && (position_ == other.getPosition());
  }

  bool Feature::operator!=(const Feature& other) const
  {
    return !(*this == other);
  }

  OrientedFeature::OrientedFeature() : Feature()
  {
    type_ = -2;
  }

  OrientedFeature::OrientedFeature(const std::string& name, const fccl::Vector& position) : Feature(name, position)
  {
    type_ = -2;
  }

  OrientedFeature::~OrientedFeature()
  {
  }

  Point::Point() : Feature()
  {
    type_ = 1;
  }

  bool OrientedFeature::operator==(const Feature& other) const
  {
    if(type_ != other.getType())
      return false;
    
    try 
    {
      const OrientedFeature* other_pointer = dynamic_cast<const OrientedFeature*>(&other);

      assert(other_pointer);

      return (name_.compare(other_pointer->getName()) == 0)
          && (position_ == other_pointer->getPosition())
          && (this->getOrientation() == other_pointer->getOrientation());
    } 
    catch (std::exception& e) 
    {
      std::cout << "Exception in OrientedFeature::operator==: " << e.what();
    }

    return false;
  }

  bool OrientedFeature::operator!=(const Feature& other) const
  {
    return !(*this == other);
  }

  Point::Point(const std::string& name, const fccl::Vector& position) :
      Feature(name, position)
  {
    type_ = 1;
  }

  Point::~Point()
  {
    type_ = 1;
  }

  void Point::changeReferenceFrame(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
  }

  Plane::Plane() : OrientedFeature()
  {
    type_ = 2;
  }

  Plane::Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal) : OrientedFeature(name, position), normal_(normal)
  {
    type_ = 2;
  }

  Plane::~Plane()
  {
  }

  const fccl::Vector& Plane::getNormal() const
  {
    return normal_;
  }

  void Plane::setNormal(const fccl::Vector& normal)
  {
    normal_ = normal;
  }

  const fccl::Vector& Plane::getOrientation() const
  {
    return getNormal();
  }

  void Plane::setOrientation(const fccl::Vector& orientation)
  {
    setNormal(orientation);
  }
 
  void Plane::changeReferenceFrame(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
    normal_.changeReferenceFrame(transform);
  }

  Line::Line() : OrientedFeature()
  {
    type_ = 3;
  }

  Line::Line(const std::string& name, const fccl::Vector& position, const fccl::Vector& direction) : OrientedFeature(name, position), direction_(direction)
  {
    type_ = 3;
  }

  Line::~Line()
  {
  }

  const fccl::Vector& Line::getDirection() const
  {
    return direction_;
  }

  void Line::setDirection(const fccl::Vector& direction)
  {
    direction_ = direction;
  }

  const fccl::Vector& Line::getOrientation() const
  {
    return getDirection();
  } 

  void Line::setOrientation(const fccl::Vector& orientation)
  {
    setDirection(orientation);
  }

  void Line::changeReferenceFrame(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
    direction_.changeReferenceFrame(transform);
  }

} // namespace fccl
