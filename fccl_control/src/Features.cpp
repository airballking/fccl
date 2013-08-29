#include <fccl_control/Features.h>

namespace fccl
{
  Feature::Feature()
  {
    name_.reserve(STRING_SIZE);
  }

  Feature::Feature(const std::string& name, const fccl::Vector& position) :
      name_(name), position_(position)
  {
    name_.reserve(STRING_SIZE);
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

  OrientedFeature::OrientedFeature() : Feature()
  {
  }

  OrientedFeature::OrientedFeature(const std::string& name, const fccl::Vector& position) : Feature(name, position)
  {
  }

  OrientedFeature::~OrientedFeature()
  {
  }

  Point::Point() : Feature()
  {
  }

  Point::Point(const std::string& name, const fccl::Vector& position) :
      Feature(name, position)
  {
  }

  Point::~Point()
  {
  }

  void Point::changeReferenceFrame(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
  }

  Plane::Plane() : OrientedFeature()
  {
  }

  Plane::Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal) : OrientedFeature(name, position), normal_(normal)
  {
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
  }

  Line::Line(const std::string& name, const fccl::Vector& position, const fccl::Vector& direction) : OrientedFeature(name, position), direction_(direction)
  {
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
