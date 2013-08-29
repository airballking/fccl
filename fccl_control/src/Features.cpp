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

  Plane::Plane() : Feature()
  {
  }

  Plane::Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal) : Feature(name, position), normal_(normal)
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

  void Plane::changeReferenceFrame(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
    normal_.changeReferenceFrame(transform);
  }
} // namespace fccl
