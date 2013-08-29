#include <fccl_control/Features.h>

namespace fccl
{
  Feature::Feature()
  {
    name_.reserve(STRING_SIZE);
  }

  Feature::Feature(const std::string& name, const fccl::Position& position) :
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

  const fccl::Position& Feature::getPosition() const
  {
    return position_;
  }

  void Feature::setPosition(const fccl::Position& position)
  {
    position_ = position;
  }

  Point::Point() : Feature()
  {
  }

  Point::Point(const std::string& name, const fccl::Position& position) :
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
 
} // namespace fccl
