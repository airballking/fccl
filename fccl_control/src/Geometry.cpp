#include <fccl_control/Geometry.h>

namespace fccl
{
  Transform::Transform()
  {
    parent_frame_.reserve(STRING_SIZE);
    child_frame_.reserve(STRING_SIZE);
  }

  Transform::Transform(const std::string& parent_frame, const std::string& child_frame, const KDL::Frame transform) : 
      parent_frame_(parent_frame), child_frame_(child_frame), transform_(transform)
  {
    parent_frame_.reserve(STRING_SIZE);
    child_frame_.reserve(STRING_SIZE);
  }

  Transform::~Transform()
  {
  }

  const std::string& Transform::getParentFrame() const
  { 
    return parent_frame_;
  }

  void Transform::setParentFrame(const std::string& parent_frame)
  { 
    parent_frame_ = parent_frame;
  }

  const std::string& Transform::getChildFrame() const
  {
    return child_frame_;
  }

  void Transform::setChildFrame(const std::string child_frame)
  {
    child_frame_ = child_frame;
  }

  const KDL::Frame& Transform::getTransform() const
  {
    return transform_;
  }

  void Transform::setTransform(const KDL::Frame& transform)
  {
    transform_ = transform;
  }

  Vector::Vector()
  {
    frame_name_.reserve(STRING_SIZE);
  }

  Vector::Vector(const KDL::Vector& vector, const std::string& frame_name) :
      frame_name_(frame_name), vector_(vector)
  {
    frame_name_.reserve(STRING_SIZE);
  }
  
  Vector::~Vector() {}

  const std::string& Vector::getFrameName() const
  {
    return frame_name_;
  }

  void Vector::setFrameName(const std::string& frame_name)
  {
    frame_name_ = frame_name;
  }

  const KDL::Vector& Vector::getVector() const
  {
    return vector_;
  }

  void Vector::setVector(const KDL::Vector& vector)
  {
    vector_ = vector;
  }

  bool Vector::isTransformApplicable(const fccl::Transform& transform) const
  {
    return (frame_name_.compare(transform.getParentFrame()) == 0);
  }

  void Vector::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(isTransformApplicable(transform));

    frame_name_ = transform.getChildFrame();
    vector_ = transform.getTransform() * vector_;
  }
} // namespace fccl
