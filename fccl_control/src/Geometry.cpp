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

  Position::Position()
  {
    frame_name_.reserve(STRING_SIZE);
  }

  Position::Position(const KDL::Vector& position, const std::string& frame_name) :
      frame_name_(frame_name), position_(position)
  {
    frame_name_.reserve(STRING_SIZE);
  }
  
  Position::~Position() {}

  const std::string& Position::getFrameName() const
  {
    return frame_name_;
  }

  void Position::setFrameName(const std::string& frame_name)
  {
    frame_name_ = frame_name;
  }

  const KDL::Vector& Position::getPosition() const
  {
    return position_;
  }

  void Position::setPosition(const KDL::Vector& position)
  {
    position_ = position;
  }

  bool Position::isTransformApplicable(const fccl::Transform& transform) const
  {
    return (frame_name_.compare(transform.getParentFrame()) == 0);
  }

  void Position::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(isTransformApplicable(transform));

    frame_name_ = transform.getChildFrame();
    position_ = transform.getTransform() * position_;
  }
} // namespace fccl
