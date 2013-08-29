#ifndef FCCL_CONTROL_GEOMETRY_H
#define FCCL_CONTROL_GEOMETRY_H

#include <fccl_control/String.h>
#include <kdl/frames.hpp>

namespace fccl
{
  class Transform
  {
    public:
      Transform()
      {
        parent_frame_.reserve(STRING_SIZE);
        child_frame_.reserve(STRING_SIZE);
      }

      Transform(const std::string& parent_frame, const std::string& child_frame,
          const KDL::Frame transform) : 
          parent_frame_(parent_frame), child_frame_(child_frame), transform_(transform)
      {
        parent_frame_.reserve(STRING_SIZE);
        child_frame_.reserve(STRING_SIZE);
      }

      virtual ~Transform() {}

      const std::string& getParentFrame() const
      { 
        return parent_frame_;
      }

      void setParentFrame(const std::string& parent_frame)
      { 
        parent_frame_ = parent_frame;
      }

      const std::string& getChildFrame() const
      {
        return child_frame_;
      }

      void setChildFrame(const std::string child_frame)
      {
        child_frame_ = child_frame;
      }

      const KDL::Frame& getTransform() const
      {
        return transform_;
      }

      void setTransform(const KDL::Frame& transform)
      {
        transform_ = transform;
      }

    private:
      // frame w.r.t. kinematic objects are defined before transformation
      std::string parent_frame_;

      // frame w.r.t. kinematic objects are defined after transformation
      std::string child_frame_;

      // actual numeric representation of transform
      KDL::Frame transform_;
  };

  class Position
  {
    public:
      Position()
      {
        frame_name_.reserve(STRING_SIZE);
      }

      Position(const KDL::Vector& position, const std::string& frame_name) :
          frame_name_(frame_name), position_(position)
      {
        frame_name_.reserve(STRING_SIZE);
      }
      
      virtual ~Position() {}

      const std::string& getFrameName() const
      {
        return frame_name_;
      }

      void setFrameName(const std::string& frame_name)
      {
        frame_name_ = frame_name;
      }

      const KDL::Vector& getPosition() const
      {
        return position_;
      }

      void setPosition(const KDL::Vector& position)
      {
        position_ = position;
      }
 
    private:
      // frame w.r.t. the position is defined
      std::string frame_name_;

      // actual numeric representation of position
      KDL::Vector position_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_GEOMETRY_H
