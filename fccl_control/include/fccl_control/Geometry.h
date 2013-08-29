#ifndef FCCL_CONTROL_GEOMETRY_H
#define FCCL_CONTROL_GEOMETRY_H

#include <fccl_control/String.h>
#include <kdl/frames.hpp>

namespace fccl
{
  class Transform
  {
    public:
      Transform();
      Transform(const std::string& parent_frame, const std::string& child_frame,
          const KDL::Frame transform); 

      virtual ~Transform();

      const std::string& getParentFrame() const;
      void setParentFrame(const std::string& parent_frame);

      const std::string& getChildFrame() const;
      void setChildFrame(const std::string child_frame);

      const KDL::Frame& getTransform() const;
      void setTransform(const KDL::Frame& transform);

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
      Position();
      Position(const KDL::Vector& position, const std::string& frame_name);
      
      virtual ~Position();

      const std::string& getFrameName() const;
      void setFrameName(const std::string& frame_name);

      const KDL::Vector& getPosition() const;
      void setPosition(const KDL::Vector& position);
 
      bool isTransformApplicable(const fccl::Transform& transform) const;
      void changeReferenceFrame(const fccl::Transform& transform);

    private:
      // frame w.r.t. the position is defined
      std::string frame_name_;

      // actual numeric representation of position
      KDL::Vector position_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_GEOMETRY_H
