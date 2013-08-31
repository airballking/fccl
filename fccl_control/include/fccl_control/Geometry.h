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

  class Vector
  {
    public:
      Vector();
      Vector(const KDL::Vector& vector, const std::string& frame_name);
      
      virtual ~Vector();

      const std::string& getFrameName() const;
      void setFrameName(const std::string& frame_name);

      const KDL::Vector& getVector() const;
      void setVector(const KDL::Vector& vector);
 
      bool isTransformApplicable(const fccl::Transform& transform) const;
      void changeReferenceFrame(const fccl::Transform& transform);
 
      bool operator==(const Vector &other) const;
      bool operator!=(const Vector &other) const; 

    private:
      // frame w.r.t. the vector is defined
      std::string frame_name_;

      // actual numeric representation of vector
      KDL::Vector vector_;
  };

} // namespace fccl
#endif // FCCL_CONTROL_GEOMETRY_H
