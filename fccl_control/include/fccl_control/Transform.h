#ifndef FCCL_CONTROL_TRANSFORM_H
#define FCCL_CONTROL_TRANSFORM_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>

namespace fccl
{
  class Transform
  {
    public:
      Transform();
      Transform(const fccl::Transform& other);
      Transform(const std::string& reference_frame, const std::string& target_frame,
          const KDL::Frame& transform); 
      Transform(unsigned long reference_id, unsigned long target_id,
          const KDL::Frame& transform);
 
      virtual ~Transform();

      unsigned long getReferenceID() const;
      void setReferenceID(unsigned long reference_id);
      void setReferenceFrame(const std::string& reference_frame);

      unsigned long getTargetID() const;
      void setTargetID(unsigned long target_id);
      void setTargetFrame(const std::string& target_frame);

      const KDL::Frame& getTransform() const;
      void setTransform(const KDL::Frame& transform);

      bool operator==(const fccl::Transform& other) const;
      bool operator!=(const fccl::Transform& other) const;

      bool semanticsEqual(const fccl::Transform& other) const;
      bool numericsEqual(const fccl::Transform& other) const;

      fccl::Transform inverse() const;
      
      bool postMultiplicationPossible(const fccl::Transform& other_post) const;
      bool preMultiplicationPossible(const fccl::Transform& other_pre) const;

      friend fccl::Transform operator*(const fccl::Transform& lhs,
          const fccl::Transform& rhs); 

      friend std::ostream& operator<<(std::ostream& os, 
          const fccl::Transform& transform);

    private:
      // frame w.r.t. kinematic objects are defined before transformation,
      // sometimes also 'parent_frame' called
      unsigned long reference_id_;

      // frame w.r.t. kinematic objects are defined after transformation,
      // also called 'child_frame'
      unsigned long target_id_;

      // actual numeric representation of transform
      KDL::Frame transform_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_TRANSFORM_H
