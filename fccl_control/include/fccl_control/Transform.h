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
      Transform(std::size_t reference_id, std::size_t target_id,
          const KDL::Frame& transform);
 
      virtual ~Transform();

      std::size_t getReferenceID() const;
      void setReferenceID(std::size_t reference_id);
      void setReferenceFrame(const std::string& reference_frame);

      std::size_t getTargetID() const;
      void setTargetID(std::size_t target_id);
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
      std::size_t reference_id_;

      // frame w.r.t. kinematic objects are defined after transformation,
      // also called 'child_frame'
      std::size_t target_id_;

      // actual numeric representation of transform
      KDL::Frame transform_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_TRANSFORM_H
