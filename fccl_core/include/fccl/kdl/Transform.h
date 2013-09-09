#ifndef FCCL_KDL_TRANSFORM_H
#define FCCL_KDL_TRANSFORM_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    class Transform
    {
      public:
        Transform();
        Transform(const fccl::kdl::Transform& other);
        Transform(const std::string& reference_frame, const std::string& target_frame,
            const KDL::Frame& transform); 
        Transform(std::size_t reference_id, std::size_t target_id,
            const KDL::Frame& transform);
   
        virtual ~Transform();
  
        fccl::kdl::Transform& operator=(const fccl::kdl::Transform& rhs);
  
        std::size_t getReferenceID() const;
        void setReferenceID(std::size_t reference_id);
        void setReferenceFrame(const std::string& reference_frame);
  
        std::size_t getTargetID() const;
        void setTargetID(std::size_t target_id);
        void setTargetFrame(const std::string& target_frame);
  
        const KDL::Frame& getTransform() const;
        void setTransform(const KDL::Frame& transform);
  
        bool operator==(const fccl::kdl::Transform& other) const;
        bool operator!=(const fccl::kdl::Transform& other) const;
  
        bool semanticsEqual(const fccl::kdl::Transform& other) const;
        bool numericsEqual(const fccl::kdl::Transform& other) const;
  
        fccl::kdl::Transform inverse() const;
        
        bool postMultiplicationPossible(const fccl::kdl::Transform& other_post) const;
        bool preMultiplicationPossible(const fccl::kdl::Transform& other_pre) const;
  
        friend fccl::kdl::Transform operator*(const fccl::kdl::Transform& lhs,
            const fccl::kdl::Transform& rhs); 
  
        friend std::ostream& operator<<(std::ostream& os, 
            const fccl::kdl::Transform& transform);
  
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
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_TRANSFORM_H
