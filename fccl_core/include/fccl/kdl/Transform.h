#ifndef FCCL_KDL_TRANSFORM_H
#define FCCL_KDL_TRANSFORM_H

#include <fccl/kdl/Semantics.h>
#include <kdl/frames.hpp>
#include <string>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    class Transform : public SemanticObject1x1
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
  
        const KDL::Frame& getTransform() const;
        void setTransform(const KDL::Frame& transform);
  
        bool numericsEqual(const fccl::kdl::Transform& other) const;
        virtual bool operator==(const fccl::kdl::Transform& other) const;
        virtual bool operator!=(const fccl::kdl::Transform& other) const;
  
        fccl::kdl::Transform inverse() const;
        
        bool postMultiplicationPossible(const fccl::kdl::Transform& other_post) const;
        bool preMultiplicationPossible(const fccl::kdl::Transform& other_pre) const;
  
        friend fccl::kdl::Transform operator*(const fccl::kdl::Transform& lhs,
            const fccl::kdl::Transform& rhs); 
  
        friend std::ostream& operator<<(std::ostream& os, 
            const fccl::kdl::Transform& transform);
  
      private:
        // actual numeric representation of transform
        KDL::Frame transform_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_TRANSFORM_H
