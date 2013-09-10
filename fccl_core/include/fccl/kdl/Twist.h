#ifndef FCCL_KDL_TWIST_H
#define FCCL_KDL_TWIST_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>

namespace fccl
{
  namespace kdl
  {
    class Twist : public SemanticObject1x1
    {
      public:
        Twist();
        Twist(const fccl::kdl::Twist& other);
        Twist(const std::string& reference_name, const std::string&
            target_name, const KDL::Twist& twist);
        Twist(std::size_t reference_id, std::size_t target_id,
            const KDL::Twist& twist);
  
        virtual ~Twist();
  
        fccl::kdl::Twist& operator=(const fccl::kdl::Twist& rhs);
  
        const KDL::Twist& getTwist() const;
        void setTwist(const KDL::Twist& twist);
  
        bool operator==(const fccl::kdl::Twist& other) const;
        bool operator!=(const fccl::kdl::Twist& other) const;
  
        bool numericsEqual(const fccl::kdl::Twist& other) const;
  
        void changeReferenceFrame(const fccl::kdl::Transform& transform);
        bool multiplicationPossible(const fccl::kdl::Transform& transform) const;
  
        friend fccl::kdl::Twist operator*(const fccl::kdl::Transform& lhs,
            const fccl::kdl::Twist& rhs);
  
        friend std::ostream& operator<<(std::ostream& os, 
            const fccl::kdl::Twist& twist);
   
      private:
        // numeric representation of the derivative 
        KDL::Twist twist_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_TWIST_H
