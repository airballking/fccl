#ifndef FCCL_CONTROL_TWIST_H
#define FCCL_CONTROL_TWIST_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>
#include <fccl_control/Transform.h>
#include <Eigen/Core>

namespace fccl
{
  class Twist
  {
    public:
      Twist();
      Twist(const fccl::Twist& other);
      Twist(const std::string& reference_name, const std::string&
          target_name, const KDL::Twist& twist);
      Twist(std::size_t reference_id, std::size_t target_id,
          const KDL::Twist& twist);

      std::size_t getReferenceID() const;
      void setReferenceID(std::size_t reference_id);
      void setReferenceName(const std::string& reference_name);

      std::size_t getTargetID() const;
      void setTargetID(std::size_t target_id);
      void setTargetName(const std::string& target_name);

      const KDL::Twist& getTwist() const;
      void setTwist(const KDL::Twist& twist);

      bool operator==(const fccl::Twist& other) const;
      bool operator!=(const fccl::Twist& other) const;

      bool semanticsEqual(const fccl::Twist& other) const;
      bool numericsEqual(const fccl::Twist& other) const;

      void changeReferenceFrame(const fccl::Transform& transform);
      bool multiplicationPossible(const fccl::Transform& transform) const;

      friend fccl::Twist operator*(const fccl::Transform& lhs,
          const fccl::Twist& rhs);

      friend std::ostream& operator<<(std::ostream& os, 
          const fccl::Twist& twist);
 
    private:
      // reference frame w.r.t. the twist is defined
      std::size_t reference_id_;

      // entity/frame of which this is the twist
      std::size_t target_id_;

      // numeric representation of the derivative 
      KDL::Twist twist_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_TWIST_H
