#ifndef FCCL_CONTROL_VECTOR_H
#define FCCL_CONTROL_VECTOR_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>
#include <fccl_control/Transform.h>

namespace fccl
{
  class Vector
  {
    public:
      Vector();
      Vector(const fccl::Vector& other);
      Vector(const std::string& reference_name, const std::string& target_name,
          const KDL::Vector& vector);
      Vector(unsigned long reference_id, unsigned long target_id,
          const KDL::Vector& vector);
      
      virtual ~Vector();

      unsigned long getReferenceID() const;
      void setReferenceID(unsigned long reference_id);
      void setReferenceName(const std::string& reference_name);

      unsigned long getTargetID() const;
      void setTargetID(unsigned long target_id);
      void setTargetName(const std::string& target_name);

      const KDL::Vector& getVector() const;
      void setVector(const KDL::Vector& vector);
 
      bool operator==(const fccl::Vector& other) const;
      bool operator!=(const fccl::Vector& other) const;

      bool semanticsEqual(const fccl::Vector& other) const;
      bool numericsEqual(const fccl::Vector& other) const;

      void changeReferenceFrame(const fccl::Transform& transform);
      bool multiplicationPossible(const fccl::Transform& transform) const;

      friend fccl::Vector operator*(const fccl::Transform& lhs,
          const fccl::Vector& rhs);

      friend std::ostream& operator<<(std::ostream& os, 
          const fccl::Vector& vector);
 
    private:
      // reference frame w.r.t. the vector is defined
      unsigned long reference_id_;

      // name/id of entity this vector represents, e.g. name of point of which
      // this is the position, translational velocity, angular velocity etc.
      unsigned long target_id_;

      // actual numeric representation of vector
      KDL::Vector vector_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_VECTOR_H
