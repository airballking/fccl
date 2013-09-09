#include <fccl/kdl/Transform.h>
#include <fccl/utils/Hashing.h>
#include <fccl/utils/Printing.h>

namespace fu = fccl::utils;

namespace fccl
{
  namespace kdl
  {
    Transform::Transform()
    {
    }
  
    Transform::Transform(const fccl::kdl::Transform& other) :
        reference_id_(other.getReferenceID()), target_id_(other.getTargetID()),
        transform_(other.getTransform())
    {
    }
   
    Transform::Transform(const std::string& reference_frame, const std::string& target_frame, const KDL::Frame& transform) :
        reference_id_(fu::hash(reference_frame)), target_id_(fu::hash(target_frame)), 
        transform_(transform)
    {
    }
  
    Transform::Transform(std::size_t reference_id, std::size_t target_id, const KDL::Frame& transform) :
        reference_id_(reference_id), target_id_(target_id), transform_(transform)
    {
    }
  
    Transform::~Transform()
    {
    }
  
    fccl::kdl::Transform& Transform::operator=(const fccl::kdl::Transform& rhs)
    {
      // need to check for self-allocation
      if(this != &rhs)
      {
        this->setReferenceID(rhs.getReferenceID());
        this->setTargetID(rhs.getTargetID());
        this->setTransform(rhs.getTransform());
      }
  
      return *this;
    }
  
  
    std::size_t Transform::getReferenceID() const
    {
      return reference_id_;
    }
  
    void Transform::setReferenceID(std::size_t reference_id) 
    {
      reference_id_ = reference_id;
    }
  
    void Transform::setReferenceFrame(const std::string& reference_frame)
    { 
      reference_id_ = fu::hash(reference_frame);
    }
  
    std::size_t Transform::getTargetID() const
    {
      return target_id_;
    }
  
    void Transform::setTargetID(std::size_t target_id)
    {
      target_id_ = target_id;
    }
  
    void Transform::setTargetFrame(const std::string& target_frame)
    {
      target_id_ = fu::hash(target_frame);
    }
  
    const KDL::Frame& Transform::getTransform() const
    {
      return transform_;
    }
  
    void Transform::setTransform(const KDL::Frame& transform)
    {
      transform_ = transform;
    }
  
    bool Transform::operator==(const fccl::kdl::Transform& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }
  
    bool Transform::operator!=(const fccl::kdl::Transform& other) const
    {
      return !(*this == other);
    }
  
    bool Transform::semanticsEqual(const fccl::kdl::Transform& other) const
    {
      return (reference_id_ == other.getReferenceID()
          && target_id_ == other.getTargetID());
    }
   
    bool Transform::numericsEqual(const fccl::kdl::Transform& other) const
    {
      return KDL::Equal(transform_, other.getTransform());
    }
  
    fccl::kdl::Transform Transform::inverse() const
    {
      return Transform(target_id_, reference_id_, transform_.Inverse());
    }
  
    bool Transform::postMultiplicationPossible(const fccl::kdl::Transform& other_post) const
    {
      return (target_id_ == other_post.getReferenceID());
    }
   
    bool Transform::preMultiplicationPossible(const fccl::kdl::Transform& other_pre) const
    {
      return other_pre.postMultiplicationPossible(*this);
    }
  
    fccl::kdl::Transform operator*(const fccl::kdl::Transform& lhs, const fccl::kdl::Transform& rhs)
    {
      assert(lhs.postMultiplicationPossible(rhs));
  
      return Transform(lhs.getReferenceID(), rhs.getTargetID(), lhs.getTransform()*rhs.getTransform());
    }
  
    std::ostream& operator<<(std::ostream& os, const fccl::kdl::Transform& transform)
    {
      using fccl::utils::operator<<;

      os << "transform:\n" << transform.getTransform() << "\n";
      os << "reference: " << transform.getReferenceID() << "\n";
      os << "target: " << transform.getTargetID() << "\n";
  
      return os;
    }
  } // namespace kdl
} // namespace fccl
