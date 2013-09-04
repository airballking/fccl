#include <fccl_control/Transform.h>
#include <fccl_control/Hashing.h>

namespace fccl
{
  Transform::Transform()
  {
  }

  Transform::Transform(const Transform& other) :
      reference_id_(other.getReferenceID()), target_id_(other.getTargetID()),
      transform_(other.getTransform())
  {
  }
 
  Transform::Transform(const std::string& reference_frame, const std::string& target_frame, const KDL::Frame& transform) :
      reference_id_(hash(reference_frame)), target_id_(hash(target_frame)), 
      transform_(transform)
  {
  }

  Transform::Transform(unsigned long reference_id, unsigned long target_id, const KDL::Frame& transform) :
      reference_id_(reference_id), target_id_(target_id), transform_(transform)
  {
  }

  Transform::~Transform()
  {
  }

  unsigned long Transform::getReferenceID() const
  {
    return reference_id_;
  }

  void Transform::setReferenceID(unsigned long reference_id) 
  {
    reference_id_ = reference_id;
  }

  void Transform::setReferenceFrame(const std::string& reference_frame)
  { 
    reference_id_ = hash(reference_frame);
  }

  unsigned long Transform::getTargetID() const
  {
    return target_id_;
  }

  void Transform::setTargetID(unsigned long target_id)
  {
    target_id_ = target_id;
  }

  void Transform::setTargetFrame(const std::string& target_frame)
  {
    target_id_ = hash(target_frame);
  }

  const KDL::Frame& Transform::getTransform() const
  {
    return transform_;
  }

  void Transform::setTransform(const KDL::Frame& transform)
  {
    transform_ = transform;
  }

  bool Transform::operator==(const fccl::Transform& other) const
  {
    return semanticsEqual(other) && numericsEqual(other);
  }

  bool Transform::operator!=(const fccl::Transform& other) const
  {
    return !(*this == other);
  }

  bool Transform::semanticsEqual(const fccl::Transform& other) const
  {
    return (reference_id_ == other.getReferenceID()
        && target_id_ == other.getTargetID());
  }
 
  bool Transform::numericsEqual(const fccl::Transform& other) const
  {
    return KDL::Equal(transform_, other.getTransform());
  }

  fccl::Transform Transform::inverse() const
  {
    return Transform(target_id_, reference_id_, transform_.Inverse());
  }

  bool Transform::postMultiplicationPossible(const fccl::Transform& other_post) const
  {
    return (target_id_ == other_post.getReferenceID());
  }
 
  bool Transform::preMultiplicationPossible(const fccl::Transform& other_pre) const
  {
    return other_pre.postMultiplicationPossible(*this);
  }

  fccl::Transform operator*(const fccl::Transform& lhs, const fccl::Transform& rhs)
  {
    assert(lhs.postMultiplicationPossible(rhs));

    return Transform(lhs.getReferenceID(), rhs.getTargetID(), lhs.getTransform()*rhs.getTransform());
  }

  std::ostream& operator<<(std::ostream& os, const Transform& transform)
  {
    KDL::Frame f = transform.getTransform();
    os << "p:\n  " << f.p.x() << " " << f.p.y() << " " << f.p.z() << "\n";
    os << "M:\n";
    for(unsigned int i=0; i<3; i++)
    {
      os << "  ";
      for(unsigned int j=0; j<3; j++)
        os << f.M(i,j) << " ";
      os << "\n";
    }   
    os << "reference: " << transform.getReferenceID() << "\n";
    os << "target: " << transform.getTargetID() << "\n";

    return os;
  }

} // namespace fccl
