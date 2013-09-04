#include <fccl_control/Twist.h>
#include <fccl_control/Hashing.h>

namespace fccl
{
  Twist::Twist()
  {
  }

  Twist::Twist(const fccl::Twist& other) :
      reference_id_(other.getReferenceID()), target_id_(other.getTargetID()),
      twist_(other.getTwist())
  {
  }

  Twist::Twist(const std::string& reference_name, const std::string&
      target_name, const KDL::Twist& twist) :
      reference_id_(hash(reference_name)), target_id_(hash(target_name)),
      twist_(twist)
  {
  }

  Twist::Twist(std::size_t reference_id, std::size_t target_id,
      const KDL::Twist& twist) :
      reference_id_(reference_id), target_id_(target_id), twist_(twist)
  {
  }

  std::size_t Twist::getReferenceID() const
  {
    return reference_id_;
  }

  void Twist::setReferenceID(std::size_t reference_id)
  {
    reference_id_ = reference_id;
  }

  void Twist::setReferenceName(const std::string& reference_name)
  {
    reference_id_ = hash(reference_name);
  }

  std::size_t Twist::getTargetID() const
  {
    return target_id_;
  }

  void Twist::setTargetID(std::size_t target_id)
  {
    target_id_ = target_id;
  }

  void Twist::setTargetName(const std::string& target_name)
  {
    target_id_ = hash(target_name);
  }

  const KDL::Twist& Twist::getTwist() const
  {
    return twist_;
  }

  void Twist::setTwist(const KDL::Twist& twist)
  {
    twist_ = twist;
  } 

  bool Twist::operator==(const fccl::Twist& other) const
  {
    return semanticsEqual(other) && numericsEqual(other);
  }

  bool Twist::operator!=(const fccl::Twist& other) const
  {
    return !(*this == other);
  }

  bool Twist::semanticsEqual(const fccl::Twist& other) const
  {
    return reference_id_ == other.getReferenceID() &&
        target_id_ == other.getTargetID();
  }

  bool Twist::numericsEqual(const fccl::Twist& other) const
  {
    return KDL::Equal(twist_, other.getTwist());
  }

  void Twist::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(multiplicationPossible(transform));

    twist_ = transform.getTransform() * twist_;
    reference_id_ = transform.getReferenceID();
  }

  bool Twist::multiplicationPossible(const fccl::Transform& transform) const
  {
    return reference_id_ == transform.getTargetID();
  }

  fccl::Twist operator*(const fccl::Transform& lhs, const fccl::Twist& rhs)
  {
    fccl::Twist result(rhs);
    result.changeReferenceFrame(lhs);
    return result;
  }

  std::ostream& operator<<(std::ostream& os, const fccl::Twist& twist)
  {
    KDL::Twist t = twist.getTwist();
    KDL::Vector v = t.vel;

    os << "translation: [ " << v.x() << " " << v.y() << " " << v.z() << " ]\n";
    v = t.rot;
    os << "rotation: [ " << v.x() << " " << v.y() << " " << v.z() << " ]\n";
    os << "reference: " << twist.getReferenceID() << "\n";
    os << "target: " << twist.getTargetID() << "\n";
  }

} // namespace fccl
