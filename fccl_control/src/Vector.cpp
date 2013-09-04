#include <fccl_control/Vector.h>
#include <fccl_control/Hashing.h>

namespace fccl
{
  Vector::Vector()
  {
  }

  Vector::Vector(const fccl::Vector& other) :
      reference_id_(other.getReferenceID()), target_id_(other.getTargetID()),
      vector_(other.getVector())
  {
  }

  Vector::Vector(const std::string& reference_name, const std::string& target_name, const KDL::Vector& vector) :
      reference_id_(hash(reference_name)), target_id_(hash(target_name)),
      vector_(vector)
  {
  }
  
  Vector::Vector(std::size_t reference_id, std::size_t target_id, const KDL::Vector& vector) :
      reference_id_(reference_id), target_id_(target_id), vector_(vector)
  {
  }

  Vector::~Vector() {}

  std::size_t Vector::getReferenceID() const
  {
    return reference_id_;
  }

  void Vector::setReferenceID(std::size_t reference_id)
  {
    reference_id_ = reference_id;
  }

  void Vector::setReferenceName(const std::string& reference_name)
  {
    reference_id_ = hash(reference_name);
  }

  std::size_t Vector::getTargetID() const
  {
    return target_id_;
  }

  void Vector::setTargetID(std::size_t target_id)
  {
    target_id_ = target_id;
  }

  void Vector::setTargetName(const std::string& target_name)
  {
    target_id_ = hash(target_name);
  }

  const KDL::Vector& Vector::getVector() const
  {
    return vector_;
  }

  void Vector::setVector(const KDL::Vector& vector)
  {
    vector_ = vector;
  }

  bool Vector::operator==(const Vector &other) const 
  {
    return semanticsEqual(other) && numericsEqual(other);
  }

  bool Vector::operator!=(const Vector &other) const
  {
    return !(*this == other);
  }

  bool Vector::semanticsEqual(const fccl::Vector& other) const
  {
    return reference_id_ == other.getReferenceID() && 
        target_id_ == other.getTargetID();
  }

  bool Vector::numericsEqual(const fccl::Vector& other) const
  {
    return KDL::Equal(vector_, other.getVector());
  }

  void Vector::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(multiplicationPossible(transform));

    reference_id_ = transform.getReferenceID();
    vector_ = transform.getTransform() * vector_;
  }

  bool Vector::multiplicationPossible(const fccl::Transform& transform) const
  {
    return (reference_id_ == transform.getTargetID());
  }

  fccl::Vector operator*(const fccl::Transform& lhs, const fccl::Vector& rhs)
  {
    Vector result(rhs);
    result.changeReferenceFrame(lhs);
    return result;
  }

  std::ostream& operator<<(std::ostream& os, const fccl::Vector& vector)
  {
    KDL::Vector v = vector.getVector();
    os << "[ " << v.x() << " " << v.y() << " " << v.z() << " ]\n";
    os << "  refererence: " << vector.getReferenceID() << "\n";
    os << "  target: " << vector.getTargetID() << "\n";
  }
} // namespace fccl
