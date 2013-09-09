#include <fccl/kdl/Twist.h>
#include <fccl/utils/Hashing.h>
#include <fccl/utils/Printing.h>

namespace fu = fccl::utils;

namespace fccl
{
  namespace kdl
  {
    Twist::Twist()
    {
    }
  
    Twist::Twist(const fccl::kdl::Twist& other) :
        reference_id_(other.getReferenceID()), target_id_(other.getTargetID()),
        twist_(other.getTwist())
    {
    }
  
    Twist::Twist(const std::string& reference_name, const std::string&
        target_name, const KDL::Twist& twist) :
        reference_id_(fu::hash(reference_name)), target_id_(fu::hash(target_name)),
        twist_(twist)
    {
    }
  
    Twist::Twist(std::size_t reference_id, std::size_t target_id,
        const KDL::Twist& twist) :
        reference_id_(reference_id), target_id_(target_id), twist_(twist)
    {
    }
  
    Twist::~Twist()
    {
    }
  
    fccl::kdl::Twist& Twist::operator=(const fccl::kdl::Twist& rhs)
    {
      // need to check for self-allocation
      if(this != &rhs)
      {
        this->setReferenceID(rhs.getReferenceID());
        this->setTargetID(rhs.getTargetID());
        this->setTwist(rhs.getTwist());
      }
    
      return *this;
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
      reference_id_ = fu::hash(reference_name);
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
      target_id_ = fu::hash(target_name);
    }
  
    const KDL::Twist& Twist::getTwist() const
    {
      return twist_;
    }
  
    void Twist::setTwist(const KDL::Twist& twist)
    {
      twist_ = twist;
    } 
  
    bool Twist::operator==(const fccl::kdl::Twist& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }
  
    bool Twist::operator!=(const fccl::kdl::Twist& other) const
    {
      return !(*this == other);
    }
  
    bool Twist::semanticsEqual(const fccl::kdl::Twist& other) const
    {
      return reference_id_ == other.getReferenceID() &&
          target_id_ == other.getTargetID();
    }
  
    bool Twist::numericsEqual(const fccl::kdl::Twist& other) const
    {
      return KDL::Equal(twist_, other.getTwist());
    }
  
    void Twist::changeReferenceFrame(const fccl::kdl::Transform& transform)
    {
      assert(multiplicationPossible(transform));
  
      twist_ = transform.getTransform() * twist_;
      reference_id_ = transform.getReferenceID();
    }
  
    bool Twist::multiplicationPossible(const fccl::kdl::Transform& transform) const
    {
      return reference_id_ == transform.getTargetID();
    }
  
    fccl::kdl::Twist operator*(const fccl::kdl::Transform& lhs, const fccl::kdl::Twist& rhs)
    {
      fccl::kdl::Twist result(rhs);
      result.changeReferenceFrame(lhs);
      return result;
    }
  
    std::ostream& operator<<(std::ostream& os, const fccl::kdl::Twist& twist)
    {
      using fccl::utils::operator<<;

      os << "translation: " << twist.getTwist().vel << "\n";
      os << "rotation: " << twist.getTwist().rot << "\n";
      os << "reference: " << twist.getReferenceID() << "\n";
      os << "target: " << twist.getTargetID();
    }
  } // namespace kdl
} // namespace fccl
