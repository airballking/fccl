#include <fccl/kdl/Twist.h>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace kdl
  {
    Twist::Twist() :
        SemanticObject1x1()
    {
    }
  
    Twist::Twist(const fccl::kdl::Twist& other) :
        SemanticObject1x1(other), twist_(other.getTwist())
    {
    }
  
    Twist::Twist(const std::string& reference_name, const std::string&
        target_name, const KDL::Twist& twist) :
        SemanticObject1x1(reference_name, target_name), twist_(twist)
    {
    }
  
    Twist::Twist(std::size_t reference_id, std::size_t target_id,
        const KDL::Twist& twist) :
        SemanticObject1x1(reference_id, target_id), twist_(twist)
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
  
    bool Twist::numericsEqual(const fccl::kdl::Twist& other) const
    {
      return KDL::Equal(twist_, other.getTwist());
    }
  
    void Twist::changeReferenceFrame(const fccl::kdl::Transform& transform)
    {
      assert(multiplicationPossible(transform));
  
      setTwist(transform.getTransform() * twist_);
      setReferenceID(transform.getReferenceID());
    }
  
    bool Twist::multiplicationPossible(const fccl::kdl::Transform& transform) const
    {
      return getReferenceID() == transform.getTargetID();
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
