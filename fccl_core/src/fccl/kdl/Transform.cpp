#include <fccl/kdl/Transform.h>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace kdl
  {
    Transform::Transform() : SemanticObject1x1()
    {
    }
  
    Transform::Transform(const fccl::kdl::Transform& other) :
        SemanticObject1x1(other), transform_(other.getTransform())
    {
    }
   
    Transform::Transform(const SemanticObject1x1& semantics, 
            const KDL::Frame& transform) :
        SemanticObject1x1(semantics), transform_(transform)
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
        this->setSemantics(rhs.getSemantics());
        this->setTransform(rhs.getTransform());
      }
  
      return *this;
    }
  
    const KDL::Frame& Transform::getTransform() const
    {
      return transform_;
    }
  
    void Transform::setTransform(const KDL::Frame& transform)
    {
      transform_ = transform;
    }

    bool Transform::numericsEqual(const fccl::kdl::Transform& other) const
    {
      return KDL::Equal(getTransform(), other.getTransform());
    }

    bool Transform::operator==(const fccl::kdl::Transform& other) const
    {
      return (semanticsEqual(other) && numericsEqual(other));
    }

    bool Transform::operator!=(const fccl::kdl::Transform& other) const
    {
      return !(*this == other);
    }
 
    fccl::kdl::Transform Transform::inverse() const
    {
      return Transform(getSemantics().inverse(), getTransform().Inverse());
    }
  
    bool Transform::postMultiplicationPossible(const fccl::kdl::Transform& other_post) const
    {
      return (getTargetID() == other_post.getReferenceID());
    }
   
    bool Transform::preMultiplicationPossible(const fccl::kdl::Transform& other_pre) const
    {
      return other_pre.postMultiplicationPossible(*this);
    }
  
    fccl::kdl::Transform operator*(const fccl::kdl::Transform& lhs, const fccl::kdl::Transform& rhs)
    {
      assert(lhs.postMultiplicationPossible(rhs));
  
      Transform result;
      result.setReferenceID(lhs.getReferenceID());
      result.setTargetID(rhs.getTargetID());
      result.setTransform(lhs.getTransform() * rhs.getTransform());

      return result;
//      return Transform(lhs.getReferenceID(), rhs.getTargetID(), lhs.getTransform()*rhs.getTransform());
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
