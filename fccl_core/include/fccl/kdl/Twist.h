#ifndef FCCL_KDL_TWIST_H
#define FCCL_KDL_TWIST_H

#include <kdl/frames.hpp>
#include <string>
#include <iostream>
#include <fccl/kdl/Transform.h>
#include <fccl/semantics/TwistSemantics.h>
#include <fccl/utils/Printing.h> 

namespace fccl
{
  namespace kdl
  {
    class Twist
    {
      public:
        const KDL::Twist& numerics() const
        {
          return twist_;
        }
 
        KDL::Twist& numerics()
        {
          return twist_;
        }
 
        const fccl::semantics::TwistSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::TwistSemantics& semantics()
        {
          return semantics_;
        }
     
        bool equals(const Twist& other) const
        {
          return semantics().equals(other.semantics()) &&
              KDL::Equal(numerics(), other.numerics());
        }
  
        void changeReferenceFrame(const Transform& transform)
        {
          semantics().changeReferenceFrame(transform.semantics());

          numerics() = transform.numerics() * numerics();
        }
          
        bool changeReferencePossible(const Transform& transform)
        {
          return semantics().changeReferencePossible(transform.semantics());
        }
           
      public:
        //semantics
        fccl::semantics::TwistSemantics semantics_;
        // numerics
        KDL::Twist twist_;
    };

    inline KDL::JntArray& TwistToJntArray(const KDL::Twist& twist, KDL::JntArray& joints)
    {
      assert(joints.rows() == 6);

      for(unsigned int i=0; i<6; i++)
        joints(i) = twist(i);
      
      return joints;
    }

    inline KDL::Twist JntArrayToTwist(const KDL::JntArray& joints, KDL::Twist& twist)
    {
      assert(joints.rows() == 6);

      for(unsigned int i=0; i<6; i++)
        twist(i) = joints(i);
   
      return twist;
    }

    inline std::ostream& operator<<(std::ostream& os, const Twist& twist)
    {
      using fccl::utils::operator<<;

      os << "numerics:\n" << twist.numerics() << "\n";
      os << "semantics:\n" << twist.semantics();

      return os;
    }

  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_TWIST_H
