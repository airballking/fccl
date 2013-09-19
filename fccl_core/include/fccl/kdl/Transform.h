#ifndef FCCL_KDL_TRANSFORM_H
#define FCCL_KDL_TRANSFORM_H

#include <fccl/semantics/TransformSemantics.h>
#include <kdl/frames.hpp>
#include <string>
#include <iostream>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace kdl
  {
    class Transform
    {
      public:
        const KDL::Frame& numerics() const
        {
          return transform_;
        }

        KDL::Frame& numerics()
        {
          return transform_;
        }
 
        const fccl::semantics::TransformSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::TransformSemantics& semantics()
        {
          return semantics_;
        }
       
        Transform inverse() const
        {
          Transform result(*this);
          result.invert();
 
          return result;
        }

        bool equals(const Transform& other) const
        {
          return semantics().equals(other.semantics()) &&
              KDL::Equal(numerics(), other.numerics());
        }

        void invert()
        {
          numerics() = numerics().Inverse();
          semantics().invert(); 
        }
        
      private:
        // semantics
        fccl::semantics::TransformSemantics semantics_;
        // numerics
        KDL::Frame transform_;
    };

    inline std::ostream& operator<<(std::ostream& os, const Transform& transform)
    {
      using fccl::utils::operator<<;

      os << "numerics:\n" << transform.numerics() << "\n";
      os << "semantics:\n" << transform.semantics();
 
      return os;
    }
 
    inline bool areMultipliable(const Transform& lhs, const Transform& rhs)
    {
      return areMultipliable(lhs.semantics(), rhs.semantics());
    }

    inline Transform multiply(const Transform& lhs, const Transform& rhs)
    {
      Transform result;

      result.semantics() = multiply(lhs.semantics(), rhs.semantics());
      result.numerics() = lhs.numerics() * rhs.numerics();

      return result;
    }

  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_TRANSFORM_H
