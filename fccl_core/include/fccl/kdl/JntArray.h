#ifndef FCCL_KDL_JNTARRAY_H
#define FCCL_KDL_JNTARRAY_H

#include <kdl/jntarray.hpp>
#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/utils/Printing.h>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    class JntArray
    {
      public:
        const KDL::JntArray& numerics() const
        {
          return data_;
        }

        KDL::JntArray& numerics()
        {
          return data_;
        }

        const fccl::semantics::JntArraySemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::JntArraySemantics& semantics()
        {
          return semantics_;
        }
       
        bool equals(const JntArray& other) const
        {
          assert(isValid());
          assert(other.isValid());

          return semantics().equals(other.semantics()) &&
              KDL::Equal(numerics(), other.numerics());
        }
        
        bool isValid() const
        {
          return numerics().rows() == semantics().size();
        }

        void init(const std::vector<std::string>& joint_names)
        {
          resize(joint_names.size());
          semantics().init(joint_names);
        }

        void init(const fccl::semantics::JntArraySemantics& semantics)
        {
          resize(semantics.size());
          this->semantics() = semantics;
        }

        std::size_t size() const
        {
          assert(isValid());

          return semantics().size();
        }

        void resize(std::size_t new_size)
        {
          numerics().resize(new_size);
          KDL::SetToZero(numerics());
          semantics().resize(new_size);
        }

        void partialAssignment(std::size_t start, std::size_t elements,
            const JntArray& other)
        {
          semantics().partialAssignment(start, elements, other.semantics());

          numerics().data.segment(start, elements) = other.numerics().data;
        }
 
      private:
        // semantics
        fccl::semantics::JntArraySemantics semantics_;
        // numerics
        KDL::JntArray data_;
    }; 

    inline std::ostream& operator<<(std::ostream& os, const JntArray& jnt_array)
    {
      using fccl::utils::operator<<;

      os << "numerics:\n" << jnt_array.numerics() << "\n";
      os << "semantics:\n" << jnt_array.semantics();

      return os;
    }

    inline void substract(const JntArray& lhs, const JntArray& rhs, JntArray& res)
    {
      assert(lhs.semantics().equals(rhs.semantics()));
      assert(lhs.semantics().equals(res.semantics()));
      assert(lhs.isValid());
      assert(rhs.isValid());
      assert(res.isValid());

      res.numerics().data = lhs.numerics().data - rhs.numerics().data;
    }

    inline void add(const JntArray& lhs, const JntArray& rhs, JntArray& res)
    {
      assert(lhs.semantics().equals(rhs.semantics()));
      assert(lhs.semantics().equals(res.semantics()));
      assert(lhs.isValid());
      assert(rhs.isValid());
      assert(res.isValid());

      res.numerics().data = lhs.numerics().data + rhs.numerics().data;
    }
 
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JNTARRAY_H
