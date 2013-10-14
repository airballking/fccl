#ifndef FCCL_KDL_JACOBIAN_H
#define FCCL_KDL_JACOBIAN_H

#include <fccl/semantics/JacobianSemantics.h>
#include <kdl/jacobian.hpp>
#include <fccl/utils/Printing.h>
#include <fccl/kdl/Transform.h>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    class Jacobian
    {
      public:
        const KDL::Jacobian& numerics() const
        {
          return numerics_;
        }

        KDL::Jacobian& numerics()
        {
          return numerics_;
        }

        const fccl::semantics::JacobianSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::JacobianSemantics& semantics() 
        {
          return semantics_;
        }

        bool equals(const Jacobian& other) const
        {
          assert(isValid());
          assert(other.isValid());

          // the check for the numerics is not using KDL::Equal because
          // I could not convince the compile of its existance :/
          return semantics().equals(other.semantics()) &&
              numerics().data.isApprox(other.numerics().data);
        }

        void init(const std::vector<std::string>& joint_names,
            const std::string& reference_name, const std::string& target_name)
        {
          resize(joint_names.size());
          semantics().init(joint_names, reference_name, target_name); 
        }

        bool isValid() const
        {
          return numerics().columns() == semantics().joints().size();
        }

        void resize(std::size_t columns)
        {
          numerics().resize(columns);
          semantics().resize(columns);
        }

        std::size_t size() const
        {
          assert(isValid());

          return semantics().size();
        }

        void changeReferenceFrame(const Transform& transform)
        {
          semantics().changeReferenceFrame(transform.semantics());

          numerics().changeRefFrame(transform.numerics());
        }
          
        bool changeReferencePossible(const Transform& transform)
        {
          return semantics().changeReferencePossible(transform.semantics());
        }
 
      private:
        fccl::semantics::JacobianSemantics semantics_;
        KDL::Jacobian numerics_;
    };

    inline std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian)
    {
      using fccl::utils::operator<<;

      os << "numerics:\n" << jacobian.numerics() << "\n";
      os << "semantics:\n" << jacobian.semantics();

      return os;
    }
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_JACOBIAN_H
