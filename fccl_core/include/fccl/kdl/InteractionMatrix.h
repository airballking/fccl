#ifndef FCCL_KDL_INTERACTION_MATRIX_H
#define FCCL_KDL_INTERACTION_MATRIX_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Core> 
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>
#include <fccl/semantics/InteractionMatrixSemantics.h>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace kdl
  {
    class InteractionMatrix
    {
      public:
        const Eigen::Matrix< double, Eigen::Dynamic, 6>& numerics() const
        {
          return data_;
        }

        Eigen::Matrix< double, Eigen::Dynamic, 6>& numerics()
        {
          return data_;
        }

        const fccl::semantics::InteractionMatrixSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::InteractionMatrixSemantics& semantics()
        {
          return semantics_;
        }

        bool equals(const InteractionMatrix& other) const
        {
          return semantics().equals(other.semantics()) &&
              numerics().isApprox(other.numerics());
        }

        void resize(std::size_t number_of_rows)
        {
          numerics().resize(number_of_rows, 6);
 
          semantics().resize(number_of_rows);
        }

        std::size_t size() const
        {
          assert(isValid());

          return semantics().size();
        }

        bool isValid() const
        {
          return numerics().rows() == semantics().size();
        }
  
        void changeReferenceFrame(const fccl::kdl::Transform& transform)
        {
          semantics().changeReferenceFrame(transform.semantics());

          numerics() = numerics() * calcTwistProjectorTranspose(transform.numerics());
        }

        bool changeReferencePossible(const fccl::kdl::Transform& transform) const
        {
          return semantics().changeReferencePossible(transform.semantics());
        }
  
      private:
        // semantics
        fccl::semantics::InteractionMatrixSemantics semantics_;
        // actual numeric representation of interaction matrix
        Eigen::Matrix< double, Eigen::Dynamic, 6 > data_;

        Eigen::Matrix<double, 6, 6> calcTwistProjectorTranspose(const KDL::Frame& transform) const;
    };

    inline std::ostream& operator<<(std::ostream& os,
        const InteractionMatrix& interaction_matrix)
    {
      using fccl::utils::operator<<;

      os << "semantics: " << interaction_matrix.semantics() << "\n";
      os << "numerics: " << interaction_matrix.numerics();

      return os;
    }
 
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_INTERACTION_MATRIX_H
