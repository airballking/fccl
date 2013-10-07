#ifndef FCCL_KDL_JOINT_MAPPING_MATRIX_H
#define FCCL_KDL_JOINT_MAPPING_MATRIX_H

#include <fccl/semantics/JointMappingSemantics.h>
#include <fccl/utils/Printing.h>
#include <Eigen/Core>

namespace fccl
{
  namespace kdl
  {
    class JointMappingMatrix
    {
      public:
        const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& numerics() const
        {
          return data_;
        }

        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& numerics()
        {
          return data_;
        }

        const fccl::semantics::JointMappingSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::JointMappingSemantics& semantics()
        {
          return semantics_;
        }

        bool equals(const JointMappingMatrix& other) const
        {
          return semantics().equals(other.semantics()) &&
            numerics().isApprox(other.numerics());
        }

        void resize(std::size_t rows, std::size_t columns)
        {
          numerics().resize(rows, columns);

          semantics().row_joints().resize(rows);
          semantics().column_joints().resize(columns);
        }
  
        bool isValid() const
        {
          return (semantics().row_joints().size() == numerics().rows()) &&
              (semantics().column_joints().size() == numerics().cols());
        }
    
  
      private:
        // semantics of the JointMappingMatrices
        fccl::semantics::JointMappingSemantics semantics_;
        //actual numeric representation of the matrix
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > data_;
    };

    inline std::ostream& operator<<(std::ostream& os,
            const JointMappingMatrix& joint_mapping_matrix)
    {
      using fccl::utils::operator<<;

      os << "numerics:\n" << joint_mapping_matrix.numerics() << "\n";
      os << "semantics:\n" << joint_mapping_matrix.semantics();

      return os;
    }
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_JOINT_MAPPING_MATRIX_H
