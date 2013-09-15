#ifndef FCCL_KDL_JOINT_MAPPING_MATRIX_H
#define FCCL_KDL_JOINT_MAPPING_MATRIX_H

#include <fccl/kdl/Semantics.h>
#include <Eigen/Core>

namespace fccl
{
  namespace kdl
  {
    class JointMappingMatrix : public SemanticObjectNxM
    {
      JointMappingMatrix();
      JointMappingMatrix(const JointMappingMatrix& other);
      JointMappingMatrix(const SemanticObjectNxM& semantics, 
          const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& data);
 
      ~JointMappingMatrix();
      const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& getData() const;
      void setData(const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& data);

      virtual std::pair<std::size_t, std::size_t> size() const;
      virtual void resize(const std::pair<std::size_t, std::size_t>& new_size);

      bool isValid() const;
  
      std::size_t rows() const;
      std::size_t columns() const;

      bool rowIndexValid(std::size_t row) const;
      bool columnIndexValid(std::size_t column) const;
 
      double& operator()(std::size_t row, std::size_t column);
      double operator()(std::size_t row, std::size_t column) const;
  
      bool operator==(const JointMappingMatrix& other) const;
      bool operator!=(const JointMappingMatrix& other) const;

      bool numericsEqual(const JointMappingMatrix& other) const;

      friend std::ostream& operator<<(std::ostream& os, 
          const JointMappingMatrix& joint_mapping_matrix);
 
      //actual numeric representation of the matrix
      Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > data_;
    };
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_JOINT_MAPPING_MATRIX_H
