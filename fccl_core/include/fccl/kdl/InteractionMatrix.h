#ifndef FCCL_KDL_INTERACTION_MATRIX_H
#define FCCL_KDL_INTERACTION_MATRIX_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Core> 
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>

namespace fccl
{
  namespace kdl
  {
    class InteractionMatrix : public SemanticObject1xN
    {
      public:
        InteractionMatrix();
        InteractionMatrix(const fccl::kdl::InteractionMatrix& other);
        InteractionMatrix(const std::string& reference_name, const 
            std::vector<std::string>& target_names, 
            const Eigen::Matrix<double, Eigen::Dynamic, 6>& data);
        InteractionMatrix(std::size_t reference_id, const std::vector<std::size_t>&
            target_ids, const Eigen::Matrix<double, Eigen::Dynamic, 6>& data);
  
        virtual ~InteractionMatrix();
  
        fccl::kdl::InteractionMatrix& operator=(const fccl::kdl::InteractionMatrix& rhs);
  
        const Eigen::Matrix< double, Eigen::Dynamic, 6>& getData() const;
        void setData(const Eigen::Matrix< double, Eigen::Dynamic, 6>& data);
  
        void resize(unsigned int number_of_rows);
  
        unsigned int rows() const;
        unsigned int columns() const;
  
        double& operator()(unsigned int row, unsigned int column);
        double operator()(unsigned int row, unsigned int column) const;
  
        fccl::kdl::Twist getRow(unsigned int row) const;
        void setRow(unsigned int row, const fccl::kdl::Twist& twist);
  
        bool operator==(const fccl::kdl::InteractionMatrix& other) const;
        bool operator!=(const fccl::kdl::InteractionMatrix& other) const;
  
        bool numericsEqual(const fccl::kdl::InteractionMatrix& other) const;
  
        void changeReferenceFrame(const fccl::kdl::Transform& transform);
        bool transformationPossible(const fccl::kdl::Transform& transform) const;
  
        friend std::ostream& operator<<(std::ostream& os, 
            const fccl::kdl::InteractionMatrix& interaction_matrix);
  
      private:
        // reference frame w.r.t. the twists are defined
        std::size_t reference_id_;
  
        // name of the functions for which we're holding the derivatives
        std::vector<std::size_t> target_ids_;
  
        // actual numeric representation of interaction matrix
        Eigen::Matrix< double, Eigen::Dynamic, 6 > data_;
    };
  
    // auxiliary function providing transposed twist transformation matrix
    Eigen::Matrix< double, 6, 6> getTransposedTwistTransformationMatrix(const KDL::Frame& frame);
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_INTERACTION_MATRIX_H
