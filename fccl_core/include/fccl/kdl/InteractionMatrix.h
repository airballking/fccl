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
    class InteractionMatrix
    {
      public:
        InteractionMatrix();
        InteractionMatrix(const fccl::kdl::InteractionMatrix& other);
        InteractionMatrix(const std::string& reference_name, unsigned int number_of_rows);
        InteractionMatrix(std::size_t reference_id, unsigned int number_of_rows);
        InteractionMatrix(const std::string& reference_name, const 
            std::vector<std::string>& target_names, 
            const Eigen::Matrix<double, Eigen::Dynamic, 6>& data);
        InteractionMatrix(std::size_t reference_id, const std::vector<std::size_t>&
            target_ids, const Eigen::Matrix<double, Eigen::Dynamic, 6>& data);
  
        virtual ~InteractionMatrix();
  
        fccl::kdl::InteractionMatrix& operator=(const fccl::kdl::InteractionMatrix& rhs);
  
        std::size_t getReferenceID() const;
        void setReferenceID(std::size_t reference_id);
        void setReferenceName(const std::string& reference_name);
  
        const std::vector<std::size_t>& getTargetIDs() const;
        void setTargetIDs(const std::vector<std::size_t>& target_ids);
        void setTargetNames(const std::vector<std::string>& target_names);
        void setTargetID(unsigned int row, std::size_t target_id);
        void setTargetName(unsigned int row, const std::string& target_name);
  
        const Eigen::Matrix< double, Eigen::Dynamic, 6>& getData() const;
        void setData(const Eigen::Matrix< double, Eigen::Dynamic, 6>& data);
  
        void resize(unsigned int number_of_rows);
  
        unsigned int rows() const;
        unsigned int columns() const;
  
        double& operator()(unsigned int row, unsigned int column);
        double operator()(unsigned int row, unsigned int column) const;
  
        fccl::kdl::Twist getRow(unsigned int row) const;
        void setRow(unsigned int row, const fccl::kdl::Twist& twist);
  
        bool hasDerivative(const std::string& target_name) const;
        bool hasDerivative(std::size_t target_id) const;
        bool hasDerivative(const fccl::kdl::Twist& twist) const;
  
        unsigned int getRowNumber(const std::string& target_name) const;
        unsigned int getRowNumber(std::size_t target_id) const;
        unsigned int getRowNumber(const fccl::kdl::Twist& twist) const;
  
        bool operator==(const fccl::kdl::InteractionMatrix& other) const;
        bool operator!=(const fccl::kdl::InteractionMatrix& other) const;
  
        bool semanticsEqual(const fccl::kdl::InteractionMatrix& other) const;
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
