#include <fccl/kdl/InteractionMatrix.h>

namespace fccl
{
  namespace kdl
  {
    InteractionMatrix::InteractionMatrix() : SemanticObject1xN()
    {
    }
  
    InteractionMatrix::InteractionMatrix(const fccl::kdl::InteractionMatrix& other) :
        SemanticObject1xN(other), data_(other.getData())
    {
      assert(rows() == targets()); 
    }
  
    InteractionMatrix::InteractionMatrix(const std::string& reference_name, const 
            std::vector<std::string>& target_names, 
            const Eigen::Matrix<double, Eigen::Dynamic, 6>& data) :
        SemanticObject1xN(reference_name, target_names), data_(data)
    {
      assert(rows() == targets()); 
    }
  
    InteractionMatrix::InteractionMatrix(std::size_t reference_id, const std::vector<std::size_t>& target_ids, const Eigen::Matrix<double, Eigen::Dynamic, 6>& data) :
        SemanticObject1xN(reference_id, target_ids), data_(data)
    {
      assert(rows() == targets()); 
    }
  
    InteractionMatrix::~InteractionMatrix()
    {
    }
  
    fccl::kdl::InteractionMatrix& InteractionMatrix::operator=(const fccl::kdl::InteractionMatrix& rhs)
    {
      // need to check for self-allocation
      if(this != &rhs)
      {
        this->resize(rhs.rows());
        this->setReferenceID(rhs.getReferenceID());
        this->setTargetIDs(rhs.getTargetIDs());
        this->setData(rhs.getData());
      }
  
      return *this;
    }
  
    const Eigen::Matrix< double, Eigen::Dynamic, 6>& InteractionMatrix::getData() const
    {
      return data_;
    }
  
    void InteractionMatrix::setData(const Eigen::Matrix< double, Eigen::Dynamic, 6>& data)
    {
      assert(rows() == data.rows());
      assert(columns() == data.cols());
  
      data_ = data;
    }
  
    void InteractionMatrix::resize(unsigned int number_of_rows)
    {
      resizeTargets(number_of_rows);
      data_.resize(number_of_rows, 6);
    }
  
    unsigned int InteractionMatrix::rows() const
    {
      return data_.rows();
    }
  
    unsigned int InteractionMatrix::columns() const
    {
      return data_.cols();
    }
  
    double& InteractionMatrix::operator()(unsigned int row, unsigned int column)
    {
      assert(row < rows());
      assert(column < columns());
  
      return data_(row, column); 
    }
  
    double InteractionMatrix::operator()(unsigned int row, unsigned int column) const
    {
      assert(row < rows());
      assert(column < columns());
  
      return data_(row, column); 
    }
  
    fccl::kdl::Twist InteractionMatrix::getRow(unsigned int row) const
    {
      assert(row < rows());
      assert(rows() == targets());
   
      Twist result;
      result.setReferenceID(getReferenceID());
      result.setTargetID(getTargetID(row));
  
      KDL::Vector trans(data_(row,0), data_(row,1), data_(row,2));
      KDL::Vector angular(data_(row,3), data_(row,4), data_(row,5));
  
      result.setTwist(KDL::Twist(trans, angular)); 
    
      return result;
    }
  
    void InteractionMatrix::setRow(unsigned int row, const fccl::kdl::Twist& twist)
    {
      assert(row < rows());
      assert(rows() == targets());
  
      setTargetID(row, twist.getTargetID());
      setReferenceID(twist.getReferenceID());
  
      data_(row, 0) = twist.getTwist().vel.x();
      data_(row, 1) = twist.getTwist().vel.y();
      data_(row, 2) = twist.getTwist().vel.z();
      data_(row, 3) = twist.getTwist().rot.x();
      data_(row, 4) = twist.getTwist().rot.y();
      data_(row, 5) = twist.getTwist().rot.z();
  
    }
  
    bool InteractionMatrix::operator==(const fccl::kdl::InteractionMatrix& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }
  
    bool InteractionMatrix::operator!=(const fccl::kdl::InteractionMatrix& other) const
    {
      return !(*this == other);
    }
  
    bool InteractionMatrix::numericsEqual(const fccl::kdl::InteractionMatrix& other) const
    {
      return getData().isApprox(other.getData());
    }
  
    void InteractionMatrix::changeReferenceFrame(const fccl::kdl::Transform& transform)
    {
      // take care of semantics
      assert(transformationPossible(transform));
      setReferenceID(transform.getReferenceID());
  
      // copy frame because some of Eigens functions do not guarantee constness
      KDL::Frame f = transform.getTransform();
  
      // (transposed) Rotation matrix of f
      Eigen::Matrix3d Rt = Eigen::Map<Eigen::Matrix3d>(f.M.data);
    
      double x = f.p.x(), y = f.p.y(), z = f.p.z();
    
      // Skew symmetric matrix of p, [p]_x for expressing a cross product
      Eigen::Matrix3d px;
      px << 0, -z,  y,
            z,  0, -x,
           -y,  x,  0;
    
      // the inverse twist projection matrix
      Eigen::Matrix<double, 6, 6> Mi;
      Mi.block<3,3>(0,0) = Rt;
      Mi.block<3,3>(3,3) = Rt;
      Mi.block<3,3>(3,0) = -Rt*px;
      Mi.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
  
      // actual multiplication
      setData(getData() * Mi);
    }
  
    bool InteractionMatrix::transformationPossible(const fccl::kdl::Transform& transform) const
    {
      return getReferenceID() == transform.getTargetID();
    }
  
    std::ostream& operator<<(std::ostream& os, const fccl::kdl::InteractionMatrix& interaction_matrix)
    {
      os << "data:\n";
      for(unsigned int row=0; row<interaction_matrix.rows(); row++)
      {
        for(unsigned int column=0; column<interaction_matrix.columns(); column++)
        {
          os << "  " << interaction_matrix(row, column);
        }
        os << "    (target_id: " << interaction_matrix.getTargetIDs()[row] << ")\n";
      }  
  
      os << "reference: " << interaction_matrix.getReferenceID() << "\n";
    }
  } // namespace kdl
} // namespace fccl
