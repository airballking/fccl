#include <fccl_kdl/InteractionMatrix.h>
#include <fccl_utils/Hashing.h>
#include <fccl_utils/Equalities.h>

namespace fccl
{

  InteractionMatrix::InteractionMatrix()
  {
  }

  InteractionMatrix::InteractionMatrix(const fccl::InteractionMatrix& other) :
      reference_id_(other.getReferenceID()), target_ids_(other.getTargetIDs()),
      data_(other.getData())
  {
  }

  InteractionMatrix::InteractionMatrix(const std::string& reference_name, unsigned int number_of_rows) :
      reference_id_(hash(reference_name))
  {
    target_ids_.resize(number_of_rows);
    data_.resize(number_of_rows, 6);
  }

  InteractionMatrix::InteractionMatrix(std::size_t reference_id, unsigned int number_of_rows) :
      reference_id_(reference_id)
   {
    target_ids_.resize(number_of_rows);
    data_.resize(number_of_rows, 6);
  }

  InteractionMatrix::InteractionMatrix(const std::string& reference_name, const 
      std::vector<std::string>& target_names, 
      const Eigen::Matrix<double, Eigen::Dynamic, 6>& data) :
      reference_id_(hash(reference_name)), data_(data)
  {
    assert(data_.rows() == target_names.size());

    target_ids_.resize(target_names.size());
    for(unsigned int i=0; i<target_names.size(); i++)
      target_ids_[i] = hash(target_names[i]);
  }

  InteractionMatrix::InteractionMatrix(std::size_t reference_id, const std::vector<std::size_t>& target_ids, const Eigen::Matrix<double, Eigen::Dynamic, 6>& data) :
      reference_id_(reference_id), target_ids_(target_ids), data_(data)
  {
    assert(target_ids_.size() == data_.rows());
  }

  InteractionMatrix::~InteractionMatrix()
  {
  }

  fccl::InteractionMatrix& InteractionMatrix::operator=(const fccl::InteractionMatrix& rhs)
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

  std::size_t InteractionMatrix::getReferenceID() const
  {
    return reference_id_;
  }

  void InteractionMatrix::setReferenceID(std::size_t reference_id)
  {
    reference_id_ = reference_id;
  }

  void InteractionMatrix::setReferenceName(const std::string& reference_name)
  {
    reference_id_ = hash(reference_name);
  }

  const std::vector<std::size_t>& InteractionMatrix::getTargetIDs() const
  {
    return target_ids_;
  }

  void InteractionMatrix::setTargetIDs(const std::vector<std::size_t> target_ids)
  {
    assert(target_ids_.size() == target_ids.size());
  
    for(unsigned int i=0; i<target_ids_.size(); i++)
      target_ids_[i] = target_ids[i];
  }

  void InteractionMatrix::setTargetNames(const std::vector<std::string> target_names)
  {
    assert(target_ids_.size() == target_names.size());

    for(unsigned int i=0; i<target_ids_.size(); i++)
      target_ids_[i] = hash(target_names[i]);
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
    target_ids_.resize(number_of_rows);
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

  fccl::Twist InteractionMatrix::getRow(unsigned int row) const
  {
    assert(row < rows());
    assert(rows() == target_ids_.size());
 
    fccl::Twist result;
    result.setReferenceID(reference_id_);
    result.setTargetID(target_ids_[row]);

    KDL::Vector trans(data_(row,0), data_(row,1), data_(row,2));
    KDL::Vector angular(data_(row,3), data_(row,4), data_(row,5));

    result.setTwist(KDL::Twist(trans, angular)); 
  
    return result;
  }

  void InteractionMatrix::setRow(unsigned int row, const fccl::Twist& twist)
  {
    assert(row < rows());
    assert(rows() == target_ids_.size());

    target_ids_[row] = twist.getTargetID();
    reference_id_ = twist.getReferenceID();

    data_(row, 0) = twist.getTwist().vel.x();
    data_(row, 1) = twist.getTwist().vel.y();
    data_(row, 2) = twist.getTwist().vel.z();
    data_(row, 3) = twist.getTwist().rot.x();
    data_(row, 4) = twist.getTwist().rot.y();
    data_(row, 5) = twist.getTwist().rot.z();

  }

  bool InteractionMatrix::hasDerivative(const std::string& target_name) const
  {
    return hasDerivative(hash(target_name));  
  }

  bool InteractionMatrix::hasDerivative(std::size_t target_id) const
  {
    return (getRowNumber(target_id) < rows());
  }

  bool InteractionMatrix::hasDerivative(const fccl::Twist& twist) const
  {
    return hasDerivative(twist.getReferenceID());
  }

  unsigned int InteractionMatrix::getRowNumber(const std::string& target_name) const
  {
    return getRowNumber(hash(target_name));
  }

  unsigned int InteractionMatrix::getRowNumber(std::size_t target_id) const
  {
    for(unsigned int i=0; i<target_ids_.size(); i++)
      if(target_id == target_ids_[i])
        return i;

    // search failed, return number of rows as error code
    return rows();
  }

  unsigned int InteractionMatrix::getRowNumber(const fccl::Twist& twist) const
  {
    return getRowNumber(twist.getTargetID());
  }

  bool InteractionMatrix::operator==(const fccl::InteractionMatrix& other) const
  {
    return semanticsEqual(other) && numericsEqual(other);
  }

  bool InteractionMatrix::operator!=(const fccl::InteractionMatrix& other) const
  {
    return !(*this == other);
  }

  bool InteractionMatrix::semanticsEqual(const fccl::InteractionMatrix& other) const
  {
    return target_ids_ == other.getTargetIDs()
        && reference_id_ == other.getReferenceID();
  }

  bool InteractionMatrix::numericsEqual(const fccl::InteractionMatrix& other) const
  {
    return data_.isApprox(other.getData());
  }

  void InteractionMatrix::changeReferenceFrame(const fccl::Transform& transform)
  {
    // take care of semantics
    assert(transformationPossible(transform));
    reference_id_ = transform.getReferenceID();

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
    data_ = data_*Mi;
  }

  bool InteractionMatrix::transformationPossible(const fccl::Transform& transform) const
  {
    return reference_id_ == transform.getTargetID();
  }

  std::ostream& operator<<(std::ostream& os, const fccl::InteractionMatrix& interaction_matrix)
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

} // namespace fccl
