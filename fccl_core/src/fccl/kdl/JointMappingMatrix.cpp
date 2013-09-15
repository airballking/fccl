#include <fccl/kdl/JointMappingMatrix.h>
#include <fccl/utils/Printing.h>

namespace fccl
{
  namespace kdl
  {
    JointMappingMatrix::JointMappingMatrix() : SemanticObjectNxM()
    {
      assert(isValid());
    }

    JointMappingMatrix::JointMappingMatrix(const JointMappingMatrix& other) :
        SemanticObjectNxM(other), data_(other.getData())
    {
      assert(isValid());
    }

    JointMappingMatrix::JointMappingMatrix(const SemanticObjectNxM& semantics, 
            const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& data) :
        SemanticObjectNxM(semantics), data_(data)
    {
      assert(isValid());
    }

    JointMappingMatrix::~JointMappingMatrix()
    {
    }

    const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& JointMappingMatrix::getData() const
    {
      return data_;
    }

    void JointMappingMatrix::setData(const Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& data)
    {
      assert(rows() == data.rows());
      assert(columns() == data.cols());

      data_ = data;
    }

    std::pair<std::size_t, std::size_t> JointMappingMatrix::size() const
    {
      assert(isValid());
      return SemanticObjectNxM::size();
    }

    void JointMappingMatrix::resize(const std::pair<std::size_t, std::size_t>& new_size)
    {
      SemanticObjectNxM::resize(new_size);
      data_.resize(new_size.first, new_size.second);
    }

    bool JointMappingMatrix::isValid() const
    {
      return (getData().rows() == sizeTargets()) && (getData().cols() == sizeReferences());
    }

    std::size_t JointMappingMatrix::rows() const
    {
      assert(data_.rows() == sizeTargets());

      return data_.rows();
    }

    std::size_t JointMappingMatrix::columns() const
    {
      assert(data_.cols() == sizeReferences());

      return data_.cols();
    }

    bool JointMappingMatrix::rowIndexValid(std::size_t row) const
    {
      return row < rows();
    }

    bool JointMappingMatrix::columnIndexValid(std::size_t column) const
    {
      return column < columns();
    }    

    double& JointMappingMatrix::operator()(std::size_t row, std::size_t column)
    {
      assert(rowIndexValid(row));
      assert(columnIndexValid(column));

      return data_(row, column);
    }

    double JointMappingMatrix::operator()(std::size_t row, std::size_t column) const
    {
      assert(rowIndexValid(row));
      assert(columnIndexValid(column));

      return data_(row, column);
    }

    bool JointMappingMatrix::operator==(const JointMappingMatrix& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }

    bool JointMappingMatrix::operator!=(const JointMappingMatrix& other) const
    {
      return !(*this == other);
    }

    bool JointMappingMatrix::numericsEqual(const JointMappingMatrix& other) const
    {
      return getData().isApprox(other.getData());
    }

    std::ostream& operator<<(std::ostream& os, const JointMappingMatrix& joint_mapping_matrix)
    {
      os << "data:\n" << joint_mapping_matrix.getData() << "\n";
      os << "semantics:\n" << joint_mapping_matrix.getSemantics();

      return os;
    }
  } // namespace kdl 
} // namespace fccl
