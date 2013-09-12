#include <fccl/kdl/Jacobian.h>

namespace fccl
{
  namespace kdl
  {
    Jacobian::Jacobian() : 
        SemanticObject1xN()
    {
    }

    Jacobian::Jacobian(std::size_t columns) :
        SemanticObject1xN()
    {
      resize(columns);
    }

    Jacobian::Jacobian(const Jacobian& other) :
        SemanticObject1xN(other), jacobian_(other.getData())
    {
      assert(isValid());
    }

    Jacobian::Jacobian(const SemanticObject1xN& semantics, 
            const KDL::Jacobian& data) :
        SemanticObject1xN(semantics), jacobian_(data)
    {
      assert(isValid());
    }

    Jacobian::~Jacobian()
    {
    }

    Jacobian& Jacobian::operator=(const Jacobian& other)
    {
      if(this != &other)
      {
        assert(other.isValid());

        setSemantics(other.getSemantics());
        setData(other.getData());
      }

      return *this;
    }

    const KDL::Jacobian& Jacobian::getData() const
    {
      return jacobian_;
    }

    void Jacobian::setData(const KDL::Jacobian& jacobian)
    {
      assert(rows() == jacobian.rows());
      assert(columns() == jacobian.columns());

      jacobian_ = jacobian;
    }

    void Jacobian::resize(std::size_t rows)
    {
      jacobian_.resize(rows);
      SemanticObject1xN::resize(rows);
    }

    std::size_t Jacobian::size() const
    {
      return columns();
    }

    bool Jacobian::isValid() const
    {
      return (SemanticObject1xN::size() == jacobian_.columns());
    }
  
    std::size_t Jacobian::rows() const
    {
      return jacobian_.rows();
    }

    std::size_t Jacobian::columns() const
    {
      assert(isValid());

      return jacobian_.columns();
    }

    bool Jacobian::rowIndexValid(std::size_t row) const
    {
      return row < rows();
    }

    bool Jacobian::columnIndexValid(std::size_t column) const
    {
      return column < columns();
    }
 
    double& Jacobian::operator()(unsigned int row, unsigned int column)
    {
      assert(rowIndexValid(row));
      assert(columnIndexValid(column));

      return jacobian_(row, column);
    }

    double Jacobian::operator()(unsigned int row, unsigned int column) const
    {
      assert(rowIndexValid(row));
      assert(columnIndexValid(column));

      return jacobian_(row, column);
    }

    fccl::kdl::Twist Jacobian::getColumn(unsigned int column) const
    {
      assert(columnIndexValid(column));

      fccl::kdl::Twist result;
      result.setReferenceID(getReferenceID());
      result.setTargetID(getTargetID(column));
      result.setTwist(getData().getColumn(column));

      return result;
    }

    void Jacobian::setColumn(unsigned int column, const fccl::kdl::Twist& twist)
    {
      assert(columnIndexValid(column));
      assert(twist.getReferenceID() == getReferenceID());
      
      setTargetID(column, twist.getTargetID());
      jacobian_.setColumn(column, twist.getTwist());
    }

    bool Jacobian::operator==(const Jacobian& other) const
    {
      return semanticsEqual(other) && numericsEqual(other);
    }

    bool Jacobian::operator!=(const Jacobian& other) const
    {
      return !(*this == other);
    }

    bool Jacobian::numericsEqual(const Jacobian& other) const
    {
      return getData() == other.getData();
    }

    void Jacobian::changeReferenceFrame(const fccl::kdl::Transform& transform)
    {
      assert(transformationPossible(transform));

      jacobian_.changeRefFrame(transform.getTransform());
    }

    bool Jacobian::transformationPossible(const fccl::kdl::Transform& transform) const
    {
      return getReferenceID() == transform.getTargetID();
    }

    std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian)
    {
      os << "data:\n";
      os << jacobian.getData().data << "\n";
      os << "semantics:\n";
      os << jacobian.getSemantics();
      return os;
    }
  } // namespace kdl 
} // namespace fccl
