#include <fccl/kdl/JntArray.h>

namespace fccl
{
  namespace kdl
  {
    JntArray::JntArray() : SemanticObjectN()
    {
    }

    JntArray::JntArray(const JntArray& other) : 
        SemanticObjectN(other), data_(other.getData())
    {
      assert(other.isValid());
    }

    JntArray::JntArray(const SemanticObjectN& semantics, const KDL::JntArray& data) :
        SemanticObjectN(semantics), data_(data)
    {
      assert(isValid());
    }

    JntArray::~JntArray()
    {
    }

    JntArray& JntArray::operator=(const JntArray& other)
    {
      assert(other.isValid());

      setSemantics(other.getSemantics());
      setData(other.getData());
    }

    SemanticObjectN JntArray::getSemantics() const
    {
      return SemanticObjectN(*this);
    }

    void JntArray::setSemantics(const SemanticObjectN& semantics)
    {
      SemanticObjectN* semantic_pointer = 
          static_cast<SemanticObjectN*>(this);

      *semantic_pointer = semantics;
    }

    const KDL::JntArray& JntArray::getData() const
    {
      return data_;
    }

    void JntArray::setData(const KDL::JntArray& data)
    {
      assert(data.rows() == size());

      data_ = data;
    }

    bool JntArray::numericsEqual(const JntArray& other) const
    {
      return KDL::Equal(getData(), other.getData());
    }

    bool JntArray::operator==(const JntArray& other) const
    {
      return (semanticsEqual(other) && numericsEqual(other));
    }

    bool JntArray::operator!=(const JntArray& other) const
    {
      return !(*this == other);
    }

    void JntArray::resize(std::size_t new_size)
    {
      SemanticObjectN::resize(new_size);
      data_.resize(new_size);
    }

    bool JntArray::isValid() const
    {
      return SemanticObjectN::size() == size();
    }

    std::ostream& operator<<(std::ostream& os, const JntArray& jnt_array)
    {
      os << "joints\n******\n";
      for(unsigned int i=0; i<jnt_array.size(); i++)
      {
        os << jnt_array.data_(i) << " " << jnt_array.getTargetName(i) << " (";
        os << jnt_array.getTargetID(i) << ")";
        if(i < (jnt_array.size() - 1 ))
          os << "\n";
      }
 
      return os;
    }
  } // namespace kdl
} // namespace fccl
