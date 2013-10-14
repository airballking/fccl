#ifndef FCCL_SEMANTICS_JNTARRAY_SEMANTICS_H
#define FCCL_SEMANTICS_JNTARRAY_SEMANTICS_H

#include <fccl/semantics/SemanticsBase.h>
#include <iostream>
#include <string>
#include <vector>

namespace fccl
{
  namespace semantics
  {
    class JntArraySemantics
    {
      public:
        void init(const std::vector<std::string>& joint_names)
        {
          resize(joint_names.size());

          for(std::size_t i=0; i<size(); i++)
            this->operator()(i).setName(joint_names[i]);
        }

        std::size_t size() const
        {
          return joints_.rows();
        }

        void resize(std::size_t new_size)
        {
          joints_.resize(new_size, 1);
        }

        const SemanticsBase& operator()(std::size_t index) const
        {
          return joints_(index, 0);
        }

        SemanticsBase& operator()(std::size_t index)
        {
          return joints_(index, 0);
        }

        Eigen::Matrix< SemanticsBase, Eigen::Dynamic, 1>& joints()
        {
          return joints_;
        }

        const Eigen::Matrix< SemanticsBase, Eigen::Dynamic, 1>& joints() const
        {
          return joints_;
        }

        bool equals(const JntArraySemantics& other) const
        {
          if(size() != other.size())
            return false;

          for(std::size_t i=0; i<size(); i++)
            if(!(*this)(i).equals(other(i)))
              return false;

          return true;
        }

        void partialAssignment(std::size_t start_index, std::size_t numberOfElements,
            const JntArraySemantics& other)
        {
          joints().segment(start_index, numberOfElements) =
              other.joints();
        }
          
      private:
        Eigen::Matrix< SemanticsBase, Eigen::Dynamic, 1> joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const JntArraySemantics& obj)
    {
      for(std::size_t i=0; i<obj.size(); i++)
      {
        os << obj(i);
        if(i < (obj.size() - 1))
          os << "\n";
      }
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_JNTARRAY_SEMANTICS_H
