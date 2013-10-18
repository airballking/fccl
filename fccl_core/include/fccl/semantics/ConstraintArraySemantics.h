#ifndef FCCL_SEMANTICS_CONSTRAINT_ARRAY_SEMANTICS_H
#define FCCL_SEMANTICS_CONSTRAINT_ARRAY_SEMANTICS_H

#include <fccl/semantics/ConstraintSemantics.h>
#include <iostream>
#include <string>
#include <vector>

namespace fccl
{
  namespace semantics
  {
    class ConstraintArraySemantics
    {
      public:
        Eigen::Matrix< ConstraintSemantics, Eigen::Dynamic, 1>& constraints()
        {
          return constraints_;
        }

        const Eigen::Matrix< ConstraintSemantics, Eigen::Dynamic, 1>& constraints() const
        {
          return constraints_;
        }

        bool equals(const ConstraintArraySemantics& other) const
        {
          if(size() != other.size())
            return false;

          for(std::size_t i=0; i<size(); i++)
            if(!(*this)(i).equals(other(i)))
              return false;

          return true;
        }
 
        const ConstraintSemantics& operator()(std::size_t index) const
        {
          return constraints_(index, 0);
        }

        ConstraintSemantics& operator()(std::size_t index)
        {
          return constraints_(index, 0);
        }

        void init(const std::vector<ConstraintSemantics>& constraints)
        {
          resize(constraints.size());
          for(std::size_t i=0; i<size(); i++)
            this->operator()(i) = constraints[i];
        }

        std::size_t size() const
        {
          return constraints_.rows();
        }

        void resize(std::size_t new_size)
        {
          constraints_.resize(new_size, 1);
        }
 
        bool hasCommonReference() const
        {
          if(size() > 0)
          {
            SemanticsBase common_reference = this->operator()(0).reference();
            for(std::size_t i=0; i<size(); i++)
              if(!this->operator()(i).reference().equals(common_reference))
                return false;
          }

          return true; 
        }

        bool isValid() const
        {
          for(std::size_t i=0; i<size(); i++)
            if(!this->operator()(i).isValid())
              return false;

          return hasCommonReference();
        }

        // NOT REAL-TIME-SAFE
        JntArray joints() const
        {
          JntArray result;
          result.init(names());

          return result;
        } 

        // NOT REAL-TIME-SAFE
        std::vector<std::string> names() const
        {
          std::vector<std::string> result;

          for(std::size_t i=0; i<size(); i++)
            result.push_back(this->operator()(i).getName());

          return result;
        }

      private:
        Eigen::Matrix< ConstraintSemantics, Eigen::Dynamic, 1> constraints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const ConstraintArraySemantics& obj)
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
#endif // FCCL_SEMANTICS_CONSTRAINT_ARRAY_SEMANTICS_H
