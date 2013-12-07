#ifndef FCCL_SEMANTICS_SEMANTICS_BASE_H
#define FCCL_SEMANTICS_SEMANTICS_BASE_H

#include <Eigen/Core>
#include <string>
#include <iostream>
#include <fccl/utils/Hashing.h>

namespace fccl
{
  namespace semantics
  {
    class SemanticsBase
    {
      public:
        std::size_t getID() const
        {
          return id_;
        }

        void setID(std::size_t id)
        {
          id_ = id;
        }
     
        const std::string& getName() const
        {
          return fccl::utils::Hasher::retrieveValue(getID());
        }

        void setName(const std::string& name)
        {
          setID(fccl::utils::Hasher::hash(name));
        }

        virtual bool equals(const SemanticsBase& other) const
        {
          return getID() == other.getID();
        }

        const SemanticsBase& semantics() const
        {
          return *this;
        }

        SemanticsBase& semantics()
        {
          return *this;
        }

      private:
        std::size_t id_;
    };

    inline std::ostream& operator<<(std::ostream& os, const SemanticsBase& obj)
    {
      os << obj.getName() << " (" << obj.getID() << ")";
      return os;
    }

    inline bool operator==(const SemanticsBase& lhs, const SemanticsBase& rhs)
    { 
      return lhs.equals(rhs);
    }

    inline bool operator!=(const SemanticsBase& lhs, const SemanticsBase& rhs)
    {
      return !operator==(lhs,rhs);
    }

    // necessary to use SemanticsBase as key in std::map
    inline bool operator<(const SemanticsBase& lhs, const SemanticsBase& rhs)
    { 
      return lhs.getID() < rhs.getID();
    }

    inline bool operator>(const SemanticsBase& lhs, const SemanticsBase& rhs)
    {
      return operator<(rhs,lhs);
    }

    inline bool operator<=(const SemanticsBase& lhs, const SemanticsBase& rhs)
    {
      return !operator>(lhs,rhs);
    }

    inline bool operator>=(const SemanticsBase& lhs, const SemanticsBase& rhs)
    {
      return !operator<(lhs,rhs);
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_SEMANTICS_BASE_H
