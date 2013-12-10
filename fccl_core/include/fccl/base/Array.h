#ifndef FCCL_BASE_ARRAY_H
#define FCCL_BASE_ARRAY_H

#include <Eigen/Core>
#include <boost/type_traits.hpp>

using namespace std;

namespace fccl
{
  namespace base
  {
// TODO(Georg): add policy for validity checking
// TODO(Georg): add policy for updates
    template <class T, class SemanticsPolicy>
    class Array
    {
      public:
        template <class U>
        void init(const Array<U, SemanticsPolicy>& semantics_array)
        {
          BOOST_STATIC_ASSERT((boost::is_base_of<SemanticsPolicy, U>::value));
          BOOST_STATIC_ASSERT((boost::is_base_of<SemanticsPolicy, T>::value));

          this->resize(semantics_array.size());

          for(size_t i=0; i<this->size(); i++)
            // TODO(Georg): refactor this using as<SemanticsPolicy>()
            this->operator()(i).semantics() = semantics_array(i).semantics();
        }

        size_t size() const
        {
          this->data().rows();
        }

        void resize(size_t new_size)
        {
          this->data().resize(new_size, 1);
        }

        const T& operator()(size_t index) const
        {
          return this->data_(index, 0);
        }

        T& operator()(size_t index)
        {
          return this->data_(index, 0);
        }

        Eigen::Matrix< T, Eigen::Dynamic, 1>& data()
        {
          return this->data_;
        }

        const Eigen::Matrix< T, Eigen::Dynamic, 1>& data() const
        {
          return this->data_;
        }

        bool equals(const Array& other) const
        {
          if(this->size() != other.size())
            return false;

          for(size_t i=0; i<this->size(); i++)
            if(!(*this)(i).equals(other(i)))
              return false;

          return true;
        }

        void partialAssignment(size_t start, size_t elements, const Array& other)
        {
          this->data().segment(start, elements) = other.data();
        }

        Array& operator+=(const Array& rhs)
        {
          this->data() += rhs.data();
          return *this;
        }

        Array& operator-=(const Array& rhs)
        {
          this->data() -= rhs.data();
          return *this;
        }
 
      private:
        Eigen::Matrix<T, Eigen::Dynamic, 1> data_;
    };

    template<class T, class SemanticsPolicy>
    inline ostream& operator<<(ostream& os, const Array<T, SemanticsPolicy>& obj)
    {
      for(size_t i=0; i<obj.size(); i++)
        os << obj(i) << "\n";

      return os;
    }

    template<class T, class SemanticsPolicy>
    inline Array<T, SemanticsPolicy> operator+(Array<T, SemanticsPolicy> lhs,
        const Array<T, SemanticsPolicy>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T, class SemanticsPolicy>
    inline Array<T, SemanticsPolicy> operator-(Array<T, SemanticsPolicy> lhs,
        const Array<T, SemanticsPolicy>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_ARRAY_H
