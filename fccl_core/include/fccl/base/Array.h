#ifndef FCCL_BASE_ARRAY_H
#define FCCL_BASE_ARRAY_H

#include <Eigen/Core>

using namespace std;

namespace fccl
{
  namespace base
  {
// TODO(Georg): add policy for validity checking
// TODO(Georg): add policy for updates
    template <class T>
    class Array
    {
      public:
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

    template<class T>
    inline ostream& operator<<(ostream& os, const Array<T>& obj)
    {
      for(size_t i=0; i<obj.size(); i++)
        os << obj(i) << "\n";

      return os;
    }

    template<class T>
    inline Array<T> operator+(Array<T> lhs, const Array<T>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T>
    inline Array<T> operator-(Array<T> lhs, const Array<T>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_ARRAY_H
