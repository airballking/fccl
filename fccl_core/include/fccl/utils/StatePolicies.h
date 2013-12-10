#ifndef FCCL_UTILS_STATE_POLICIES_H
#define FCCL_UTILS_STATE_POLICIES_H

#include <iostream>
#include <fccl/utils/Equalities.h>

using namespace std;

namespace fccl
{
  namespace utils
  {
    template <class T>
    class PositionState
    {
      public: 
        const T& position() const 
        { 
          return this->position_;
        }

        T& position()
        {
          return this->position_;
        }

        virtual bool equals(const PositionState& other) const
        {
          return areEqual(this->position(), other.position());
        }

        PositionState& operator+=(const PositionState& rhs)
        {
          this->position() += rhs.position();
          return *this;
        }

        PositionState& operator-=(const PositionState& rhs)
        {
          this->position() -= rhs.position();
          return *this;
        }

      protected:
        T position_;

        ~PositionState() {}
    };

    // TODO(Georg): think about solving this through composition
    template <class T>
    class VelocityState : public PositionState<T>
    {
      public:
        const T& velocity() const
        {
          return this->velocity_;
        }
 
        T& velocity()
        {
          return this->velocity_;
        }

        virtual bool equals(const VelocityState& other) const
        {
          return static_cast< const PositionState<T>& >(*this).equals(
              static_cast< const PositionState<T>& >(other)) &&
              areEqual(this->velocity(), other.velocity());
        }

        VelocityState& operator+=(const VelocityState& rhs)
        {
          static_cast< PositionState<T>& >(*this) +=
              static_cast< const PositionState<T>& >(rhs);
          this->velocity() += rhs.velocity();
          return *this;
        }

        VelocityState& operator-=(const VelocityState& rhs)
        {
          static_cast< PositionState<T>& >(*this) -=
              static_cast< const PositionState<T>& >(rhs);
          this->velocity() -= rhs.velocity();
          return *this;
        }
     
      protected:
        T velocity_;
        
        ~VelocityState() {}
    };
 
    template <class T>
    class AccelerationState : public VelocityState<T>
    {
      public:
        const T& acceleration() const
        {
          return this->acceleration_;
        }
 
        T& acceleration()
        {
          return this->acceleration_;
        }

        virtual bool equals(const AccelerationState& other) const
        {
          return static_cast< const VelocityState<T>& >(*this).equals(
              static_cast< const VelocityState<T>& >(other)) &&
              areEqual(this->acceleration(), other.acceleration());
        }

        AccelerationState& operator+=(const AccelerationState& rhs)
        {
          static_cast< VelocityState<T>& >(*this) +=
              static_cast< const VelocityState<T>& >(rhs);
          this->acceleration() += rhs.acceleration();
          return *this;
        }

        AccelerationState& operator-=(const AccelerationState& rhs)
        {
          static_cast< VelocityState<T>& >(*this) -=
              static_cast< const VelocityState<T>& >(rhs);
          this->acceleration() -= rhs.acceleration();
          return *this;
        }
      
      protected:
        T acceleration_;
        
        ~AccelerationState() {}
    };

    template <class T>
    class JerkState : public AccelerationState<T>
    {
      public:
        const T& jerk() const
        {
          return this->jerk_;
        }
 
        T& jerk()
        {
          return this->jerk_;
        }

        virtual bool equals(const JerkState& other) const
        {
          return static_cast< const AccelerationState<T>& >(*this).equals(
              static_cast< const AccelerationState<T>& >(other)) &&
              areEqual(this->jerk(), other.jerk());
        }

        JerkState& operator+=(const JerkState& rhs)
        {
          static_cast< AccelerationState<T>& >(*this) +=
              static_cast< const AccelerationState<T>& >(rhs);
          this->jerk() += rhs.jerk();
          return *this;
        }

        JerkState& operator-=(const JerkState& rhs)
        {
          static_cast< AccelerationState<T>& >(*this) -=
              static_cast< const AccelerationState<T>& >(rhs);
          this->jerk() -= rhs.jerk();
          return *this;
        }
      
      protected:
        T jerk_;
        
        ~JerkState() {}
    };


    template<class T>
    inline ostream& operator<<(ostream& os, const PositionState<T>& state)
    {
      os << "position: " << state.position();
      return os;
    }

    template<class T>
    inline ostream& operator<<(ostream& os, const VelocityState<T>& state)
    {
      os << static_cast< const PositionState<T>& >(state) << "\n";
      os << "velocity: " << state.velocity();
      return os;
    }

    template<class T>
    inline ostream& operator<<(ostream& os, const AccelerationState<T>& state)
    {
      os << static_cast< const VelocityState<T>& >(state) << "\n";
      os << "acceleration: " << state.acceleration();
      return os;
    }

    template<class T>
    inline ostream& operator<<(ostream& os, const JerkState<T>& state)
    {
      os << static_cast< const AccelerationState<T>& >(state) << "\n";
      os << "jerk: " << state.jerk();
      return os;
    }

    template<class T>
    inline PositionState<T> operator+(PositionState<T> lhs, 
        const PositionState<T>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T>
    inline VelocityState<T> operator+(VelocityState<T> lhs, 
        const VelocityState<T>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T>
    inline AccelerationState<T> operator+(AccelerationState<T> lhs, 
        const AccelerationState<T>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T>
    inline JerkState<T> operator+(JerkState<T> lhs, 
        const JerkState<T>& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    template<class T>
    inline PositionState<T> operator-(PositionState<T> lhs, 
        const PositionState<T>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    template<class T>
    inline VelocityState<T> operator-(VelocityState<T> lhs, 
        const VelocityState<T>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    template<class T>
    inline AccelerationState<T> operator-(AccelerationState<T> lhs, 
        const AccelerationState<T>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    template<class T>
    inline JerkState<T> operator-(JerkState<T> lhs, 
        const JerkState<T>& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    typedef PositionState<double> DoublePositionState;
    typedef VelocityState<double> DoubleVelocityState;
    typedef AccelerationState<double> DoubleAccelerationState;
    typedef JerkState<double> DoubleJerkState;
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_STATE_POLICIES_H
