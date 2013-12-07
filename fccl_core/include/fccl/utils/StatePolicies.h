#ifndef FCCL_UTILS_STATE_POLICIES_H
#define FCCL_UTILS_STATE_POLICIES_H

#include <limits>
#include <math.h>
#include <iostream>

using namespace std;

namespace fccl
{
  namespace utils
  {
    // TODO(Georg): move this to another file
    inline bool areEqual(double a, double b)
    {
      return fabs(a-b) < std::numeric_limits<double>::epsilon();
    }

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

      protected:
        T position_;

        ~PositionState() {}

        const PositionState& state() const 
        {
          return *this;
        }
    };

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
      
      protected:
        T velocity_;
        
        ~VelocityState() {}
        
        const VelocityState& state() const
        {
          return *this;
        }
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
      
      protected:
        T acceleration_;
        
        ~AccelerationState() {}
        
        const AccelerationState& state() const
        {
          return *this;
        }
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

    typedef PositionState<double> DoublePositionState;
    typedef VelocityState<double> DoubleVelocityState;
    typedef AccelerationState<double> DoubleAccelerationState;
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_STATE_POLICIES_H
