#ifndef FCCL_BASE_JOINT_CONSTRAINT_H
#define FCCL_BASE_JOINT_CONSTRAINT_H

#include <fccl/kdl/Joint.h>

using namespace fccl::kdl;

namespace fccl
{
  namespace base
  {
    // TODO(Georg): connect constraint-values with traits of template T
    // TODO(Georg): extract calculation of weight and desired value in policy
    template<class T>
    class Constraint
    {
      public:
        const T& output() const
        {
          return output_;
        }

        T& output()
        {
          return output_;
        }

        double lowerBoundary() const
        {
          return lower_boundary_;
        }

        double& lowerBoundary() 
        {
          return lower_boundary_;
        }

        double upperBoundary() const
        {
          return upper_boundary_;
        }

        double& upperBoundary() 
        {
          return upper_boundary_;
        }

        bool equals(const Constraint& other) const
        {
          using fccl::utils::areEqual;

          return output().equals(other.output()) && 
              areEqual(lowerBoundary(), other.lowerBoundary()) &&
              areEqual(upperBoundary(), other.upperBoundary());
        }

        double calculateWeight() const
        {
          if(output().position() > upperBoundary() || 
              output().position() < lowerBoundary())
          {
            return 1.0;
          }
          else
          {
            double w_lo = (1/margin())*(-upperBoundary() + output().position())+1;
            double w_hi = (1/margin())*( lowerBoundary() - output().position())+1;
        
            w_lo = (w_lo > 0.0) ? w_lo : 0.0;
            w_hi = (w_hi > 0.0) ? w_hi : 0.0;
        
            return (w_lo > w_hi) ? w_lo : w_hi;
          }
        }

        double calculateDesiredOutput() const
        {
          if(output().position() > adjustedUpperBoundary())
          {
            return adjustedUpperBoundary();
          }
          else if(output().position() < adjustedLowerBoundary())
          {
            return adjustedLowerBoundary();
          }
          else // constraint is fulfilled
          {
            return output().position();
          }
        }

      private:
        // internal state of the output-constraint
        T output_;
        double lower_boundary_;
        double upper_boundary_;

        double adjustedUpperBoundary() const
        {
          return upperBoundary() - margin();
        }

        double adjustedLowerBoundary() const
        {
          return lowerBoundary() + margin();
        }

        double defaultMargin() const
        {
          // TODO(Georg): refactor this param into something constraint-specific
          return 0.05;
        }

        double margin() const
        {
          // adjust default-margin if range is too small
          return (range() < 2*defaultMargin()) ? range()/2 : defaultMargin();
        }

        double range() const
        {
          return upperBoundary() - lowerBoundary();
        }
    };

    typedef Constraint<PositionJoint> JointConstraint;
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_JOINT_CONSTRAINT_H
