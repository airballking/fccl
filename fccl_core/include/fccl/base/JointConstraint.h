#ifndef FCCL_BASE_JOINT_CONSTRAINT_H
#define FCCL_BASE_JOINT_CONSTRAINT_H

#include <fccl/kdl/Joint.h>

namespace fccl
{
  namespace base
  {
    // TODO(Georg): think about templating Constraint
    class JointConstraint
    {
      public:
        const fccl::kdl::Joint& joint() const
        {
          return joint_;
        }

        fccl::kdl::Joint& joint()
        {
          return joint_;
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

        bool equals(const JointConstraint& other) const
        {
          using fccl::kdl::areEqual;

          return joint().equals(other.joint()) && 
              areEqual(lowerBoundary(), other.lowerBoundary()) &&
              areEqual(upperBoundary(), other.upperBoundary());
        }

        double calculateWeight() const
        {
          if(joint().position() > upperBoundary() || 
              joint().position() < lowerBoundary())
          {
            return 1.0;
          }
          else
          {
            double w_lo = (1/margin())*(-upperBoundary() + joint().position())+1;
            double w_hi = (1/margin())*( lowerBoundary() - joint().position())+1;
        
            w_lo = (w_lo > 0.0) ? w_lo : 0.0;
            w_hi = (w_hi > 0.0) ? w_hi : 0.0;
        
            return (w_lo > w_hi) ? w_lo : w_hi;
          }
        }

        double calculateDesiredOutput() const
        {
          if(joint().position() > adjustedUpperBoundary())
          {
            return adjustedUpperBoundary();
          }
          else if(joint().position() < adjustedLowerBoundary())
          {
            return adjustedLowerBoundary();
          }
          else
          {
            return joint().position();
          }
        }

      private:
        // internal state of the joint-constraint
        fccl::kdl::Joint joint_;
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
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_JOINT_CONSTRAINT_H
