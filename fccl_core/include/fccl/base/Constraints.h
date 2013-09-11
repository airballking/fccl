#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl/base/Features.h>
#include <fccl/base/ConstraintTypes.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/InteractionMatrix.h>
#include <fccl/kdl/Semantics.h>
#include <string>
#include <iostream>

namespace fccl
{
  namespace base
  {
    class Constraint : public fccl::kdl::SemanticObject1x1
    {
      public:
        Constraint();
        Constraint(const Constraint& other);
        Constraint(const SemanticObject1x1& semantics, const fccl::base::Feature& 
            tool_feature, const fccl::base::Feature& object_feature,
            double lower_boundary, double upper_boundary);
  
        virtual ~Constraint();
  
        Constraint& operator=(const Constraint& rhs);
  
        const fccl::base::Feature& getToolFeature() const;
        void setToolFeature(const fccl::base::Feature& tool_feature);
  
        const fccl::base::Feature& getObjectFeature() const;
        void setObjectFeature(const fccl::base::Feature& object_feature);
  
        double getLowerBoundary() const;
        void setLowerBoundary(double lower_boundary);
  
        double getUpperBoundary() const;
        void setUpperBoundary(double upper_boundary);
  
        int getType() const;
  
        virtual double calculateValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) const;
  
        const fccl::kdl::InteractionMatrix& calculateFirstDerivative(const fccl::kdl::Transform&
            tool_transform, const fccl::kdl::Transform& object_transform, double 
            delta=0.001); 
  
        bool operator==(const Constraint& other) const;
        bool operator!=(const Constraint& other) const;
  
        virtual bool semanticsEqual(const Constraint& other) const;
        bool numericsEqual(const Constraint& other) const;
  
        friend std::ostream& operator<<(std::ostream& os, const Constraint& constraint);
  
      protected:
        // feature on the controllable tool
        fccl::base::Feature tool_feature_;
     
        // feature on the non-controllable object
        fccl::base::Feature object_feature_;
  
        // lower boundary for output of feature function
        double lower_boundary_;
  
        // upper boundary for output of feature function
        double upper_boundary_;
  
        // type information to discriminate between constraints
        int type_;
  
        // memory to store first derivative of constraint
        fccl::kdl::InteractionMatrix first_derivative_;
    };
  
    class AboveConstraint : public Constraint
    {
      public:
        AboveConstraint();
        AboveConstraint(const AboveConstraint& other);
        AboveConstraint(const SemanticObject1x1& semantics, 
            const fccl::base::Feature& tool_feature,
            const fccl::base::Feature& object_feature,
            double lower_boundary, double upper_boundary);
  
        virtual ~AboveConstraint();
  
        virtual double calculateValue(const fccl::kdl::Transform& tool_transform,
            const fccl::kdl::Transform& object_transform) const;
    };
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
