#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl_base/Features.h>
#include <fccl_base/ConstraintTypes.h>
#include <fccl_kdl/Transform.h>
#include <fccl_kdl/InteractionMatrix.h>
#include <string>

namespace fccl
{
  class Constraint
  {
    public:
      Constraint();
      Constraint(const Constraint& other);
      Constraint(const std::string& reference_name, const std::string& name,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
          double lower_boundary, double upper_boundary);
      Constraint(std::size_t reference_id, std::size_t id,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature, 
          double lower_boundary, double upper_boundary);

      virtual ~Constraint();

      Constraint& operator=(const Constraint& rhs);

      std::size_t getReferenceID() const;
      void setReferenceName(const std::string& reference_name);
      void setReferenceID(std::size_t reference_id);

      std::size_t getID() const;
      void setName(const std::string& name);
      void setID(std::size_t id);

      const fccl::Feature& getToolFeature() const;
      void setToolFeature(const fccl::Feature& tool_feature);

      const fccl::Feature& getObjectFeature() const;
      void setObjectFeature(const fccl::Feature& object_feature);

      double getLowerBoundary() const;
      void setLowerBoundary(double lower_boundary);

      double getUpperBoundary() const;
      void setUpperBoundary(double upper_boundary);

      int getType() const;

      virtual double calculateValue(const fccl::Transform& tool_transform,
          const fccl::Transform& object_transform) const;

      const InteractionMatrix& calculateFirstDerivative(const fccl::Transform&
          tool_transform, const fccl::Transform& object_transform, double 
          delta=0.001); 

      bool operator==(const Constraint& other) const;
      bool operator!=(const Constraint& other) const;

      bool semanticsEqual(const Constraint& other) const;
      bool numericsEqual(const Constraint& other) const;

      friend std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

    protected:
      // feature on the controllable tool
      fccl::Feature tool_feature_;
   
      // feature on the non-controllable object
      fccl::Feature object_feature_;

      // lower boundary for output of feature function
      double lower_boundary_;

      // upper boundary for output of feature function
      double upper_boundary_;

      // reference frame w.r.t. relative feature functions are calculated
      std::size_t reference_id_;

      // identifier of this constraint, e.g. hash-value calculated from its name
      // in the knowledge base
      std::size_t id_;

      // type information to discriminate between constraints
      int type_;

      // memory to store first derivative of constraint
      fccl::InteractionMatrix first_derivative_;
  };

  class AboveConstraint : public Constraint
  {
    public:
      AboveConstraint();
      AboveConstraint(const AboveConstraint& other);
      AboveConstraint(const std::string& reference_name, const std::string& name,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
          double lower_boundary, double upper_boundary);
      AboveConstraint(std::size_t reference_id, std::size_t id,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature, 
          double lower_boundary, double upper_boundary); 

      virtual ~AboveConstraint();

      virtual double calculateValue(const fccl::Transform& tool_transform,
          const fccl::Transform& object_transform) const;
  };

} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
