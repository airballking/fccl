#ifndef FCCL_BASE_CONSTRAINTS_H
#define FCCL_BASE_CONSTRAINTS_H

#include <fccl_base/Features.h>
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
      Constraint(const std::string& reference_name, int type,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
          double lower_boundary, double upper_boundary,
          const fccl::Transform& tool_transform, const fccl::Transform& object_transform);
      Constraint(std::size_t reference_id, int type,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature, 
          double lower_boundary, double upper_boundary, 
          const fccl::Transform& tool_transform, const fccl::Transform& object_transform);

      virtual ~Constraint();

      Constraint& operator=(const Constraint& rhs);

      std::size_t getReferenceID() const;
      void setReferenceName(const std::string& reference_name);
      void setReferenceID(std::size_t reference_id);

      const fccl::Feature& getToolFeature() const;
      void setToolFeature(const fccl::Feature& tool_feature);

      const fccl::Feature& getObjectFeature() const;
      void setObjectFeature(const fccl::Feature& object_feature);

      double getLowerBoundary() const;
      void setLowerBoundary(double lower_boundary);

      double getUpperBoundary() const;
      void setUpperBoundary(double upper_boundary);

      const fccl::Transform& getToolTransform() const;
      void setToolTransform(const fccl::Transform& tool_transform);

      const fccl::Transform& getObjectTransform() const;
      void setObjectTransform(const fccl::Transform& object_transform);

      int getType() const;
      void setType(int type);

      virtual double calculateValue() const;
      InteractionMatrix calculateDerivate() const;

      bool operator==(const Constraint& other) const;
      bool operator!=(const Constraint& other) const;

      bool semanticsEqual(const Constraint& other) const;
      bool numericsEqual(const Constraint& other) const;

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

      // type information to discriminate between constraints
      int type_;
 
      // transform between reference_frame_ of this constraint and reference
      // frame of tool_feature_
      fccl::Transform T_tool_feature_in_constraint_frame_;

      // transform between reference_frame_ of this constraint and reference
      // frame of object_feature_
      fccl::Transform T_object_feature_in_constraint_frame_;
 };
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINTS_H
