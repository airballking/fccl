#ifndef FCCL_BASE_CONSTRAINT_FUNCTIONS_H
#define FCCL_BASE_CONSTRAINT_FUNCTIONS_H

#include <fccl/base/Features.h>
#include <fccl/kdl/Transform.h>
#include <fccl/semantics/SemanticsBase.h>

namespace fccl
{
  namespace base
  {
    typedef double (*ConstraintFunction) 
        (const fccl::semantics::SemanticsBase& view_frame,
         const Feature& tool_feature, const Feature& object_feature,
         const fccl::kdl::Transform& tool_transform,
         const fccl::kdl::Transform& object_transform);

    Feature transformFeature(const fccl::semantics::SemanticsBase& target_frame,
         const fccl::kdl::Transform& transform, const Feature& feature);

    double above(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double below(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double left(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double right(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double behind(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double infront(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);

    double perpendicular(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform);
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_CONSTRAINT_FUNCTIONS_H
