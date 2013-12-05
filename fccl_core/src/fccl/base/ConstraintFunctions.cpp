#include <fccl/base/ConstraintFunctions.h>

using namespace fccl::kdl;
using namespace fccl::semantics;

namespace fccl
{
  namespace base
  {
    double above(const SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const Transform& tool_transform, const Transform& object_transform)
    {
      assert(view_frame.equals(tool_transform.semantics().reference()));
      assert(view_frame.equals(object_transform.semantics().reference()));
  
      Feature tool = tool_feature;
      Feature object = object_feature;
      
      tool.changeReferenceFrame(tool_transform);
      object.changeReferenceFrame(object_transform);
  
      return tool.position().z() - object.position().z();
    }

    double below(const SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const Transform& tool_transform, const Transform& object_transform)
    {
      return -above(view_frame, tool_feature, object_feature, tool_transform, object_transform); 
    }

    double left(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      // TODO(Georg): refactor the beginnning into an external method
      //              because we will re-use it several times
      assert(view_frame.equals(tool_transform.semantics().reference()));
      assert(view_frame.equals(object_transform.semantics().reference()));
  
      Feature tool = tool_feature;
      Feature object = object_feature;
      
      tool.changeReferenceFrame(tool_transform);
      object.changeReferenceFrame(object_transform);

      return tool.position().y() - object.position().y();
    }

    double right(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      return -left(view_frame, tool_feature, object_feature, tool_transform, object_transform);
    }
  } // namespace base
} // namespace fccl
