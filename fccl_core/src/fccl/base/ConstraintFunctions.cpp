#include <fccl/base/ConstraintFunctions.h>

using namespace fccl::kdl;
using namespace fccl::semantics;

namespace fccl
{
  namespace base
  {
    const double infinity = 100;

    double above(const SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const Transform& tool_transform, const Transform& object_transform)
    {
        Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
        Feature object = transformFeature(view_frame, object_transform,
            object_feature);

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
      Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
      Feature object = transformFeature(view_frame, object_transform,
          object_feature);

      return tool.position().y() - object.position().y();
    }

    double right(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      return -left(view_frame, tool_feature, object_feature, tool_transform, object_transform);
    }

    double behind(const SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const Transform& tool_transform, const Transform& object_transform)
    {
        Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
        Feature object = transformFeature(view_frame, object_transform,
            object_feature);

        return tool.position().x() - object.position().x();
    }

    double infront(const SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const Transform& tool_transform, const Transform& object_transform)
    {
      return -behind(view_frame, tool_feature, object_feature, tool_transform, object_transform);
    }

    double perpendicular(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      assert(tool_feature.isOrientationValid());
      assert(object_feature.isOrientationValid());

      Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
      Feature object = transformFeature(view_frame, object_transform,
          object_feature);

      // returning the cos() of the angle between the two vectors
      return KDL::dot(tool.orientation(), object.orientation()) /
          (tool.orientation().Norm() * object.orientation().Norm());
    }

    double pointing(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      assert(tool_feature.isOrientationValid());

      Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
      Feature object = transformFeature(view_frame, object_transform,
          object_feature);

      KDL::Vector connector = object.position() - tool.position();

//      double len = tool.orientation().Norm() * connector.Norm();
//
//      if(len < KDL::epsilon)
//      {
//        return 0.0;
//      }
//      else
//      {
//        // returning the sin() of the angle between the two vectors
//        return (tool.orientation() * connector).Norm() / len;
//      }

      double denominator = tool.orientation().Norm() * connector.Norm();
      
      if(denominator < KDL::epsilon)
      {
        // SINGULARITY: (a) THE TWO FEATURES ARE PRACTICALLY ON THE SAME SPOT!!
        //              OR (b) AT LEAST ONE OF THE VECTORS HASE LENGTH 0 
        return 0.0;
      }
      else
      {
        return KDL::dot(tool.orientation(), connector) - denominator;
      }
    }

    double distance(const fccl::semantics::SemanticsBase& view_frame,
        const Feature& tool_feature, const Feature& object_feature,
        const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform)
    {
      Feature tool = transformFeature(view_frame, tool_transform, tool_feature);
      Feature object = transformFeature(view_frame, object_transform,
            object_feature);

      KDL::Vector connector = object.position() - tool.position();
      
      return connector.Norm();
    }


    Feature transformFeature(const fccl::semantics::SemanticsBase& target_frame,
         const fccl::kdl::Transform& transform, const Feature& feature)
    {
      assert(target_frame.equals(transform.semantics().reference()));
  
      Feature result = feature;
      result.changeReferenceFrame(transform);

      return result;
    }
  } // namespace base
} // namespace fccl
