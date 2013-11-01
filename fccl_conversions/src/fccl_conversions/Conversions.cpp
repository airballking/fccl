#include <fccl_conversions/Conversions.h>

namespace fccl
{
  namespace conversions
  {
    void toMsg(const fccl::base::Feature& feature, fccl_msgs::Feature& msg)
    {
      toMsg(feature.semantics().name(), msg.name);
      toMsg(feature.semantics().reference(), msg.reference);
      toMsg(feature.semantics().type(), msg.type);
      toMsg(feature.position(), msg.position);
      toMsg(feature.orientation(), msg.direction); 
    }

    void toMsg(const fccl::base::Constraint& constraint, fccl_msgs::Constraint& msg)
    {
      toMsg(constraint.semantics().name(), msg.name);
      toMsg(constraint.semantics().reference(), msg.reference);
      toMsg(constraint.semantics().type(), msg.function);
      toMsg(constraint.toolFeature(), msg.tool_feature);
      toMsg(constraint.objectFeature(), msg.object_feature);
      toMsg(constraint.lowerBoundary(), msg.lower_boundary);
      toMsg(constraint.upperBoundary(), msg.upper_boundary);
    }

    void toMsg(const KDL::Vector& vector, geometry_msgs::Vector3& msg)
    {
      msg.x = vector.x();
      msg.y = vector.y();
      msg.z = vector.z();
    }

    void toMsg(const fccl::semantics::SemanticsBase& base, std_msgs::String& msg)
    {
      msg.data = base.getName();
    }

    void toMsg(const fccl::semantics::FeatureTypes& type, std_msgs::UInt8& msg)
    {
      msg.data = type; 
    }

    void toMsg(double value, std_msgs::Float64& msg)
    {
      msg.data = value;
    }

    void fromMsg(const fccl_msgs::Feature& msg, fccl::base::Feature& feature)
    {
      fromMsg(msg.name, feature.semantics().name());
      fromMsg(msg.reference, feature.semantics().reference());
      fromMsg(msg.type, feature.semantics().type());
      fromMsg(msg.position, feature.position());
      fromMsg(msg.direction, feature.orientation());
    }

    void fromMsg(const fccl_msgs::Constraint& msg, fccl::base::Constraint& constraint)
    {
      fromMsg(msg.name, constraint.semantics().name());
      fromMsg(msg.reference, constraint.semantics().reference());
      fromMsg(msg.function, constraint.semantics().type());
      fromMsg(msg.tool_feature, constraint.toolFeature());
      fromMsg(msg.object_feature, constraint.objectFeature());
      fromMsg(msg.lower_boundary, constraint.lowerBoundary());
      fromMsg(msg.upper_boundary, constraint.upperBoundary()); 
    }

    void fromMsg(const geometry_msgs::Vector3& msg, KDL::Vector& vector)
    {
      vector.x(msg.x);
      vector.y(msg.y);
      vector.z(msg.z);
    }

    void fromMsg(const std_msgs::String& msg, fccl::semantics::SemanticsBase& base)
    {
      base.setName(msg.data);
    }

    void fromMsg(const std_msgs::UInt8& msg, fccl::semantics::FeatureTypes& type)
    {
      type = static_cast<fccl::semantics::FeatureTypes>(msg.data);
    }

    void fromMsg(const std_msgs::Float64& msg, double& value)
    {
      value = msg.data;
    }

    fccl_msgs::Feature toMsg(const fccl::base::Feature& feature)
    {
      fccl_msgs::Feature msg;
      toMsg(feature, msg);
      return msg;
    }

    fccl_msgs::Constraint toMsg(const fccl::base::Constraint& constraint)
    {
      fccl_msgs::Constraint msg;
      toMsg(constraint, msg);
      return msg;
    }

    fccl::base::Feature fromMsg(const fccl_msgs::Feature& msg)
    {
      fccl::base::Feature feature;
      fromMsg(msg, feature);
      return feature;
    }

    fccl::base::Constraint fromMsg(const fccl_msgs::Constraint& msg)
    {
      fccl::base::Constraint constraint;
      fromMsg(msg, constraint);
      return constraint;
    }
  } // namespace conversions
} // namespace fccl
