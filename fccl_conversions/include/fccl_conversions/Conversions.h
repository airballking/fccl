#ifndef FCCL_CONVERSIONS_CONVERSIONS_H
#define FCCL_CONVERSIONS_CONVERSIONS_H

#include <fccl/base/Features.h>
#include <fccl/base/Constraints.h>
#include <fccl/base/ConstraintArray.h>
#include <fccl/kdl/KinematicChain.h>
#include <kdl/frames.hpp>
#include <vector>
#include <urdf/model.h>

#include <fccl_msgs/Feature.h>
#include <fccl_msgs/Constraint.h>
#include <fccl_msgs/KinematicChain.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include <stdexcept>

namespace fccl
{
  namespace conversions
  {
    // Custom exception thrown whenever message conversions encounters an error.
    class ConversionException : public std::runtime_error
    {
      public:
        ConversionException(const std::string& message) : 
            std::runtime_error(message) { }
    };

    // in-place message conversions
    void toMsg(const fccl::base::ConstraintArray& constraints, 
        std::vector<fccl_msgs::Constraint>& msg);  
    void toMsg(const fccl::base::Constraint& constraint, fccl_msgs::Constraint& msg);
    void toMsg(const fccl::base::Feature& feature, fccl_msgs::Feature& msg);

    void toMsg(const KDL::Vector& vector, geometry_msgs::Vector3& msg);
    void toMsg(const fccl::semantics::SemanticsBase& base, std_msgs::String& msg);
    void toMsg(const fccl::semantics::FeatureTypes& type, std_msgs::UInt8& msg);
    void toMsg(double value, std_msgs::Float64& msg);

    void fromMsg(const std::vector<fccl_msgs::Constraint>& msg,
        fccl::base::ConstraintArray& constraints);
    void fromMsg(const fccl_msgs::Constraint& msg, fccl::base::Constraint& constraint);
    void fromMsg(const fccl_msgs::Feature& msg, fccl::base::Feature& feature);

    void fromMsg(const fccl_msgs::KinematicChain& msg, const urdf::Model& urdf,
        fccl::kdl::KinematicChain& kinematics) throw (ConversionException);

    void fromMsg(const geometry_msgs::Vector3& msg, KDL::Vector& vector);
    void fromMsg(const std_msgs::String& msg, fccl::semantics::SemanticsBase& base);
    void fromMsg(const std_msgs::UInt8& msg, fccl::semantics::FeatureTypes& type);
    void fromMsg(const std_msgs::Float64& msg, double& value);

    // message conversions with memory allocation
    std::vector<fccl_msgs::Constraint> toMsg(const fccl::base::ConstraintArray&
        constraints);
    fccl_msgs::Constraint toMsg(const fccl::base::Constraint& constraint);
    fccl_msgs::Feature toMsg(const fccl::base::Feature& feature);

    fccl::base::ConstraintArray fromMsg(const std::vector<fccl_msgs::Constraint>&
        msg);
    fccl::base::Constraint fromMsg(const fccl_msgs::Constraint& msg);
    fccl::base::Feature fromMsg(const fccl_msgs::Feature& msg);
    fccl::kdl::KinematicChain fromMsg(const fccl_msgs::KinematicChain& msg,
        const urdf::Model& urdf) throw (ConversionException);
  } // namespace conversions
} // namespace fccl
#endif // FCCL_CONVERSIONS_CONVERSIONS_H
