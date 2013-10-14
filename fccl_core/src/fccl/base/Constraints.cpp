#include <fccl/base/Constraints.h>

namespace fccl
{
  namespace base
  {
    double Constraint::calculateValue(const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform) const
    {
      // IF THIS FUNCTION GETS CALLED, SOMEONE F'D UP.
      // BY THE WAY, I ALSO DON'T LIKE THIS DESIGN BUT IT'S
      // RESEARCH CODE, YOU KNOW? ;)
      assert(false);
      
      return 0.0;
    }

    const fccl::kdl::InteractionMatrix& Constraint::calculateFirstDerivative(const
        fccl::kdl::Transform& tool_transform, const fccl::kdl::Transform& 
        object_transform, double delta)
    {
      assert(delta != 0.0);
      assert(first_derivative_.size() == 1);
      
      // prepare semantics of result
      first_derivative_.semantics() = 
      semantics().calculateFirstDerivative(tool_transform.semantics());
      
      // get current constraint value around which we differentiate
      double value = calculateValue(tool_transform, object_transform);
      
      // prepare six delta-transforms to simulate delta motions of tool
      fccl::kdl::Transform T[6];
      for(unsigned int i=0; i<6; i++)
      {
        T[i].semantics().reference() = tool_transform.semantics().target();
        T[i].semantics().target() = tool_transform.semantics().target();
      }
      double cd = cos(delta);
      double sd = sin(delta);
      T[0].numerics() = KDL::Frame(KDL::Vector(delta,0,0));
      T[1].numerics() = KDL::Frame(KDL::Vector(0,delta,0));
      T[2].numerics() = KDL::Frame(KDL::Vector(0,0,delta));
      T[3].numerics() = KDL::Frame(KDL::Rotation(1,0,0,  0,cd,-sd,  0,sd,cd));
      T[4].numerics() = KDL::Frame(KDL::Rotation(cd,0,sd,  0,1,0,  -sd,0,cd));
      T[5].numerics() = KDL::Frame(KDL::Rotation(cd,-sd,0,  sd,cd,0,  0,0,1));
       
      // calculate actual numeric calculation
      double delta_r = 1.0 / delta;
      for(unsigned int i=0; i < 6; i++)
      {
        first_derivative_.numerics()(0,i) =
          (calculateValue(multiply(tool_transform, T[i]), object_transform) - value) * delta_r;
      }
  
      return first_derivative_;
    }
  
    double AboveConstraint::calculateValue(const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform) const
    {
      assert(semantics().reference().equals(tool_transform.semantics().reference()));
      assert(semantics().reference().equals(object_transform.semantics().reference()));
//      assert(tool_feature_.semantics().reference().equals(
//        tool_transform.semantics().target()));
//      assert(object_feature_.semantics().reference().equals(
//        object_transform.semantics().target());
  
      Feature tool = toolFeature();
      Feature object = objectFeature();
      
      tool.changeReferenceFrame(tool_transform);
      object.changeReferenceFrame(object_transform);
//      KDL::Vector tool_position_in_view = tool_transform.getTransform() * tool_feature_.getPosition();
//      KDL::Vector object_position_in_view = object_transform.getTransform() * object_feature_.getPosition();
  
      return  tool.position().z() - object.position().z();
    }
  } // namespace base
} // namespace fccl
