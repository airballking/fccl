#include <fccl/base/Constraints.h>

using namespace fccl::kdl;
using namespace fccl::semantics;

namespace fccl
{
  namespace base
  {
    // perform definition and init of our static function map
    const std::map<SemanticsBase, ConstraintFunction> 
       Constraint::function_map_ = Constraint::createFunctionMap();

    // register new constraint functions here
    std::map<SemanticsBase, ConstraintFunction> Constraint::createFunctionMap()
    {
      std::map<SemanticsBase, ConstraintFunction> result;

      registerConstraintFunction("above", above, result);
      registerConstraintFunction("below", below, result);
      registerConstraintFunction("left", left, result);
      registerConstraintFunction("right", right, result);
      registerConstraintFunction("behind", behind, result);
      registerConstraintFunction("infront", infront, result);
      registerConstraintFunction("perpendicular", perpendicular, result);
      registerConstraintFunction("pointing", pointing, result);
      registerConstraintFunction("distance", distance, result);

      return result;
    }

    void Constraint::registerConstraintFunction(const std::string& function_name,
        const ConstraintFunction& function,
        std::map<SemanticsBase, ConstraintFunction>& map)
    {
      SemanticsBase function_identifier;
      function_identifier.setName(function_name);
      map[function_identifier] = function;  
    }

    // NOT REAL-TIME-SAFE
    std::set<TransformSemantics> Constraint::necessaryTransforms() const
    {
      assert(isValid());

      std::set<TransformSemantics> result;
      
      TransformSemantics container;
      container.reference() = semantics().reference();

      container.target() = toolFeature().semantics().reference();
      result.insert(container);
      
      container.target() = objectFeature().semantics().reference();
      result.insert(container);

      return result;
    }
 
    const InteractionMatrix& Constraint::calculateFirstDerivative(const Transform&
        tool_transform, const Transform& object_transform, double delta)
    {
      assert(delta != 0.0);
      assert(first_derivative_.size() == 1);
      
      // prepare semantics of result
      calculateInteractionSemantics(tool_transform.semantics());
      
      // get current constraint value around which we differentiate
      double value = calculateOutputValue(tool_transform, object_transform);
//      double value = calculateValue(tool_transform, object_transform);
      
      // prepare six delta-transforms to simulate delta motions of tool
      Transform T[6];
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
          (calculateOutputValue(multiply(tool_transform, T[i]), object_transform) - value) * delta_r;
      }
  
      return first_derivative_;
    }

    void Constraint::calculateInteractionSemantics(const TransformSemantics& 
        tool_transform)
    {
      assert(first_derivative_.size() == 1);
 
      first_derivative_.semantics().twist().reference() = tool_transform.target();
      first_derivative_.semantics().twist().target() = tool_transform.target();
      first_derivative_.semantics().joints()(0) = semantics().name(); 
    }
  } // namespace base
} // namespace fccl
