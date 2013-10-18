#ifndef FCCL_KDL_KINEMATIC_CHAIN_H
#define FCCL_KDL_KINEMATIC_CHAIN_H

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/semantics/KinematicChainSemantics.h>
#include <fccl/utils/Equalities.h>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <string>
#include <vector>

namespace fccl
{
  namespace kdl
  {
    class KinematicChain
    {
      public:
        void init(const fccl::semantics::TransformSemantics& transform_semantics, 
            const urdf::Model& urdf)
        {
          extractChain(transform_semantics, urdf);

          initSemantics(transform_semantics);
   
          extractJointLimits(urdf);
  
          prepareReturnValues(transform_semantics);
        }

        const fccl::semantics::KinematicChainSemantics& semantics() const
        {
          return semantics_;
        }

        const JntArray& softLowerJointLimits() const
        {
          return soft_lower_joint_limits_;
        }

        const JntArray& softUpperJointLimits() const
        {
          return soft_upper_joint_limits_;
        }
    
        const JntArray& hardLowerJointLimits() const
        {
          return hard_lower_joint_limits_;
        }

        const JntArray& hardUpperJointLimits() const
        {
          return hard_upper_joint_limits_;
        }

        bool isValid() const
        {
          return semantics().joints().equals(softLowerJointLimits().semantics()) &&
            semantics().joints().equals(softUpperJointLimits().semantics()) &&
            semantics().joints().equals(hardLowerJointLimits().semantics()) &&
            semantics().joints().equals(hardUpperJointLimits().semantics()) &&
            semantics().joints().equals(jacobian_.semantics().joints()) &&
            semantics().transform().equals(transform_.semantics()) &&
            semantics().transform().target().equals(
                jacobian_.semantics().twist().target()) &&
            semantics().transform().reference().equals(
                jacobian_.semantics().twist().reference());
        }

        std::vector<std::string> jointNames() const
        {
          return semantics().joints().jointNames();
        }

        std::size_t size() const
        {
          return softLowerJointLimits().size();
        }

        void calculateJacobian(const JntArray& joint_state, Jacobian& jacobian);
        const Jacobian& calculateJacobian(const JntArray& joint_state);
    
        void calculateForwardKinematics(const JntArray& joint_state, 
            Transform& transform);
        const Transform& calculateForwardKinematics(const JntArray& joint_state);
    
      private:
        void extractChain(const fccl::semantics::TransformSemantics& semantics, 
            const urdf::Model& urdf);

        void extractJointLimits(const urdf::Model& urdf);

        void initSemantics(const fccl::semantics::TransformSemantics& 
            transform_semantics);

        void prepareReturnValues(const fccl::semantics::TransformSemantics& semantics);

        // semantics of the chain
        fccl::semantics::KinematicChainSemantics semantics_;
        // numerics of the chain
        KDL::Chain chain_;

        // pre-allocated memory for our return values
        JntArray soft_lower_joint_limits_;
        JntArray soft_upper_joint_limits_;
        JntArray hard_lower_joint_limits_;
        JntArray hard_upper_joint_limits_;
        Jacobian jacobian_;
        Transform transform_;
    };

    inline std::vector<std::string> extractJointNames(const KDL::Chain& chain)
    {
      std::vector<std::string> joint_names;
    
      for(unsigned int i=0; i<chain.getNrOfSegments(); i++)
        if(fccl::utils::isJoint(chain.getSegment(i)))
          joint_names.push_back(chain.getSegment(i).getJoint().getName());
 
      return joint_names;
    }
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_KINEMATIC_CHAIN_H
