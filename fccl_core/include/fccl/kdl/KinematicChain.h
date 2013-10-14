#ifndef FCCL_KDL_KINEMATIC_CHAIN_H
#define FCCL_KDL_KINEMATIC_CHAIN_H

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/JntArray.h>
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
        void init(const fccl::semantics::TransformSemantics& semantics, 
            const urdf::Model& urdf)
        {
          extractChain(semantics, urdf);
   
          extractJointLimits(urdf);
  
          prepareReturnValues(semantics);
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

        std::vector<std::string> jointNames() const
        {
          std::vector<std::string> joint_names;
          joint_names.clear();
    
          for(unsigned int i=0; i<chain_.getNrOfSegments(); i++)
            if(fccl::utils::isJoint(chain_.getSegment(i)))
              joint_names.push_back(chain_.getSegment(i).getJoint().getName());
    
          return joint_names;
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

        void prepareReturnValues(const fccl::semantics::TransformSemantics& semantics);

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
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_KINEMATIC_CHAIN_H
