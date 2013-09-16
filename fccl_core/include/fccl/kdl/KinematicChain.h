#ifndef FCCL_KDL_KINEMATIC_CHAIN_H
#define FCCL_KDL_KINEMATIC_CHAIN_H

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/JntArray.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <string>
#include <vector>
#include <map>

namespace fccl
{
  namespace kdl
  {
    class KinematicChain
    {
      public:
        KinematicChain();
        KinematicChain(const SemanticObject1x1& semantics, const urdf::Model& urdf);
        virtual ~KinematicChain();
    
        void init(const SemanticObject1x1& semantics, const urdf::Model& urdf);
    
        const JntArray& getSoftLowerJointLimits() const;
        const JntArray& getSoftUpperJointLimits() const;
    
        const JntArray& getHardLowerJointLimits() const;
        const JntArray& getHardUpperJointLimits() const;

        // not real-time safe
        std::vector<std::string> getJointNames() const;
        std::size_t getNumberOfJoints() const;
        const SemanticObject1x1& getTransformationSemantics() const;
        const SemanticObjectN& getJointSemantics() const;
        
        void calculateJacobian(const JntArray& joint_state, Jacobian& jacobian);
        const Jacobian& calculateJacobian(const JntArray& joint_state);
    
        void calculateForwardKinematics(const JntArray& joint_state, 
            Transform& transform);
        const Transform& calculateForwardKinematics(const JntArray& joint_state);
    
      private:
        void extractChain(const SemanticObject1x1& semantics, 
            const urdf::Model& urdf);

        void extractJointNames();

        void extractJointLimits(const urdf::Model& urdf);

        void prepareReturnValues(const SemanticObject1x1& semantics);

        KDL::Chain chain_;

        std::vector<std::string> joint_names_;

        JntArray soft_lower_joint_limits_;
        JntArray soft_upper_joint_limits_;
        JntArray hard_lower_joint_limits_;
        JntArray hard_upper_joint_limits_;

        // pre-allocated memory for our return values
        Jacobian jacobian_;
        Transform transform_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_KINEMATIC_CHAIN_H
