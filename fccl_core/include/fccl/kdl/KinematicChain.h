#ifndef FCCL_KDL_KINEMATIC_CHAIN_H
#define FCCL_KDL_KINEMATIC_CHAIN_H

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/JntArray.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <boost/shared_ptr.hpp>
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
    
        bool init(const SemanticObject1x1& semantics, const urdf::Model& urdf);
    
        const JntArray& getSoftLowerJointLimits() const;
        const JntArray& getSoftUpperJointLimits() const;
    
        const JntArray& getHardLowerJointLimits() const;
        const JntArray& getHardUpperJointLimits() const;

        // not real-time safe
        std::vector<std::string> getJointNames() const;
        std::size_t getNumberOfJoints() const;
        const SemanticObject1x1& getSemantics() const;
        
        void calculateJacobian(const JntArray& joint_state, Jacobian& jacobian) const;
        const Jacobian& calculateJacobian(const JntArray& joint_state);
    
        void calculateForwardKinematics(const JntArray& joint_state, 
            Transform& transform) const;
        const Transform& calculateForwardKinematics(const JntArray& joint_state);
    
      private:
        KDL::Chain extractChain(const SemanticObject1x1& semantics, 
            const urdf::Model& urdf) const;
        std::vector<std::string> extractJointNames(const KDL::Chain& chain) const;

        void initJointLimits(const urdf::Model& urdf, 
            const std::vector<std::string>& joint_names);
        void initSolvers(const KDL::Chain& chain);

        void rememberSemantics(const std::vector<std::string>& joint_names,
            const SemanticObject1x1& semantics);
        void resize(std::size_t new_size);

        boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
        
        boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    
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
