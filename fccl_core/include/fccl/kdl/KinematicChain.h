#ifndef FCCL_KDL_KINEMATIC_CHAIN_H
#define FCCL_KDL_KINEMATIC_CHAIN_H

#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/JntArray.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <map>

namespace fccl
{
  namespace kdl
  {
    typedef boost::shared_ptr<KDL::ChainFkSolverPos_recursive> FkSolverPosPtr;
    typedef boost::shared_ptr<KDL::ChainJntToJacSolver> JacobianSolverPtr;
    
    class KinematicChain : public SemanticObject1x1
    {
      public:
        KinematicChain();
        KinematicChain(const urdf::Model& model, const SemanticObject1x1& chain_semantics);
        virtual ~KinematicChain();
    
        bool init(const SemanticObject1x1& semantics, const urdf::Model& model);
    
        const JntArray& getSoftLowerJointLimits() const;
        const JntArray& getSoftUpperJointLimits() const;
    
        const JntArray& getHardLowerJointLimits() const;
        const JntArray& getHardUpperJointLimits() const;

        const std::vector<std::string>& getJointNames() const;
        std::size_t getNumberOfJoints() const;
        
        bool isValid() const;
    
        void calculateJacobian(const JntArray& joint_state, Jacobian& jacobian);
        const Jacobian& calculateJacobian(const JntArray& joint_state);
    
        void calculateForwardKinematics(const JntArray& joint_state, 
            Transform& transform);
        const Transform& calculateForwardKinematics(const JntArray& joint_state);
    
        virtual bool numericsEqual(const KinematicChain& other) const;
        
        virtual bool operator==(const KinematicChain& other) const;
        virtual bool operator!=(const KinematicChain& other) const;
    
      private:

        void createSolvers(const KDL::Chain& chain);
        bool extractJointNames(const KDL::Chain& chain);
        bool extractJointLimits(const urdf::Model& model);
        void resize(std::size_t new_size);

        FkSolverPtr jnt_to_pose_solver_;
        
        JacobianSolverPtr jnt_to_jac_solver_;
    
        JntArray soft_lower_joint_limits_;
        JntArray soft_upper_joint_limits_;
        JntArray hard_lower_joint_limits_;
        JntArray hard_upper_joint_limits_;

        std::vector<std::string> joint_names_;
    
        Jacobian jacobian_;
    
        Transform transform_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_KINEMATIC_CHAIN_H
