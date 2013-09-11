#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/JntArray.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
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
    
        bool init(const urdf::Model& model, const SemanticObject1x1& chain_semantics);
    
        const fccl::kdl::JntArray& getSoftLowerJointLimits() const;
        const fccl::kdl::JntArray& getSoftUpperJointLimits() const;
    
        const fccl::kdl::JntArray& getHardLowerJointLimits() const;
        const fccl::kdl::JntArray& getHardUpperJointLimits() const;
    
        void calculateJacobian(const fccl::kdl::JntArray& joint_state, 
            fccl::kdl::Jacobian& jacobian);
        const fccl::kdl::Jacobian& calculateJacobian(const fccl::kdl::JntArray&
            joint_state);
    
        void calculateForwardKinematics(const fccl::kdl::JntArray& joint_state, 
            fccl::kdl::Transform& transform);
        const fccl::kdl::Transform& calculateForwardKinematics(
            const fccl::kdl::JntArray& joint_state);
    
        virtual bool numericsEqual(const KinematicChain& other) const;
        
        virtual bool operator==(const KinematicChain& other) const;
        virtual bool operator!=(const KinematicChain& other) const;
    
      private:
        FkSolverPtr jnt_to_pose_solver_;
        
        JacobianSolverPtr jnt_to_jac_solver_;
    
        fccl::kdl::JntArray soft_lower_joint_limits_;
        fccl::kdl::JntArray soft_upper_joint_limits_;
        fccl::kdl::JntArray hard_lower_joint_limits_;
        fccl::kdl::JntArray hard_upper_joint_limits_;
    
        fccl::kdl::Jacobian jacobian_;
    
        fccl::kdl::Transform transform_;
    };
  } // namespace kdl
} // namespace fccl
