#include <fccl/kdl/KinematicChain.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace fccl
{
  namespace kdl
  {
    void KinematicChain::calculateJacobian(const JntArray& joint_state, Jacobian& jacobian)
    {
      assert(jacobian.semantics().equals(jacobian_.semantics()));
      assert(joint_state.semantics().equals(jacobian_.semantics().joints()));

      KDL::ChainJntToJacSolver jnt_to_jac_solver(chain_);
      int error = jnt_to_jac_solver.JntToJac(joint_state.numerics(), jacobian.numerics());

      assert(error == 0);

      jacobian.numerics().changeRefPoint(-calculateForwardKinematics(joint_state).numerics().p);
    }

    const Jacobian& KinematicChain::calculateJacobian(const JntArray& joint_state)
    {
      calculateJacobian(joint_state, jacobian_);
      return jacobian_;
    }
 
    void KinematicChain::calculateForwardKinematics(const JntArray& joint_state, 
        Transform& transform)
    {
      assert(transform.semantics().equals(transform_.semantics()));
      assert(joint_state.semantics().equals(jacobian_.semantics().joints()));

      KDL::ChainFkSolverPos_recursive jnt_to_pose_solver(chain_);
      int error = jnt_to_pose_solver.JntToCart(joint_state.numerics(), 
          transform.numerics());
 
      assert(error == 0);
    }

    const Transform& KinematicChain::calculateForwardKinematics(const JntArray& joint_state)
    {
      calculateForwardKinematics(joint_state, transform_);
      return transform_;
    }
 
    void KinematicChain::extractChain(
        const fccl::semantics::TransformSemantics& semantics, 
        const urdf::Model& urdf)
    {
      KDL::Tree tree;
      assert(kdl_parser::treeFromUrdfModel(urdf, tree));

      assert(tree.getChain(semantics.reference().getName(),
          semantics.target().getName(), chain_));
    }

    void KinematicChain::extractJointLimits(const urdf::Model& urdf)
    {
      soft_lower_joint_limits_.init(jointNames());
      soft_upper_joint_limits_.init(jointNames());
      hard_lower_joint_limits_.init(jointNames());
      hard_upper_joint_limits_.init(jointNames());

      for(std::size_t i=0; i<size(); i++)
      {
        const boost::shared_ptr<urdf::JointSafety> soft_safety_limits =
            urdf.getJoint(jointNames()[i])->safety;
        const boost::shared_ptr<urdf::JointLimits> hard_joint_limits =
            urdf.getJoint(jointNames()[i])->limits;
  
        assert(soft_safety_limits);
        assert(hard_joint_limits);
  
        soft_lower_joint_limits_.numerics()(i) = soft_safety_limits->soft_lower_limit;
        soft_upper_joint_limits_.numerics()(i) = soft_safety_limits->soft_upper_limit;
        hard_lower_joint_limits_.numerics()(i) = hard_joint_limits->lower;
        hard_upper_joint_limits_.numerics()(i) = hard_joint_limits->upper;
      }
    }

    void KinematicChain::prepareReturnValues(const fccl::semantics::TransformSemantics& semantics)
    {
      jacobian_.init(jointNames(), semantics.reference().getName(), 
          semantics.target().getName());

      transform_.semantics() = semantics;
    }

  } // namespace kdl
} // namespace fccl
