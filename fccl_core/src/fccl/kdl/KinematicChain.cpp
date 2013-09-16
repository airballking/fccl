#include <fccl/kdl/KinematicChain.h>
#include <fccl/utils/Equalities.h>
#include <kdl_parser/kdl_parser.hpp>

using namespace fccl::utils;

namespace fccl
{
  namespace kdl
  {
    KinematicChain::KinematicChain()
    {
    }

    KinematicChain::KinematicChain(const SemanticObject1x1& semantics, const urdf::Model& urdf)
    {
      init(semantics, urdf);
    }

    KinematicChain::~KinematicChain()
    {
    }
 
    void KinematicChain::init(const SemanticObject1x1& semantics, 
        const urdf::Model& urdf)
    {
      extractChain(semantics, urdf);

      extractJointNames();

      extractJointLimits(urdf);

      prepareReturnValues(semantics);
    }

    const JntArray& KinematicChain::getSoftLowerJointLimits() const
    {
      return soft_lower_joint_limits_;
    }

    const JntArray& KinematicChain::getSoftUpperJointLimits() const
    {
      return soft_upper_joint_limits_;
    }
 
    const JntArray& KinematicChain::getHardLowerJointLimits() const
    {
      return hard_lower_joint_limits_;
    }

    const JntArray& KinematicChain::getHardUpperJointLimits() const
    {
      return hard_upper_joint_limits_;
    }

    std::vector<std::string> KinematicChain::getJointNames() const
    {
      return joint_names_;
    } 
    
    std::size_t KinematicChain::getNumberOfJoints() const
    {
      return chain_.getNrOfJoints();
    }

    const SemanticObject1x1& KinematicChain::getTransformationSemantics() const
    {
      return transform_.getSemantics();
    }

    const SemanticObjectN& KinematicChain::getJointSemantics() const
    {
      return hard_lower_joint_limits_.getSemantics();
    }
    
    void KinematicChain::calculateJacobian(const JntArray& joint_state, Jacobian& jacobian)
    {
      assert(jacobian.semanticsEqual(jacobian_));
      assert(Equal(joint_state.getTargetIDs(), jacobian_.getTargetIDs()));

      KDL::ChainJntToJacSolver jnt_to_jac_solver(chain_);
      int error = jnt_to_jac_solver.JntToJac(joint_state.getData(), jacobian.jacobian_);

      // TODO(Georg): turn this into an exception
      assert(error == 0);

      jacobian.jacobian_.changeRefPoint(-calculateForwardKinematics(joint_state).getTransform().p);
    }

    const Jacobian& KinematicChain::calculateJacobian(const JntArray& joint_state)
    {
      calculateJacobian(joint_state, jacobian_);
      return jacobian_;
    }
 
    void KinematicChain::calculateForwardKinematics(const JntArray& joint_state, 
        Transform& transform)
    {
      assert(transform.semanticsEqual(transform_));
      assert(Equal(joint_state.getTargetIDs(), jacobian_.getTargetIDs()));

      KDL::ChainFkSolverPos_recursive jnt_to_pose_solver(chain_);
      int error = jnt_to_pose_solver.JntToCart(joint_state.getData(), 
          transform.transform_);
 
      // TODO(Georg): turn this into an exception
      assert(error == 0);
    }

    const Transform& KinematicChain::calculateForwardKinematics(const JntArray& joint_state)
    {
      calculateForwardKinematics(joint_state, transform_);
      return transform_;
    }
 
    void KinematicChain::extractChain(const SemanticObject1x1& semantics, 
        const urdf::Model& urdf)
    {
      // TODO(Georg): make all of these exceptions
      KDL::Tree tree;
      assert(kdl_parser::treeFromUrdfModel(urdf, tree));

      assert(tree.getChain(semantics.getReferenceName(),
          semantics.getTargetName(), chain_));
    }

    void KinematicChain::extractJointNames()
    {
      joint_names_.clear();

      for(unsigned int i=0; i<chain_.getNrOfSegments(); i++)
        if(isJoint(chain_.getSegment(i)))
          joint_names_.push_back(chain_.getSegment(i).getJoint().getName());
    }

    void KinematicChain::extractJointLimits(const urdf::Model& urdf)
    {
      SemanticObjectN joint_semantics(joint_names_);
      soft_lower_joint_limits_.init(joint_semantics);
      soft_upper_joint_limits_.init(joint_semantics);
      hard_lower_joint_limits_.init(joint_semantics);
      hard_upper_joint_limits_.init(joint_semantics);

      for(std::size_t i=0; i<joint_names_.size(); i++)
      {
        const boost::shared_ptr<urdf::JointSafety> soft_safety_limits =
            urdf.getJoint(joint_names_[i])->safety;
        const boost::shared_ptr<urdf::JointLimits> hard_joint_limits =
            urdf.getJoint(joint_names_[i])->limits;
  
        // TODO(Georg): turn these into exceptions
        assert(soft_safety_limits);
        assert(hard_joint_limits);
  
        soft_lower_joint_limits_(i) = soft_safety_limits->soft_lower_limit;
        soft_upper_joint_limits_(i) = soft_safety_limits->soft_upper_limit;
        hard_lower_joint_limits_(i) = hard_joint_limits->lower;
        hard_upper_joint_limits_(i) = hard_joint_limits->upper;
      }
    }

    void KinematicChain::prepareReturnValues(const SemanticObject1x1& semantics)
    {
      jacobian_.init(SemanticObject1xN(semantics.getReferenceName(), joint_names_));

      transform_.setSemantics(semantics);
    }

  } // namespace kdl
} // namespace fccl
