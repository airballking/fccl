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
      KDL::Chain chain = extractChain(semantics, urdf);
      std::vector<std::string> joint_names = extractJointNames(chain);

      resize(chain.getNrOfJoints());

      rememberSemantics(joint_names, semantics);

      initJointLimits(urdf, joint_names);

      initSolvers(chain);
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
      return soft_lower_joint_limits_.getTargetNames();
    } 
    
    std::size_t KinematicChain::getNumberOfJoints() const
    {
      return soft_lower_joint_limits_.size();
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
      assert(jnt_to_jac_solver_);

      int error = jnt_to_jac_solver_->JntToJac(joint_state.getData(), jacobian.jacobian_);
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
      assert(jnt_to_pose_solver_);

      int error = jnt_to_pose_solver_->JntToCart(joint_state.getData(), 
          transform.transform_);
 
      assert(error == 0);
    }

    const Transform& KinematicChain::calculateForwardKinematics(const JntArray& joint_state)
    {
      calculateForwardKinematics(joint_state, transform_);
      return transform_;
    }
 
    KDL::Chain KinematicChain::extractChain(const SemanticObject1x1& semantics, 
        const urdf::Model& urdf) const
    {
      KDL::Tree tree;
      assert(kdl_parser::treeFromUrdfModel(urdf, tree));

      KDL::Chain chain;
      assert(tree.getChain(semantics.getReferenceName(),
          semantics.getTargetName(), chain));

      return chain;
    }

    void KinematicChain::initSolvers(const KDL::Chain& chain)
    {
      jnt_to_pose_solver_ = boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain));
      jnt_to_jac_solver_ = boost::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain));
    }

    std::vector<std::string> KinematicChain::extractJointNames(const KDL::Chain& chain) const
    {
      std::vector<std::string> joint_names;

      for(unsigned int i=0; i<chain.getNrOfSegments(); i++)
        if(isJoint(chain.getSegment(i)))
          joint_names.push_back(chain.getSegment(i).getJoint().getName());

      return joint_names;
    }

    void KinematicChain::initJointLimits(const urdf::Model& urdf, 
        const std::vector<std::string>& joint_names)
    {
      for(std::size_t i=0; i<joint_names.size(); i++)
      {
        const boost::shared_ptr<urdf::JointSafety> soft_safety_limits =
            urdf.getJoint(joint_names[i])->safety;
        const boost::shared_ptr<urdf::JointLimits> hard_joint_limits =
            urdf.getJoint(joint_names[i])->limits;
  
        assert(soft_safety_limits);
        assert(hard_joint_limits);
  
        soft_lower_joint_limits_(i) = soft_safety_limits->soft_lower_limit;
        soft_upper_joint_limits_(i) = soft_safety_limits->soft_upper_limit;
        hard_lower_joint_limits_(i) = hard_joint_limits->lower;
        hard_upper_joint_limits_(i) = hard_joint_limits->upper;
      }
    }

    void KinematicChain::rememberSemantics(const std::vector<std::string>& 
        joint_names, const SemanticObject1x1& semantics)
    {
      soft_lower_joint_limits_.setTargetNames(joint_names);
      soft_upper_joint_limits_.setTargetNames(joint_names);
      hard_lower_joint_limits_.setTargetNames(joint_names);
      hard_upper_joint_limits_.setTargetNames(joint_names);

      jacobian_.setTargetNames(joint_names);
      jacobian_.setReferenceName(semantics.getReferenceName());

      transform_.setSemantics(semantics);
    }

    void KinematicChain::resize(std::size_t new_size)
    {
      soft_lower_joint_limits_.resize(new_size);
      soft_upper_joint_limits_.resize(new_size);
      hard_lower_joint_limits_.resize(new_size);
      hard_upper_joint_limits_.resize(new_size);

      jacobian_.resize(new_size);
    }

  } // namespace kdl
} // namespace fccl
