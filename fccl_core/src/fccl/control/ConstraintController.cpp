#include <fccl/control/ConstraintController.h>

namespace fccl
{
  namespace control
  {
    void ConstraintController::init(const fccl::base::ConstraintArray& constraints,
        const fccl::kdl::KinematicChain& kinematics)
    {
      assert(kinematics.isValid());
      assert(constraints.isValid());

      constraints_ = constraints;
      kinematics_ = kinematics;

      desired_joint_velocities_.init(kinematics.semantics().joints());

      null_space_projector_.init(kinematics.semantics().joints(), kinematics.semantics().joints());

      // TODO(Georg):
      //  - implement init of JointMappingMatrix 
      A_.init(constraints.semantics().joints(), kinematics.semantics().joints());

      // TODO(Georg):
      //  - implement init of InteractionMatrix
      //  - implement ConstraintsArray
      //  - add function toolTwistSemantics
      H_.init(constraints.semantics().joints(), constraints.toolTwistSemantics());

      // TODO(Georg):
      //  - implement init of Jacobian
      JR_.init(constraints.toolTwistSemantics(), kinematics.semantics().joints());
    }

    void ConstraintController::update(const JntArray& joint_state,
        const fccl::utils::TransformMap& transform_map, double delta)
    {
      constraints.update(transfrom_map, delta);

      ydot_des_ = constraints_.getDesiredTaskVelocities();

      task_weights_ = constraints_.getTaskWeights();

      // TODO(Georg):
      //  - add interpolator

      // TODO(Georg):
      //  - add control law
 
      H_ = constraints.getFirstDerivative();

      // get robot jacobian in reference frame w.r.t. which constraints are defined
      JR_ = kinematics.calculateJacobian(joint_state);
      SemanticObject1x1 transform_semantics(H_.getReferenceID(), JR_.getReferenceID());
      JR_.changeReferenceFrame(transform_map.getTransform(transform_semantics));
    
      A_ = H_ * JR_;

      solver_.solve(A_, ydot_des_, task_weights_, joint_weights_, desired_joint_velocites_, null_space_projector);
    }

    const fccl::kdl::JntArray& ConstraintController::getDesiredJointVelocities() const
    {
      return desired_joint_velocities_;
    }

    const fccl::kdl::MatrixNxN& ConstraintController::getNullSpaceProjector() const
    {
      return nullSpaceProjector_;
    }

    const std::vector<fccl::base::Constraint>& ConstraintController::getConstraints() const
    {
      return constraints_;
    }

    const fccl::kdl::KinematicChain& ConstraintController::getKinematics() const
    {
      return kinematics_;
    }
 
  } // namespace control
} // namespace fccl
