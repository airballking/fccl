#ifndef FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
#define FCCL_CONTROL_CONSTRAINT_CONTROLLER_H

#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/KinematicChain.h>
#include <fccl/base/Constraints.h>
#include <fccl/utils/TransformMap.h>
#include <fccl/kdl/JointMappingMatrix.h>
#include <vector>

namespace fccl
{
  namespace control
  {
    class ConstraintController
      public:
        void init(const fccl::base::ConstraintArray& constraints, 
            const fccl::kdl::KinematicChain& kinematics);
        void update(const fccl::kdl::JntArray& joint_state,
            const fccl::utils::TransformMap& transform_map, double delta=0.001);

        const fccl::kdl::JntArray& getDesiredJointVelocities() const;
        const fccl::kdl::JointMappingMatrix& getNullSpaceProjector() const;

        const fccl::base::ConstraintArray& getConstraints() const;
        const fccl::kdl::KinematicChain& getKinematics() const;
 
      private:
        // actual state variables of the controller
        std::vector<fccl::base::Constraint> constraints_;
        fccl::kdl::KinematicChain& kinematics_;

        // memory for output values
        fccl::kdl::JntArray desired_joint_velocities_;
        fccl::kdl::JointMappingMatrix null_space_projector_;

        // intermediates results...
        fccl::kdl::JointMappingMatrix A_;
        fccl::kdl::InteractionMatrix H_;
        fccl::kdl::Jacobian JR_;
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
