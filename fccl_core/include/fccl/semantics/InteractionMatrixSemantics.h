#ifndef FCCL_SEMANTICS_INTERACTION_MATRIX_SEMANTICS_H
#define FCCL_SEMANTICS_INTERACTION_MATRIX_SEMANTICS_H

#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/TwistSemantics.h>
#include <fccl/semantics/JacobianSemantics.h>
#include <fccl/semantics/JointMappingSemantics.h>

namespace fccl
{
  namespace semantics
  {
    class InteractionMatrixSemantics
    {
      public:
        const JntArraySemantics& joints() const
        {
          return joints_;
        }

        JntArraySemantics& joints()
        {
          return joints_;
        }

        const TwistSemantics& twist() const
        {
          return twist_;
        }

        TwistSemantics& twist()
        {
          return twist_;
        }

        std::size_t size() const
        {
          return joints().size();
        }

        void resize(std::size_t number_of_joints)
        {
          joints().resize(number_of_joints);
        }

        bool equals(const InteractionMatrixSemantics& other) const
        {
          return twist().equals(other.twist()) &&
              joints().equals(other.joints());
        }

        void init(const std::vector<std::string>& joint_names,
            const std::string& reference_name, const std::string& target_name)
        {
          joints().init(joint_names);
          twist().init(reference_name, target_name);
        }

        void changeReferenceFrame(const fccl::semantics::TransformSemantics& 
            transform_semantics)
        {
          twist().changeReferenceFrame(transform_semantics);
        }
          
        bool changeReferencePossible(const fccl::semantics::TransformSemantics& 
            transform_semantics) const
        {
          return twist().changeReferencePossible(transform_semantics);
        }

        void partialAssignment(std::size_t start, std::size_t elements,
            const InteractionMatrixSemantics& other)
        {
          assert(twist().equals(other.twist()));

          joints().partialAssignment(start, elements, other.joints());
        }
 
      private:
        TwistSemantics twist_;
        JntArraySemantics joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const InteractionMatrixSemantics& obj)
    {
      os << "twist:\n " << obj.twist() << "\n";
      os << "joints:\n " << obj.joints();
      return os;
    }

    inline bool areMultipliable(const InteractionMatrixSemantics& interaction_matrix,
        const JacobianSemantics& jacobian)
    {
      return interaction_matrix.twist().reference().equals(jacobian.twist().reference());
    }

    inline void multiply(const InteractionMatrixSemantics& interaction_matrix, 
        const JacobianSemantics& jacobian, JointMappingSemantics& result)
    {
      assert(areMultipliable(interaction_matrix, jacobian));
      
      result.row_joints() = interaction_matrix.joints();
      result.column_joints() = jacobian.joints();
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_INTERACTION_MATRIX_SEMANTICS_H
