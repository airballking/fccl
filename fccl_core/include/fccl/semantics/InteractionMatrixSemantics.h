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

        bool equals(const InteractionMatrixSemantics& other) const
        {
          return twist().equals(other.twist()) &&
              joints().equals(other.joints());
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
      return interaction_matrix.twist().equals(jacobian.twist());
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
