#ifndef FCCL_SEMANTICS_JOINT_MAPPING_SEMANTICS_H
#define FCCL_SEMANTICS_JOINT_MAPPING_SEMANTICS_H

#include <fccl/semantics/JntArraySemantics.h>

namespace fccl
{
  namespace semantics
  {
    class JointMappingSemantics
    {
      public:
        const JntArraySemantics& row_joints() const
        {
          return row_joints_;
        }

        JntArraySemantics& row_joints()
        {
          return row_joints_;
        }

        const JntArraySemantics& column_joints() const
        {
          return column_joints_;
        }

        JntArraySemantics& column_joints()
        {
          return column_joints_;
        }
 
        bool equals(const JointMappingSemantics& other) const
        {
          return row_joints().equals(other.row_joints()) &&
               column_joints().equals(other.column_joints());
        }

      private:
        JntArraySemantics row_joints_;
        JntArraySemantics column_joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const JointMappingSemantics& obj)
    {
      os << "row_joints:\n " << obj.row_joints() << "\n";
      os << "column_joints:\n " << obj.column_joints();
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_JOINT_MAPPING_SEMANTICS_H