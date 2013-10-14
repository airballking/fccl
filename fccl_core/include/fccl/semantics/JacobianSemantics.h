#ifndef FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H
#define FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H

#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/TwistSemantics.h>
#include <fccl/semantics/TransformSemantics.h>

namespace fccl
{
  namespace semantics
  {
    class JacobianSemantics
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

        bool equals(const JacobianSemantics& other) const
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

        void resize(std::size_t columns)
        {
          joints().resize(columns);
        }

        std::size_t size() const
        {
          return joints().size();
        }

        void changeReferenceFrame(const fccl::semantics::TransformSemantics& 
            transform_semantics)
        {
          twist().changeReferenceFrame(transform_semantics);
        }
          
        bool changeReferencePossible(const fccl::semantics::TransformSemantics& 
            transform_semantics)
        {
          return twist().changeReferencePossible(transform_semantics);
        }
 
      private:
        TwistSemantics twist_;
        JntArraySemantics joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const JacobianSemantics& obj)
    {
      os << "twist:\n " << obj.twist() << "\n";
      os << "joints:\n " << obj.joints();
      return os;
    }

    inline bool areMultipliable(const JacobianSemantics& lhs, const JntArraySemantics& rhs)
    {
      return lhs.joints().equals(rhs);
    }

    inline TwistSemantics multiply(const JacobianSemantics& lhs, const JntArraySemantics& rhs)
    {
      assert(areMultipliable(lhs, rhs));

      return lhs.twist();
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_JACOBIAN_SEMANTICS_H
