#ifndef FCCL_SEMANTICS_KINEMATIC_CHAIN_SEMANTICS_H
#define FCCL_SEMANTICS_KINEMATIC_CHAIN_SEMANTICS_H

#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/TransformSemantics.h>

namespace fccl
{
  namespace semantics
  {
    class KinematicChainSemantics
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

        const TransformSemantics& transform() const
        {
          return transform_;
        }

        TransformSemantics& transform()
        {
          return transform_;
        }

        bool equals(const KinematicChainSemantics& other) const
        {
          return transform().equals(other.transform()) &&
              joints().equals(other.joints());
        }

        void init(const std::vector<std::string>& joint_names,
            const std::string& reference_name, const std::string& target_name)
        {
          joints().init(joint_names);
          transform().init(reference_name, target_name);
        }

        void resize(std::size_t columns)
        {
          joints().resize(columns);
        }

        std::size_t size() const
        {
          return joints().size();
        }

      private:
        TransformSemantics transform_;
        JntArraySemantics joints_;
    };

    inline std::ostream& operator<<(std::ostream& os, const KinematicChainSemantics& obj)
    {
      os << "transform:\n " << obj.transform() << "\n";
      os << "joints:\n " << obj.joints();
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_KINEMATIC_CHAIN_SEMANTICS_H
