#ifndef FCCL_CONVERSIONS_CONVERSIONS_H
#define FCCL_CONVERSIONS_CONVERSIONS_H

#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/semantics/SemanticsBase.h>
#include <fccl/kdl/JntArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <map>

using namespace fccl::semantics;
using namespace fccl::kdl;
using namespace std;

namespace fccl
{
  namespace conversions
  {
    //! A realtime-safe joint state reader.
    /*! Assumptions:
     *    - The joint names must all appear in one JointState message.
     *    - The order of joint states in msg is irrelevant.
     *    - ('holes' are allowed), i.e. there can be other joints in the msg.
     *
     *  If the joint states were distributed over several messages,
     *  then a collection of JointStateInterpreters is required. 
     */
    class JointStateInterpreter
    {
    public:
      JointStateInterpreter();
      ~JointStateInterpreter();

      // Semantics has to contain all the relevant joint names that
      // shall be extracted when calling parseJointState.
      JointStateInterpreter(const JntArraySemantics& semantics);

      // Semantics has to contain all the relevant joint names that
      // shall be extracted when calling parseJointState.
      void init(const JntArraySemantics& semantics);


      // Parse over 'msg' and updates the corresponding values in 'joint_state'.
      // Returns false if not all joints given to constructor were present.
      bool parseJointState(const sensor_msgs::JointState& msg,
          JntArray& joint_state) const;

      // Returns the semantics the interpreter is expecting.
      const JntArraySemantics& semantics() const;
    private:
      // Typedefs and internal variables to have a map that translates
      // joint names into their respective joint index in the q-vector.
      // This is necessary because the order of joints in the JointState
      // messages that come from the robot do not need to be equal to
      // their order in KDL-style. Sidenote: Had to be added for the PR2.
      typedef map<SemanticsBase, size_t> JointNameIndexMap;
      typedef pair<SemanticsBase, size_t> JointNameIndexPair;
    
      // The actual map used for storing the joint_semantics -> index.
      JointNameIndexMap joint_semantics_to_index_map_;
    
      // Internal convenience translator single_joint_name->joint_index.
      // If the return value is greater/equal joint_name_.size() then 
      // the joint name is not in the map.
      size_t getJointIndex(const SemanticsBase& joint) const;
    
      // semantics of the joints that should be extracted from a JointState
      // message when calling parseJointState()
      JntArraySemantics joint_semantics_;
    };
  } // namespace conversions
} // namespace fccl
#endif // FCCL_CONVERSIONS_CONVERSIONS_H
