#include <fccl_conversions/JointStateInterpreter.h>

namespace fccl
{
  namespace conversions
  {
    JointStateInterpreter::JointStateInterpreter(const JntArraySemantics& semantics)
      : joint_semantics_to_index_map_(), joint_semantics_(semantics)
    {
      // building map to get from joint names to their index position in q-vector
      joint_semantics_to_index_map_.clear();
      for(size_t i=0; i<joint_semantics_.size(); i++)
        joint_semantics_to_index_map_.insert(JointNameIndexPair(joint_semantics_(i), i));
    }
    
    bool JointStateInterpreter::parseJointState(const sensor_msgs::JointState& msg,
        JntArray& q) const
    {
      assert(q.size() == joint_semantics_.size());
      assert(msg.name.size() == msg.position.size());
    
      // iterate over message and use joint names -> joint index map to check for
      // each joint whether it is part of the current kinematic chain.
      // if yes, update the corresponding entry in q.
      size_t joint_counter = 0;
      for(size_t i=0; i<msg.name.size(); i++)
      {
        // TODO(Georg): add constructor to build SemanticsBase from string
        SemanticsBase joint_semantics;
        joint_semantics.setName(msg.name[i]);
 
        size_t jnt = getJointIndex(joint_semantics);
        if(jnt < joint_semantics_.size())
        {
          assert(q.semantics()(jnt).equals(joint_semantics));

          // joint is one of the wanted ones
          q.numerics()(jnt) = msg.position[i];
          joint_counter++;
        }
      }
      return (joint_counter == joint_semantics_.size());
    }
    
    size_t JointStateInterpreter::getJointIndex(const SemanticsBase& joint) const
    {
      JointNameIndexMap::const_iterator aux_iterator =
          joint_semantics_to_index_map_.find(joint);
      if(aux_iterator != joint_semantics_to_index_map_.end())
        return aux_iterator->second; 
      else
        return joint_semantics_.size() + 1;
    }
  } // namespace conversions
} // namespace fccl
