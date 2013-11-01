# name of the constraint used for identification in knowledge base
std_msgs/String name

# name of tf-frame w.r.t. which the constraint is defined, i.e. usually the
# frame from which the relation is 'seen'
std_msgs/String reference

# name of the function used to relate the two features
std_msgs/String function

# toolFeature which HAS to be connected to the kinematic chain controlled by
# the controller
fccl_msgs/Feature tool_feature

# objectFeature which does not need to be connected to the kinematic chain, but
# but can be
fccl_msgs/Feature object_feature

# lower bound for desired output of function --the motion goal
std_msgs/Float64 lower_boundary 

# upper bound for desired output of function --the motion goal
std_msgs/Float64 upper_boundary 
